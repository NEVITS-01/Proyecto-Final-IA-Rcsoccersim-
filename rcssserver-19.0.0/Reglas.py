#!/usr/bin/env python3
"""
agente_ofensivo.py

Cliente simple con lógica de un único agente ofensivo para rcssserver (RoboCup 2D).
- Usa como referencia el cliente que ya tenías.
- Añade un filtro de Kalman para estimar posición/velocidad del balón.
- Lógica ofensiva simple: interceptar balón, decidir (shoot/dribble/pass) y ejecutar.
- Diseñado para servidor en localhost:6000 por defecto.

Cómo usar:
- Asegúrate que tu rcssserver está corriendo en localhost puerto 6000.
- Ejecuta: python3 agente_ofensivo.py
"""

import socket
import select
import threading
import time
import math
import os
from typing import Optional, Tuple

# ---------------- CONFIG ----------------
TEAM_NAME = "MiEquipo"
VERSION = 19
N_PLAYERS = 1
POSICIONES_INICIALES = [(-8, 0)]  # delantero listo cerca medio campo
SERVER_HOST = "localhost"
SERVER_PORT = 6000

RECV_TIMEOUT = 0.2
ACTION_INTERVAL = 0.08
LOG_DIR = "logs_agente"
os.makedirs(LOG_DIR, exist_ok=True)

# Parameters for our simple model
KICKABLE_DISTANCE = 1.0   # distancia aproximada para poder chutar
MAX_DASH_POWER = 100.0
GOAL_X = 52.5             # coordenada x de la portería rival (campo típico width 105 -> goal x = +52.5)
GOAL_Y = 0.0

# ---------------- utilidades ----------------
def log(msg: str, jid: int = 1):
    ts = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] Jugador{jid}: {msg}"
    print(line)
    with open(os.path.join(LOG_DIR, f"jugador_{jid}.log"), "a") as f:
        f.write(line + "\n")

def deg2rad(d): return d * math.pi / 180.0
def rad2deg(r): return r * 180.0 / math.pi
def clamp(x, a, b): return max(a, min(b, x))

# ---------------- Kalman Filter (2D pos + vel) ----------------
class SimpleKalman2D:
    """
    Estado: [x, y, vx, vy]^T.
    Matrices discretizadas con dt variable (predicción simple).
    Notación simplificada, covariancias pequeñas.
    """
    def __init__(self):
        # state
        self.x = None  # 4x1
        # covariance
        self.P = None  # 4x4
        # process noise (tunable)
        self.q_pos = 0.05
        self.q_vel = 0.5
        # measurement noise
        self.r_pos = 0.5

        self.last_t = None

    def initialize(self, px, py, t):
        self.x = [px, py, 0.0, 0.0]
        self.P = [
            [1.0,0,0,0],
            [0,1.0,0,0],
            [0,0,1.0,0],
            [0,0,0,1.0]
        ]
        self.last_t = t
        log(f"Kalman inicializado en ({px:.2f},{py:.2f}) t={t:.2f}")

    def predict(self, t):
        if self.x is None:
            return
        dt = max(1e-6, t - self.last_t)
        # state transition
        x, y, vx, vy = self.x
        nx = x + vx * dt
        ny = y + vy * dt
        nvx = vx
        nvy = vy
        self.x = [nx, ny, nvx, nvy]
        # update covariance (approx)
        q_pos = self.q_pos
        q_vel = self.q_vel
        # simple diag growth
        for i in range(4):
            for j in range(4):
                self.P[i][j] = self.P[i][j]  # keep shape
        self.P[0][0] += q_pos * dt
        self.P[1][1] += q_pos * dt
        self.P[2][2] += q_vel * dt
        self.P[3][3] += q_vel * dt
        self.last_t = t

    def update_position(self, px, py, t):
        if self.x is None:
            self.initialize(px, py, t)
            return
        # predict to time t first
        self.predict(t)
        # measurement residual
        mx = px
        my = py
        x, y, vx, vy = self.x
        # simple Kalman gain on position only (very simplified scalar gains)
        # Compute innovation covariance for pos as Ppos + R
        Sx = self.P[0][0] + self.r_pos
        Sy = self.P[1][1] + self.r_pos
        Kx = self.P[0][0] / Sx
        Ky = self.P[1][1] / Sy
        # update state (we also lightly update velocity by finite diff)
        vx_est = vx
        vy_est = vy
        # estimate new velocity as small blend between old and measured derivative
        # derivative approx (mx - x) / dt
        dt = max(1e-6, t - (self.last_t - (t - self.last_t))) if t!=self.last_t else 0.05
        if dt <= 0: dt = 0.05
        measured_vx = (mx - x) / dt
        measured_vy = (my - y) / dt
        new_x = x + Kx * (mx - x)
        new_y = y + Ky * (my - y)
        new_vx = 0.6*vx + 0.4*measured_vx
        new_vy = 0.6*vy + 0.4*measured_vy
        self.x = [new_x, new_y, new_vx, new_vy]
        # reduce covariance a bit
        self.P[0][0] *= (1 - Kx)
        self.P[1][1] *= (1 - Ky)
        # leave velocity covariances
        self.last_t = t

    def get_state(self):
        if self.x is None:
            return None
        return tuple(self.x)

# ---------------- Parser mínimo de mensajes (see) ----------------
def parse_see_ball(msg_block: str):
    """
    Busca la información de la bola en un bloque de 'see' y devuelve (dist, ang)
    dist: distancia en unidades del servidor, ang: en grados relativo al jugador (sentido positivo: izquierda?)
    Retorna None si no está.
    Ejemplo de subcadena: "(b 3.4 -12.3)"
    """
    try:
        # localizar "(b " y tomar hasta ")"
        idx = msg_block.find("(b ")
        if idx == -1:
            return None
        sub = msg_block[idx:]
        # encontrar cierre ')'
        end = sub.find(")")
        if end == -1:
            return None
        content = sub[1:end]  # "b 3.4 -12.3"
        parts = content.split()
        if len(parts) >= 3:
            d = float(parts[1])
            a = float(parts[2])
            return (d, a)
    except Exception:
        return None
    return None

# Convertir coordenadas polares relativas del servidor a coords cartesianas RELATIVAS al jugador
def polar_to_cartesian(dist, ang_deg):
    ang_rad = deg2rad(ang_deg)
    x = dist * math.cos(ang_rad)
    y = dist * math.sin(ang_rad)
    return (x, y)

# ---------------- Lógica ofensiva (árbol de decisión simplificado) ----------------
class AgentLogic:
    def __init__(self, jid:int=1):
        self.jid = jid
        self.kalman = SimpleKalman2D()
        self.last_ball_rel = None  # (dx,dy) relative to player
        self.player_dir = 0.0      # orientation degrees (approx), we keep an internal orientation
        self.player_pos = (0.0, 0.0) # not strictly available; we keep approximate
        self.time0 = time.time()

    def observe_ball(self, dist_angle, t_now):
        # dist_angle = (dist, angle_deg)
        d, a = dist_angle
        # convert to relative cartesian (player-centric)
        bx_rel_x, bx_rel_y = polar_to_cartesian(d, a)
        self.last_ball_rel = (bx_rel_x, bx_rel_y)
        # estimate global ball position using our approximate player_pos and player_dir
        # rotate relative coords by player_dir
        theta = deg2rad(self.player_dir)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # rotate and translate
        bx_global_x = self.player_pos[0] + (bx_rel_x * cos_t - bx_rel_y * sin_t)
        bx_global_y = self.player_pos[1] + (bx_rel_x * sin_t + bx_rel_y * cos_t)
        # feed kalman
        self.kalman.update_position(bx_global_x, bx_global_y, t_now)
        log(f"Observación bola rel=({bx_rel_x:.2f},{bx_rel_y:.2f}) global=({bx_global_x:.2f},{bx_global_y:.2f})", self.jid)

    def predict_ball(self, t_now):
        self.kalman.predict(t_now)
        s = self.kalman.get_state()
        if s:
            x,y,vx,vy = s
            return (x,y,vx,vy)
        return None

    def decide_and_act(self, sock:socket.socket, server_addr, last_seen_time, t_now):
        """
        Decide acción según estado:
        - Si el balón está dentro de KICKABLE_DISTANCE (aprox), girar hacia portería y kick.
        - Si no: calcular punto de intercepción (usando predicción lineal) y moverse (turn+dash) hacia allí.
        """
        state = self.kalman.get_state()
        # si no tenemos estado de bola, patrullar
        if state is None:
            # gira y dash para buscar
            sock.sendto(b"(turn 25)", server_addr)
            time.sleep(0.02)
            sock.sendto(b"(dash 40)", server_addr)
            log("Patrulla: turn + dash (no bola)", self.jid)
            return

        bx, by, bvx, bvy = state
        # estimación de posición del jugador (aprox). Para esta demo, asumimos player_pos interna es cierta.
        px, py = self.player_pos
        # distancia desde jugador a bola (global)
        dx = bx - px
        dy = by - py
        dist_player_ball = math.hypot(dx, dy)

        # angular to ball relative (in degrees), from player's forward axis (player_dir)
        angle_to_ball = rad2deg(math.atan2(dy, dx)) - self.player_dir
        # normalizar a [-180,180]
        while angle_to_ball > 180: angle_to_ball -= 360
        while angle_to_ball < -180: angle_to_ball += 360

        # Decision tree (simple):
        # If close enough -> shoot to goal
        # Else -> move to intercept point
        if dist_player_ball <= KICKABLE_DISTANCE:
            # aim to goal
            angle_to_goal = rad2deg(math.atan2(GOAL_Y - py, GOAL_X - px)) - self.player_dir
            while angle_to_goal > 180: angle_to_goal -= 360
            while angle_to_goal < -180: angle_to_goal += 360
            # rotate toward goal
            cmd_turn = f"(turn {angle_to_goal:.1f})"
            sock.sendto(cmd_turn.encode(), server_addr)
            time.sleep(0.02)
            # choose power by distance to goal: farther = more power
            dist_to_goal = math.hypot(GOAL_X - px, GOAL_Y - py)
            kick_power = clamp(80 + (dist_to_goal * 0.8), 20, 120)
            cmd_kick = f"(kick {kick_power:.1f} {GOAL_Y:.1f})"  # direction uses y? server kick takes power and angle or (kick power direction)? We'll use (kick power y) as simplified placeholder
            # Note: many servers accept (kick power direction) where direction is angle; we use a simple variant below:
            # Use kick with angle toward goal:
            goal_angle_rel = rad2deg(math.atan2(GOAL_Y - py, GOAL_X - px)) - self.player_dir
            while goal_angle_rel > 180: goal_angle_rel -= 360
            while goal_angle_rel < -180: goal_angle_rel += 360
            cmd_kick2 = f"(kick {kick_power:.1f} {goal_angle_rel:.1f})"
            sock.sendto(cmd_kick2.encode(), server_addr)
            log(f"Tirar: dist_ball={dist_player_ball:.2f} turn={angle_to_goal:.1f} kick={kick_power:.1f} angle={goal_angle_rel:.1f}", self.jid)
            return

        # If not in kick range, compute simple intercept assuming linear ball motion
        # simulate future steps and find t where agent can reach near ball
        agent_max_speed = 1.2  # approx units/sec -- tunear según server
        best_t = None
        intercept_pt = None
        for dt in [i*0.2 for i in range(1, 51)]:  # up to 10s ahead
            bx_future = bx + bvx * dt
            by_future = by + bvy * dt
            # distance agent -> future ball
            d = math.hypot(bx_future - px, by_future - py)
            # time agent needs (rough) = d / (agent_max_speed * factor)
            t_agent = d / (agent_max_speed + 1e-6)
            # if agent can arrive before or about same time as ball
            if t_agent <= dt + 0.4:  # small slack
                best_t = dt
                intercept_pt = (bx_future, by_future)
                break

        if intercept_pt is None:
            # can't intercept -> move to a strategic position: line between ball and goal at some fraction
            # compute point 1/3 from ball towards goal
            bx_state = bx
            by_state = by
            vec_to_goal = (GOAL_X - bx_state, GOAL_Y - by_state)
            ipt = (bx_state + vec_to_goal[0]*0.25, by_state + vec_to_goal[1]*0.25)
            intercept_pt = ipt
            log("No se puede interceptar a tiempo; posicionándose en línea balón-portería", self.jid)
        else:
            log(f"Intercepción posible en dt={best_t:.2f} en punto ({intercept_pt[0]:.2f},{intercept_pt[1]:.2f})", self.jid)

        # Now steer toward intercept_pt
        tx, ty = intercept_pt
        dx_t = tx - px
        dy_t = ty - py
        angle_to_target = rad2deg(math.atan2(dy_t, dx_t)) - self.player_dir
        while angle_to_target > 180: angle_to_target -= 360
        while angle_to_target < -180: angle_to_target += 360
        # compute dash power roughly proportional to distance
        dist_target = math.hypot(dx_t, dy_t)
        dash_power = clamp(20 + dist_target*12, 20, MAX_DASH_POWER)
        # send turn then dash
        sock.sendto(f"(turn {angle_to_target:.1f})".encode(), server_addr)
        time.sleep(0.02)
        sock.sendto(f"(dash {dash_power:.1f})".encode(), server_addr)
        log(f"Moviendose a intercepción: target=({tx:.2f},{ty:.2f}) dist={dist_target:.2f} turn={angle_to_target:.1f} dash={dash_power:.1f}", self.jid)

    # optional: update our internal approx of player position and orientation when we receive sense_body messages
    def update_self_state_from_sense(self, sense_msg: str):
        # Intent: buscar "(p x y dir ...)" o "((self ...))" no siempre presente; se hace heurístico
        # Ejemplo: "(sense_body (self 0 0 20 ...))" - esto depende de la versión del servidor
        # Para robustez, no confiar demasiado; si se encuentra un "(p x y)" se actualiza
        # Aquí haremos una detección simple: buscar " (p " pattern -> "(p X Y)"
        try:
            # muy heurístico
            idx = sense_msg.find("(p ")
            if idx != -1:
                sub = sense_msg[idx:]
                end = sub.find(")")
                if end != -1:
                    content = sub[1:end]  # "p X Y"
                    parts = content.split()
                    if len(parts) >= 3:
                        px = float(parts[1])
                        py = float(parts[2])
                        self.player_pos = (px, py)
                        log(f"Actualizada pos propia aprox a ({px:.2f},{py:.2f})", self.jid)
            # detect orientation token "dir" possibility: "(self X Y DIR ...)"
            # fallback: no-op
        except Exception:
            pass

# ---------------- hilo jugador ----------------
def crear_socket_local() -> socket.socket:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    s.bind(('', 0))
    return s

def jugador_thread(jid:int, start_pos:Tuple[float,float]):
    sock = crear_socket_local()
    server_addr = (SERVER_HOST, SERVER_PORT)
    init_cmd = f"(init {TEAM_NAME} (version {VERSION}))"
    try:
        sock.sendto(init_cmd.encode(), server_addr)
    except Exception as e:
        log(f"ERROR enviando init: {e}", jid)
        return
    log(f"INIT enviado: {init_cmd}", jid)

    # Esperar respuesta inicial
    respuesta = None
    start_wait = time.time()
    while time.time() - start_wait < 5:
        ready = select.select([sock], [], [], RECV_TIMEOUT)
        if ready[0]:
            try:
                data, addr = sock.recvfrom(4096)
                respuesta = data.decode()
                server_addr = (SERVER_HOST, addr[1])
                log(f"Respuesta init: {respuesta.strip()[:200]} (server puerto {addr[1]})", jid)
                break
            except Exception as e:
                log(f"ERROR recv init: {e}", jid)
                break
    if not respuesta:
        log("No se recibió respuesta del servidor al INIT. Abortando.", jid)
        sock.close()
        return

    # send initial move
    move_x, move_y = start_pos
    sock.sendto(f"(move {move_x} {move_y})".encode(), server_addr)
    log(f"Enviado MOVE inicial {move_x},{move_y}", jid)

    # instantiate logic
    logic = AgentLogic(jid=jid)
    logic.player_pos = start_pos
    last_action_time = time.time()
    last_seen_time = None

    try:
        while True:
            ready = select.select([sock], [], [], 0.01)
            if ready[0]:
                try:
                    data, addr = sock.recvfrom(8192)
                    msg = data.decode()
                except Exception:
                    msg = None

                if not msg:
                    continue
                # log partial
                log(f"RECIBIDO: {msg.strip()[:300]}", jid)

                # update self sensing if present (heuristic)
                logic.update_self_state_from_sense(msg)

                # detect play_on
                if "play_on" in msg and "init" not in msg:
                    # could set a flag; for now it's ok
                    pass

                # parse see ball
                if "(see" in msg:
                    ball = parse_see_ball(msg)
                    if ball is not None:
                        last_seen_time = time.time()
                        logic.observe_ball(ball, last_seen_time)

            # periodic decision / action
            if time.time() - last_action_time >= ACTION_INTERVAL:
                tnow = time.time()
                logic.decide_and_act(sock, server_addr, last_seen_time, tnow)
                last_action_time = tnow

            time.sleep(0.002)

    except KeyboardInterrupt:
        log("Interrupción por teclado", jid)
    except Exception as e:
        log(f"Excepción en jugador: {e}", jid)
    finally:
        try:
            sock.sendto(b"(bye)", server_addr)
        except Exception:
            pass
        sock.close()
        log("Socket cerrado", jid)

def main():
    if N_PLAYERS <= 0 or N_PLAYERS > 11:
        print("N_PLAYERS debe estar entre 1 y 11")
        return
    if len(POSICIONES_INICIALES) < N_PLAYERS:
        print("Define al menos N_PLAYERS posiciones iniciales")
        return
    threads = []
    for i in range(N_PLAYERS):
        pos = POSICIONES_INICIALES[i]
        t = threading.Thread(target=jugador_thread, args=(i+1, pos), daemon=True)
        t.start()
        threads.append(t)
        time.sleep(0.15)
    print(f"Lanzados {N_PLAYERS} agentes. Logs en '{LOG_DIR}'")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Deteniendo...")

if __name__ == "__main__":
    main()
