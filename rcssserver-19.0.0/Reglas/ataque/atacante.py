# ataque/atacante.py
"""
Jugador ofensivo:
- busca el balón,
- corre hacia él,
- cuando está cerca alinea y patea con potencia hacia la portería contraria,
- patrulla si no ve el balón.
Import robusto para evitar NameError.
"""

import time
import os
import sys
import math

# Import robusto de cliente_base (intenta importar normalmente, si falla añade rutas)
try:
    from network.cliente_base import crear_socket, conectar, escuchar, enviar
except Exception:
    this_dir = os.path.dirname(os.path.abspath(__file__))        # .../Reglas/ataque
    parent = os.path.abspath(os.path.join(this_dir, ".."))        # .../Reglas
    maybe_parent_of_parent = os.path.abspath(os.path.join(parent, ".."))
    for p in (parent, maybe_parent_of_parent):
        if p not in sys.path:
            sys.path.insert(0, p)
    from network.cliente_base import crear_socket, conectar, escuchar, enviar

from jugadores import TEAM_NAME, VERSION, JUGADORES

LOG_DIR = "logs_equipo"
os.makedirs(LOG_DIR, exist_ok=True)

KICKABLE_DISTANCE = 0.7  # distancia para considerar que "tengo el balón"
PATROL_INTERVAL = 0.6
GOAL_X = 52.5  # posición X de la portería contraria (ajusta si es necesario)
GOAL_Y = 0.0   # posición Y de la portería (centro)


def log(msg, jid):
    ts = time.strftime("%H:%M:%S")
    print(f"[{ts}] 🧨 J{jid}: {msg}")
    with open(os.path.join(LOG_DIR, f"jugador_{jid}.log"), "a") as f:
        f.write(f"[{ts}] {msg}\n")


def parse_ball(msg):
    """Extrae (dist, ang) del balón en un mensaje see. ang en grados relativos."""
    try:
        import re
        m = re.search(r"\(b\)\s*([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)", msg)
        if m:
            return float(m.group(1)), float(m.group(2))
    except Exception:
        pass
    return None, None


def _rel_ball_to_xy(dist, ang_deg):
    """Convierte (dist,ang_rel) a coordenadas (x,y) relativas al jugador."""
    rad = math.radians(ang_deg)
    x = dist * math.cos(rad)
    y = dist * math.sin(rad)
    return x, y


def _angle_from_point_to_goal(px, py, goal_x=GOAL_X, goal_y=GOAL_Y):
    """Ángulo (grados) desde (px,py) hacia la portería (en referencia al eje x)."""
    dx = goal_x - px
    dy = goal_y - py
    ang = math.degrees(math.atan2(dy, dx))
    return ang


def _normalize_angle(a):
    """Normaliza ángulo a rango (-180,180]."""
    a = (a + 180) % 360 - 180
    return a


def jugador_ofensivo(jid=1, start_pos=None):
    sock = crear_socket()
    server_addr = conectar(sock, TEAM_NAME, VERSION,
                           start_pos=start_pos, jid=jid, total_players=JUGADORES)
    log("✅ Conectado al servidor y listo para atacar", jid)

    last_seen = time.time()
    last_patrol = time.time()

    while True:
        msg = escuchar(sock)
        if msg and "(see" in msg:
            dist, ang = parse_ball(msg)
            if dist is not None:
                last_seen = time.time()
                log(f"👀 Balón detectado a {dist:.2f} m, ang {ang:.2f}°", jid)

                # Si estoy lo bastante cerca -> alinear y patear hacia el arco
                if dist <= KICKABLE_DISTANCE:
                    # posición aproximada del balón relativa al jugador
                    bx, by = _rel_ball_to_xy(dist, ang)
                    # ángulo estimado desde la posición del balón hacia la portería
                    ang_goal_est = _angle_from_point_to_goal(bx, by)
                    ang_goal_est = _normalize_angle(ang_goal_est)
                    log(f"🔁 Alineando para tiro: ang_goal_est={ang_goal_est:.1f}°", jid)

                    # Girar hacia la dirección estimada del gol
                    enviar(sock, f"(turn {ang_goal_est})", server_addr)
                    # dejar que el servidor procese la rotación
                    time.sleep(0.03)

                    # Kick potente hacia adelante (después de girar, kick 0 va en la dirección actual)
                    enviar(sock, "(kick 95 0)", server_addr)
                    log("⚽ Pateado con potencia hacia el arco (intento de gol)", jid)
                else:
                    # No está a la distancia de pateo: girar hacia el balón y avanzar
                    if abs(ang) > 3.0:
                        enviar(sock, f"(turn {ang})", server_addr)
                    fuerza = min(100, max(25, int(dist * 40)))
                    enviar(sock, f"(dash {fuerza})", server_addr)
            else:
                # Hubo see pero no (b) detectado: patrulla ligera
                if time.time() - last_patrol > PATROL_INTERVAL:
                    enviar(sock, "(turn 25)", server_addr)
                    enviar(sock, "(dash 60)", server_addr)
                    last_patrol = time.time()
        else:
            # No hay see: movimiento de búsqueda
            if time.time() - last_patrol > PATROL_INTERVAL:
                enviar(sock, "(turn 35)", server_addr)
                enviar(sock, "(dash 45)", server_addr)
                last_patrol = time.time()

        # Si no ve el balón por un tiempo, hacer búsqueda amplia
        if time.time() - last_seen > 6.0:
            enviar(sock, "(turn 60)", server_addr)
            last_seen = time.time()

        time.sleep(0.04)
