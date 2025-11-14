# estrategia/pases.py
"""
Jugador pasador (completo)
- Detecta balón (desde messages see).
- Si está cerca (KICKABLE_DISTANCE) decide:
    * Si está en zona de tiro cercana al arco rival -> alinea y TIRO potente al arco.
    * Si no, busca compañero "mejor posicionado" hacia el arco y hace pase hacia él.
- Si el balón está lejos -> girar hacia el balón y avanzar (dash).
- Patrulla/busca si no ve el balón.
- Import robusto de network.cliente_base para evitar NameError si no hay __init__.py.
Ajusta GOAL_X, KICK_POWER, y ZONA_TIRO_X según necesites.
"""

import time
import os
import sys
import math

# Intento de import robusto de cliente_base
try:
    from network.cliente_base import crear_socket, conectar, escuchar, enviar
except Exception:
    this_dir = os.path.dirname(os.path.abspath(__file__))        # .../Reglas/estrategia
    parent = os.path.abspath(os.path.join(this_dir, ".."))        # .../Reglas
    maybe_parent_of_parent = os.path.abspath(os.path.join(parent, ".."))
    for p in (parent, maybe_parent_of_parent):
        if p not in sys.path:
            sys.path.insert(0, p)
    from network.cliente_base import crear_socket, conectar, escuchar, enviar

from jugadores import TEAM_NAME, VERSION, JUGADORES

LOG_DIR = "logs_equipo"
os.makedirs(LOG_DIR, exist_ok=True)

# Parámetros configurables
KICKABLE_DISTANCE = 1.0     # distancia para considerar que "tengo el balón"
PATROL_INTERVAL = 0.6
GOAL_X = 52.5               # posición X de la portería contraria (ajusta si hace falta)
GOAL_Y = 0.0
ZONA_TIRO_X = 40.0          # si la x estimada del balón (relativa al jugador) > ZONA_TIRO_X -> tirar
KICK_POWER_SHOT = 95        # potencia para el tiro al arco
KICK_POWER_PASS = 60        # potencia para pases
MIN_PASS_DISTANCE = 5.0     # evitar pasar a compañeros muy cercanos (evita rebotes inútiles)


def log(msg, jid):
    ts = time.strftime("%H:%M:%S")
    s = f"[{ts}] J{jid}: {msg}"
    print(s)
    with open(os.path.join(LOG_DIR, f"jugador_{jid}.log"), "a") as f:
        f.write(s + "\n")


def parse_ball(msg):
    """
    Extrae (dist, ang) del balón en un mensaje see.
    ang en grados relativo al jugador. Retorna (None, None) si no encuentra.
    """
    try:
        import re
        m = re.search(r"\(b\)\s*([-+]?\d*\.?\d+)\s+([-+]?\d*\.?\d+)", msg)
        if m:
            return float(m.group(1)), float(m.group(2))
    except Exception:
        pass
    return None, None


def _rel_ball_to_xy(dist, ang_deg):
    """Convierte (dist, ang_rel) a coordenadas (x,y) relativas al jugador."""
    rad = math.radians(ang_deg)
    x = dist * math.cos(rad)
    y = dist * math.sin(rad)
    return x, y


def _angle_to_point(px, py, tx, ty):
    """Ángulo en grados desde (px,py) hacia (tx,ty)."""
    dx = tx - px
    dy = ty - py
    return math.degrees(math.atan2(dy, dx))


def _normalize_angle(a):
    """Normaliza ángulo a (-180, 180]."""
    return (a + 180) % 360 - 180


def teammate_position_estimate(teammate_jid):
    """
    Estimación simple de posiciones de compañeros en el campo basada en jid.
    Se usa para decidir a quién pasar. Puedes mejorar esto leyendo posiciones reales
    si tu servidor envía la información completa de jugadores en el see.
    """
    # Patrón de posiciones ofensivas razonables, ajusta como quieras:
    mapping = {
        1: (-40, 0),
        2: (-20, -10),
        3: (-20, 10),
        4: (-5, -12),
        5: (-5, 12),
        6: (5, -8),
        7: (5, 8),
        8: (20, -6),
        9: (20, 6),
        10: (35, 0),
        11: (-10, 0)
    }
    return mapping.get((teammate_jid - 1) % 11 + 1, (0, 0))


def choose_best_teammate_towards_goal(bx, by, my_jid):
    """
    Selecciona el compañero que esté más adelantado hacia la portería rival
    y que no esté demasiado cerca del balón (evita pases inútiles).
    Se calcula con la estimación de teammate_position_estimate.
    Devuelve (teammate_jid, tx, ty, ang_pass) o (None, None, None, None) si no encuentra.
    """
    best = None
    best_score = -1e9
    for tid in range(1, JUGADORES + 1):
        if tid == my_jid:
            continue
        tx, ty = teammate_position_estimate(tid)
        # preferir compañeros con mayor X (más cerca del arco rival)
        score = tx - abs(ty) * 0.01  # ligero penalizador por distancia lateral
        # evitar compañeros extremadamente cercanos al balón (se busca progresión)
        dist_to_ball = math.hypot(tx - bx, ty - by)
        if dist_to_ball < MIN_PASS_DISTANCE:
            continue
        if score > best_score:
            best_score = score
            best = (tid, tx, ty, _angle_to_point(bx, by, tx, ty))
    if best:
        tid, tx, ty, ang = best
        return tid, tx, ty, _normalize_angle(ang)
    return None, None, None, None


def jugador_pasador(jid=2, start_pos=None):
    sock = crear_socket()
    server_addr = conectar(sock, TEAM_NAME, VERSION,
                           start_pos=start_pos, jid=jid, total_players=JUGADORES)
    print(f"🤝 Jugador {jid} (pasador) listo en campo")

    last_seen = time.time()
    last_patrol = time.time()

    while True:
        msg = escuchar(sock)
        if msg and "(see" in msg:
            dist, ang = parse_ball(msg)
            if dist is not None:
                last_seen = time.time()
                bx, by = _rel_ball_to_xy(dist, ang)  # posición aproximada del balón relativa al jugador
                log(f"Balón detectado: dist={dist:.2f} ang={ang:.1f} → bx={bx:.2f}, by={by:.2f}", jid)

                # Si estoy cerca del balón -> decidir entre tiro o pase
                if dist <= KICKABLE_DISTANCE:
                    # Estimación: si la posición del balón (en coordenadas relativas) está
                    # suficientemente adelantada hacia la portería rival, tiramos al arco.
                    # Nota: bx es la distancia hacia adelante desde el jugador; sumando la posición
                    # real del jugador podría dar la X del campo, pero aquí aproximamos con bx.
                    if bx >= ZONA_TIRO_X:
                        # Girar hacia el arco y disparar con potencia
                        ang_goal = _normalize_angle(_angle_to_point(bx, by, GOAL_X, GOAL_Y))
                        log(f"Zona de tiro alcanzada (bx={bx:.2f}). ang_goal_est={ang_goal:.1f}° -> TIRO", jid)
                        enviar(sock, f"(turn {ang_goal})", server_addr)
                        time.sleep(0.03)
                        enviar(sock, f"(kick {KICK_POWER_SHOT} 0)", server_addr)
                        log("⚽ TIRO FUERTE AL ARCO (intento)", jid)
                    else:
                        # No hay tiro claro -> buscar compañero mejor posicionado
                        tid, tx, ty, ang_pass = choose_best_teammate_towards_goal(bx, by, jid)
                        if tid is not None:
                            log(f"Se pasa a J{tid} (estim pos {tx:.1f},{ty:.1f}) ang_pass={ang_pass:.1f}°", jid)
                            # Girar hacia la dirección del pase y patear (kick 0 hacia adelante)
                            enviar(sock, f"(turn {ang_pass})", server_addr)
                            time.sleep(0.02)
                            enviar(sock, f"(kick {KICK_POWER_PASS} 0)", server_addr)
                            log(f"⚽ Pase realizado a J{tid}", jid)
                        else:
                            # No hay compañero válido (raro): intentar avanzar y retener
                            log("No hay compañero claro para pasar -> intento de tiro o retener", jid)
                            ang_goal = _normalize_angle(_angle_to_point(bx, by, GOAL_X, GOAL_Y))
                            enviar(sock, f"(turn {ang_goal})", server_addr)
                            time.sleep(0.03)
                            enviar(sock, f"(kick {KICK_POWER_PASS} 0)", server_addr)
                else:
                    # Balón lejos: girar hacia balón y avanzar
                    if abs(ang) > 3.0:
                        enviar(sock, f"(turn {ang})", server_addr)
                    fuerza = min(90, max(20, int(dist * 35)))
                    enviar(sock, f"(dash {fuerza})", server_addr)
            else:
                # Vio see pero no encontró (b) -> pequeña patrulla
                if time.time() - last_patrol > PATROL_INTERVAL:
                    enviar(sock, "(turn 20)", server_addr)
                    enviar(sock, "(dash 50)", server_addr)
                    last_patrol = time.time()
        else:
            # No hay see: patrulla / busca
            if time.time() - last_patrol > PATROL_INTERVAL:
                enviar(sock, "(dash 40)", server_addr)
                last_patrol = time.time()

        # Si no ve balón por tiempo, búsqueda más amplia
        if time.time() - last_seen > 6.0:
            enviar(sock, "(turn 45)", server_addr)
            last_seen = time.time()

        time.sleep(0.04)
