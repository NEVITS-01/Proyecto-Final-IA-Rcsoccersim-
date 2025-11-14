# network/cliente_base.py
import socket
import select
import time
import math

SERVER_HOST = "localhost"
SERVER_PORT = 6000
RECV_TIMEOUT = 0.2

def crear_socket():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setblocking(False)
    s.bind(('', 0))
    return s

def assign_position_for_jid(jid, total):
    """Asigna una posición inicial razonable a cada jugador (hasta 11)."""
    if jid == 1:
        return (0, 0)  # atacante al centro

    posiciones = [
        (-50, 0), (-30, -12), (-30, 12),
        (-15, -10), (-15, 10), (0, -8), (0, 8),
        (15, -6), (15, 6), (35, 0), (-10, 0)
    ]
    if 1 <= jid <= len(posiciones):
        return posiciones[jid - 1]
    ang = (jid % 11) * (360 / max(11, total))
    r = 10 + (jid % 3) * 6
    x = max(-52.5, min(52.5, r * math.cos(math.radians(ang))))
    y = max(-34, min(34, r * math.sin(math.radians(ang))))
    return (x, y)

def conectar(sock, team_name="MiEquipo", version=19,
             start_pos=None, jid=None, total_players=None, wait_seconds=5):
    server_addr = (SERVER_HOST, SERVER_PORT)
    msg = f"(init {team_name} (version {version}))"
    sock.sendto(msg.encode(), server_addr)
    print(f"[network] INIT enviado a {server_addr}")

    start_time = time.time()
    while time.time() - start_time < wait_seconds:
        ready = select.select([sock], [], [], 0.25)
        if ready[0]:
            data, addr = sock.recvfrom(8192)
            text = data.decode()
            if text.strip().startswith("(init"):
                server_addr = addr
                print(f"[network] ✅ INIT confirmado por {addr}")
                break

    if start_pos is None and jid is not None and total_players is not None:
        start_pos = assign_position_for_jid(jid, total_players)

    if start_pos:
        move_cmd = f"(move {start_pos[0]} {start_pos[1]})"
        for _ in range(3):
            sock.sendto(move_cmd.encode(), server_addr)
            time.sleep(0.25)
        print(f"[network] Jugador {jid} movido a {start_pos}")

    return server_addr

def escuchar(sock, timeout=0.05):
    ready = select.select([sock], [], [], timeout)
    if ready[0]:
        data, _ = sock.recvfrom(8192)
        return data.decode()
    return None

def enviar(sock, cmd, server_addr):
    if isinstance(cmd, str):
        sock.sendto(cmd.encode(), server_addr)
    else:
        sock.sendto(cmd, server_addr)
