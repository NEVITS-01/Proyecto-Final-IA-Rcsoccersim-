# main.py
import threading
import time
from jugadores import TEAM_NAME, PUERTO, JUGADORES
from ataque.atacante import jugador_ofensivo
from estrategia.pases import jugador_pasador

def main():
    print(f"🚀 Iniciando equipo {TEAM_NAME} con {JUGADORES} jugadores en puerto {PUERTO}")

    # Jugador ofensivo principal
    hilo1 = threading.Thread(target=jugador_ofensivo, args=(1,), daemon=True)
    hilo1.start()

    # Jugador pasador
    hilo2 = threading.Thread(target=jugador_pasador, args=(2,), daemon=True)
    hilo2.start()

    # Los demás jugadores (si hay más de 2)
    for jid in range(3, JUGADORES + 1):
        hilo = threading.Thread(target=jugador_pasador, args=(jid,), daemon=True)
        hilo.start()

    print("⚽ Equipo desplegado en el campo.")

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
