"""
Visualisation en direct du RPLidar C1, branche en USB directement sur ce PC,
sans passer par le Raspberry Pi ni ROS2.

Installation (une seule fois) :
    pip install rplidar-roboticia pyserial matplotlib numpy

Utilisation :
    1. Branche le lidar sur un port USB du PC (via son adaptateur USB-UART).
    2. Regarde dans le Gestionnaire de peripheriques > Ports (COM & LPT) quel
       port COM apparait (ex: COM5). Remplace PORT_NAME ci-dessous.
    3. Lance : python visualize_lidar.py
    4. Pour arreter : ferme simplement la fenetre (ou Ctrl+C).

NOTES TECHNIQUES :
- La librairie rplidar-roboticia lit les mesures une par une (un appel
  serial.read() par echantillon de 5 octets). Le C1 envoie des donnees a un
  debit trop eleve pour que cette boucle Python suive en temps reel (surtout
  sous Windows) : le tampon deborde, la lib redemarre le scan en boucle sans
  jamais assembler un tour complet. On lit donc nous-memes par gros blocs
  (un seul read() recupere tout ce qui est disponible), puis on decoupe ce
  bloc en paquets de 5 octets avec le parsing officiel de la lib.
- On n'appelle pas lidar.start_motor() : cette methode envoie une commande
  SET_PWM (0xF0) specifique aux anciens modeles A1/A2. Sur le C1 le moteur
  est gere en interne par le firmware, et cette commande peut laisser des
  octets residuels dans le tampon -> get_health() (appele par start())
  plante alors avec "ValueError: too many values to unpack".
- L'affichage utilise matplotlib.animation.FuncAnimation plutot qu'une
  boucle manuelle avec draw_idle()/pause() : c'est la methode robuste pour
  un affichage temps reel, qui laisse la vraie boucle d'evenements du
  backend graphique gerer les rafraichissements (une boucle manuelle peut
  laisser la fenetre "geler" en blanc selon le backend/l'environnement).
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from rplidar import RPLidar, RPLidarException
from rplidar import _process_scan  # reutilise le parsing officiel de la lib

PORT_NAME = "COM5"   # <-- a adapter selon le Gestionnaire de peripheriques
BAUDRATE = 460800     # vitesse de communication du C1 (datasheet).
                       # NE PAS laisser 115200 par defaut : c'est la valeur
                       # pour les anciens modeles A1/A2, pas pour le C1.

MIN_POINTS_PER_SCAN = 5     # tours trop courts (bruit/desync) ignores
UPDATE_INTERVAL_MS = 30     # frequence de rafraichissement de la fenetre


def main():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=3)

    print("Info  :", lidar.get_info())
    print("Sante :", lidar.get_health())

    lidar._serial.reset_input_buffer()
    lidar.start("normal")

    print("Scan demarre. Ferme la fenetre (ou Ctrl+C) pour arreter proprement.")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="polar")
    ax.set_theta_zero_location("N")
    ax.set_theta_direction(-1)
    ax.set_rmax(6)  # 6 m : portee fiable garantie sur objet sombre (datasheet C1)
    scatter = ax.scatter([], [], s=4)
    title = ax.set_title("En attente du premier tour...")

    state = {
        "raw_buf": bytearray(),
        "scan_list": [],
        "last_scan": None,
        "total_scans": 0,
    }

    def update(_frame):
        n = lidar._serial.in_waiting
        if n >= 5:
            state["raw_buf"].extend(lidar._serial.read(n))

        nb_complete = len(state["raw_buf"]) // 5
        if nb_complete:
            for i in range(nb_complete):
                packet = bytes(state["raw_buf"][i * 5:(i + 1) * 5])
                try:
                    new_scan, quality, angle, distance = _process_scan(packet)
                except RPLidarException:
                    continue  # paquet corrompu / desynchronise, on l'ignore

                if new_scan:
                    if len(state["scan_list"]) > MIN_POINTS_PER_SCAN:
                        state["last_scan"] = list(state["scan_list"])
                        state["total_scans"] += 1
                    state["scan_list"] = []

                if distance > 0:
                    state["scan_list"].append((quality, angle, distance))

            state["raw_buf"] = state["raw_buf"][nb_complete * 5:]

        if state["last_scan"]:
            angles = np.radians([a for (_q, a, _d) in state["last_scan"]])
            dists_m = np.array([d / 1000.0 for (_q, _a, d) in state["last_scan"]])
            scatter.set_offsets(np.column_stack((angles, dists_m)))
            title.set_text(f"Tour {state['total_scans']} - {len(state['last_scan'])} points")

        return scatter, title

    ani = animation.FuncAnimation(
        fig, update, interval=UPDATE_INTERVAL_MS, cache_frame_data=False
    )

    try:
        plt.show()  # bloquant : garde une reference a `ani` vivante tant que la fenetre est ouverte
    except KeyboardInterrupt:
        print("Arret demande.")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("Lidar arrete proprement.")


if __name__ == "__main__":
    main()
