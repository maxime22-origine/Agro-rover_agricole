"""
============================================================
  AGROVER MR 25.26 — Téléopération Clavier (Windows)
  Connexion : OpenCR via USB → Port COM Windows (ex: COM7)
============================================================

  INSTALLATION (une seule fois) :
      pip install pyserial

  LANCEMENT :
      python agrover_teleop_windows.py

  TOUCHES :
  ┌──────────────────────────────────────┐
  │  Z  →  Avancer                      │
  │  S  →  Reculer                      │
  │  Q  →  Tourner à gauche             │
  │  D  →  Tourner à droite             │
  │  A  →  Pivoter sur place gauche     │
  │  E  →  Pivoter sur place droite     │
  │  ESPACE  →  STOP (arrêt immédiat)   │
  │  P  →  Pompe ON / OFF               │
  │  T  →  Lire capteur DHT11           │
  │  +  →  Augmenter vitesse            │
  │  -  →  Réduire vitesse              │
  │  X  →  Quitter                      │
  └──────────────────────────────────────┘
============================================================
"""

import serial
import serial.tools.list_ports
import json
import time
import msvcrt   # module Windows intégré, aucune installation requise
import os
import sys

# ============================================================
#  CONFIGURATION
# ============================================================
COM_PORT   = 'COM7'       # ← Change si besoin (vérifie dans Gestionnaire de périphériques)
BAUD_RATE  = 115200
TIMEOUT    = 0.1

SPEED_STEP = 0.05         # incrément vitesse (m/s)
SPEED_MIN  = 0.05
SPEED_MAX  = 0.26         # limite sécurité cahier des charges
TURN_RATIO = 0.6          # réduction vitesse en virage

# ============================================================
#  ÉTAT DU SYSTÈME
# ============================================================
state = {
    "speed"     : 200,           # vitesse brute Dynamixel (50-400)
    "pump"      : False,
    "temp"      : "--",
    "hum"       : "--",
    "direction" : "STOP",
    "last_cmd"  : "Aucune",
    "last_rx"   : "En attente...",
    "status"    : "",
}

ser = None

# ============================================================
#  DÉTECTION AUTOMATIQUE DU PORT COM
# ============================================================
def detect_opencr_port():
    """Cherche automatiquement le port OpenCR/STM32."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").upper()
        # OpenCR apparaît souvent avec ces noms
        if any(k in desc for k in ["OPENCR", "STM32", "ROBOTIS", "USB SERIAL"]):
            return p.device
    return None

# ============================================================
#  CONNEXION SÉRIE
# ============================================================
def connect():
    global ser
    global COM_PORT

    # Tentative détection auto
    auto = detect_opencr_port()
    if auto:
        COM_PORT = auto
        print(f"[AUTO] OpenCR détecté sur {COM_PORT}")
    else:
        print(f"[INFO] Pas de détection auto, utilisation de {COM_PORT}")

    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(1.5)
        print(f"[OK] Connexion établie sur {COM_PORT}")
        return True
    except serial.SerialException as e:
        print(f"[ERREUR] {e}")
        print("\nPorts disponibles sur cette machine :")
        for p in serial.tools.list_ports.comports():
            print(f"   {p.device} — {p.description}")
        print(f"\nModifie la variable COM_PORT dans le script.")
        return False

# ============================================================
#  ENVOI COMMANDE JSON → OPENCR
# ============================================================
def send(payload: dict):
    if not ser or not ser.is_open:
        return
    try:
        msg = json.dumps(payload) + '\n'
        ser.write(msg.encode())
        state["last_cmd"] = msg.strip()
    except Exception as e:
        state["status"] = f"Erreur envoi: {e}"

def move(direction: str):
    state["direction"] = direction
    send({"cmd": direction})

def stop():
    state["direction"] = "STOP"
    send({"cmd": "STOP"})

def pump(on: bool):
    state["pump"] = on
    send({"cmd": "PUMP", "state": 1 if on else 0})

def read_sensor():
    send({"cmd": "SENSOR"})

# ============================================================
#  LECTURE RÉPONSES OPENCR (non bloquant)
# ============================================================
def read_response():
    if not ser or not ser.is_open:
        return
    try:
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                state["last_rx"] = line
                try:
                    data = json.loads(line)
                    if "temp" in data:
                        state["temp"] = data["temp"]
                    if "hum"  in data:
                        state["hum"]  = data["hum"]
                    if "pump" in data:
                        state["pump"] = (data["pump"] == 1)
                except json.JSONDecodeError:
                    pass
    except Exception:
        pass

# ============================================================
#  AFFICHAGE TERMINAL WINDOWS
# ============================================================
def display():
    os.system('cls')

    arrows = {
        "AVANCER" : "▲  AVANCE",
        "RECULER" : "▼  RECULE",
        "GAUCHE"  : "◄  TOURNE GAUCHE",
        "DROITE"  : "►  TOURNE DROITE",
        "PIVOT_G" : "↺  PIVOT GAUCHE",
        "PIVOT_D" : "↻  PIVOT DROITE",
        "STOP"    : "■  STOP",
    }
    dir_str  = arrows.get(state["direction"], "~  " + state["direction"])
    pump_str = "ON  ●" if state["pump"] else "OFF ○"

    print("=" * 52)
    print("   AGROVER MR 25.26 — Téléopération Windows")
    print(f"   Port : {COM_PORT} @ {BAUD_RATE} bps")
    print("=" * 52)
    print()
    print(f"  Direction  :  {dir_str}")
    print(f"  Vitesse    :  {state['speed']}  (raw Dynamixel, 50-400)")
    print()
    print(f"  Pompe      :  {pump_str}")
    print(f"  Temp.      :  {state['temp']} °C")
    print(f"  Humidité   :  {state['hum']} %")
    print()
    print(f"  → Envoyé   :  {state['last_cmd']}")
    print(f"  ← Reçu     :  {state['last_rx']}")
    if state["status"]:
        print(f"\n  *** {state['status']} ***")
    print()
    print("-" * 52)
    print("  Z=Avancer  S=Reculer  Q=Gauche  D=Droite")
    print("  A=Pivot G  E=Pivot D  ESPACE=STOP")
    print("  P=Pompe    T=DHT11    +/-=Vitesse   X=Quitter")
    print("-" * 52)

# ============================================================
#  BOUCLE PRINCIPALE
# ============================================================
def main():
    global ser

    print("=" * 52)
    print("  AGROVER MR 25.26 — Démarrage téléopération")
    print("=" * 52)

    if not connect():
        print("\nAppuie sur ENTRÉE pour quitter.")
        input()
        sys.exit(1)

    # Ping initial
    send({"cmd": "PING"})
    time.sleep(0.3)
    read_response()

    print("\n[OK] Robot prêt ! Appuie sur une touche pour commencer...")
    msvcrt.getch()

    last_refresh = 0

    try:
        while True:
            # Lecture réponses OpenCR
            read_response()

            # Rafraîchissement écran (~10 Hz)
            now = time.time()
            if now - last_refresh > 0.1:
                display()
                last_refresh = now

            # Lecture touche clavier (non bloquant)
            if msvcrt.kbhit():
                raw = msvcrt.getch()

                # Touches spéciales (flèches) → préfixe b'\xe0'
                if raw in (b'\xe0', b'\x00'):
                    raw2 = msvcrt.getch()
                    if   raw2 == b'H': raw = b'z'   # ↑ = avancer
                    elif raw2 == b'P': raw = b's'   # ↓ = reculer
                    elif raw2 == b'K': raw = b'q'   # ← = gauche
                    elif raw2 == b'M': raw = b'd'   # → = droite

                key = raw.decode('utf-8', errors='ignore').lower()
                spd = state["speed"]

                state["status"] = ""

                if   key == 'z': move("AVANCER")
                elif key == 's': move("RECULER")
                elif key == 'q': move("GAUCHE")
                elif key == 'd': move("DROITE")
                elif key == 'a': move("PIVOT_G")
                elif key == 'e': move("PIVOT_D")

                elif key == ' ':                    # STOP
                    stop()
                    state["status"] = "STOP !"

                elif key == 'p':                    # Pompe ON/OFF
                    pump(not state["pump"])
                    state["status"] = f"Pompe {'ON' if state['pump'] else 'OFF'}"

                elif key == 't':                    # DHT11
                    read_sensor()
                    state["status"] = "Lecture DHT11..."

                elif key == '+':                    # Vitesse +
                    state["speed"] = min(state["speed"] + 50, 400)
                    send({"cmd": "SPEED", "val": state["speed"]})
                    state["status"] = f"Vitesse : {state['speed']}"

                elif key == '-':                    # Vitesse -
                    state["speed"] = max(state["speed"] - 50, 50)
                    send({"cmd": "SPEED", "val": state["speed"]})
                    state["status"] = f"Vitesse : {state['speed']}"

                elif key == 'x':                    # Quitter
                    state["status"] = "Arrêt en cours..."
                    display()
                    break

            time.sleep(0.02)

    except KeyboardInterrupt:
        pass

    finally:
        print("\n\nArrêt sécurisé...")
        stop()
        pump(False)
        time.sleep(0.3)
        if ser and ser.is_open:
            ser.close()
        print("Connexion fermée. Au revoir !")

# ============================================================
if __name__ == '__main__':
    main()
