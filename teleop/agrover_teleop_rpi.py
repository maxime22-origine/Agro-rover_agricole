#!/usr/bin/env python3
"""
============================================================
  AGROVER MR 25.26 — Téléopération Clavier (Raspberry Pi)
  Compatible SSH — sans curses
  Liaison : USB  /dev/ttyACM0  @ 115200 bps
============================================================

  LANCEMENT :
      python3 agrover_teleop_rpi.py

  TOUCHES :
    Z → Avancer        S → Reculer
    Q → Gauche         D → Droite
    A → Pivot gauche   E → Pivot droite
    ESPACE → STOP
    P → Pompe ON/OFF   T → Lire DHT11
    + → Vitesse +      - → Vitesse -
    X → Quitter
============================================================
"""

import serial
import json
import time
import sys
import os
import threading
import tty
import termios

# ============================================================
#  CONFIGURATION
# ============================================================
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE   = 115200
TIMEOUT     = 0.5

SPEED_STEP  = 50
SPEED_MIN   = 50
SPEED_MAX   = 400

# ============================================================
#  ÉTAT GLOBAL
# ============================================================
state = {
    "speed"     : 200,
    "pump"      : False,
    "temp"      : "--",
    "hum"       : "--",
    "direction" : "STOP",
    "last_cmd"  : "Aucune",
    "last_rx"   : "En attente...",
    "status"    : "",
}

ser  = None
lock = threading.Lock()
running = True

# ============================================================
#  CONNEXION SÉRIE
# ============================================================
def connect_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(1.5)
        return True
    except serial.SerialException as e:
        print(f"[ERREUR] Impossible d'ouvrir {SERIAL_PORT} : {e}")
        return False

# ============================================================
#  ENVOI COMMANDE JSON
# ============================================================
def send_cmd(payload: dict):
    with lock:
        if not ser or not ser.is_open:
            return
        try:
            msg = json.dumps(payload) + '\n'
            ser.write(msg.encode('utf-8'))
            state["last_cmd"] = msg.strip()
        except Exception as e:
            state["status"] = f"Erreur envoi: {e}"

def cmd_move(direction):
    state["direction"] = direction
    send_cmd({"cmd": direction})

def cmd_stop():
    state["direction"] = "STOP"
    send_cmd({"cmd": "STOP"})

def cmd_pump(on):
    state["pump"] = on
    send_cmd({"cmd": "PUMP", "state": 1 if on else 0})

def cmd_sensor():
    send_cmd({"cmd": "SENSOR"})

def cmd_speed(val):
    state["speed"] = val
    send_cmd({"cmd": "SPEED", "val": val})

# ============================================================
#  THREAD LECTURE RÉPONSES OPENCR
# ============================================================
def read_thread():
    while running:
        if ser and ser.is_open:
            try:
                with lock:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    state["last_rx"] = line
                    try:
                        data = json.loads(line)
                        if "temp" in data: state["temp"] = data["temp"]
                        if "hum"  in data: state["hum"]  = data["hum"]
                        if "pump" in data: state["pump"] = (data["pump"] == 1)
                    except json.JSONDecodeError:
                        pass
            except Exception:
                pass
        time.sleep(0.05)

# ============================================================
#  AFFICHAGE
# ============================================================
ARROWS = {
    "AVANCER" : "▲  AVANCER",
    "RECULER" : "▼  RECULER",
    "GAUCHE"  : "◄  GAUCHE",
    "DROITE"  : "►  DROITE",
    "PIVOT_G" : "↺  PIVOT GAUCHE",
    "PIVOT_D" : "↻  PIVOT DROITE",
    "STOP"    : "■  STOP",
}

def display():
    os.system('clear')
    pump_str = "ON  ●" if state["pump"] else "OFF ○"
    dir_str  = ARROWS.get(state["direction"], state["direction"])
    bar_len  = int((state["speed"] - SPEED_MIN) / (SPEED_MAX - SPEED_MIN) * 20)
    bar      = "█" * bar_len + "░" * (20 - bar_len)

    print("=" * 54)
    print("   AGROVER MR 25.26 — Téléopération RPi (SSH)")
    print(f"   Port : {SERIAL_PORT} @ {BAUD_RATE} bps")
    print("=" * 54)
    print()
    print(f"  Direction  :  {dir_str}")
    print(f"  Vitesse    :  {state['speed']:3d}  [{bar}]")
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
    print("-" * 54)
    print("  Z=Avancer  S=Reculer  Q=Gauche  D=Droite")
    print("  A=PivotG   E=PivotD   ESPACE=STOP")
    print("  P=Pompe    T=DHT11    +/-=Vitesse   X=Quitter")
    print("-" * 54)

# ============================================================
#  LECTURE CLAVIER (1 caractère sans ENTRÉE)
# ============================================================
def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

# ============================================================
#  BOUCLE PRINCIPALE
# ============================================================
def main():
    global running

    print("=" * 54)
    print("  AGROVER MR 25.26 — Démarrage téléopération")
    print("=" * 54)
    print(f"\n  Connexion sur {SERIAL_PORT}...")

    if not connect_serial():
        print("\n  Vérifie que l'OpenCR est bien branché en USB.")
        sys.exit(1)

    print(f"  [OK] Connecté sur {SERIAL_PORT}")
    print("\n  Appuie sur ENTRÉE pour commencer...")
    input()

    # Thread lecture série
    t = threading.Thread(target=read_thread, daemon=True)
    t.start()

    # Ping initial
    send_cmd({"cmd": "PING"})
    time.sleep(0.3)

    display()

    try:
        while True:
            key = get_key().lower()
            state["status"] = ""

            if   key == 'z': cmd_move("AVANCER")
            elif key == 's': cmd_move("RECULER")
            elif key == 'q': cmd_move("GAUCHE")
            elif key == 'd': cmd_move("DROITE")
            elif key == 'a': cmd_move("PIVOT_G")
            elif key == 'e': cmd_move("PIVOT_D")
            elif key == ' ':
                cmd_stop()
                state["status"] = "STOP !"
            elif key == 'p':
                cmd_pump(not state["pump"])
                state["status"] = f"Pompe {'ON' if state['pump'] else 'OFF'}"
            elif key == 't':
                cmd_sensor()
                state["status"] = "Lecture DHT11..."
            elif key == '+':
                new = state["speed"] + SPEED_STEP
                cmd_speed(new if new <= SPEED_MAX else SPEED_MAX)
                state["status"] = f"Vitesse : {state['speed']}"
            elif key == '-':
                new = state["speed"] - SPEED_STEP
                cmd_speed(new if new >= SPEED_MIN else SPEED_MIN)
                state["status"] = f"Vitesse : {state['speed']}"
            elif key == 'x':
                state["status"] = "Arrêt en cours..."
                display()
                break

            display()

    except KeyboardInterrupt:
        pass

    finally:
        running = False
        print("\n\nArrêt sécurisé...")
        cmd_stop()
        cmd_pump(False)
        time.sleep(0.3)
        if ser and ser.is_open:
            ser.close()
        print("Connexion fermée. Au revoir !")

if __name__ == '__main__':
    main()
