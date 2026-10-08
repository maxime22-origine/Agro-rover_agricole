"""
============================================================
  AGROVER MR 25.26 — IHM Web Complète (v3.0)
  Flask : caméra MJPEG + pompe (série) + gestion subprocess ROS2
  ROS2  : via rosbridge WebSocket (port 9090) → côté navigateur
============================================================
  Démarrage :
    cd ~/agrover_ihm && python3 app.py

  Prérequis RPi :
    sudo apt install ros-humble-rosbridge-suite
    pip3 install flask flask-socketio pyserial opencv-python --break-system-packages
============================================================
"""

import os, json, time, threading, subprocess
import cv2, serial
from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO

# ── Configuration ────────────────────────────────────────────
SERIAL_PORT    = '/dev/ttyACM0'
SERIAL_BAUD    = 115200
CAM_DEVICE     = 0
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
JPEG_QUALITY   = 70
MAP_SAVE_PATH  = '/home/rover/maps/agrover_map'
ROS_DOMAIN_ID  = '0'
ROSBRIDGE_PORT = 9090

app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('AGROVER_SECRET_KEY') or os.urandom(24).hex()
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

# ── État global ───────────────────────────────────────────────
g_ser          = None
g_serial_lock  = threading.Lock()
g_latest_frame = None
g_cam_lock     = threading.Lock()
g_ros_proc     = None     # processus ros2 launch en cours
g_bridge_proc  = None     # processus rosbridge
g_current_mode = 'idle'   # idle | bringup | slam | nav


# ═══════════════════════════════════════════════════════════════
#  SERIAL  (OpenCR — pompe uniquement, le reste passe par ROS2)
# ═══════════════════════════════════════════════════════════════

def init_serial():
    global g_ser
    try:
        g_ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"[SERIAL] Connecté : {SERIAL_PORT}")
    except Exception as e:
        print(f"[SERIAL] Indisponible ({e}) — pompe désactivée.")
        g_ser = None


def serial_send(cmd: dict) -> dict:
    if g_ser is None:
        return {"error": "no_serial"}
    try:
        with g_serial_lock:
            g_ser.write((json.dumps(cmd) + '\n').encode())
            resp = g_ser.readline().decode().strip()
        return json.loads(resp) if resp else {"ack": "ok"}
    except Exception as e:
        return {"error": str(e)}


# ═══════════════════════════════════════════════════════════════
#  CAMÉRA  (OpenCV + stream MJPEG)
# ═══════════════════════════════════════════════════════════════

def camera_loop():
    global g_latest_frame
    cap = cv2.VideoCapture(CAM_DEVICE, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 30)
    print(f"[CAM] /dev/video{CAM_DEVICE} ouvert")
    while True:
        ok, frame = cap.read()
        if ok:
            frame = cv2.flip(frame, -1)          # caméra montée à l'envers
            with g_cam_lock:
                g_latest_frame = frame
        else:
            time.sleep(0.05)


def generate_frames():
    while True:
        with g_cam_lock:
            frame = g_latest_frame
        if frame is None:
            time.sleep(0.033)
            continue
        ok, buf = cv2.imencode('.jpg', frame,
                               [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                   + buf.tobytes() + b'\r\n')
        time.sleep(0.033)


# ═══════════════════════════════════════════════════════════════
#  ROSBRIDGE  (démarrage automatique au lancement Flask)
# ═══════════════════════════════════════════════════════════════

def start_rosbridge():
    global g_bridge_proc
    if g_bridge_proc and g_bridge_proc.poll() is None:
        print("[ROSBRIDGE] Déjà actif.")
        return
    cmd = ['ros2', 'launch', 'rosbridge_server',
           'rosbridge_websocket_launch.xml',
           f'port:={ROSBRIDGE_PORT}']
    env = {**os.environ, 'ROS_DOMAIN_ID': ROS_DOMAIN_ID}
    try:
        g_bridge_proc = subprocess.Popen(
            cmd, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[ROSBRIDGE] PID={g_bridge_proc.pid}, port={ROSBRIDGE_PORT}")
    except FileNotFoundError:
        print("[ROSBRIDGE] 'ros2' introuvable — "
              "source /opt/ros/humble/setup.bash avant de lancer app.py")


# ═══════════════════════════════════════════════════════════════
#  GESTION MODES ROS2
# ═══════════════════════════════════════════════════════════════

def kill_ros_launch():
    global g_ros_proc, g_current_mode
    if g_ros_proc and g_ros_proc.poll() is None:
        g_ros_proc.terminate()
        try:
            g_ros_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            g_ros_proc.kill()
        print("[ROS2] Processus terminé.")
    g_ros_proc     = None
    g_current_mode = 'idle'


def launch_ros_mode(mode: str, extra_args: list = None) -> dict:
    global g_ros_proc, g_current_mode
    kill_ros_launch()
    if mode == 'stop':
        return {'status': 'stopped', 'mode': 'idle'}

    launch_map = {
        'bringup': 'agrover_bringup.launch.py',
        'slam'   : 'agrover_slam.launch.py',
        'nav'    : 'agrover_nav.launch.py',
    }
    if mode not in launch_map:
        return {'error': f'Mode inconnu : {mode}'}

    cmd = ['ros2', 'launch', 'agrover_ros', launch_map[mode]]
    if extra_args:
        cmd += extra_args
    env = {**os.environ, 'ROS_DOMAIN_ID': ROS_DOMAIN_ID}
    try:
        g_ros_proc     = subprocess.Popen(
            cmd, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        g_current_mode = mode
        print(f"[ROS2] Mode '{mode}' → PID {g_ros_proc.pid}")
        return {'status': 'launched', 'mode': mode, 'pid': g_ros_proc.pid}
    except Exception as e:
        return {'error': str(e)}


# ═══════════════════════════════════════════════════════════════
#  ROUTES
# ═══════════════════════════════════════════════════════════════

@app.route('/')
def index():
    return render_template('index.html', rosbridge_port=ROSBRIDGE_PORT)


@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/pump', methods=['POST'])
def api_pump():
    state = int(request.json.get('state', 0))
    return jsonify(serial_send({'cmd': 'PUMP', 'state': state}))


@app.route('/api/launch', methods=['POST'])
def api_launch():
    data  = request.json or {}
    mode  = data.get('mode', 'stop')
    extra = []
    if mode == 'nav':
        map_f = data.get('map', f'{MAP_SAVE_PATH}.yaml')
        extra = [f'map:={map_f}']
    return jsonify(launch_ros_mode(mode, extra))


@app.route('/api/save_map', methods=['POST'])
def api_save_map():
    data = request.json or {}
    path = data.get('path', MAP_SAVE_PATH)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
           '-f', path, '--ros-args', '-p', 'use_sim_time:=false']
    env = {**os.environ, 'ROS_DOMAIN_ID': ROS_DOMAIN_ID}
    try:
        r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=15)
        if r.returncode == 0:
            return jsonify({'status': 'saved', 'path': path})
        return jsonify({'status': 'error', 'detail': r.stderr}), 500
    except subprocess.TimeoutExpired:
        return jsonify({'status': 'error', 'detail': 'timeout'}), 500
    except Exception as e:
        return jsonify({'status': 'error', 'detail': str(e)}), 500


@app.route('/api/status')
def api_status():
    ros_ok    = g_ros_proc    is not None and g_ros_proc.poll()    is None
    bridge_ok = g_bridge_proc is not None and g_bridge_proc.poll() is None
    return jsonify({
        'ros_running'   : ros_ok,
        'bridge_running': bridge_ok,
        'mode'          : g_current_mode,
        'ros_pid'       : g_ros_proc.pid    if ros_ok    else None,
        'bridge_pid'    : g_bridge_proc.pid if bridge_ok else None,
        'serial_ok'     : g_ser is not None,
    })


@app.route('/api/ping', methods=['POST'])
def api_ping():
    return jsonify(serial_send({'cmd': 'PING'}))


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=" * 55)
    print("   AGROVER MR 25.26  —  IHM v3.0")
    print("=" * 55)
    init_serial()
    threading.Thread(target=camera_loop, daemon=True).start()
    start_rosbridge()
    print(f"[FLASK]  http://0.0.0.0:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
