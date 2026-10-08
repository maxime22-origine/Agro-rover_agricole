#!/usr/bin/env python3
"""
============================================================
  AGROVER MR 25.26 — Nœud ROS2 principal
  Rôle : Pont entre ROS2 et l'OpenCR via port série
============================================================

  Ce nœud :
    - Souscrit à /cmd_vel  → convertit en commandes JSON → OpenCR
    - Lit les encodeurs depuis OpenCR  → publie /odom + TF
    - Souscrit à /agrover/pump  → contrôle pompe
    - Publie /agrover/dht11  → température & humidité

  INSTALLATION :
    cd ~/agrover_ws
    colcon build --packages-select agrover_ros
    source install/setup.bash
    ros2 launch agrover_ros agrover_bringup.launch.py
============================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import Temperature, RelativeHumidity

from tf2_ros import TransformBroadcaster

import serial
import json
import math
import threading
import time

from .kinematics import AgroverKinematics


class AgroverNode(Node):

    def __init__(self):
        super().__init__('agrover_node')

        # ── Paramètres ROS2 ──────────────────────────────────────
        self.declare_parameter('serial_port',   '/dev/ttyACM0')
        self.declare_parameter('baud_rate',      115200)
        self.declare_parameter('wheel_radius',   0.05)    # m  ← À MESURER
        self.declare_parameter('track_width',    0.30)    # m  ← À MESURER
        self.declare_parameter('odom_frame',    'odom')
        self.declare_parameter('base_frame',    'base_link')
        self.declare_parameter('publish_tf',     True)

        port      = self.get_parameter('serial_port').value
        baud      = self.get_parameter('baud_rate').value
        r         = self.get_parameter('wheel_radius').value
        L         = self.get_parameter('track_width').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # ── Modèle cinématique ───────────────────────────────────
        self.kinematics = AgroverKinematics(
            wheel_radius=r,
            track_width=L
        )

        # ── État odométrie ───────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ── Connexion série ──────────────────────────────────────
        self.ser  = None
        self.lock = threading.Lock()
        self._connect_serial(port, baud)

        # ── Publishers ───────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.temp_pub = self.create_publisher(Temperature, '/agrover/temperature', 10)
        self.hum_pub  = self.create_publisher(RelativeHumidity, '/agrover/humidity', 10)

        # ── Subscribers ──────────────────────────────────────────
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.pump_sub = self.create_subscription(
            Bool, '/agrover/pump', self.pump_callback, 10)

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(0.05,  self._read_serial)   # 20 Hz lecture série
        self.create_timer(5.0,   self._request_sensor) # 0.2 Hz DHT11

        self.get_logger().info(
            f'AGROVER Node démarré | Port: {port} | r={r}m | L={L}m'
        )

    # ════════════════════════════════════════════════════════════
    #  CONNEXION SÉRIE
    # ════════════════════════════════════════════════════════════
    def _connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(1.5)
            self.get_logger().info(f'OpenCR connecté sur {port}')
            self._send({'cmd': 'PING'})
        except serial.SerialException as e:
            self.get_logger().error(f'Erreur série : {e}')

    # ════════════════════════════════════════════════════════════
    #  ENVOI JSON → OPENCR
    # ════════════════════════════════════════════════════════════
    def _send(self, payload: dict):
        with self.lock:
            if self.ser and self.ser.is_open:
                try:
                    msg = json.dumps(payload) + '\n'
                    self.ser.write(msg.encode('utf-8'))
                except Exception as e:
                    self.get_logger().warn(f'Erreur TX: {e}')

    # ════════════════════════════════════════════════════════════
    #  CALLBACK cmd_vel  →  OPENCR
    # ════════════════════════════════════════════════════════════
    def cmd_vel_callback(self, msg: Twist):
        """
        Reçoit Twist de Nav2 ou du télé-opérateur.
        Convertit en commandes DXL via modèle cinématique inverse.
        """
        v     = msg.linear.x
        omega = msg.angular.z

        dxl = self.kinematics.twist_to_dxl(v, omega)

        self._send({
            "cmd": "MOVE_VEL",
            "id1": dxl["id1"],
            "id2": dxl["id2"],
            "id3": dxl["id3"],
            "id4": dxl["id4"]
        })

    # ════════════════════════════════════════════════════════════
    #  CALLBACK pompe
    # ════════════════════════════════════════════════════════════
    def pump_callback(self, msg: Bool):
        self._send({"cmd": "PUMP", "state": 1 if msg.data else 0})

    # ════════════════════════════════════════════════════════════
    #  LECTURE SÉRIE (timer 20 Hz)
    # ════════════════════════════════════════════════════════════
    def _read_serial(self):
        if not self.ser or not self.ser.is_open:
            return

        with self.lock:
            try:
                while self.ser.in_waiting:
                    raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if raw:
                        self._parse_opencr(raw)
            except Exception:
                pass

    def _parse_opencr(self, raw: str):
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        # Données odométrie (encodeurs)
        if 'odom' in data:
            self._update_odometry(data['odom'])

        # Données capteurs DHT11
        if 'temp' in data:
            t = Temperature()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.base_frame
            t.temperature = float(data['temp'])
            self.temp_pub.publish(t)

        if 'hum' in data:
            h = RelativeHumidity()
            h.header.stamp = self.get_clock().now().to_msg()
            h.header.frame_id = self.base_frame
            h.relative_humidity = float(data['hum']) / 100.0
            self.hum_pub.publish(h)

    # ════════════════════════════════════════════════════════════
    #  ODOMÉTRIE
    # ════════════════════════════════════════════════════════════
    def _update_odometry(self, odom_data: dict):
        """
        Reçoit {'vl': float, 'vr': float} depuis OpenCR.
        Calcule et publie la position + TF.
        """
        v_left  = odom_data.get('vl', 0.0)
        v_right = odom_data.get('vr', 0.0)

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        # Intégration cinématique
        self.x, self.y, self.theta, v, omega = \
            self.kinematics.update_odometry(
                self.x, self.y, self.theta,
                v_left, v_right, dt
            )

        # Quaternion depuis yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = now.to_msg()

        # ── Publier /odom ────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = omega

        # Covariance diagonale (incertitude odométrie skid-steer)
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.05   # yaw
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        # ── Publier TF odom → base_link ──────────────────────────
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self.tf_broadcaster.sendTransform(tf)

    # ════════════════════════════════════════════════════════════
    #  REQUÊTE CAPTEUR DHT11 (timer 5s)
    # ════════════════════════════════════════════════════════════
    def _request_sensor(self):
        self._send({"cmd": "SENSOR"})

    # ════════════════════════════════════════════════════════════
    #  DESTRUCTION PROPRE
    # ════════════════════════════════════════════════════════════
    def destroy_node(self):
        self._send({"cmd": "STOP"})
        self._send({"cmd": "PUMP", "state": 0})
        time.sleep(0.3)
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


# ════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = AgroverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
