"""
============================================================
  AGROVER MR 25.26 — Modèle Cinématique (Skid-Steer 4 roues)
============================================================

  ARCHITECTURE MOTEURS :
  ┌─────────────────────────────────────┐
  │           AVANT DU ROBOT            │
  │  ID2 (FL) ────────────── ID1 (FR)   │
  │     │                       │       │
  │  ID3 (RL) ────────────── ID4 (RR)   │
  │           ARRIÈRE DU ROBOT          │
  └─────────────────────────────────────┘

  CONVENTION DE SIGNES (firmware vérifié) :
    - Avancer : ID1=-spd, ID2=-spd, ID3=+spd, ID4=+spd
    - Côté gauche (forward) : ID2 négatif, ID3 positif
    - Côté droit  (forward) : ID1 négatif, ID4 positif

  MODÈLE SKID-STEER :
    v_robot    = (v_R + v_L) / 2
    ω_robot    = (v_R - v_L) / L

  CINÉMATIQUE INVERSE (Twist → vitesses roues) :
    v_L = v - ω * L/2
    v_R = v + ω * L/2

  UNITÉ DYNAMIXEL :
    1 unité = 0.229 rpm = 0.02398 rad/s

  CONVERSION m/s ↔ DXL :
    dxl = (v_ms / r) / 0.02398
    v_ms = dxl * 0.02398 * r
============================================================
"""

import math


# ── Constantes Dynamixel XL430 ──────────────────────────────
DXL_VEL_UNIT = 0.229 * 2.0 * math.pi / 60.0   # rad/s par unité
MAX_DXL_VEL  = 400    # limite sécurité (unités DXL)
MIN_DXL_VEL  = 0


class AgroverKinematics:
    """
    Modèle cinématique du rover AGROVER MR 25.26.

    Paramètres à mesurer sur le robot physique :
      - wheel_radius : rayon de la roue en mètres (ex: 0.05 m)
      - track_width  : distance entre roues gauche et droite en mètres (ex: 0.30 m)
    """

    def __init__(self, wheel_radius: float = 0.0325, track_width: float = 0.155):
        self.r = wheel_radius    # rayon roue (m)
        self.L = track_width     # voie (m)

    # ── Cinématique inverse ─────────────────────────────────────
    def twist_to_wheels(self, v: float, omega: float):
        """
        Convertit une commande Twist (v, ω) en vitesses roues (m/s).

        Args:
            v     : vitesse linéaire (m/s)  — positif = avancer
            omega : vitesse angulaire (rad/s) — positif = tourner gauche

        Returns:
            (v_left, v_right) en m/s
        """
        v_left  = v - omega * self.L / 2.0
        v_right = v + omega * self.L / 2.0
        return v_left, v_right

    # ── Cinématique directe ─────────────────────────────────────
    def wheels_to_twist(self, v_left: float, v_right: float):
        """
        Convertit les vitesses roues (m/s) en Twist (v, ω).

        Returns:
            (v, omega)
        """
        v     = (v_right + v_left) / 2.0
        omega = (v_right - v_left) / self.L
        return v, omega

    # ── Conversions m/s ↔ DXL ──────────────────────────────────
    def ms_to_dxl(self, v_ms: float) -> int:
        """
        Convertit une vitesse en m/s vers les unités Dynamixel.
        Le signe indique le sens de rotation.
        """
        omega_rad_s = v_ms / self.r
        raw = omega_rad_s / DXL_VEL_UNIT
        return int(raw)

    def dxl_to_ms(self, dxl_val: int) -> float:
        """
        Convertit des unités Dynamixel vers m/s.
        """
        omega_rad_s = dxl_val * DXL_VEL_UNIT
        return omega_rad_s * self.r

    # ── Twist complet → commandes DXL ──────────────────────────
    def twist_to_dxl(self, v: float, omega: float):
        """
        Convertit Twist → (dxl_left, dxl_right).
        Tient compte de la convention de signes du firmware :
          - Côté gauche : ID2 = -dxl_L, ID3 = +dxl_L
          - Côté droit  : ID1 = -dxl_R, ID4 = +dxl_R

        Returns:
            dict avec id1, id2, id3, id4
        """
        v_left, v_right = self.twist_to_wheels(v, omega)
        dxl_L = self.ms_to_dxl(v_left)
        dxl_R = self.ms_to_dxl(v_right)

        # Appliquer la convention de signes du firmware
        return {
            "id1": -dxl_R,   # front-right  (forward = négatif)
            "id2": -dxl_L,   # front-left (forward = négatif)
            "id3":  dxl_L,   # rear-left   (forward = positif)
            "id4":  dxl_R,   # rear-right  (forward = positif)
        }

    # ── Odométrie ───────────────────────────────────────────────
    def update_odometry(self, x: float, y: float, theta: float,
                        v_left: float, v_right: float, dt: float):
        """
        Met à jour la position du robot (intégration Euler).

        Args:
            x, y, theta : position actuelle (m, m, rad)
            v_left      : vitesse roue gauche (m/s)
            v_right     : vitesse roue droite (m/s)
            dt          : pas de temps (s)

        Returns:
            (x, y, theta, v, omega) mis à jour
        """
        v, omega = self.wheels_to_twist(v_left, v_right)

        delta_s     = v * dt
        delta_theta = omega * dt

        # Intégration avec angle à mi-pas (meilleure précision)
        x     += delta_s * math.cos(theta + delta_theta / 2.0)
        y     += delta_s * math.sin(theta + delta_theta / 2.0)
        theta += delta_theta

        # Normaliser theta dans [-π, π]
        theta = math.atan2(math.sin(theta), math.cos(theta))

        return x, y, theta, v, omega

    # ── DXL présent → m/s (lecture encodeurs) ──────────────────
    def present_vel_to_ms(self, vel1: int, vel2: int,
                          vel3: int, vel4: int):
        """
        Convertit les vitesses présentes lues sur les 4 moteurs
        en vitesses gauche/droite en m/s.

        Convention de signes du firmware :
          - Gauche  : ID2 forward=négatif, ID3 forward=positif
          - Droite  : ID1 forward=négatif, ID4 forward=positif

        Args:
            vel1..vel4 : vitesses DXL lues (signées, int32)

        Returns:
            (v_left, v_right) en m/s
        """
        # Normaliser le signe pour que "positif = avancer"
        dxl_left  = (-vel2 + vel3) / 2.0
        dxl_right = (-vel1 + vel4) / 2.0

        v_left  = self.dxl_to_ms(dxl_left)
        v_right = self.dxl_to_ms(dxl_right)

        return v_left, v_right
