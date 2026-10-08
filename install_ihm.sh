#!/bin/bash
# ============================================================
#  AGROVER MR 25.26 — Script d'installation IHM v3.0
#  À exécuter sur le Raspberry Pi (Ubuntu 22.04 + ROS2 Humble)
#  Usage : chmod +x install_ihm.sh && sudo ./install_ihm.sh
# ============================================================

set -e  # Arrêt immédiat en cas d'erreur

# ── Couleurs terminal ────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

ok()   { echo -e "${GREEN}  ✓ $1${NC}"; }
info() { echo -e "${CYAN}  ▶ $1${NC}"; }
warn() { echo -e "${YELLOW}  ⚠ $1${NC}"; }
err()  { echo -e "${RED}  ✗ $1${NC}"; exit 1; }

echo -e "${BOLD}"
echo "  ╔══════════════════════════════════════════════════╗"
echo "  ║   AGROVER MR 25.26 — Installation IHM v3.0      ║"
echo "  ║   ROS2 Humble + Flask + rosbridge + Nav2         ║"
echo "  ╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

# ── Vérifications préalables ─────────────────────────────────
[ "$(id -u)" -eq 0 ] || err "Ce script doit être exécuté en root (sudo ./install_ihm.sh)"

# Vérifier que ROS2 Humble est installé
if [ ! -f /opt/ros/humble/setup.bash ]; then
    err "ROS2 Humble non détecté. Installez-le d'abord :\n  https://docs.ros.org/en/humble/Installation.html"
fi

source /opt/ros/humble/setup.bash
ok "ROS2 Humble détecté : $(ros2 --version 2>&1 | head -1)"

# Détecter l'utilisateur réel (pas root)
REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo ubuntu)}"
HOME_DIR=$(eval echo "~$REAL_USER")
info "Utilisateur cible : $REAL_USER ($HOME_DIR)"

# ── 1. Mise à jour apt ────────────────────────────────────────
echo ""
echo -e "${BOLD}[1/7] Mise à jour des paquets apt${NC}"
apt-get update -qq && ok "Cache apt mis à jour"

# ── 2. Paquets ROS2 ──────────────────────────────────────────
echo ""
echo -e "${BOLD}[2/7] Paquets ROS2${NC}"

ROS2_PKGS=(
    ros-humble-rosbridge-suite        # WebSocket bridge ROS2 ↔ navigateur
    ros-humble-rplidar-ros            # Driver LiDAR RPLidar
    ros-humble-slam-toolbox           # SLAM (cartographie)
    ros-humble-nav2-bringup           # Navigation autonome Nav2
    ros-humble-nav2-map-server        # map_saver_cli
    ros-humble-twist-mux              # Multiplexeur cmd_vel
    ros-humble-robot-state-publisher  # TF depuis URDF
    ros-humble-joint-state-publisher  # Joint states
    ros-humble-xacro                  # Parser URDF xacro
    ros-humble-tf2-ros                # Transformations TF2
    ros-humble-tf2-tools              # tf2_echo, tf2_monitor
    ros-humble-rqt-robot-monitor      # Monitoring RQT (optionnel)
)

for pkg in "${ROS2_PKGS[@]}"; do
    if dpkg -l "$pkg" &>/dev/null; then
        ok "$pkg (déjà installé)"
    else
        info "Installation $pkg..."
        apt-get install -y -qq "$pkg" && ok "$pkg"
    fi
done

# ── 3. Dépendances Python ─────────────────────────────────────
echo ""
echo -e "${BOLD}[3/7] Paquets Python${NC}"

PYTHON_PKGS=(
    flask
    flask-socketio
    pyserial
    opencv-python-headless   # Headless = sans GUI, adapté RPi
    numpy                    # Requis par OpenCV
    eventlet                 # Backend async pour Flask-SocketIO
)

for pkg in "${PYTHON_PKGS[@]}"; do
    info "pip install $pkg..."
    pip3 install --quiet --break-system-packages "$pkg" && ok "$pkg"
done

# ── 4. Création des répertoires ───────────────────────────────
echo ""
echo -e "${BOLD}[4/7] Création des répertoires${NC}"

# Répertoire de l'IHM
IHM_DIR="$HOME_DIR/agrover_ihm"
mkdir -p "$IHM_DIR/templates"
mkdir -p "$IHM_DIR/static"
chown -R "$REAL_USER:$REAL_USER" "$IHM_DIR"
ok "Répertoire IHM : $IHM_DIR"

# Répertoire des cartes Nav2
MAPS_DIR="/home/rover/maps"
mkdir -p "$MAPS_DIR"
chown -R "$REAL_USER:$REAL_USER" "$MAPS_DIR" 2>/dev/null || true
ok "Répertoire cartes : $MAPS_DIR"

# Répertoire logs
LOG_DIR="/var/log/agrover"
mkdir -p "$LOG_DIR"
chown "$REAL_USER:$REAL_USER" "$LOG_DIR"
ok "Répertoire logs : $LOG_DIR"

# ── 5. Copie des fichiers IHM ────────────────────────────────
echo ""
echo -e "${BOLD}[5/7] Déploiement des fichiers IHM${NC}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Copier app.py
if [ -f "$SCRIPT_DIR/agrover_ihm/app.py" ]; then
    cp "$SCRIPT_DIR/agrover_ihm/app.py" "$IHM_DIR/app.py"
    chown "$REAL_USER:$REAL_USER" "$IHM_DIR/app.py"
    ok "app.py copié"
else
    warn "app.py introuvable dans $SCRIPT_DIR/agrover_ihm/ — copiez-le manuellement."
fi

# Copier index.html
if [ -f "$SCRIPT_DIR/agrover_ihm/templates/index.html" ]; then
    cp "$SCRIPT_DIR/agrover_ihm/templates/index.html" "$IHM_DIR/templates/index.html"
    chown "$REAL_USER:$REAL_USER" "$IHM_DIR/templates/index.html"
    ok "templates/index.html copié"
else
    warn "templates/index.html introuvable — copiez-le manuellement."
fi

# ── 6. Règle udev pour OpenCR ────────────────────────────────
echo ""
echo -e "${BOLD}[6/7] Règle udev OpenCR (ttyACM0)${NC}"

UDEV_RULE='/etc/udev/rules.d/99-opencr.rules'
if [ ! -f "$UDEV_RULE" ]; then
    cat > "$UDEV_RULE" << 'EOF'
# OpenCR (STM32 CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", \
    SYMLINK+="ttyOpenCR", MODE="0666", GROUP="dialout"

# Fallback générique STM32 CDC
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", MODE="0666", GROUP="dialout"
EOF
    udevadm control --reload-rules
    udevadm trigger
    ok "Règle udev créée : $UDEV_RULE"
else
    ok "Règle udev déjà présente"
fi

# Ajouter l'utilisateur au groupe dialout (accès port série sans sudo)
if ! groups "$REAL_USER" | grep -q dialout; then
    usermod -aG dialout "$REAL_USER"
    ok "Utilisateur $REAL_USER ajouté au groupe dialout"
else
    ok "$REAL_USER déjà dans le groupe dialout"
fi

# ── 7. Service systemd agrover-ihm ───────────────────────────
echo ""
echo -e "${BOLD}[7/7] Service systemd agrover-ihm${NC}"

SERVICE_FILE='/etc/systemd/system/agrover-ihm.service'
cat > "$SERVICE_FILE" << EOF
[Unit]
Description=AGROVER MR 25.26 — IHM Web v3.0
After=network.target

[Service]
Type=simple
User=$REAL_USER
WorkingDirectory=$IHM_DIR
Environment="ROS_DOMAIN_ID=0"
Environment="PYTHONUNBUFFERED=1"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && python3 $IHM_DIR/app.py'
Restart=on-failure
RestartSec=5
StandardOutput=append:$LOG_DIR/ihm.log
StandardError=append:$LOG_DIR/ihm_error.log

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
ok "Service agrover-ihm créé"

# Activer le service (démarrage automatique au boot)
systemctl enable agrover-ihm
ok "Service activé (démarrage automatique au boot)"

# ── Résumé final ──────────────────────────────────────────────
echo ""
echo -e "${BOLD}${GREEN}"
echo "  ╔══════════════════════════════════════════════════╗"
echo "  ║   Installation terminée avec succès !            ║"
echo "  ╚══════════════════════════════════════════════════╝"
echo -e "${NC}"

echo -e "${BOLD}Commandes utiles :${NC}"
echo ""
echo "  Démarrer l'IHM maintenant :"
echo -e "    ${CYAN}sudo systemctl start agrover-ihm${NC}"
echo ""
echo "  Suivre les logs en direct :"
echo -e "    ${CYAN}tail -f $LOG_DIR/ihm.log${NC}"
echo ""
echo "  Statut du service :"
echo -e "    ${CYAN}sudo systemctl status agrover-ihm${NC}"
echo ""
echo "  Accéder à l'IHM depuis le réseau :"
echo -e "    ${CYAN}http://$(hostname -I | awk '{print $1}'):5000${NC}"
echo ""

warn "Redémarrez la session ou lancez 'newgrp dialout' pour activer le groupe série."
echo ""

# Proposer de démarrer immédiatement
read -r -p "  Démarrer l'IHM maintenant ? [o/N] " REPLY
if [[ "$REPLY" =~ ^[Oo]$ ]]; then
    systemctl start agrover-ihm
    sleep 2
    if systemctl is-active --quiet agrover-ihm; then
        ok "IHM démarrée → http://$(hostname -I | awk '{print $1}'):5000"
    else
        warn "Échec du démarrage — consultez : journalctl -u agrover-ihm -n 30"
    fi
fi

echo ""
