/*
 * ============================================================
 *  AGROVER MR 25.26 — Firmware OpenCR 1.0  (version ROS2)
 *  Autonomous Agricultural Rover — JUNIA HEI4 MR
 * ============================================================
 *
 *  Librairies requises (Arduino IDE) :
 *    - DynamixelSDK   (ROBOTIS)
 *    - DHT sensor library  (Adafruit)
 *
 *  Composants pilotés :
 *    - 4x Dynamixel XL430-W250 via TTL Port 3 (DEVICE_NAME "3")
 *        ID 1 = Avant  gauche   ID 2 = Avant  droit
 *        ID 3 = Arrière gauche  ID 4 = Arrière droit
 *    - DHT11  : Température & Humidité → GPIO PIN 7
 *    - Pompe  : Relais 5V              → GPIO PIN 6
 *
 *  Communication :
 *    - Serial  (USB,  115200 bps) → PC Windows / Arduino IDE
 *    - Serial1 (UART, 115200 bps) → Raspberry Pi 4 (ROS2)
 *    Le firmware écoute les deux ports simultanément.
 *
 *  Commandes CHAR (Serial Monitor ou script simple) :
 *    z = Avancer          s = Reculer
 *    q = Tourner gauche   d = Tourner droite
 *    a = Pivoter gauche   e = Pivoter droite
 *    x = STOP
 *    p = Pompe ON/OFF     t = Lire DHT11
 *    + = Vitesse +50      - = Vitesse -50
 *
 *  Commandes JSON (script Python / ROS2 node) :
 *    {"cmd":"AVANCER"}
 *    {"cmd":"RECULER"}
 *    {"cmd":"GAUCHE"}
 *    {"cmd":"DROITE"}
 *    {"cmd":"PIVOT_G"}
 *    {"cmd":"PIVOT_D"}
 *    {"cmd":"STOP"}
 *    {"cmd":"PUMP","state":1}              ← 1=ON, 0=OFF
 *    {"cmd":"SENSOR"}                      ← lecture DHT11 immédiate
 *    {"cmd":"SPEED","val":200}             ← vitesse brute 50-400
 *    {"cmd":"PING"}                        ← test connexion
 *    {"cmd":"MOVE_VEL","id1":n,"id2":n,"id3":n,"id4":n}
 *                                          ← NOUVEAU : vitesses DXL directes depuis ROS2
 *                                            n = valeur signée en unités Dynamixel
 *                                            (positif = sens horaire, 1 unité ≈ 0.229 RPM)
 *
 *  Flux odométrie (vers RPi toutes les 50 ms) :
 *    {"odom":{"vl":x.xxxx,"vr":y.yyyy}}
 *    vl = vitesse roues gauches en m/s (moyenne front_left + rear_left)
 *    vr = vitesse roues droites en m/s (moyenne front_right + rear_right)
 *
 *  Auteurs : NGON Vinny Juniors / SAGNA Mouhamadou Bassirou
 *  Date    : Avril–Mai 2026
 * ============================================================
 */

#include <DynamixelSDK.h>
#include <DHT.h>

// ============================================================
//  CONFIGURATION DYNAMIXEL
// ============================================================
#define PROTOCOL_VERSION     2.0
#define DXL_BAUDRATE         1000000
#define DEVICE_NAME          "3"        // TTL Port 3 de l'OpenCR

// Adresses registres XL430-W250
#define ADDR_OPERATING_MODE      11
#define ADDR_TORQUE_ENABLE       64
#define ADDR_GOAL_VELOCITY      104
#define ADDR_PRESENT_VELOCITY   128     // ← NOUVEAU : lecture vitesse encodeur (4 octets, signé)

// IDs moteurs
#define DXL_ID1   1    // Avant  droit
#define DXL_ID2   2    // Avant  gauche
#define DXL_ID3   3    // Arrière gauche
#define DXL_ID4   4    // Arrière droit

// ============================================================
//  CONVERSION VITESSE DYNAMIXEL ↔ m/s
//  1 unité DXL = 0.229 RPM = 0.229 × 2π/60 rad/s ≈ 0.02398 rad/s
//  v_lin (m/s) = ω (rad/s) × WHEEL_RADIUS
//  !! MESURER LE RAYON RÉEL !!
// ============================================================
#define WHEEL_RADIUS_M   0.0325f     // rayon roue en mètres  (À MESURER)
#define DXL_VEL_UNIT     0.023980f // rad/s par unité DXL (0.229*2π/60)
// Facteur global unité DXL → m/s :
#define DXL_TO_MS        (DXL_VEL_UNIT * WHEEL_RADIUS_M)   // ≈ 0.001199 m/s / unité

// ============================================================
//  PINS CAPTEUR & POMPE
// ============================================================
#define DHT_PIN    7    // GPIO PIN 7 — DHT11 DATA (pull-up 10kΩ)
#define PUMP_PIN   6    // GPIO PIN 6 — Relais pompe (HIGH = ON)
#define DHT_TYPE   DHT11

// ============================================================
//  PORTS SÉRIE
// ============================================================
#define SERIAL_USB   Serial    // USB  → PC Windows (test local)
#define SERIAL_RPI   Serial1   // UART → Raspberry Pi 4

// ============================================================
//  PARAMÈTRES VITESSE (commandes CHAR et haut-niveau)
// ============================================================
#define DEFAULT_SPEED   200    // Unités brutes Dynamixel (≈ 45 RPM)
#define MAX_SPEED       400    // Limite de sécurité
#define MIN_SPEED        50
#define SPEED_STEP       50

// ============================================================
//  PARAMÈTRES SÉCURITÉ & TIMING
// ============================================================
#define CMD_TIMEOUT_MS    500   // Arrêt auto si silence > 500 ms (mode ROS2)
#define SENSOR_PERIOD    2000   // Envoi DHT11 toutes les 2 s
#define ODOM_PERIOD        50   // Envoi odométrie toutes les 50 ms (20 Hz)

// ============================================================
//  OBJETS GLOBAUX
// ============================================================
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
//  VARIABLES D'ÉTAT
// ============================================================
int           g_speed          = DEFAULT_SPEED;
bool          g_pump           = false;
bool          g_motors_ok      = false;
bool          g_ros_mode       = false;   // true dès qu'un MOVE_VEL est reçu
String        g_last_move      = "STOP";
unsigned long g_last_cmd_ms    = 0;
unsigned long g_last_sensor_ms = 0;
unsigned long g_last_odom_ms   = 0;

// ============================================================
//  LECTURE VITESSE PRÉSENTE D'UN MOTEUR
//  Retourne une valeur signée (complément à 2, 32 bits)
// ============================================================
int32_t readVelocity(uint8_t id) {
  uint32_t raw = 0;
  uint8_t  dxl_error = 0;
  int dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler, id, ADDR_PRESENT_VELOCITY, &raw, &dxl_error);

  if (dxl_comm_result != COMM_SUCCESS) return 0;
  if (dxl_error != 0)                 return 0;

  // Interprétation complément à 2 (le registre est signé sur 32 bits)
  return (int32_t)raw;
}

// ============================================================
//  CONTRÔLE MOTEURS (commandes haut-niveau)
//  Convention VALIDÉE sur le robot :
//    Avancer : ID1=-spd  ID2=-spd  ID3=+spd  ID4=+spd
// ============================================================
void setVelocity(int v1, int v2, int v3, int v4) {
  packetHandler->write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY, (uint32_t)v1);
  packetHandler->write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_VELOCITY, (uint32_t)v2);
  packetHandler->write4ByteTxRx(portHandler, DXL_ID3, ADDR_GOAL_VELOCITY, (uint32_t)v3);
  packetHandler->write4ByteTxRx(portHandler, DXL_ID4, ADDR_GOAL_VELOCITY, (uint32_t)v4);
}

void avancer() {
  setVelocity(-g_speed, -g_speed, g_speed, g_speed);
  g_last_move = "AVANCER";
}

void reculer() {
  setVelocity(g_speed, g_speed, -g_speed, -g_speed);
  g_last_move = "RECULER";
}

void tournerGauche() {
  setVelocity(-g_speed, -(int)(g_speed * 0.3), (int)(g_speed * 0.3), g_speed);
  g_last_move = "GAUCHE";
}

void tournerDroite() {
  setVelocity(-(int)(g_speed * 0.3), -g_speed, g_speed, (int)(g_speed * 0.3));
  g_last_move = "DROITE";
}

void pivoterGauche() {
  setVelocity(g_speed, (int)(g_speed * 0.3), -(int)(g_speed * 0.3), -g_speed);
  g_last_move = "PIVOT_G";
}

void pivoterDroite() {
  setVelocity((int)(g_speed * 0.3), g_speed, -g_speed, -(int)(g_speed * 0.3));
  g_last_move = "PIVOT_D";
}

void stopper() {
  setVelocity(0, 0, 0, 0);
  g_last_move = "STOP";
}

// ============================================================
//  NOUVEAU : MOVE_VEL — commande directe depuis ROS2
//  Le nœud ROS2 (agrover_node.py) envoie les 4 vitesses DXL
//  déjà calculées par AgroverKinematics.twist_to_dxl()
//  Convention : id1=avant_gauche, id2=avant_droit,
//               id3=arrière_gauche, id4=arrière_droit
// ============================================================
void handleMoveVel(int id1, int id2, int id3, int id4) {
  setVelocity(id1, id2, id3, id4);
  g_last_move  = "MOVE_VEL";
  g_ros_mode   = true;    // Activer le watchdog ROS2
  g_last_cmd_ms = millis();
}

// ============================================================
//  NOUVEAU : ODOMÉTRIE
//  Lit les 4 encodeurs et publie vl/vr en m/s vers le RPi
//
//  Convention vitesse présente (même signe que GOAL_VELOCITY) :
//    moteurs gauches  : ID2 (avant), ID3 (arrière)  → positif = avancer
//    moteurs droits   : ID1 (avant), ID4 (arrière)  → négatif = avancer
//
//  Pour récupérer la vitesse "avancer" uniforme :
//    v_left_raw  = (-vel2 + vel3) / 2
//    v_right_raw = (-vel1 + vel4) / 2
//  (annule le signe inversé des moteurs gauches 1, voir convention avancer)
//
//  Puis conversion en m/s :  v_m/s = raw * DXL_TO_MS
// ============================================================
void sendOdometry() {
  if (!g_motors_ok) return;

  int32_t vel1 = readVelocity(DXL_ID1);
  int32_t vel2 = readVelocity(DXL_ID2);
  int32_t vel3 = readVelocity(DXL_ID3);
  int32_t vel4 = readVelocity(DXL_ID4);

  // Moyenne par côté (avant + arrière) avec correction de signe
  // D'après la convention validée : avancer => ID2=-spd, ID3=+spd
  //   ⟹ vitesse réelle gauche = (-vel2 + vel3) / 2
  //   ⟹ vitesse réelle droite = (-vel1 + vel4) / 2  (ID1=-spd, ID4=+spd)
  float left_raw  = (float)(-vel2 + vel3) * 0.5f;
  float right_raw = (float)(-vel1 + vel4) * 0.5f;

  float vl = left_raw  * DXL_TO_MS;
  float vr = right_raw * DXL_TO_MS;

  // JSON → Raspberry Pi (nœud ROS2 agrover_node.py lit cette ligne)
  SERIAL_RPI.print(F("{\"odom\":{\"vl\":"));
  SERIAL_RPI.print(vl, 4);
  SERIAL_RPI.print(F(",\"vr\":"));
  SERIAL_RPI.print(vr, 4);
  SERIAL_RPI.println(F("}}"));
}

// ============================================================
//  CONTRÔLE POMPE
// ============================================================
void setPump(bool on) {
  g_pump = on;
  digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  SERIAL_USB.print(F("[POMPE] "));
  SERIAL_USB.println(on ? F("ON") : F("OFF"));
}

// ============================================================
//  LECTURE & ENVOI CAPTEUR DHT11
// ============================================================
void sendSensorData(Stream &dest) {
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    dest.println(F("{\"error\":\"dht11_fail\"}"));
    SERIAL_USB.println(F("[WARN] DHT11 : lecture échouée"));
    return;
  }

  dest.print(F("{\"temp\":"));   dest.print(temp, 1);
  dest.print(F(",\"hum\":"));    dest.print(hum,  1);
  dest.print(F(",\"pump\":"));   dest.print(g_pump ? 1 : 0);
  dest.print(F(",\"speed\":")); dest.print(g_speed);
  dest.print(F(",\"move\":\"")); dest.print(g_last_move);
  dest.println(F("\"}"));
}

// ============================================================
//  ENVOI SUR LES DEUX PORTS SÉRIE
// ============================================================
void sendBoth(const __FlashStringHelper* msg) {
  SERIAL_USB.println(msg);
  SERIAL_RPI.println(msg);
}

// ============================================================
//  TRAITEMENT D'UNE COMMANDE REÇUE
//  Accepte CHAR simples, JSON haut-niveau ET MOVE_VEL ROS2
// ============================================================
void handleCommand(const String& raw, Stream &src) {

  // --- Commandes CHAR simples (1 caractère) ---
  if (raw.length() == 1) {
    char c = raw.charAt(0);
    switch (c) {
      case 'z': case 'Z': avancer();       src.println(F("{\"ack\":\"AVANCER\"}")); break;
      case 's': case 'S': reculer();       src.println(F("{\"ack\":\"RECULER\"}")); break;
      case 'q': case 'Q': tournerGauche(); src.println(F("{\"ack\":\"GAUCHE\"}"));  break;
      case 'd': case 'D': tournerDroite(); src.println(F("{\"ack\":\"DROITE\"}"));  break;
      case 'a': case 'A': pivoterGauche(); src.println(F("{\"ack\":\"PIVOT_G\"}")); break;
      case 'e': case 'E': pivoterDroite(); src.println(F("{\"ack\":\"PIVOT_D\"}")); break;
      case 'x': case 'X': stopper();       src.println(F("{\"ack\":\"STOP\"}"));    break;
      case 'p': case 'P':
        setPump(!g_pump);
        src.print(F("{\"ack\":\"PUMP\",\"state\":"));
        src.print(g_pump ? 1 : 0);
        src.println(F("}"));
        break;
      case 't': case 'T':
        sendSensorData(src);
        break;
      case '+':
        g_speed = (g_speed + SPEED_STEP > MAX_SPEED ? MAX_SPEED : g_speed + SPEED_STEP);
        src.print(F("{\"ack\":\"SPEED\",\"val\":"));
        src.print(g_speed);
        src.println(F("}"));
        break;
      case '-':
        g_speed = (g_speed - SPEED_STEP < MIN_SPEED ? MIN_SPEED : g_speed - SPEED_STEP);
        src.print(F("{\"ack\":\"SPEED\",\"val\":"));
        src.print(g_speed);
        src.println(F("}"));
        break;
      default: break;
    }
    g_last_cmd_ms = millis();
    return;
  }

  // --- Commandes JSON ---
  if (raw.indexOf('{') < 0) return;

  SERIAL_USB.print(F("[RX JSON] "));
  SERIAL_USB.println(raw);

  // ── MOVE_VEL (NOUVEAU — prioritaire car appelé fréquemment par ROS2) ──
  if (raw.indexOf("\"MOVE_VEL\"") >= 0) {
    // Extraction de id1, id2, id3, id4
    // Format attendu : {"cmd":"MOVE_VEL","id1":N,"id2":N,"id3":N,"id4":N}
    int v1 = 0, v2 = 0, v3 = 0, v4 = 0;

    int idx;
    idx = raw.indexOf("\"id1\":");
    if (idx >= 0) v1 = raw.substring(idx + 6).toInt();
    idx = raw.indexOf("\"id2\":");
    if (idx >= 0) v2 = raw.substring(idx + 6).toInt();
    idx = raw.indexOf("\"id3\":");
    if (idx >= 0) v3 = raw.substring(idx + 6).toInt();
    idx = raw.indexOf("\"id4\":");
    if (idx >= 0) v4 = raw.substring(idx + 6).toInt();

    handleMoveVel(v1, v2, v3, v4);
    // Pas d'ACK pour MOVE_VEL (appelé à 20 Hz — inutile de saturer le bus)
    return;
  }

  // ── Commandes haut-niveau ──
  if      (raw.indexOf("\"AVANCER\"") >= 0) { avancer();       src.println(F("{\"ack\":\"AVANCER\"}")); }
  else if (raw.indexOf("\"RECULER\"") >= 0) { reculer();       src.println(F("{\"ack\":\"RECULER\"}")); }
  else if (raw.indexOf("\"GAUCHE\"")  >= 0) { tournerGauche(); src.println(F("{\"ack\":\"GAUCHE\"}"));  }
  else if (raw.indexOf("\"DROITE\"")  >= 0) { tournerDroite(); src.println(F("{\"ack\":\"DROITE\"}"));  }
  else if (raw.indexOf("\"PIVOT_G\"") >= 0) { pivoterGauche(); src.println(F("{\"ack\":\"PIVOT_G\"}")); }
  else if (raw.indexOf("\"PIVOT_D\"") >= 0) { pivoterDroite(); src.println(F("{\"ack\":\"PIVOT_D\"}")); }
  else if (raw.indexOf("\"STOP\"")    >= 0) { stopper();       src.println(F("{\"ack\":\"STOP\"}"));    }

  else if (raw.indexOf("\"PUMP\"") >= 0) {
    int idx = raw.indexOf("\"state\":");
    if (idx >= 0) {
      int val = raw.substring(idx + 8).toInt();
      setPump(val == 1);
    }
    src.print(F("{\"ack\":\"PUMP\",\"state\":"));
    src.print(g_pump ? 1 : 0);
    src.println(F("}"));
  }

  else if (raw.indexOf("\"SPEED\"") >= 0) {
    int idx = raw.indexOf("\"val\":");
    if (idx >= 0) {
      int val = raw.substring(idx + 6).toInt();
      g_speed = (val < MIN_SPEED ? MIN_SPEED : (val > MAX_SPEED ? MAX_SPEED : val));
    }
    src.print(F("{\"ack\":\"SPEED\",\"val\":"));
    src.print(g_speed);
    src.println(F("}"));
  }

  else if (raw.indexOf("\"SENSOR\"") >= 0) {
    sendSensorData(src);
  }

  else if (raw.indexOf("\"PING\"") >= 0) {
    src.println(F("{\"ack\":\"PONG\",\"status\":\"ok\"}"));
  }

  else {
    src.println(F("{\"error\":\"unknown_cmd\"}"));
  }

  g_last_cmd_ms = millis();
}

// ============================================================
//  LECTURE DES DEUX PORTS SÉRIE
// ============================================================
void readSerialPorts() {
  if (SERIAL_USB.available()) {
    String raw = SERIAL_USB.readStringUntil('\n');
    raw.trim();
    if (raw.length() > 0) handleCommand(raw, SERIAL_USB);
  }

  if (SERIAL_RPI.available()) {
    String raw = SERIAL_RPI.readStringUntil('\n');
    raw.trim();
    if (raw.length() > 0) handleCommand(raw, SERIAL_RPI);
  }
}

// ============================================================
//  INITIALISATION DYNAMIXEL
// ============================================================
void initMotors() {
  portHandler   = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    SERIAL_USB.println(F("[ERREUR] Impossible d'ouvrir le port Dynamixel !"));
    return;
  }
  if (!portHandler->setBaudRate(DXL_BAUDRATE)) {
    SERIAL_USB.println(F("[ERREUR] Impossible de régler le baudrate Dynamixel !"));
    return;
  }

  // Désactiver couple → mode vitesse → réactiver couple
  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0);

  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 1);

  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1);

  stopper();
  g_motors_ok = true;

  SERIAL_USB.println(F("[OK] 4 moteurs Dynamixel initialisés"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  SERIAL_USB.begin(115200);
  SERIAL_RPI.begin(115200);
  delay(200);

  // Pompe — OFF par défaut (sécurité)
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // DHT11
  dht.begin();

  // Moteurs
  initMotors();

  // Bannière
  SERIAL_USB.println(F("============================================"));
  SERIAL_USB.println(F("  AGROVER MR 25.26 — OpenCR Firmware ROS2  "));
  SERIAL_USB.println(F("============================================"));
  SERIAL_USB.println(F("  CHAR : z/s/q/d/a/e/x/p/t/+/-"));
  SERIAL_USB.println(F("  JSON : AVANCER RECULER GAUCHE DROITE"));
  SERIAL_USB.println(F("         STOP PUMP SPEED SENSOR PING"));
  SERIAL_USB.println(F("  ROS2 : MOVE_VEL id1 id2 id3 id4"));
  SERIAL_USB.println(F("  ODOM : publiée vers RPi toutes les 50 ms"));
  SERIAL_USB.println(F("============================================"));

  SERIAL_RPI.println(F("{\"status\":\"ready\",\"firmware\":\"AGROVER_v3.0_ROS2\"}"));

  g_last_odom_ms   = millis();
  g_last_sensor_ms = millis();
}

// ============================================================
//  BOUCLE PRINCIPALE
// ============================================================
void loop() {
  unsigned long now = millis();

  // 1. Lire commandes (USB Windows + UART RPi)
  readSerialPorts();

  // 2. Watchdog ROS2 — arrêt auto si plus de MOVE_VEL depuis 500 ms
  //    (uniquement en mode ROS2, pas en mode CHAR manuel)
  if (g_ros_mode
      && g_last_move != "STOP"
      && (now - g_last_cmd_ms > CMD_TIMEOUT_MS)) {
    stopper();
    g_ros_mode = false;
    SERIAL_USB.println(F("[WATCHDOG] Timeout ROS2 — STOP automatique"));
    SERIAL_RPI.println(F("{\"warn\":\"watchdog_stop\"}"));
  }

  // 3. Envoi odométrie vers RPi (20 Hz = toutes les 50 ms)
  if (now - g_last_odom_ms >= ODOM_PERIOD) {
    sendOdometry();
    g_last_odom_ms = now;
  }

  // 4. Envoi périodique DHT11 vers RPi (toutes les 2 s)
  if (now - g_last_sensor_ms >= SENSOR_PERIOD) {
    sendSensorData(SERIAL_RPI);
    g_last_sensor_ms = now;
  }
}
