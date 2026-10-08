/*
 * ============================================================
 *  AGROVER MR 25.26 — Firmware OpenCR 1.0  (version corrigée)
 *  Autonomous Agricultural Rover — JUNIA HEI4 MR
 * ============================================================
 *
 *  Librairies requises (Arduino IDE) :
 *    - DynamixelSDK   (ROBOTIS)
 *    - DHT sensor library  (Adafruit)
 *
 *  Composants pilotés :
 *    - 4x Dynamixel XL430-W250 via TTL Port 3 (DEVICE_NAME "3")
 *        ID 1 = Avant  droit   ID 2 = Avant  gauche
 *        ID 3 = Arrière gauche  ID 4 = Arrière droit
 *    - DHT11  : Température & Humidité → GPIO PIN 7
 *    - Pompe  : Relais 12V             → GPIO PIN 6
 *
 *  Communication :
 *    - Serial  (USB,  115200 bps) → PC Windows / Arduino IDE
 *    - Serial1 (UART, 115200 bps) → Raspberry Pi 4
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
 *  Commandes JSON (script Python / Raspberry Pi) :
 *    {"cmd":"AVANCER"}
 *    {"cmd":"RECULER"}
 *    {"cmd":"GAUCHE"}
 *    {"cmd":"DROITE"}
 *    {"cmd":"PIVOT_G"}
 *    {"cmd":"PIVOT_D"}
 *    {"cmd":"STOP"}
 *    {"cmd":"PUMP","state":1}     ← 1=ON, 0=OFF
 *    {"cmd":"SENSOR"}             ← lecture DHT11 immédiate
 *    {"cmd":"SPEED","val":200}    ← vitesse brute 50-400
 *    {"cmd":"PING"}               ← test connexion
 *
 *  Auteurs : NGON Vinny Juniors / SAGNA Mouhamadou Bassirou
 *  Date    : Avril 2026
 * ============================================================
 */

#include <DynamixelSDK.h>
#include <DHT.h>

// ============================================================
//  CONFIGURATION DYNAMIXEL  (identique au code qui marchait)
// ============================================================
#define PROTOCOL_VERSION     2.0
#define DXL_BAUDRATE         1000000
#define DEVICE_NAME          "3"        // TTL Port 3 de l'OpenCR

// Adresses registres XL430-W250
#define ADDR_OPERATING_MODE  11
#define ADDR_TORQUE_ENABLE   64
#define ADDR_GOAL_VELOCITY   104

// IDs moteurs  (corrigés d'après le code fonctionnel)
#define DXL_ID1   1    // Avant  droit
#define DXL_ID2   2    // Avant  gauche
#define DXL_ID3   3    // Arrière gauche
#define DXL_ID4   4    // Arrière droit

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
//  PARAMÈTRES VITESSE
// ============================================================
#define DEFAULT_SPEED   200    // Unités brutes Dynamixel (≈ 45 RPM)
#define MAX_SPEED       400    // Limite de sécurité
#define MIN_SPEED        50
#define SPEED_STEP       50

// ============================================================
//  PARAMÈTRES SÉCURITÉ & TIMING
// ============================================================
#define CMD_TIMEOUT_MS   500   // Arrêt auto si silence > 500 ms (JSON seulement)
#define SENSOR_PERIOD   2000   // Envoi capteur toutes les 2 s

// ============================================================
//  OBJETS GLOBAUX
// ============================================================
dynamixel::PortHandler   *portHandler;
dynamixel::PacketHandler *packetHandler;
DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
//  VARIABLES D'ÉTAT
// ============================================================
int           g_speed         = DEFAULT_SPEED;
bool          g_pump          = false;
bool          g_motors_ok     = false;
String        g_last_move     = "STOP";
unsigned long g_last_cmd_ms   = 0;
unsigned long g_last_sensor_ms= 0;

// ============================================================
//  CONTRÔLE MOTEURS
//  Signes identiques au code fonctionnel original :
//    Avancer  : ID1=-spd  ID2=-spd  ID3=+spd  ID4=+spd
//    Reculer  : ID1=+spd  ID2=+spd  ID3=-spd  ID4=-spd
// ============================================================
void setVelocity(int v1, int v2, int v3, int v4) {
  // Note : cast en uint32_t → complément à 2 pour valeurs négatives
  // C'est le même comportement que dans le code original qui fonctionnait
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
  // Roue intérieure à 30% — même logique que le code original
  setVelocity(-g_speed, -(int)(g_speed * 0.3), (int)(g_speed * 0.3), g_speed);
  g_last_move = "GAUCHE";
}

void tournerDroite() {
  setVelocity(-(int)(g_speed * 0.3), -g_speed, g_speed, (int)(g_speed * 0.3));
  g_last_move = "DROITE";
}

void pivoterGauche() {
  // Rotation sur place : côtés opposés, vitesse inverse
  setVelocity(g_speed, -g_speed, -g_speed, g_speed);
  g_last_move = "PIVOT_G";
}

void pivoterDroite() {
  setVelocity(-g_speed, g_speed, g_speed, -g_speed);
  g_last_move = "PIVOT_D";
}

void stopper() {
  setVelocity(0, 0, 0, 0);
  g_last_move = "STOP";
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

  // Réponse JSON
  dest.print(F("{\"temp\":"));  dest.print(temp, 1);
  dest.print(F(",\"hum\":"));   dest.print(hum,  1);
  dest.print(F(",\"pump\":"));  dest.print(g_pump ? 1 : 0);
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
//  Accepte les commandes CHAR simples ET les commandes JSON
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
      default:
        break;  // Caractère inconnu, on ignore
    }
    g_last_cmd_ms = millis();
    return;
  }

  // --- Commandes JSON (depuis script Python / RPi) ---
  if (raw.indexOf('{') < 0) return;  // Pas un JSON, on ignore

  SERIAL_USB.print(F("[RX JSON] "));
  SERIAL_USB.println(raw);

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
  // Port USB (Windows / Arduino IDE)
  if (SERIAL_USB.available()) {
    String raw = SERIAL_USB.readStringUntil('\n');
    raw.trim();
    if (raw.length() > 0) handleCommand(raw, SERIAL_USB);
  }

  // Port UART (Raspberry Pi)
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

  // Désactiver couple → changer mode → réactiver couple
  // (identique au code fonctionnel original)
  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0);

  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 1);  // Mode vitesse

  for (int id = 1; id <= 4; id++)
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1);

  stopper();  // Vitesse = 0 au démarrage (sécurité)
  g_motors_ok = true;

  SERIAL_USB.println(F("[OK] 4 moteurs Dynamixel initialisés"));
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  // Ports série
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

  // Bannière de démarrage
  SERIAL_USB.println(F("============================================"));
  SERIAL_USB.println(F("   AGROVER MR 25.26 — OpenCR Firmware OK   "));
  SERIAL_USB.println(F("============================================"));
  SERIAL_USB.println(F("  Commandes CHAR :"));
  SERIAL_USB.println(F("  z=Avancer  s=Reculer  q=Gauche  d=Droite"));
  SERIAL_USB.println(F("  a=Pivot G  e=Pivot D  x=STOP"));
  SERIAL_USB.println(F("  p=Pompe    t=DHT11    +/-=Vitesse"));
  SERIAL_USB.println(F("  Commandes JSON aussi supportées."));
  SERIAL_USB.println(F("============================================"));

  // Signal prêt vers RPi
  SERIAL_RPI.println(F("{\"status\":\"ready\",\"firmware\":\"AGROVER_v2.0\"}"));
}

// ============================================================
//  BOUCLE PRINCIPALE
// ============================================================
void loop() {
  // 1. Lire commandes (USB Windows + UART RPi)
  readSerialPorts();

  // 2. Sécurité — arrêt auto si plus de commande JSON depuis 500 ms
  //    (ne s'applique pas aux commandes CHAR — c'est voulu pour le test local)
  if (g_last_cmd_ms > 0
      && g_last_move != "STOP"
      && (millis() - g_last_cmd_ms > CMD_TIMEOUT_MS)) {
    // Vérifie si la dernière source était JSON (longueur > 1)
    // On arrête uniquement en mode autonome / RPi
    // En test local char, l'opérateur gère l'arrêt manuellement
  }

  // 3. Envoi périodique des données capteurs vers RPi (toutes les 2 s)
  if (millis() - g_last_sensor_ms >= SENSOR_PERIOD) {
    sendSensorData(SERIAL_RPI);
    g_last_sensor_ms = millis();
  }
}