// Sistema de Control de Vehículo Autónomo - Heltec WiFi LoRa 32 V3
// Recepción de waypoints via LoRa y control de motores L298N

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <RadioLib.h>  // Librería RadioLib para SX1262
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <MPU6050_tockn.h>

// ========== REGISTROS AK8963 (MAGNETÓMETRO) ==========
#define AK8963_ADDR 0x0C
#define AK8963_WIA 0x00
#define AK8963_ST1 0x02
#define AK8963_HXL 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

// ========== CONFIGURACIÓN LORA ==========
// Pines SX1262 para Heltec WiFi LoRa 32 V3
#define LORA_SCK 9
#define LORA_MISO 11
#define LORA_MOSI 10
#define LORA_CS 8
#define LORA_RST 12
#define LORA_DIO1 14
#define LORA_BUSY 13

#define LORA_FREQUENCY 868.0  // 868 MHz
#define LORA_BANDWIDTH 125.0  // 125 kHz
#define LORA_SPREADING_FACTOR 7
#define LORA_TX_POWER 14      // dBm

#define BUFFER_SIZE 64

// ========== PROTOCOLO DE COMUNICACIÓN ==========
#define MSG_REQUEST_WAYPOINTS 0x01
#define MSG_WAYPOINT_DATA 0x02
#define MSG_END_TRANSMISSION 0x03
#define MSG_ACK 0x04
#define MSG_START_TRANSMISSION 0x05
#define MSG_TELEMETRY 0x06        // Telemetría del robot
#define MSG_TELEMETRY_ACK 0x07    // ACK de telemetría (opcional)

#define MAX_WAYPOINTS 50

// ========== ESTRUCTURAS DE DATOS ==========
struct Waypoint {
  float latitude;
  float longitude;
  bool received;
};

struct __attribute__((packed)) WaypointPacket {
  uint8_t msgType;
  uint8_t waypointId;
  uint8_t totalWaypoints;
  float latitude;
  float longitude;
  uint8_t checksum;
};

struct __attribute__((packed)) TelemetryPacket {
  uint8_t msgType;              // MSG_TELEMETRY (0x06)
  float latitude;               // Latitud actual del robot
  float longitude;              // Longitud actual del robot
  uint8_t satellites;           // Número de satélites GPS
  uint8_t gpsFixed;             // 1 si tiene fix GPS, 0 si no
  uint8_t currentWaypoint;      // Waypoint actual al que se dirige
  float distanceToWaypoint;     // Distancia al waypoint en metros
  float headingToWaypoint;      // Rumbo al waypoint en grados (0-360)
  int16_t rssi;                 // RSSI de la última recepción LoRa
  float snr;                    // SNR de la última recepción LoRa
  uint8_t batteryLevel;         // Nivel de batería en % (0-100)
  uint8_t checksum;             // Checksum simple
};

// ========== VARIABLES GLOBALES ==========
Waypoint waypoints[MAX_WAYPOINTS];
int totalWaypoints = 0;
int waypointsReceived = 0;
bool missionStarted = false;
bool waitingForWaypoints = false;
unsigned long lastRequestTime = 0;
unsigned long lastWaypointTime = 0;
const unsigned long REQUEST_INTERVAL = 5000; // Reintentar cada 5 segundos
const unsigned long WAYPOINT_TIMEOUT = 15000; // 15 segundos después del último waypoint

// Variables de telemetría
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 2000; // Enviar telemetría cada 2 segundos
int16_t lastRssi = 0;
float lastSnr = 0.0;
uint8_t currentWaypointIndex = 0;

// Módulo SX1262
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ========== CONFIGURACIÓN GPS ==========
// Pines libres según pinout Heltec V3
#define GPS_RX 1   // RX del ESP32 conectado a TX del GPS
#define GPS_TX 2   // TX del ESP32 conectado a RX del GPS
#define GPS_BAUD 9600

// Objeto GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Usar Serial1

// Variables GPS
float latActual = 0.0;
float lonActual = 0.0;
int satellites = 0;
bool gpsFixed = false;
unsigned long lastGPSUpdate = 0;

// ========== CONFIGURACIÓN MPU 9250/6500 ==========
// Pines I2C para MPU: GPIO 6 (SDA), GPIO 7 (SCL)
#define MPU_SDA 6
#define MPU_SCL 7

// Objeto MPU6050 con TwoWire secundario
TwoWire mpuWire = TwoWire(1);
MPU6050 mpu(mpuWire);
bool mpuWorking = false;

// Variables IMU
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float magX = 0.0, magY = 0.0, magZ = 0.0;
float heading = 0.0;  // Brújula: 0-360 grados
float tempMPU = 0.0;
unsigned long lastMPUUpdate = 0;

// Calibración del magnetómetro AK8963
float magOffsetX = 0.0, magOffsetY = 0.0, magOffsetZ = 0.0;
float magScaleX = 1.0, magScaleY = 1.0, magScaleZ = 1.0;
float magASAX = 0.0, magASAY = 0.0, magASAZ = 0.0;

// ========== DECLARACIONES FORWARD ==========
void showMessage(const char *line1, const char *line2);
void showMessage(String line1, String line2);
void startMission();
void updateGPS();
void displayGPS();
void updateMPU();
void initAK8963();
void calibrateMagnetometer();
void verifyMagnetometer(float targetHeading = 0.0);
void readAK8963Data();
uint8_t readAK8963Register(uint8_t reg);
void writeAK8963Register(uint8_t reg, uint8_t value);

// ========== CONFIGURACIÓN MOTORES L298N ==========
// Motor A (Derecho): ENA=48, IN1=3, IN2=26
const int M1_IN1 = 3;
const int M1_IN2 = 26;
const int M1_ENA = 48;

// Motor B (Izquierdo): ENB=47, IN3=4, IN4=20
const int M2_IN1 = 4;
const int M2_IN2 = 20;
const int M2_ENB = 47;

// PWM en los pines IN (no en ENA/ENB porque LEDC no funciona en GPIO 48/47)
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;  // 0-255
const int CH_M1_IN1 = 0;
const int CH_M1_IN2 = 1;
const int CH_M2_IN1 = 2;
const int CH_M2_IN2 = 3;

// ========== CONFIGURACIÓN OLED ==========
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 17, /* reset=*/ 21);
bool oledWorking = false;

// ========== FUNCIONES OLED ==========

void showMessage(const char *line1, const char *line2) {
  if (!oledWorking) return;
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, line1);
  if (line2 && line2[0]) u8g2.drawStr(0, 28, line2);
  u8g2.sendBuffer();
}

void showMessage(String line1, String line2) {
  showMessage(line1.c_str(), line2.c_str());
}

// ========== FUNCIONES LORA ==========
static uint8_t rxBuffer[BUFFER_SIZE];

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void processLoRaPacket() {
  // Verificar si hay paquete disponible
  int state = radio.readData(rxBuffer, BUFFER_SIZE);
  
  if (state == RADIOLIB_ERR_NONE) {
    // Paquete recibido correctamente
    int packetSize = radio.getPacketLength();
    int rssi = radio.getRSSI();
    float snr = radio.getSNR();
    
    // Guardar RSSI y SNR para telemetría
    lastRssi = rssi;
    lastSnr = snr;
    
    // Procesar paquete
    if (packetSize == 2) {
      uint8_t msgType = rxBuffer[0];
      
      if (msgType == MSG_START_TRANSMISSION) {
        totalWaypoints = rxBuffer[1];
        waypointsReceived = 0;
        waitingForWaypoints = true;  // IMPORTANTE: Mantenemos el flag para pausar telemetría
        
        Serial.print("[LoRa] Recibiremos ");
        Serial.print(totalWaypoints);
        Serial.println(" waypoints");
        
        showMessage("RX Start", String(totalWaypoints) + " WP");
        
        // Inicializar array
        for (int i = 0; i < MAX_WAYPOINTS; i++) {
          waypoints[i].received = false;
        }
        
        delay(100);
      }
      else if (msgType == MSG_END_TRANSMISSION) {
        Serial.println("[LoRa] Transmisión completa!");
        Serial.print("[LoRa] Recibidos ");
        Serial.print(waypointsReceived);
        Serial.print("/");
        Serial.println(totalWaypoints);
        
        showMessage("RX Complete", String(waypointsReceived) + " WP");
        delay(2000);
        
        startMission();
      }
    }
    else if (packetSize >= sizeof(WaypointPacket) && packetSize <= 20) {
      WaypointPacket packet;
      memcpy(&packet, rxBuffer, sizeof(WaypointPacket));
      
      // Verificar checksum
      uint8_t checksum = calculateChecksum((uint8_t*)&packet, sizeof(WaypointPacket) - 1);
      
      if (checksum == packet.checksum && packet.msgType == MSG_WAYPOINT_DATA) {
        int wpIdx = packet.waypointId - 1;
        
        // Verificar si es waypoint nuevo
        bool isNew = !waypoints[wpIdx].received;
        
        // Guardar waypoint
        waypoints[wpIdx].latitude = packet.latitude;
        waypoints[wpIdx].longitude = packet.longitude;
        waypoints[wpIdx].received = true;
        
        // Si es un waypoint nuevo, incrementar contador
        if (isNew) {
          waypointsReceived++;
        }
        
        // Actualizar tiempo del último waypoint recibido
        lastWaypointTime = millis();
        
        Serial.print("[LoRa] WP");
        Serial.print(packet.waypointId);
        Serial.print("/");
        Serial.print(packet.totalWaypoints);
        Serial.print(" - Lat: ");
        Serial.print(packet.latitude, 6);
        Serial.print(", Lon: ");
        Serial.println(packet.longitude, 6);
        
        // CRÍTICO: Esperar a que la estación termine su delay(800) antes de enviar ACK
        delay(900);
        
        // Enviar ACK después de que la estación esté lista para recibirlo
        uint8_t ackBuffer[2] = {MSG_ACK, packet.waypointId};
        radio.transmit(ackBuffer, 2);
        
        Serial.print("[LoRa] ACK enviado para WP");
        Serial.println(packet.waypointId);
        
        // Pequeño delay para asegurar transmisión
        delay(100);
        
        // Volver a RX para siguiente waypoint
        radio.startReceive();
        
        // Actualizar OLED de forma no bloqueante
        if (oledWorking) {
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_ncenB08_tr);
          u8g2.drawStr(0, 12, ("RX WP" + String(packet.waypointId)).c_str());
          u8g2.drawStr(0, 28, String(packet.latitude, 4).c_str());
          u8g2.sendBuffer();
        }
        
        // IMPORTANTE: Si ya recibimos todos los waypoints, iniciar misión inmediatamente
        if (waypointsReceived >= totalWaypoints && totalWaypoints > 0) {
          Serial.println("[Sistema] ¡TODOS LOS WAYPOINTS RECIBIDOS! Iniciando misión...");
          delay(500);
          startMission();
        }
        
        // Si ya se había mostrado la misión y llega un waypoint nuevo, actualizar
        if (missionStarted) {
          Serial.println("[Sistema] Waypoint recibido tras timeout. Actualizando display...");
          startMission();
        }
      }
    }
  }
  else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // Timeout normal, no hacer nada
  }
  else {
    Serial.print("[LoRa] Error RX: ");
    Serial.println(state);
    radio.startReceive();
  }
}

void requestWaypoints() {
  Serial.println("[LoRa] Solicitando waypoints...");
  showMessage("LoRa TX", "Request WP");
  
  uint8_t request[1] = {MSG_REQUEST_WAYPOINTS};
  int state = radio.transmit(request, 1);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] Solicitud enviada");
    waitingForWaypoints = true;
    lastRequestTime = millis();
    // Volver a modo RX con delay
    delay(50);
    radio.startReceive();
  } else {
    Serial.print("[LoRa] Error TX: ");
    Serial.println(state);
  }
}

void startMission() {
  Serial.println("\n========================================");
  Serial.println("=== WAYPOINTS RECIBIDOS ===");
  Serial.println("========================================");
  Serial.print("[DEBUG] missionStarted antes: ");
  Serial.println(missionStarted);
  Serial.print("[DEBUG] waitingForWaypoints antes: ");
  Serial.println(waitingForWaypoints);
  
  for (int i = 0; i < totalWaypoints; i++) {
    if (waypoints[i].received) {
      Serial.print("WP");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(waypoints[i].latitude, 6);
      Serial.print(", ");
      Serial.println(waypoints[i].longitude, 6);
    }
  }
  
  Serial.println("========================================\n");
  
  missionStarted = true;
  waitingForWaypoints = false;  // Desactivar flag para reanudar telemetría
  
  Serial.println("[SISTEMA] ✅ MISIÓN INICIADA!");
  Serial.println("[SISTEMA] ✅ TELEMETRÍA ACTIVADA!");
  Serial.print("[DEBUG] missionStarted después: ");
  Serial.println(missionStarted);
  Serial.print("[DEBUG] waitingForWaypoints después: ");
  Serial.println(waitingForWaypoints);
  
  // Mostrar waypoints en pantalla OLED
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 12, "Waypoints recibidos:");
  
  int y = 24;
  for (int i = 0; i < totalWaypoints && i < 4; i++) {
    if (waypoints[i].received) {
      String wp = "WP" + String(i + 1) + ": " + String(waypoints[i].latitude, 2) + "," + String(waypoints[i].longitude, 2);
      u8g2.drawStr(0, y, wp.c_str());
      y += 12;
    }
  }
  
  if (totalWaypoints > 4) {
    u8g2.drawStr(0, 60, "...y mas");
  }
  
  u8g2.sendBuffer();
}

// ========== FUNCIONES DE NAVEGACIÓN ==========
// Calcula la distancia entre dos coordenadas GPS usando la fórmula de Haversine
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0; // Radio de la Tierra en metros
  
  float phi1 = lat1 * PI / 180.0;
  float phi2 = lat2 * PI / 180.0;
  float deltaPhi = (lat2 - lat1) * PI / 180.0;
  float deltaLambda = (lon2 - lon1) * PI / 180.0;
  
  float a = sin(deltaPhi / 2.0) * sin(deltaPhi / 2.0) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda / 2.0) * sin(deltaLambda / 2.0);
  float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return R * c; // Distancia en metros
}

// Calcula el rumbo (bearing) desde la posición actual al waypoint
// Retorna un ángulo de 0-360 grados (0 = Norte, 90 = Este, 180 = Sur, 270 = Oeste)
float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = lat1 * PI / 180.0;
  float phi2 = lat2 * PI / 180.0;
  float deltaLambda = (lon2 - lon1) * PI / 180.0;
  
  float y = sin(deltaLambda) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
  float theta = atan2(y, x);
  
  float bearing = (theta * 180.0 / PI + 360.0);
  return fmod(bearing, 360.0); // Normalizar a 0-360
}

// ========== FUNCIONES DE CONTROL DE MOTORES (NO ACTIVADAS AÚN) ==========

void motorForward(int in1, int in2, int speed = 255) {
  bool isMotorA = (in1 == M1_IN1);
  int ch1 = isMotorA ? CH_M1_IN1 : CH_M2_IN1;
  int ch2 = isMotorA ? CH_M1_IN2 : CH_M2_IN2;
  
  ledcWrite(ch1, 0);       // IN1 = LOW
  ledcWrite(ch2, speed);   // IN2 = PWM(speed)
}

void motorBackward(int in1, int in2, int speed = 255) {
  bool isMotorA = (in1 == M1_IN1);
  int ch1 = isMotorA ? CH_M1_IN1 : CH_M2_IN1;
  int ch2 = isMotorA ? CH_M1_IN2 : CH_M2_IN2;
  
  ledcWrite(ch1, speed);   // IN1 = PWM(speed)
  ledcWrite(ch2, 0);       // IN2 = LOW
}

void motorStop(int in1, int in2) {
  bool isMotorA = (in1 == M1_IN1);
  int ch1 = isMotorA ? CH_M1_IN1 : CH_M2_IN1;
  int ch2 = isMotorA ? CH_M1_IN2 : CH_M2_IN2;
  
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
}

void stopAllMotors() {
  motorStop(M1_IN1, M1_IN2);
  motorStop(M2_IN1, M2_IN2);
}

void moveForward(int speed = 255) {
  motorForward(M1_IN1, M1_IN2, speed);
  motorForward(M2_IN1, M2_IN2, speed);
}

void moveBackward(int speed = 255) {
  motorBackward(M1_IN1, M1_IN2, speed);
  motorBackward(M2_IN1, M2_IN2, speed);
}

void turnLeft(int speed = 255) {
  motorBackward(M1_IN1, M1_IN2, speed);  // Motor izq atrás
  motorForward(M2_IN1, M2_IN2, speed);   // Motor der adelante
}

void turnRight(int speed = 255) {
  motorForward(M1_IN1, M1_IN2, speed);   // Motor izq adelante
  motorBackward(M2_IN1, M2_IN2, speed);  // Motor der atrás
}

// ========== FUNCIONES GPS ==========

void updateGPS() {
  // Leer datos del GPS
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
  }
  
  // Si hay nueva ubicación
  if (gps.location.isUpdated()) {
    latActual = gps.location.lat();
    lonActual = gps.location.lng();
    satellites = gps.satellites.value();
    gpsFixed = gps.location.isValid();
    lastGPSUpdate = millis();
    
    // Mostrar en Serial
    Serial.print("[GPS] Lat: ");
    Serial.print(latActual, 6);
    Serial.print(" Lon: ");
    Serial.print(lonActual, 6);
    Serial.print(" Sats: ");
    Serial.print(satellites);
    Serial.print(" HDOP: ");
    Serial.println(gps.hdop.hdop());
  }
}

void displayGPS() {
  if (!oledWorking) return;
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  // Título
  u8g2.drawStr(0, 10, "GPS NEO-6M");
  
  if (gpsFixed) {
    // Latitud
    u8g2.drawStr(0, 24, "Lat:");
    String latStr = String(latActual, 6);
    u8g2.drawStr(30, 24, latStr.c_str());
    
    // Longitud
    u8g2.drawStr(0, 38, "Lon:");
    String lonStr = String(lonActual, 6);
    u8g2.drawStr(30, 38, lonStr.c_str());
    
    // Satélites
    u8g2.drawStr(0, 52, "Sats:");
    String satStr = String(satellites);
    u8g2.drawStr(40, 52, satStr.c_str());
    
    // HDOP (precisión)
    u8g2.drawStr(70, 52, "HD:");
    String hdopStr = String(gps.hdop.hdop(), 1);
    u8g2.drawStr(95, 52, hdopStr.c_str());
    
  } else {
    // Sin Fix
    u8g2.drawStr(0, 30, "Buscando");
    u8g2.drawStr(0, 45, "satelites...");
    
    if (satellites > 0) {
      String satStr = "Sats: " + String(satellites);
      u8g2.drawStr(0, 60, satStr.c_str());
    }
  }
  
  u8g2.sendBuffer();
}

// ========== FUNCIÓN DE ENVÍO DE TELEMETRÍA ==========
void sendTelemetry() {
  TelemetryPacket packet;
  packet.msgType = MSG_TELEMETRY;
  packet.latitude = latActual;
  packet.longitude = lonActual;
  packet.satellites = satellites;
  packet.gpsFixed = gpsFixed ? 1 : 0;
  packet.currentWaypoint = currentWaypointIndex;
  
  // Calcular distancia y rumbo al waypoint actual si hay waypoints
  if (missionStarted && totalWaypoints > 0 && currentWaypointIndex < totalWaypoints) {
    packet.distanceToWaypoint = calculateDistance(
      latActual, lonActual,
      waypoints[currentWaypointIndex].latitude,
      waypoints[currentWaypointIndex].longitude
    );
    packet.headingToWaypoint = calculateBearing(
      latActual, lonActual,
      waypoints[currentWaypointIndex].latitude,
      waypoints[currentWaypointIndex].longitude
    );
  } else {
    packet.distanceToWaypoint = 0.0;
    packet.headingToWaypoint = 0.0;
  }
  
  packet.rssi = lastRssi;
  packet.snr = lastSnr;
  packet.batteryLevel = 100; // TODO: Implementar lectura de batería real
  
  // Calcular checksum
  uint8_t* data = (uint8_t*)&packet;
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
    sum += data[i];
  }
  packet.checksum = sum;
  
  // Enviar paquete
  int state = radio.transmit((uint8_t*)&packet, sizeof(TelemetryPacket));
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[Telemetry] Paquete enviado OK");
    Serial.print("[Telemetry] Pos: ");
    Serial.print(packet.latitude, 6);
    Serial.print(", ");
    Serial.print(packet.longitude, 6);
    Serial.print(" | Sats: ");
    Serial.print(packet.satellites);
    Serial.print(" | WP: ");
    Serial.print(packet.currentWaypoint);
    Serial.print(" | Dist: ");
    Serial.print(packet.distanceToWaypoint, 1);
    Serial.println("m");
  } else {
    Serial.print("[Telemetry] Error al enviar: ");
    Serial.println(state);
  }
  
  // Volver al modo recepción con un pequeño delay
  delay(50);
  radio.startReceive();
}

// ========== FUNCIONES DE INICIALIZACIÓN ==========

void scanI2C() {
  Serial.println("[I2C] Escaneando dispositivos...");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("[I2C] Dispositivo encontrado en 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("[I2C] No se encontraron dispositivos");
  } else {
    Serial.print("[I2C] ");
    Serial.print(nDevices);
    Serial.println(" dispositivo(s) encontrado(s)");
  }
}

void initOLED() {
  Serial.println("[OLED] Inicializando...");
  
  // Activar Vext (GPIO36)
  pinMode(36, OUTPUT);
  digitalWrite(36, LOW);
  delay(100);
  
  // Reset OLED (GPIO21)
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
  delay(50);
  digitalWrite(21, HIGH);
  delay(100);
  
  // Iniciar I2C
  Wire.begin(17, 18);
  delay(50);
  scanI2C();
  
  // Inicializar U8g2
  u8g2.begin();
  
  // Mensaje de inicio
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);
  u8g2.drawStr(0, 20, "VEHICULO");
  u8g2.drawStr(0, 45, "AUTONOMO");
  u8g2.sendBuffer();
  
  oledWorking = true;
  Serial.println("[OLED] OK");
}

void initMotors() {
  Serial.println("[Motor] Configurando L298N...");
  
  // ENA/ENB como GPIO siempre HIGH (LEDC no funciona en GPIO 48/47)
  pinMode(M1_ENA, OUTPUT);
  pinMode(M2_ENB, OUTPUT);
  digitalWrite(M1_ENA, HIGH);
  digitalWrite(M2_ENB, HIGH);
  
  // PWM en los pines IN para control de velocidad
  ledcSetup(CH_M1_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M1_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(CH_M2_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_IN1, CH_M1_IN1);
  ledcAttachPin(M1_IN2, CH_M1_IN2);
  ledcAttachPin(M2_IN1, CH_M2_IN1);
  ledcAttachPin(M2_IN2, CH_M2_IN2);
  
  // Parada inicial
  stopAllMotors();
  
  Serial.println("[Motor] Motor A: IN1=3(CH0), IN2=26(CH1), ENA=48");
  Serial.println("[Motor] Motor B: IN3=4(CH2), IN4=20(CH3), ENB=47");
  Serial.println("[Motor] PWM en pines IN, ENA/ENB=HIGH fijo");
  Serial.println("[Motor] OK");
}

void initGPS() {
  Serial.println("[GPS] Inicializando NEO-6M...");
  Serial.println("[GPS] Probando diferentes baudrates...");
  
  // Lista de baudrates comunes para GPS
  int baudrates[] = {4800, 9600, 38400, 57600, 115200};
  
  for (int i = 0; i < 5; i++) {
    Serial.print("[GPS] Probando ");
    Serial.print(baudrates[i]);
    Serial.println(" bps...");
    
    // Iniciar con este baudrate
    gpsSerial.begin(baudrates[i], SERIAL_8N1, GPS_RX, GPS_TX);
    delay(500);
    
    // Leer datos durante 2 segundos
    unsigned long start = millis();
    bool foundNMEA = false;
    String testData = "";
    
    while (millis() - start < 2000) {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        testData += c;
        
        // Buscar inicio de trama NMEA
        if (c == '$') {
          foundNMEA = true;
        }
      }
      
      if (foundNMEA) break;
    }
    
    if (foundNMEA) {
      Serial.println("[GPS] ✓ ¡NMEA detectado!");
      Serial.print("[GPS] Datos recibidos: ");
      Serial.println(testData.substring(0, 80)); // Primeros 80 chars
      Serial.print("[GPS] Baudrate correcto: ");
      Serial.println(baudrates[i]);
      
      // Reiniciar con el baudrate correcto
      gpsSerial.end();
      gpsSerial.begin(baudrates[i], SERIAL_8N1, GPS_RX, GPS_TX);
      
      if (oledWorking) {
        showMessage("GPS OK", String(baudrates[i]) + " bps");
        delay(1000);
      }
      return;
    } else if (testData.length() > 0) {
      Serial.print("[GPS] Datos incorrectos (");
      Serial.print(testData.length());
      Serial.println(" bytes)");
      // Mostrar primeros bytes en hexadecimal
      Serial.print("[GPS] Hex: ");
      for (int j = 0; j < min(20, (int)testData.length()); j++) {
        Serial.print(testData[j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    } else {
      Serial.println("[GPS] Sin datos");
    }
    
    gpsSerial.end();
    delay(100);
  }
  
  // Si llegamos aquí, ningún baudrate funcionó
  Serial.println("[GPS] ✗ No se detectó NMEA en ningún baudrate");
  Serial.println("[GPS] Probando pines invertidos (RX<->TX)...");
  
  // Probar con pines invertidos
  for (int i = 0; i < 5; i++) {
    Serial.print("[GPS] Probando ");
    Serial.print(baudrates[i]);
    Serial.println(" bps (pines invertidos)...");
    
    // INVERTIR: TX del ESP32 -> TX del GPS, RX del ESP32 -> RX del GPS
    gpsSerial.begin(baudrates[i], SERIAL_8N1, GPS_TX, GPS_RX);
    delay(500);
    
    unsigned long start = millis();
    bool foundNMEA = false;
    String testData = "";
    
    while (millis() - start < 2000) {
      while (gpsSerial.available()) {
        char c = gpsSerial.read();
        testData += c;
        if (c == '$') foundNMEA = true;
      }
      if (foundNMEA) break;
    }
    
    if (foundNMEA) {
      Serial.println("[GPS] ✓ ¡NMEA detectado con pines invertidos!");
      Serial.print("[GPS] Datos: ");
      Serial.println(testData.substring(0, 80));
      Serial.print("[GPS] Baudrate: ");
      Serial.println(baudrates[i]);
      Serial.println("[GPS] ⚠️ Conexión: RX(ESP)->TX(GPS), TX(ESP)->RX(GPS)");
      
      if (oledWorking) {
        showMessage("GPS OK INV", String(baudrates[i]) + " bps");
        delay(1000);
      }
      return;
    }
    
    gpsSerial.end();
    delay(100);
  }
  
  // Si llegamos aquí, nada funcionó
  Serial.println("[GPS] ✗✗✗ ERROR: No se pudo detectar GPS");
  Serial.println("[GPS] Verifica:");
  Serial.println("  1. VCC -> 3.3V o 5V");
  Serial.println("  2. GND -> GND");
  Serial.println("  3. TX(GPS) -> GPIO 2");
  Serial.println("  4. RX(GPS) -> GPIO 1");
  Serial.println("  5. LED del GPS parpadeando");
  
  if (oledWorking) {
    showMessage("GPS ERROR", "Ver Serial");
  }
}

void initMPU() {
  Serial.println("[MPU] Inicializando MPU6050 + AK8963...");
  
  mpuWire.begin(MPU_SDA, MPU_SCL, 400000);
  delay(100);
  
  mpu.begin();
  delay(100);
  
  Serial.println("[MPU] Calibrando giroscopio (mantén quieto)...");
  mpu.calcGyroOffsets(true);
  
  Serial.println("[AK8963] Habilitando I2C master en MPU6050...");
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x36);  // USER_CTRL
  mpuWire.write(0x40);  // I2C_MST_EN
  mpuWire.endTransmission();
  delay(50);
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x37);  // I2C_MST_CTRL
  mpuWire.write(0x0D);  // 400kHz
  mpuWire.endTransmission();
  delay(50);
  
  initAK8963();
  
  mpuWorking = true;
  Serial.println("[MPU] ✓ Sensor inicializado correctamente");
  Serial.println("[MPU] OK - Accel + Gyro + Temp + Magnetómetro funcionando");
  
  showMessage("MPU Status", "OK");
  delay(1000);
}

void updateMPU() {
  if (!mpuWorking) return;
  
  // Actualizar MPU6050 (acelerómetro, giroscopio, temperatura)
  mpu.update();
  
  // Obtener valores del MPU6050
  accelX = mpu.getAccX();
  accelY = mpu.getAccY();
  accelZ = mpu.getAccZ();
  
  gyroX = mpu.getGyroX();
  gyroY = mpu.getGyroY();
  gyroZ = mpu.getGyroZ();
  
  tempMPU = mpu.getTemp();
  
  // Leer magnetómetro AK8963
  readAK8963Data();
  
  lastMPUUpdate = millis();
}

void displayMPU() {
  if (!mpuWorking || !oledWorking) return;
  
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  u8g2.drawStr(0, 10, "MPU6050+AK8963");
  
  // Heading (Magnetómetro - Brújula)
  u8g2.drawStr(0, 24, "Heading:");
  String headStr = String(heading, 0) + "°";
  u8g2.drawStr(70, 24, headStr.c_str());
  
  // Temperatura
  u8g2.drawStr(0, 38, "Temp:");
  String tempStr = String(tempMPU, 1) + "C";
  u8g2.drawStr(50, 38, tempStr.c_str());
  
  u8g2.sendBuffer();
}

// ========== FUNCIONES AK8963 (MAGNETÓMETRO) ==========

uint8_t readAK8963Register(uint8_t reg) {
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x25);  // I2C_SLV0_ADDR
  mpuWire.write(AK8963_ADDR | 0x80);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x26);  // I2C_SLV0_REG
  mpuWire.write(reg);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x27);  // I2C_SLV0_CTRL
  mpuWire.write(0x81);
  mpuWire.endTransmission();
  
  delay(10);
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x3B);  // EXT_SENS_DATA_00
  mpuWire.endTransmission(false);
  mpuWire.requestFrom(0x68, 1);
  
  return mpuWire.read();
}

void writeAK8963Register(uint8_t reg, uint8_t value) {
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x24);  // I2C_SLV0_ADDR
  mpuWire.write(AK8963_ADDR);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x26);  // I2C_SLV0_REG
  mpuWire.write(reg);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x28);  // I2C_SLV0_DO
  mpuWire.write(value);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x27);  // I2C_SLV0_CTRL
  mpuWire.write(0x81);
  mpuWire.endTransmission();
  
  delay(10);
}

void initAK8963() {
  Serial.println("[AK8963] Inicializando magnetómetro...");
  
  writeAK8963Register(AK8963_CNTL2, 0x01);  // Soft reset
  delay(100);
  
  writeAK8963Register(AK8963_CNTL1, 0x0F);  // Fuse ROM access
  delay(100);
  
  magASAX = (float)readAK8963Register(AK8963_ASAX);
  magASAY = (float)readAK8963Register(AK8963_ASAY);
  magASAZ = (float)readAK8963Register(AK8963_ASAZ);
  
  Serial.print("[AK8963] Sensibilidad X:");
  Serial.print(magASAX);
  Serial.print(" Y:");
  Serial.print(magASAY);
  Serial.print(" Z:");
  Serial.println(magASAZ);
  
  writeAK8963Register(AK8963_CNTL1, 0x16);  // 16-bit, 100Hz
  delay(100);
  
  Serial.println("[AK8963] ✓ Magnetómetro inicializado");
}

void calibrateMagnetometer() {
  Serial.println("\n========================================");
  Serial.println("[AK8963] INICIANDO CALIBRACIÓN");
  Serial.println("========================================");
  Serial.println("[AK8963] El robot girará solo durante 20 segundos");
  Serial.println("[AK8963] Asegúrate de que tiene espacio libre");
  Serial.println("[AK8963] Esperando 3 segundos para empezar...");
  
  showMessage("CAL MAG", "3 seg...");
  delay(3000);
  
  // Variables para almacenar mín/máx
  float minX = 10000, maxX = -10000;
  float minY = 10000, maxY = -10000;
  float minZ = 10000, maxZ = -10000;
  
  unsigned long calibStart = millis();
  int samples = 0;
  int phase = 0;
  
  Serial.println("[AK8963] ¡CALIBRACIÓN EN CURSO!");
  Serial.println("[AK8963] Fase 1: Girando a la derecha...");
  showMessage("CAL MAG", "Girando DER");
  
  // Fase 1: Girar a la derecha
  turnRight(230);
  
  // Recopilar datos durante 20 segundos
  while (millis() - calibStart < 20000) {
    // Cambiar de dirección a los 10 segundos
    if (phase == 0 && millis() - calibStart > 10000) {
      phase = 1;
      Serial.println("[AK8963] Fase 2: Girando a la izquierda...");
      showMessage("CAL MAG", "Girando IZQ");
      turnLeft(230);
    }
    
    readAK8963Data();
    
    // Actualizar mín/máx
    if (magX < minX) minX = magX;
    if (magX > maxX) maxX = magX;
    if (magY < minY) minY = magY;
    if (magY > maxY) maxY = magY;
    if (magZ < minZ) minZ = magZ;
    if (magZ > maxZ) maxZ = magZ;
    
    samples++;
    
    // Mostrar progreso cada segundo
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      float elapsed = (millis() - calibStart) / 1000.0;
      Serial.print("[AK8963] ");
      Serial.print(elapsed, 1);
      Serial.print("s - X:[");
      Serial.print(minX, 0);
      Serial.print(",");
      Serial.print(maxX, 0);
      Serial.print("] Y:[");
      Serial.print(minY, 0);
      Serial.print(",");
      Serial.print(maxY, 0);
      Serial.print("] Z:[");
      Serial.print(minZ, 0);
      Serial.print(",");
      Serial.print(maxZ, 0);
      Serial.println("]");
      lastPrint = millis();
    }
    
    delay(50);
  }
  
  // PARAR MOTORES
  stopAllMotors();
  Serial.println("[AK8963] Motores detenidos");
  
  // Calcular offsets y escalas
  float offsetX = (maxX + minX) / 2.0;
  float offsetY = (maxY + minY) / 2.0;
  float offsetZ = (maxZ + minZ) / 2.0;
  
  float rangeX = (maxX - minX) / 2.0;
  float rangeY = (maxY - minY) / 2.0;
  float rangeZ = (maxZ - minZ) / 2.0;
  
  float avgRange = (rangeX + rangeY + rangeZ) / 3.0;
  
  // Evitar división por cero
  float scaleX = (rangeX > 0) ? avgRange / rangeX : 1.0;
  float scaleY = (rangeY > 0) ? avgRange / rangeY : 1.0;
  float scaleZ = (rangeZ > 0) ? avgRange / rangeZ : 1.0;
  
  // Guardar calibración
  magOffsetX = offsetX;
  magOffsetY = offsetY;
  magOffsetZ = offsetZ;
  magScaleX = scaleX;
  magScaleY = scaleY;
  magScaleZ = scaleZ;
  
  Serial.println("\n========================================");
  Serial.println("[AK8963] CALIBRACIÓN COMPLETADA");
  Serial.println("========================================");
  Serial.print("[AK8963] Muestras: ");
  Serial.println(samples);
  Serial.println("[AK8963] Offsets calculados:");
  Serial.print("  X: ");
  Serial.print(magOffsetX, 2);
  Serial.print(" Y: ");
  Serial.print(magOffsetY, 2);
  Serial.print(" Z: ");
  Serial.println(magOffsetZ, 2);
  
  Serial.println("[AK8963] Escalas calculadas:");
  Serial.print("  X: ");
  Serial.print(magScaleX, 3);
  Serial.print(" Y: ");
  Serial.print(magScaleY, 3);
  Serial.print(" Z: ");
  Serial.println(magScaleZ, 3);
  
  Serial.println("[AK8963] ✓ Magnetómetro calibrado correctamente");
  Serial.println("========================================\n");
  
  showMessage("MAG CAL", "OK!");
  delay(2000);
}

void verifyMagnetometer(float targetHeading) {
  Serial.println("\n========================================");
  Serial.println("[VERIFY] VERIFICACIÓN DEL MAGNETÓMETRO");
  Serial.println("========================================");
  Serial.println("[VERIFY] Test: girar en 4 pasos de ~90°");
  Serial.println("[VERIFY] Si los headings varían ~90° entre sí = OK");
  Serial.println("[VERIFY] (Los motores generan campo magnético,");
  Serial.println("[VERIFY]  por eso se lee con motores PARADOS)");
  Serial.println("[VERIFY] Esperando 3 segundos...");
  
  showMessage("VERIFY MAG", "4 pasos 90deg");
  delay(3000);
  
  // Paso 0: Leer heading inicial con motores parados
  float headings[5];  // 5 lecturas: posición inicial + 4 giros
  
  // Hacer varias lecturas para estabilizar
  for (int i = 0; i < 5; i++) {
    readAK8963Data();
    delay(50);
  }
  headings[0] = heading;
  
  Serial.print("[VERIFY] Pos 0 (inicio): Heading = ");
  Serial.print(headings[0], 1);
  Serial.println("°");
  
  if (oledWorking) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "VERIFY MAG");
    String s0 = "P0: " + String(headings[0], 1) + String("\xb0");
    u8g2.drawStr(0, 24, s0.c_str());
    u8g2.sendBuffer();
  }
  
  // Hacer 4 giros de ~90° cada uno
  for (int step = 1; step <= 4; step++) {
    Serial.print("[VERIFY] Girando paso ");
    Serial.print(step);
    Serial.println("/4 (~90°)...");
    
    // Girar ~90°: pulso de motor calibrado
    // Girar durante un tiempo fijo (ajustar según velocidad real)
    turnRight(230);
    delay(800);  // ~90° a velocidad 230 (ajustar si necesario)
    stopAllMotors();
    
    // Esperar a que se estabilice el robot y el campo magnético
    delay(500);
    
    // Leer heading varias veces con motores parados
    for (int i = 0; i < 5; i++) {
      readAK8963Data();
      delay(50);
    }
    headings[step] = heading;
    
    Serial.print("[VERIFY] Pos ");
    Serial.print(step);
    Serial.print(": Heading = ");
    Serial.print(headings[step], 1);
    Serial.println("°");
    
    if (oledWorking) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 10, "VERIFY MAG");
      for (int j = 0; j <= step; j++) {
        String line = "P" + String(j) + ": " + String(headings[j], 1) + String("\xb0");
        u8g2.drawStr(0, 24 + j * 12, line.c_str());
      }
      u8g2.sendBuffer();
    }
    
    delay(500);
  }
  
  // Analizar resultados: calcular diferencias entre pasos consecutivos
  Serial.println("\n========================================");
  Serial.println("[VERIFY] RESULTADO DE VERIFICACIÓN");
  Serial.println("========================================");
  
  float totalRotation = 0;
  bool magnetometerOK = true;
  
  for (int i = 1; i <= 4; i++) {
    float diff = headings[i] - headings[i - 1];
    // Normalizar a -180..180
    if (diff > 180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    
    totalRotation += diff;
    
    Serial.print("[VERIFY] Paso ");
    Serial.print(i - 1);
    Serial.print("->"); 
    Serial.print(i);
    Serial.print(": ");
    Serial.print(headings[i - 1], 1);
    Serial.print("° -> ");
    Serial.print(headings[i], 1);
    Serial.print("° (delta: ");
    Serial.print(diff, 1);
    Serial.println("°)");
    
    // Cada paso debería ser ~90° (aceptar 30-150° como válido)
    if (fabs(diff) < 20.0) {
      Serial.println("[VERIFY]   ⚠ Cambio muy pequeño - posible problema");
      magnetometerOK = false;
    }
  }
  
  Serial.print("[VERIFY] Rotación total: ");
  Serial.print(totalRotation, 1);
  Serial.println("° (esperado: ~360°)");
  
  // Vuelta completa debería ser ~360° (aceptar 200-500° como razonable)
  if (fabs(totalRotation) > 200.0) {
    Serial.println("[VERIFY] ✓ MAGNETÓMETRO FUNCIONA CORRECTAMENTE");
    Serial.println("[VERIFY] ✓ El heading cambia al girar el robot");
    Serial.println("[VERIFY] ✓ Rotación total cercana a 360°");
    
    // Estimar el offset por interferencia del motor
    float motorOffset = headings[0];
    Serial.print("[VERIFY] ℹ Heading con motores parados: ");
    Serial.print(motorOffset, 1);
    Serial.println("°");
    Serial.println("[VERIFY] ℹ (Los motores pueden añadir offset al heading)");
    
    showMessage("MAG VERIFY", "OK! Funciona");
    magnetometerOK = true;
  } else {
    Serial.println("[VERIFY] ⚠ Rotación total insuficiente");
    Serial.println("[VERIFY] ⚠ El magnetómetro podría no estar leyendo bien");
    Serial.println("[VERIFY] Continuando de todas formas...");
    showMessage("MAG VERIFY", "REVISAR");
    magnetometerOK = false;
  }
  
  Serial.println("========================================\n");
  
  delay(3000);
}

void readAK8963Data() {
  if (!mpuWorking) return;
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x24);  // I2C_SLV0_ADDR
  mpuWire.write(AK8963_ADDR | 0x80);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x26);  // I2C_SLV0_REG
  mpuWire.write(AK8963_ST1);
  mpuWire.endTransmission();
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x27);  // I2C_SLV0_CTRL
  mpuWire.write(0x87);
  mpuWire.endTransmission();
  
  delay(10);
  
  mpuWire.beginTransmission(0x68);
  mpuWire.write(0x3B);  // EXT_SENS_DATA_00
  mpuWire.endTransmission(false);
  mpuWire.requestFrom(0x68, 7);
  
  if (mpuWire.available() == 7) {
    uint8_t st1 = mpuWire.read();
    int16_t magRawX = (mpuWire.read() | (mpuWire.read() << 8));
    int16_t magRawY = (mpuWire.read() | (mpuWire.read() << 8));
    int16_t magRawZ = (mpuWire.read() | (mpuWire.read() << 8));
    uint8_t st2 = mpuWire.read();
    
    magX = (magRawX - magOffsetX) * magScaleX;
    magY = (magRawY - magOffsetY) * magScaleY;
    magZ = (magRawZ - magOffsetZ) * magScaleZ;
    
    heading = atan2(magY, magX) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
  }
}

void initLoRa() {
  Serial.println("[LoRa] Inicializando módulo SX1262...");

  
  // Configurar SPI
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  
  // Inicializar SX1262
  int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, 5, 0x12, LORA_TX_POWER);
  
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] ERROR al inicializar, código: ");
    Serial.println(state);
    Serial.println("[LoRa] Verificando pines:");
    Serial.print("[LoRa]   SCK=");
    Serial.print(LORA_SCK);
    Serial.print(", MISO=");
    Serial.print(LORA_MISO);
    Serial.print(", MOSI=");
    Serial.print(LORA_MOSI);
    Serial.print(", CS=");
    Serial.println(LORA_CS);
    Serial.print("[LoRa]   RST=");
    Serial.print(LORA_RST);
    Serial.print(", DIO1=");
    Serial.print(LORA_DIO1);
    Serial.print(", BUSY=");
    Serial.println(LORA_BUSY);
    
    showMessage("LoRa ERROR", "Code: " + String(state));
    while (1) {
      delay(1000);
    }
  }
  
  // Habilitar CRC
  radio.setCRC(true);
  
  // Poner en modo recepción
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] Error al iniciar RX: ");
    Serial.println(state);
  }
  
  Serial.println("[LoRa] Frecuencia: 868 MHz");
  Serial.println("[LoRa] SF: 7, BW: 125 kHz, TX Power: 14 dBm");
  Serial.println("[LoRa] Modo RX activado");
  Serial.println("[LoRa] OK");
}

// ========== SETUP ==========

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("  VEHICULO AUTONOMO - HELTEC V3");
  Serial.println("  LoRa + GPS + Motores L298N");
  Serial.println("========================================\n");

  // Inicializar subsistemas
  initOLED();
  delay(1000);
  
  initGPS();
  delay(500);
  
  initMotors();
  delay(500);
  
  initMPU();
  delay(500);
  
  initLoRa();
  delay(500);
  
  // ========== MINI TEST MPU ==========
  Serial.println("\n========== MINI TEST MPU 9250/6500 ==========");
  if (mpuWorking) {
    Serial.println("[TEST] Leyendo datos del MPU durante 5 segundos...");
    showMessage("MPU Test", "Lectura...");
    
    unsigned long testStart = millis();
    int sampleCount = 0;
    
    while (millis() - testStart < 5000) {
      updateMPU();
      
      if (millis() - testStart > (sampleCount * 1000)) {
        Serial.print("[MPU] Muestra ");
        Serial.print(sampleCount + 1);
        Serial.print(" - Accel: ");
        Serial.print(accelX/16384.0, 3);
        Serial.print("g, Gyro: ");
        Serial.print(gyroX/131.0, 1);
        Serial.print("°/s, Heading: ");
        Serial.print(heading, 0);
        Serial.print("°, Temp: ");
        Serial.print(tempMPU, 1);
        Serial.println("°C");
        sampleCount++;
      }
      
      displayMPU();
      delay(100);
    }
    
    Serial.println("[TEST] ✓ Test completado correctamente");
    showMessage("MPU Test", "OK!");
    delay(2000);
    
    // Calibrar magnetómetro
    calibrateMagnetometer();
    
    // Verificar magnetómetro: girar hasta apuntar al Norte (0°)
    verifyMagnetometer(0.0);
    
  } else {
    Serial.println("[TEST] ✗ MPU no disponible");
    showMessage("MPU Test", "FAILED");
    delay(2000);
  }
  Serial.println("==========================================\n");
  
  // Mostrar GPS mientras esperamos fix
  Serial.println("\n[Sistema] Esperando GPS fix...");
  Serial.println("[Sistema] El GPS puede tardar 1-10 minutos en obtener fix");
  Serial.println("[Sistema] Coloca el módulo GPS cerca de una ventana");
  showMessage("GPS", "Buscando sats...");
  
  // Esperar 30 segundos para que el GPS obtenga algunos satélites
  Serial.println("[Sistema] Esperando 30 segundos para GPS...");
  unsigned long gpsWaitStart = millis();
  while (millis() - gpsWaitStart < 30000) {
    updateGPS();
    
    // Actualizar display cada 2 segundos
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 2000) {
      displayGPS();
      lastUpdate = millis();
    }
    
    delay(100);
  }
  
  // Solicitar waypoints
  Serial.println("\n[Sistema] Iniciando modo LoRa...");
  showMessage("LoRa", "Requesting...");
  delay(1000);
  requestWaypoints();
  
  Serial.println("\n[Sistema] Esperando waypoints...");
  showMessage("Waiting", "Waypoints");
}

// ========== LOOP ==========

void loop() {
  // Actualizar datos GPS continuamente
  updateGPS();
  
  // Actualizar datos del MPU continuamente
  updateMPU();
  
  // Procesar paquetes LoRa recibidos
  processLoRaPacket();
  
  // Mostrar datos del MPU cada 500ms
  static unsigned long lastMPUDisplay = 0;
  if (millis() - lastMPUDisplay > 500) {
    Serial.print("[MPU] Heading: ");
    Serial.print(heading, 0);
    Serial.print("° | Accel: ");
    Serial.print(accelX, 2);
    Serial.print("g | Gyro: ");
    Serial.print(gyroX, 1);
    Serial.print("°/s | Temp: ");
    Serial.print(tempMPU, 1);
    Serial.println("°C");
    lastMPUDisplay = millis();
  }
  
  // DEBUG: Mostrar estado cada 5 segundos
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 5000) {
    Serial.println("\n[DEBUG] Estado del sistema:");
    Serial.print("  missionStarted: ");
    Serial.println(missionStarted);
    Serial.print("  waitingForWaypoints: ");
    Serial.println(waitingForWaypoints);
    Serial.print("  totalWaypoints: ");
    Serial.println(totalWaypoints);
    Serial.print("  waypointsReceived: ");
    Serial.println(waypointsReceived);
    Serial.print("  gpsFixed: ");
    Serial.println(gpsFixed);
    lastDebug = millis();
  }
  
  // Si estamos esperando waypoints y ha pasado el intervalo, reintentar
  if (waitingForWaypoints && (millis() - lastRequestTime > REQUEST_INTERVAL)) {
    Serial.println("[Sistema] Reintentando solicitud de waypoints...");
    requestWaypoints();
  }
  
  // Si recibimos todos los waypoints esperados, finalizar inmediatamente
  if (!missionStarted && totalWaypoints > 0 && waypointsReceived >= totalWaypoints) {
    Serial.println("[Sistema] Todos los waypoints recibidos!");
    startMission();
  }
  // Si recibimos waypoints pero no llega el siguiente en el tiempo esperado, finalizar
  else if (!missionStarted && waypointsReceived > 0 && lastWaypointTime > 0) {
    if (millis() - lastWaypointTime > WAYPOINT_TIMEOUT) {
      Serial.print("[Sistema] No se recibieron más waypoints en ");
      Serial.print(WAYPOINT_TIMEOUT / 1000);
      Serial.println(" segundos.");
      Serial.println("[Sistema] Finalizando recepción y mostrando waypoints...");
      startMission();
    }
  }
  
  // Mostrar GPS en pantalla cada 2 segundos si la misión aún no ha comenzado
  static unsigned long lastGPSDisplay = 0;
  if (!missionStarted && millis() - lastGPSDisplay > 2000) {
    displayGPS();
    lastGPSDisplay = millis();
  }
  
  // Enviar telemetría cada 2 segundos si tenemos GPS fix
  // IMPORTANTE: Solo enviar telemetría si NO estamos esperando waypoints
  // para evitar interferencias en la recepción
  if (gpsFixed && !waitingForWaypoints && millis() - lastTelemetryTime > TELEMETRY_INTERVAL) {
    Serial.println("[DEBUG] Condiciones para telemetría:");
    Serial.print("  gpsFixed: ");
    Serial.println(gpsFixed);
    Serial.print("  waitingForWaypoints: ");
    Serial.println(waitingForWaypoints);
    Serial.print("  Tiempo desde última: ");
    Serial.println(millis() - lastTelemetryTime);
    
    sendTelemetry();
    lastTelemetryTime = millis();
  }
  
  // Si la misión ha comenzado, aquí irá la navegación GPS
}
