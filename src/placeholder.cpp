// Sistema de Control de Vehículo Autónomo - Heltec WiFi LoRa 32 V3
// Recepción de waypoints via LoRa y control de motores L298N

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <RadioLib.h>  // Librería RadioLib para SX1262

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

// Módulo SX1262
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);

// ========== DECLARACIONES FORWARD ==========
void showMessage(const char *line1, const char *line2);
void showMessage(String line1, String line2);
void startMission();

// ========== CONFIGURACIÓN MOTORES L298N ==========
// Motor A (Motor Izquierdo)
const int M1_IN1 = 19;
const int M1_IN2 = 26;
const int M1_ENA = 48;

// Motor B (Motor Derecho)
const int M2_IN1 = 21;
const int M2_IN2 = 20;
const int M2_ENB = 47;

// PWM
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_ENA = 0;
const int PWM_CHANNEL_ENB = 1;

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
    
    // Procesar paquete
    if (packetSize == 2) {
      uint8_t msgType = rxBuffer[0];
      
      if (msgType == MSG_START_TRANSMISSION) {
        totalWaypoints = rxBuffer[1];
        waypointsReceived = 0;
        waitingForWaypoints = false;
        
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
    // Volver a modo RX
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

// ========== FUNCIONES DE CONTROL DE MOTORES (NO ACTIVADAS AÚN) ==========

void motorForward(int in1, int in2, int speed = 255) {
  bool isMotorA = (in1 == M1_IN1);
  int pwm_channel = isMotorA ? PWM_CHANNEL_ENA : PWM_CHANNEL_ENB;
  
  ledcWrite(pwm_channel, speed);
  delay(10);
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorBackward(int in1, int in2, int speed = 255) {
  bool isMotorA = (in1 == M1_IN1);
  int pwm_channel = isMotorA ? PWM_CHANNEL_ENA : PWM_CHANNEL_ENB;
  
  ledcWrite(pwm_channel, speed);
  delay(10);
  
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorStop(int in1, int in2) {
  bool isMotorA = (in1 == M1_IN1);
  int pwm_channel = isMotorA ? PWM_CHANNEL_ENA : PWM_CHANNEL_ENB;
  
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(5);
  ledcWrite(pwm_channel, 0);
}

void stopAllMotors() {
  motorStop(M1_IN1, M1_IN2);
  motorStop(M2_IN1, M2_IN2);
}

void moveForward(int speed = 200) {
  motorForward(M1_IN1, M1_IN2, speed);
  motorForward(M2_IN1, M2_IN2, speed);
}

void moveBackward(int speed = 200) {
  motorBackward(M1_IN1, M1_IN2, speed);
  motorBackward(M2_IN1, M2_IN2, speed);
}

void turnLeft(int speed = 150) {
  motorBackward(M1_IN1, M1_IN2, speed);  // Motor izq atrás
  motorForward(M2_IN1, M2_IN2, speed);   // Motor der adelante
}

void turnRight(int speed = 150) {
  motorForward(M1_IN1, M1_IN2, speed);   // Motor izq adelante
  motorBackward(M2_IN1, M2_IN2, speed);  // Motor der atrás
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
  
  // Pines de dirección
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  
  // PWM para pines Enable
  ledcSetup(PWM_CHANNEL_ENA, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_ENB, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(M1_ENA, PWM_CHANNEL_ENA);
  ledcAttachPin(M2_ENB, PWM_CHANNEL_ENB);
  
  // Parada inicial
  stopAllMotors();
  
  Serial.println("[Motor] Motor A: IN1=19, IN2=26, ENA=48");
  Serial.println("[Motor] Motor B: IN1=21, IN2=20, ENB=47");
  Serial.println("[Motor] OK");
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
  Serial.println("  Recepción LoRa + Control L298N");
  Serial.println("========================================\n");

  // Inicializar subsistemas
  initOLED();
  delay(1000);
  
  initMotors();
  delay(500);
  
  initLoRa();
  delay(500);
  
  // Solicitar waypoints
  showMessage("LoRa", "Requesting...");
  delay(1000);
  requestWaypoints();
  
  Serial.println("\n[Sistema] Esperando waypoints...");
  showMessage("Waiting", "Waypoints");
}

// ========== LOOP ==========

void loop() {
  // Procesar paquetes LoRa recibidos
  processLoRaPacket();
  
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
  
  // Si la misión ha comenzado, mantener waypoints en pantalla
  // No hacer nada más por ahora
}
