# Protocolo LoRa para Receptor de Waypoints (Vehículo)

## Configuración LoRa del Vehículo
- **Frecuencia**: 868 MHz (Europa)
- **Spreading Factor**: 7
- **Bandwidth**: 125 kHz

## Pines LoRa (Heltec WiFi LoRa 32 V2)
```cpp
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26
```

## Protocolo de Comunicación

### Tipos de Mensajes
```cpp
#define MSG_REQUEST_WAYPOINTS 0x01   // Vehículo → Estación
#define MSG_WAYPOINT_DATA 0x02       // Estación → Vehículo
#define MSG_END_TRANSMISSION 0x03    // Estación → Vehículo
#define MSG_ACK 0x04                 // Vehículo → Estación
#define MSG_START_TRANSMISSION 0x05  // Estación → Vehículo
```

### Estructura del Paquete de Waypoint
```cpp
struct WaypointPacket {
  uint8_t msgType;        // 0x02 (MSG_WAYPOINT_DATA)
  uint8_t waypointId;     // ID del waypoint (1-based)
  uint8_t totalWaypoints; // Total de waypoints a recibir
  float latitude;         // Latitud
  float longitude;        // Longitud
  uint8_t checksum;       // XOR de todos los bytes anteriores
};
```

## Flujo de Comunicación

### 1. Solicitar Waypoints
El vehículo envía una solicitud a la estación:
```cpp
LoRa.beginPacket();
LoRa.write(MSG_REQUEST_WAYPOINTS);
LoRa.endPacket();
```

### 2. Recibir START_TRANSMISSION
La estación responde con el inicio de transmisión:
```cpp
// Esperar paquete
int packetSize = LoRa.parsePacket();
if (packetSize >= 2) {
  uint8_t msgType = LoRa.read();
  uint8_t totalWaypoints = LoRa.read();
  
  if (msgType == MSG_START_TRANSMISSION) {
    Serial.println("Recibiremos " + String(totalWaypoints) + " waypoints");
  }
}
```

### 3. Recibir Waypoints
Por cada waypoint:

```cpp
int packetSize = LoRa.parsePacket();
if (packetSize == sizeof(WaypointPacket)) {
  WaypointPacket packet;
  LoRa.readBytes((uint8_t*)&packet, sizeof(WaypointPacket));
  
  // Verificar checksum
  uint8_t calculatedChecksum = calculateChecksum((uint8_t*)&packet, sizeof(WaypointPacket) - 1);
  
  if (calculatedChecksum == packet.checksum && packet.msgType == MSG_WAYPOINT_DATA) {
    // Waypoint válido, guardar
    Serial.println("Waypoint " + String(packet.waypointId) + "/" + String(packet.totalWaypoints));
    Serial.println("Lat: " + String(packet.latitude, 6));
    Serial.println("Lon: " + String(packet.longitude, 6));
    
    // Guardar waypoint
    waypoints[packet.waypointId - 1].latitude = packet.latitude;
    waypoints[packet.waypointId - 1].longitude = packet.longitude;
    
    // Enviar ACK
    LoRa.beginPacket();
    LoRa.write(MSG_ACK);
    LoRa.write(packet.waypointId);
    LoRa.endPacket();
    
    Serial.println("ACK enviado para waypoint " + String(packet.waypointId));
  } else {
    Serial.println("ERROR: Checksum inválido!");
  }
}
```

### 4. Recibir END_TRANSMISSION
```cpp
int packetSize = LoRa.parsePacket();
if (packetSize) {
  uint8_t msgType = LoRa.read();
  
  if (msgType == MSG_END_TRANSMISSION) {
    Serial.println("Transmisión completa!");
    // Proceder con la misión
  }
}
```

## Función de Checksum
```cpp
uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}
```

## Código Completo de Ejemplo para el Vehículo

```cpp
#include <LoRa.h>

#define LORA_FREQUENCY 868E6
#define MAX_WAYPOINTS 50

struct Waypoint {
  float latitude;
  float longitude;
  bool received;
};

struct WaypointPacket {
  uint8_t msgType;
  uint8_t waypointId;
  uint8_t totalWaypoints;
  float latitude;
  float longitude;
  uint8_t checksum;
};

Waypoint waypoints[MAX_WAYPOINTS];
int totalWaypoints = 0;
int waypointsReceived = 0;

void setup() {
  Serial.begin(115200);
  
  // Inicializar LoRa
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error LoRa");
    while (1);
  }
  
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  
  // Solicitar waypoints
  requestWaypoints();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  
  if (packetSize) {
    if (packetSize == 2) {
      // START o END transmission
      uint8_t msgType = LoRa.read();
      
      if (msgType == 0x05) { // START_TRANSMISSION
        totalWaypoints = LoRa.read();
        Serial.println("Recibiremos " + String(totalWaypoints) + " waypoints");
        waypointsReceived = 0;
      } 
      else if (msgType == 0x03) { // END_TRANSMISSION
        Serial.println("Transmisión completa!");
        Serial.println("Recibidos " + String(waypointsReceived) + "/" + String(totalWaypoints));
        
        // Iniciar misión
        startMission();
      }
    }
    else if (packetSize == sizeof(WaypointPacket)) {
      // Waypoint data
      WaypointPacket packet;
      LoRa.readBytes((uint8_t*)&packet, sizeof(WaypointPacket));
      
      uint8_t checksum = calculateChecksum((uint8_t*)&packet, sizeof(WaypointPacket) - 1);
      
      if (checksum == packet.checksum && packet.msgType == 0x02) {
        // Guardar waypoint
        int idx = packet.waypointId - 1;
        waypoints[idx].latitude = packet.latitude;
        waypoints[idx].longitude = packet.longitude;
        waypoints[idx].received = true;
        waypointsReceived++;
        
        Serial.println("Waypoint " + String(packet.waypointId) + " OK");
        
        // Enviar ACK
        LoRa.beginPacket();
        LoRa.write((uint8_t)0x04); // MSG_ACK
        LoRa.write(packet.waypointId);
        LoRa.endPacket();
      }
    }
  }
}

void requestWaypoints() {
  Serial.println("Solicitando waypoints...");
  LoRa.beginPacket();
  LoRa.write((uint8_t)0x01); // MSG_REQUEST_WAYPOINTS
  LoRa.endPacket();
}

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void startMission() {
  Serial.println("=== WAYPOINTS RECIBIDOS ===");
  for (int i = 0; i < totalWaypoints; i++) {
    if (waypoints[i].received) {
      Serial.print("WP" + String(i+1) + ": ");
      Serial.print(waypoints[i].latitude, 6);
      Serial.print(", ");
      Serial.println(waypoints[i].longitude, 6);
    }
  }
  Serial.println("=========================");
  
  // Aquí va tu código de navegación
}
```

## Parámetros de Reintentos
- **Máximo de reintentos**: 3
- **Timeout para ACK**: 5 segundos
- **Delay entre reintentos**: 500 ms

## Notas Importantes
1. Asegúrate de que ambos dispositivos usen la misma configuración LoRa
2. El checksum es XOR de todos los bytes del paquete (excepto el campo checksum)
3. Los IDs de waypoint son 1-based (empiezan en 1, no en 0)
4. Siempre enviar ACK aunque el waypoint ya se haya recibido antes
5. La estación reintentará hasta 3 veces si no recibe ACK