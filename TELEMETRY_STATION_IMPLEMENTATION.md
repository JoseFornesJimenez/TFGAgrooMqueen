# Implementación de Telemetría en la Estación Base

## 📡 Descripción General

El robot ahora transmite telemetría cada 2 segundos cuando tiene GPS fix. La estación base debe recibir y mostrar esta información en tiempo real.

---

## 🔧 Cambios Necesarios en la Estación Base

### 1. Añadir Nuevos Tipos de Mensaje al Protocolo

Agrega estos dos nuevos tipos de mensaje a tu código:

```cpp
// En la sección de definiciones de protocolo
#define MSG_TELEMETRY 0x06        // Telemetría del robot
#define MSG_TELEMETRY_ACK 0x07    // ACK de telemetría (opcional)
```

---

### 2. Añadir la Estructura de Datos de Telemetría

**⚠️ IMPORTANTE:** Esta estructura debe ser **idéntica** a la del robot. El `__attribute__((packed))` es **obligatorio** para evitar problemas de alineación de memoria.

```cpp
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
```

---

### 3. Variables Globales para Almacenar Telemetría

Añade estas variables globales para guardar la última telemetría recibida:

```cpp
// Variables para almacenar última telemetría
float robotLatitude = 0.0;
float robotLongitude = 0.0;
uint8_t robotSatellites = 0;
bool robotGpsFixed = false;
uint8_t robotCurrentWaypoint = 0;
float robotDistanceToWaypoint = 0.0;
float robotHeadingToWaypoint = 0.0;
int16_t robotRssi = 0;
float robotSnr = 0.0;
uint8_t robotBatteryLevel = 0;
unsigned long lastTelemetryReceived = 0;
```

---

### 4. Función para Procesar Telemetría Recibida

Añade esta función que valida y procesa el paquete de telemetría:

```cpp
void processTelemetry(uint8_t* data, int size) {
  if (size != sizeof(TelemetryPacket)) {
    Serial.println("[Telemetry] Tamaño de paquete incorrecto");
    return;
  }
  
  TelemetryPacket* packet = (TelemetryPacket*)data;
  
  // Validar checksum
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) {
    sum += data[i];
  }
  
  if (sum != packet->checksum) {
    Serial.println("[Telemetry] Checksum inválido");
    return;
  }
  
  // Actualizar variables globales
  robotLatitude = packet->latitude;
  robotLongitude = packet->longitude;
  robotSatellites = packet->satellites;
  robotGpsFixed = packet->gpsFixed;
  robotCurrentWaypoint = packet->currentWaypoint;
  robotDistanceToWaypoint = packet->distanceToWaypoint;
  robotHeadingToWaypoint = packet->headingToWaypoint;
  robotRssi = packet->rssi;
  robotSnr = packet->snr;
  robotBatteryLevel = packet->batteryLevel;
  lastTelemetryReceived = millis();
  
  // Mostrar en Serial
  Serial.println("\n========== TELEMETRÍA ROBOT ==========");
  Serial.print("Posición: ");
  Serial.print(robotLatitude, 6);
  Serial.print(", ");
  Serial.println(robotLongitude, 6);
  Serial.print("Satélites: ");
  Serial.print(robotSatellites);
  Serial.print(" | GPS Fix: ");
  Serial.println(robotGpsFixed ? "SI" : "NO");
  Serial.print("Waypoint actual: ");
  Serial.print(robotCurrentWaypoint + 1);
  Serial.print(" | Distancia: ");
  Serial.print(robotDistanceToWaypoint, 1);
  Serial.println(" m");
  Serial.print("Rumbo: ");
  Serial.print(robotHeadingToWaypoint, 1);
  Serial.println("°");
  Serial.print("RSSI: ");
  Serial.print(robotRssi);
  Serial.print(" dBm | SNR: ");
  Serial.print(robotSnr, 1);
  Serial.println(" dB");
  Serial.print("Batería: ");
  Serial.print(robotBatteryLevel);
  Serial.println("%");
  Serial.println("======================================\n");
  
  // Mostrar en OLED
  displayTelemetry();
}
```

---

### 5. Función para Mostrar Telemetría en OLED

Esta función muestra la telemetría en la pantalla OLED de la estación:

```cpp
void displayTelemetry() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  
  // Título
  u8g2.drawStr(0, 10, "TELEMETRIA ROBOT");
  
  if (robotGpsFixed) {
    // Línea 1: Posición
    String pos = String(robotLatitude, 4) + "," + String(robotLongitude, 4);
    u8g2.setFont(u8g2_font_5x7_tr);  // Fuente pequeña para coordenadas
    u8g2.drawStr(0, 22, pos.c_str());
    
    // Línea 2: Waypoint y distancia
    u8g2.setFont(u8g2_font_ncenB08_tr);
    String wp = "WP" + String(robotCurrentWaypoint + 1) + " " + String(robotDistanceToWaypoint, 0) + "m";
    u8g2.drawStr(0, 34, wp.c_str());
    
    // Línea 3: Rumbo y satélites
    String nav = String(robotHeadingToWaypoint, 0) + "° S:" + String(robotSatellites);
    u8g2.drawStr(0, 46, nav.c_str());
    
    // Línea 4: Radio y batería
    String radio = "R:" + String(robotRssi) + " B:" + String(robotBatteryLevel) + "%";
    u8g2.drawStr(0, 58, radio.c_str());
    
  } else {
    u8g2.drawStr(0, 35, "Sin GPS Fix");
    String sats = "Sats: " + String(robotSatellites);
    u8g2.drawStr(0, 50, sats.c_str());
  }
  
  u8g2.sendBuffer();
}
```

---

### 6. Modificar la Función de Recepción LoRa

En tu función que procesa paquetes LoRa recibidos (probablemente `processLoRaPacket()` o similar), añade el caso para telemetría:

```cpp
void processLoRaPacket() {
  int packetSize = radio.getPacketLength();
  if (packetSize <= 0) return;
  
  uint8_t rxBuffer[256];
  int state = radio.readData(rxBuffer, packetSize);
  
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("[LoRa] Error al leer datos: ");
    Serial.println(state);
    return;
  }
  
  int rssi = radio.getRSSI();
  float snr = radio.getSNR();
  
  // Identificar tipo de mensaje
  uint8_t msgType = rxBuffer[0];
  
  if (msgType == MSG_TELEMETRY) {
    // Procesar telemetría
    processTelemetry(rxBuffer, packetSize);
    
    // OPCIONAL: Enviar ACK
    // sendTelemetryAck();
  }
  else if (msgType == MSG_REQUEST_WAYPOINTS) {
    // Tu código existente para REQUEST_WAYPOINTS
    // ...
  }
  else if (msgType == MSG_ACK) {
    // Tu código existente para ACK
    // ...
  }
  // ... resto de mensajes
}
```

---

### 7. (OPCIONAL) Enviar ACK de Telemetría

Si quieres confirmar la recepción de telemetría (no es estrictamente necesario):

```cpp
void sendTelemetryAck() {
  uint8_t ackBuffer[2];
  ackBuffer[0] = MSG_TELEMETRY_ACK;
  ackBuffer[1] = robotCurrentWaypoint;
  
  int state = radio.transmit(ackBuffer, 2);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] ACK telemetría enviado");
  }
  
  radio.startReceive();
}
```

---

## 📊 Datos que Recibirás

Cada 2 segundos (cuando el robot tenga GPS fix), recibirás:

| Campo | Tipo | Descripción |
|-------|------|-------------|
| `latitude` | `float` | Latitud del robot (ej: 37.280464) |
| `longitude` | `float` | Longitud del robot (ej: -5.921275) |
| `satellites` | `uint8_t` | Número de satélites GPS (0-12+) |
| `gpsFixed` | `uint8_t` | 1 = tiene fix GPS, 0 = sin fix |
| `currentWaypoint` | `uint8_t` | Índice del waypoint actual (0-49) |
| `distanceToWaypoint` | `float` | Distancia en metros al waypoint |
| `headingToWaypoint` | `float` | Rumbo en grados (0-360) al waypoint |
| `rssi` | `int16_t` | RSSI de la señal LoRa (dBm) |
| `snr` | `float` | SNR de la señal LoRa (dB) |
| `batteryLevel` | `uint8_t` | Batería en % (actualmente fijo a 100) |

---

## 🔍 Verificación

Para verificar que funciona correctamente:

1. **En el Serial Monitor del robot** verás:
   ```
   [Telemetry] Paquete enviado OK
   [Telemetry] Pos: 37.280464, -5.921275 | Sats: 6 | WP: 0 | Dist: 245.3m
   ```

2. **En el Serial Monitor de la estación** verás:
   ```
   ========== TELEMETRÍA ROBOT ==========
   Posición: 37.280464, -5.921275
   Satélites: 6 | GPS Fix: SI
   Waypoint actual: 1 | Distancia: 245.3 m
   Rumbo: 87.5°
   RSSI: -45 dBm | SNR: 8.2 dB
   Batería: 100%
   ======================================
   ```

3. **En el OLED de la estación** verás la telemetría actualizada cada 2 segundos.

---

## ⚠️ Notas Importantes

1. **Estructura empaquetada**: El `__attribute__((packed))` es **obligatorio** en ambos lados (robot y estación) para que la estructura tenga el mismo tamaño de memoria.

2. **Checksum**: Se valida automáticamente. Si falla, el paquete se descarta.

3. **Frecuencia**: El robot envía telemetría cada 2 segundos solo cuando tiene GPS fix.

4. **Batería**: Actualmente está fija en 100%. Se implementará lectura real más adelante.

5. **Timeout de telemetría**: Puedes añadir una verificación de timeout:
   ```cpp
   // En el loop()
   if (millis() - lastTelemetryReceived > 10000) {
     // Han pasado más de 10 segundos sin telemetría
     showMessage("WARNING", "No telemetry");
   }
   ```

---

## 📝 Resumen de Implementación

1. ✅ Añadir `MSG_TELEMETRY` y `MSG_TELEMETRY_ACK` al protocolo
2. ✅ Añadir estructura `TelemetryPacket` con `__attribute__((packed))`
3. ✅ Añadir variables globales para almacenar telemetría
4. ✅ Implementar función `processTelemetry()`
5. ✅ Implementar función `displayTelemetry()`
6. ✅ Modificar `processLoRaPacket()` para manejar `MSG_TELEMETRY`
7. ⚪ (Opcional) Implementar `sendTelemetryAck()`

---

## 🚀 Siguiente Paso

Una vez implementada la recepción de telemetría en la estación, podrás monitorizar en tiempo real:
- La posición GPS del robot
- Su progreso hacia cada waypoint
- La calidad de la señal LoRa
- El estado del GPS

Esto será **fundamental** cuando implementemos la navegación autónoma, ya que podrás verificar que el robot se está moviendo correctamente hacia los waypoints.

---

**¿Dudas o problemas?** El robot ya está transmitiendo telemetría. Solo falta que la estación la reciba y muestre. 📡
