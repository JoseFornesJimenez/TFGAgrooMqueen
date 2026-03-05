## 🚜 AgroMqueen — Vehículo Autónomo de Navegación GPS

Robot autónomo basado en **Heltec WiFi LoRa 32 V3** (ESP32-S3) que recibe waypoints por LoRa desde una estación base, navega de forma autónoma usando GPS + magnetómetro HMC5883L + giroscopio MPU-6500, y envía telemetría en tiempo real.

> **Repositorio:** [https://git.pandauniverse.es/panda/AgroMqueen](https://git.pandauniverse.es/panda/AgroMqueen)

---

## 📋 Índice

- [Descripción General](#-descripción-general)
- [Hardware](#-hardware)
- [Pinout Completo](#-pinout-completo)
- [Arquitectura del Software](#-arquitectura-del-software)
- [Protocolo LoRa](#-protocolo-lora)
- [Secuencia de Arranque](#-secuencia-de-arranque)
- [Calibración del Magnetómetro](#-calibración-del-magnetómetro)
- [Control de Motores y PID](#-control-de-motores-y-pid)
- [Navegación GPS](#-navegación-gps)
- [WiFi Telnet](#-wifi-telnet)
- [Dependencias](#-dependencias)
- [Compilar y Subir](#-compilar-y-subir)
- [Notas Técnicas](#-notas-técnicas)

---

## 🔍 Descripción General

El sistema funciona en dos partes:

| Componente | Hardware | Función |
|---|---|---|
| **Estación Base** | Heltec WiFi LoRa 32 **V2** | Recibe waypoints por Serial, los envía por LoRa, muestra telemetría |
| **Vehículo (este repo)** | Heltec WiFi LoRa 32 **V3** | Recibe waypoints, navega con GPS+brújula, controla motores, envía telemetría |

### Flujo de operación

```
Estación Base                          Vehículo
     │                                     │
     │  ◄── REQUEST_WAYPOINTS (0x01) ──── │  (el vehículo solicita los waypoints)
     │                                     │
     │ ── START_TRANSMISSION (0x05) ────► │  (la estación anuncia cuántos waypoints)
     │ ── WAYPOINT_DATA (0x02) ─────────► │
     │  ◄── ACK (0x04) ────────────────── │  (el vehículo confirma cada waypoint)
     │ ── WAYPOINT_DATA (0x02) ─────────► │
     │  ◄── ACK (0x04) ────────────────── │
     │ ── END_TRANSMISSION (0x03) ──────► │
     │                                     │
     │  ◄── TELEMETRY (0x06) ──────────── │  (cada 2s con GPS fix)
```

---

## 🔧 Hardware

| Componente | Modelo | Función |
|---|---|---|
| MCU | Heltec WiFi LoRa 32 V3 (ESP32-S3) | Controlador principal + LoRa SX1262 |
| GPS | NEO-6M | Posicionamiento por satélite |
| Magnetómetro | HMC5883L | Brújula magnética (I2C directo, 0x1E) |
| IMU | MPU-6500 | Giroscopio Z para fusión complementaria (0x68) |
| Motor Driver | L298N | Control de 2 motores DC |
| Pantalla | OLED SSD1306 128×64 | Información en tiempo real |
| Motores | 2× DC con reductora | Tracción diferencial |

---

## 📌 Pinout Completo

### OLED (I2C Principal — Software I2C)
| Señal | GPIO | Nota |
|---|---|---|
| SDA | 17 | |
| SCL | 18 | |
| RST | 21 | |
| Vext | 36 | Alimentación pantalla (LOW = ON) |

### GPS NEO-6M (UART — HardwareSerial 1)
| Señal | GPIO | Nota |
|---|---|---|
| RX (ESP ← GPS TX) | 1 | |
| TX (ESP → GPS RX) | 2 | |
| Baudrate | Auto-detect | Prueba 9600, 4800, 38400, 57600, 115200 |

### HMC5883L + MPU-6500 (I2C Hardware — Wire)
| Señal | GPIO | Nota |
|---|---|---|
| SDA | 6 | |
| SCL | 7 | |
| Frecuencia | 400 kHz | |
| HMC5883L addr | 0x1E | Magnetómetro |
| MPU-6500 addr | 0x68 | Giroscopio Z (WHO_AM_I=0x70) |

### LoRa SX1262 (SPI — integrado en la placa)
| Señal | GPIO |
|---|---|
| SCK | 9 |
| MISO | 11 |
| MOSI | 10 |
| CS | 8 |
| RST | 12 |
| DIO1 | 14 |
| BUSY | 13 |

### Motores L298N
| Señal | GPIO | Canal LEDC | Nota |
|---|---|---|---|
| Motor A IN1 | 3 | CH0 | PWM para velocidad |
| Motor A IN2 | 26 | CH1 | PWM para velocidad |
| Motor A ENA | 48 | — | GPIO HIGH fijo (LEDC no funciona) |
| Motor B IN1 | 4 | CH2 | PWM para velocidad |
| Motor B IN2 | 20 | CH3 | PWM para velocidad |
| Motor B ENB | 47 | — | GPIO HIGH fijo (LEDC no funciona) |

> ⚠️ **GPIO 48 y 47** no soportan LEDC PWM en el ESP32-S3. Por eso se usan como enable fijo y el PWM se aplica en los pines IN.

> ⚠️ **GPIO 19** (USB D-) y **GPIO 45** (strapping pin) NO se pueden usar como salidas en el ESP32-S3.

---

## 🏗 Arquitectura del Software

```
placeholder.cpp
├── Inicialización (setup)
│   ├── oledInit()              → Pantalla OLED por Software I2C
│   ├── WiFi AP + Telnet        → AP "AgroMqueen" / telnet 192.168.4.1:23
│   ├── gpsAutoDetect()         → Auto-detección de baudrate del GPS
│   ├── motorsInit()            → LEDC PWM en pines IN, ENA/ENB = HIGH
│   ├── hmcInit()               → HMC5883L por I2C hardware (400 kHz)
│   ├── mpuInit()               → MPU-6500 por I2C, calibra bias giroscopio
│   ├── loraInit()              → SX1262 a 868 MHz, CRC deshabilitado
│   ├── Test HMC (8s)           → Muestra valores raw para verificar cableado
│   ├── hmcLoadCalibration()    → Carga cal. desde flash NVS; si no hay, calibra a mano
│   ├── detectAndSaveTurnSign() → Detecta si turnRight sube o baja el heading
│   ├── faceNorth()             → Verifica magnetómetro orientando al Norte
│   ├── Espera GPS (60s)
│   └── requestWaypoints()      → Solicita waypoints por LoRa, activa navegación
│
├── Bucle principal (loop)
│   ├── telnetHandle()          → Acepta/lee conexiones WiFi Telnet
│   ├── GPS feed                → Lee tramas NMEA del GPS
│   ├── navigate()              → Navegación GPS activa hacia waypoints
│   ├── Auto-retry waypoints    → Cada 60s si no hay ruta activa
│   └── sendTelemetry()         → Envía posición/estado cada 2s
│
└── Funciones de movimiento
    ├── driveStraight()         → Avance recto con PID (fusión gyro+HMC)
    ├── returnToOrigin()        → Avanza, gira 180°, vuelve al origen por GPS
    └── faceNorth()             → Orienta al Norte con control proporcional
```

---

## 📡 Protocolo LoRa

### Configuración de Radio
| Parámetro | Valor |
|---|---|
| Frecuencia | 868.0 MHz (Europa) |
| Ancho de Banda | 125 kHz |
| Spreading Factor | 7 |
| Potencia TX | 14 dBm |
| CRC hardware | Deshabilitado (checksum propio en protocolo) |

### Tipos de Mensaje
| Código | Nombre | Dirección | Descripción |
|---|---|---|---|
| `0x01` | REQUEST_WAYPOINTS | Vehículo → Estación | Solicitar waypoints |
| `0x02` | WAYPOINT_DATA | Estación → Vehículo | Datos de un waypoint |
| `0x03` | END_TRANSMISSION | Estación → Vehículo | Fin de transmisión |
| `0x04` | ACK | Vehículo → Estación | Confirmación de waypoint |
| `0x05` | START_TRANSMISSION | Estación → Vehículo | Inicio de transmisión |
| `0x06` | TELEMETRY | Vehículo → Estación | Telemetría en tiempo real |

### Estructura WaypointPacket (12 bytes)
```cpp
struct __attribute__((packed)) WaypointPacket {
  uint8_t msgType;         // 0x02
  uint8_t waypointId;      // 1-based
  uint8_t totalWaypoints;
  float   latitude;        // 4 bytes
  float   longitude;       // 4 bytes
  uint8_t checksum;        // XOR de los bytes anteriores
};
```

### Estructura TelemetryPacket
```cpp
struct __attribute__((packed)) TelemetryPacket {
  uint8_t msgType;              // 0x06
  float   latitude;
  float   longitude;
  uint8_t satellites;
  uint8_t gpsFixed;             // 1 = fix, 0 = no fix
  uint8_t currentWaypoint;
  float   distanceToWaypoint;   // metros
  float   headingToWaypoint;    // grados (0-360)
  int16_t rssi;
  float   snr;
  uint8_t batteryLevel;         // 0-100%
  uint8_t checksum;             // Suma de todos los bytes anteriores
};
```

---

## 🚀 Secuencia de Arranque

1. **OLED** — Inicializa pantalla, muestra "VEHICULO AUTONOMO"
2. **WiFi AP** — Crea AP "AgroMqueen" (pass: agro1234), inicia servidor Telnet en puerto 23
3. **GPS** — Auto-detección de baudrate (9600, 4800, 38400, 57600, 115200)
4. **Motores** — Configura LEDC PWM en pines IN, ENA/ENB como GPIO HIGH
5. **HMC5883L** — Inicializa magnetómetro por I2C (verifica ID 'H','4','3')
6. **MPU-6500** — Inicializa giroscopio, calibra bias Z con 500 muestras en reposo
7. **LoRa** — Inicializa SX1262 a 868 MHz, SF7, 125 kHz, CRC deshabilitado
8. **Test HMC (8s)** — Muestra valores crudos en Serial/Telnet para verificar
9. **Calibración HMC** — Carga desde flash NVS; si no hay datos guardados, calibra a mano (20s)
10. **DetectTurnSign** — Gira brevemente para detectar si turnRight sube o baja el heading
11. **faceNorth** — Orienta el robot al Norte para verificar el magnetómetro
12. **Espera GPS** — Hasta 60 segundos para adquisición de satélites
13. **Solicitar waypoints** — Pide waypoints por LoRa; si recibe, activa navegación

---

## 🧭 Calibración del Magnetómetro

El HMC5883L se conecta directamente al bus I2C hardware (GPIO 6/7, dirección 0x1E). La calibración se realiza **manualmente, con motores parados**, para evitar interferencias electromagnéticas.

### Proceso de calibración (manual, sin motores)
1. La pantalla OLED muestra "CALIBRA HMC — GIRA A MANO — 20 segundos..."
2. El operario **gira el robot a mano** describiendo varios giros de 360° durante 20s
3. Se recogen valores min/max de magX y magY
4. Se calculan **offsets** (centro de la elipse) y **escalas** (normalización esférica)
5. Los valores se **guardan en flash NVS** (Preferences) para no recalibrar en cada arranque

### Configuración del sensor
- 8 muestras promedio, 75 Hz, modo continuo
- Ganancia: ±1.3 Gauss (1090 LSB/G)
- Remapeo físico: sensor_X apunta izquierda, sensor_Y apunta atrás

### Fusión complementaria (en driveStraight)
```
fusedAngle = α × (fusedAngle + gyroZ × dt) + (1-α) × hmcDelta
```
- α = 0.20 (20% giroscopio, 80% magnetómetro)
- GYRO_CLAMP = 8°/s (descarta picos de vibración de motores)

> ⚠️ La calibración se guarda en NVS. Al arrancar, si hay calibración guardada se carga automáticamente y no es necesario repetirla.

---

## ⚙️ Control de Motores y PID

### Enfoque PWM
El ESP32-S3 no permite usar LEDC PWM en los **GPIO 48 y 47** (ENA/ENB del L298N). La solución:
- **ENA/ENB** → `digitalWrite(HIGH)` siempre activados
- **IN1, IN2, IN3, IN4** → LEDC PWM a 5 kHz, resolución 8 bits (0-255)

### Funciones de movimiento
| Función | Motor A (Derecho) | Motor B (Izquierdo) |
|---|---|---|
| `moveForward(speed)` | Adelante | Adelante |
| `moveBackward(speed)` | Atrás | Atrás |
| `turnRight(speed)` | Adelante | Atrás |
| `turnLeft(speed)` | Atrás | Adelante |
| `motorsStop()` | Parado | Parado |

### driveStraight — Avance recto con PID de fusión

```
PID: error = desviación acumulada respecto al heading inicial
     corrección = Kp*err + Ki*integral + Kd*derivada
     Motor A (derecho) = baseSpd - corrección
     Motor B (izquierdo) = baseSpd + corrección
```

| Parámetro | Valor | Nota |
|---|---|---|
| Kp | 2.5 | Proporcional |
| Ki | 0.05 | Integral |
| Kd | 0.0 | Derivativo |
| MAX_CORRECTION | 40 | Límite de corrección PWM |
| DEAD_BAND | 2° | Sin corrección dentro de este rango |
| SPEED_MS_PER_M | 2500 ms/m | Ajustar según medición real a PWM=200 |

### returnToOrigin — Prueba de retorno automático
1. Guarda posición GPS actual como origen
2. Avanza con `driveStraight()` el tiempo indicado
3. Gira 180° usando el magnetómetro
4. Navega de vuelta al origen guiado por GPS

---

## 🗺 Navegación GPS

La función `navigate()` se ejecuta en el loop principal y gestiona la ruta completa:

| Parámetro | Valor |
|---|---|
| WAYPOINT_REACH_M | 2.0 m — radio para considerar waypoint alcanzado |
| BASE_SPEED | 200 PWM — velocidad normal |
| TURN_SPEED | 160 PWM — velocidad al girar |
| HEADING_TOL_DEG | ±15° — tolerancia de rumbo para avanzar recto |

**Lógica de navegación:**
- Si la distancia al waypoint < 2 m → avanza al siguiente WP
- Si el error de rumbo > 15° → gira (derecha o izquierda según gTurnSign)
- Si el error de rumbo ≤ 15° → avanza; reduce velocidad a 120 si dist < 3 m
- Al completar todos los WPs → para motores y muestra "RUTA COMPLETADA"

---

## 📶 WiFi Telnet

El vehículo crea un punto de acceso WiFi para depuración inalámbrica:

| Parámetro | Valor |
|---|---|
| SSID | AgroMqueen |
| Contraseña | agro1234 |
| IP | 192.168.4.1 |
| Puerto Telnet | 23 |

Conectar con: `telnet 192.168.4.1` (PuTTY, Windows telnet, o `nc`)

Al conectar se vuelca automáticamente el **buffer de arranque** (últimos 2 KB de log).

### Comandos disponibles por Telnet

| Comando | Función |
|---|---|
| `navegar` | Pide waypoints al base station y arranca la navegación |
| `wp` | Lista los waypoints almacenados |
| `saltar` | Salta al siguiente waypoint |
| `calibrar` | Recalibra el magnetómetro HMC5883L (manual, 20s) |
| `retorno` | Prueba: avanza 8s y vuelve al origen por GPS |
| `estado` | Muestra heading, GPS fix y estado de navegación |
| `parar` | Para motores y desactiva la navegación |
| `ayuda` | Muestra lista de comandos |

---

## 📦 Dependencias

Definidas en `platformio.ini`:

```ini
[env:heltec_wifi_lora_32_V3]
platform = espressif32
board = heltec_wifi_lora_32_V3
framework = arduino
lib_deps =
    olikraus/U8g2@^2.34.14
    jgromes/RadioLib@^6.6.0
    mikalhart/TinyGPSPlus@^1.0.3
```

| Librería | Uso |
|---|---|
| **U8g2** | Pantalla OLED SSD1306 por Software I2C |
| **RadioLib** | Módulo LoRa SX1262 (Heltec V3) |
| **TinyGPS++** | Parsing de tramas NMEA del GPS |

> El magnetómetro HMC5883L y el giroscopio MPU-6500 se controlan directamente mediante registros I2C (sin librería externa).

---

## 🔨 Compilar y Subir

```bash
# Compilar
pio run

# Subir (ajusta el puerto COM)
pio run --target upload --upload-port COM20

# Monitor serial
pio device monitor --port COM20 --baud 115200

# También puedes monitorear por WiFi:
# telnet 192.168.4.1
```

---

## 📝 Notas Técnicas

### GPIOs problemáticos en ESP32-S3
- **GPIO 19**: Pin USB D- → no usar como salida
- **GPIO 45**: Strapping pin → no usar como salida
- **GPIO 48, 47**: LEDC PWM no funciona → usar solo como GPIO digital

### Framework Arduino
El proyecto usa **Arduino Core v2.x** para ESP32 (API vieja de LEDC):
```cpp
ledcSetup(channel, freq, resolution);
ledcAttachPin(pin, channel);
ledcWrite(channel, duty);
```

### Acceso al HMC5883L
El HMC5883L se lee directamente por I2C hardware. Los datos llegan en orden **X, Z, Y** (¡Z antes que Y!):
- Registros 0x03-0x04: X_MSB, X_LSB
- Registros 0x05-0x06: Z_MSB, Z_LSB ← Z primero
- Registros 0x07-0x08: Y_MSB, Y_LSB

### Remapeo físico del HMC5883L
Con la orientación actual del sensor en el robot:
- Norte (frente) = `-sensor_Y`
- Este (derecha) = `-sensor_X`

### Telemetría
- Se envía cada **2 segundos** siempre (con o sin GPS fix)
- Incluye: posición, satélites, waypoint actual, distancia, rumbo, RSSI, SNR, batería (fijo al 100% — TODO)

### detectAndSaveTurnSign
Al arrancar, el robot gira brevemente a la derecha y mide el cambio de heading para determinar si `turnRight()` aumenta (+1) o disminuye (-1) el heading. Este signo se usa en toda la lógica de navegación y corrección PID.
