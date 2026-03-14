// =============================================================
//  AgroMqueen — Firmware Vehículo Autónomo
//  Placa: Heltec WiFi LoRa 32 V3 (ESP32-S3)
//  Magnetómetro: HMC5883L (I2C directo, sin librería externa)
// =============================================================

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <U8g2lib.h>
#include <math.h>
#include <Preferences.h>
#include <WiFi.h>

// =============================================================
//  WIFI TELNET — depuración inalámbrica
//  Conecta al AP "AgroMqueen" con pass "agro1234"
//  Luego: telnet 192.168.4.1  (PuTTY, Windows telnet, o nc)
//  Todo lo que va a Serial también llega por WiFi.
// =============================================================
#define WIFI_SSID   "AgroMqueen"
#define WIFI_PASS   "agro1234"
#define TELNET_PORT 23

static WiFiServer  telnetServer(TELNET_PORT);
static WiFiClient  telnetClient;

// Buffer circular para guardar los últimos 2 KB de log de arranque.
// Se vuelca al cliente en cuanto conecta, así no se pierde nada.
#define WLOG_BUF_SIZE 2048
static char     wlogBuf[WLOG_BUF_SIZE];
static uint16_t wlogHead = 0;   // posición de escritura (circular)
static uint16_t wlogLen  = 0;   // cuántos bytes hay guardados

static void wlogBufPush(const char *msg) {
    for (const char *p = msg; *p; p++) {
        wlogBuf[wlogHead] = *p;
        wlogHead = (wlogHead + 1) % WLOG_BUF_SIZE;
        if (wlogLen < WLOG_BUF_SIZE) wlogLen++;
    }
}

static void wlogBufFlush(WiFiClient &c) {
    if (wlogLen == 0) return;
    uint16_t start = (wlogLen < WLOG_BUF_SIZE)
                     ? 0
                     : wlogHead;  // cuando está lleno el inicio es head
    for (uint16_t i = 0; i < wlogLen; i++) {
        c.write((uint8_t)wlogBuf[(start + i) % WLOG_BUF_SIZE]);
    }
}

// Envía a Serial + telnet + buffer circular
static void wlog(const char *msg) {
    ::Serial.print(msg);
    wlogBufPush(msg);
    if (telnetClient && telnetClient.connected()) {
        telnetClient.print(msg);
    }
}

static void wlogf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    wlog(buf);
}

// Buffer para leer comandos del cliente telnet
static char     cmdBuf[64];
static uint8_t  cmdLen = 0;

// Callback — se llama cuando llega un comando completo por telnet
// Devuelve true si el comando requiere acción bloqueante (el caller para el loop)
static bool (*telnetCmdCallback)(const char *cmd) = nullptr;

// Acepta nuevas conexiones, vuelca el buffer de arranque y lee comandos entrantes
static void telnetHandle() {
    if (telnetServer.hasClient()) {
        if (telnetClient && telnetClient.connected()) {
            telnetClient.stop();
        }
        telnetClient = telnetServer.accept();
        telnetClient.println("\r\n=== AgroMqueen Telnet ===");
        telnetClient.println("Comandos: calibrar | retorno | estado | ayuda");
        telnetClient.println("--- Log de arranque ---");
        wlogBufFlush(telnetClient);
        telnetClient.println("--- Tiempo real ---");
        cmdLen = 0;
    }

    // Leer bytes entrantes y acumular hasta '\n'
    while (telnetClient && telnetClient.connected() && telnetClient.available()) {
        char c = (char)telnetClient.read();
        if (c == '\r') continue;  // ignorar CR
        if (c == '\n' || cmdLen >= sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen] = '\0';
            cmdLen = 0;
            // Trim espacios
            char *cmd = cmdBuf;
            while (*cmd == ' ') cmd++;
            // Procesar — se gestiona en processTelnetCmd() llamado desde loop
            if (telnetCmdCallback) telnetCmdCallback(cmd);
        } else {
            cmdBuf[cmdLen++] = c;
        }
    }
}

// =============================================================
//  PINES
// =============================================================
// OLED (Software I2C)
#define PIN_OLED_SDA   17
#define PIN_OLED_SCL   18
#define PIN_OLED_RST   21
#define PIN_VEXT       36

// GPS (UART1)
#define PIN_GPS_RX      1
#define PIN_GPS_TX      2

// I2C Hardware — HMC5883L
#define PIN_I2C_SDA     6
#define PIN_I2C_SCL     7

// Motores L298N
// ENA/ENB → GPIO HIGH fijo (GPIO 47/48 no soportan LEDC en ESP32-S3)
// PWM aplicado en pines IN
#define PIN_A_IN1       3
#define PIN_A_IN2      26
#define PIN_A_ENA      48   // fijo HIGH
#define PIN_B_IN1       4
#define PIN_B_IN2      20
#define PIN_B_ENB      47   // fijo HIGH

// LoRa SX1262 (integrado en la placa)
#define PIN_LORA_SCK    9
#define PIN_LORA_MISO  11
#define PIN_LORA_MOSI  10
#define PIN_LORA_CS     8
#define PIN_LORA_RST   12
#define PIN_LORA_DIO1  14
#define PIN_LORA_BUSY  13

// =============================================================
//  HMC5883L — REGISTROS (I2C addr 0x1E)
// =============================================================
#define HMC_ADDR       0x1E
#define HMC_REG_CFG_A  0x00   // Config A: avg muestras, tasa, modo
#define HMC_REG_CFG_B  0x01   // Config B: ganancia
#define HMC_REG_MODE   0x02   // Modo de operación
#define HMC_REG_DATA   0x03   // Primer byte de datos (X_MSB)
#define HMC_REG_STATUS 0x09   // Estado (bit 0 = DRDY)
#define HMC_REG_ID_A   0x0A   // Identificación: 'H','4','3'

// Calibración (se actualiza en hmcCalibrate)
static int16_t hmcOffX  = 0,    hmcOffY  = 0;
static float   hmcScaleX = 1.0f, hmcScaleY = 1.0f;
static float   filteredHeading = 0.0f;
static bool    headingReady    = false;

// =============================================================
//  MPU-6500 — REGISTROS (I2C addr 0x68, mismo bus que HMC)
// =============================================================
#define MPU_ADDR        0x68
#define MPU_REG_PWR     0x6B   // Power management
#define MPU_REG_GYRO_CFG 0x1B  // Gyro config (full scale)
#define MPU_REG_SMPLRT  0x19   // Sample rate divider
#define MPU_REG_GYRO_Z  0x47   // GYRO_ZOUT_H (seguido de GYRO_ZOUT_L)
#define MPU_REG_WHO_AM_I 0x75  // Debe devolver 0x70 (MPU-6500)

// Bias del giroscopio Z (grados/s), calibrado en reposo
static float gyroZbias = 0.0f;
// Escala: MPU-6500 a ±250°/s → 131.0 LSB/(°/s)
static const float GYRO_SCALE = 131.0f;

// =============================================================
//  PROTOCOLO LORA
// =============================================================
#define MSG_REQUEST_WP  0x01
#define MSG_WP_DATA     0x02
#define MSG_END_TX      0x03
#define MSG_ACK         0x04
#define MSG_START_TX    0x05
#define MSG_TELEMETRY   0x06
#define MAX_WAYPOINTS   20

struct __attribute__((packed)) WaypointPacket {
    uint8_t msgType;
    uint8_t waypointId;     // 1-based
    uint8_t totalWaypoints;
    float   latitude;
    float   longitude;
    uint8_t checksum;       // XOR de todos los bytes anteriores
};

struct __attribute__((packed)) TelemetryPacket {
    uint8_t msgType;
    float   latitude;
    float   longitude;
    uint8_t satellites;
    uint8_t gpsFixed;
    uint8_t currentWaypoint;
    float   distanceToWaypoint;
    float   headingToWaypoint;
    int16_t rssi;
    float   snr;
    uint8_t batteryLevel;
    uint8_t checksum;
};

// =============================================================
//  ESTADO GLOBAL
// =============================================================
struct Waypoint { float lat, lon; };
Waypoint waypoints[MAX_WAYPOINTS];
uint8_t  waypointCount   = 0;
uint8_t  currentWaypoint = 0;
bool     navActive       = false;
// +1 si turnRight() sube el heading, -1 si lo baja.
// Se detecta automáticamente al arrancar (detectTurnSign).
static int gTurnSign = -1;

#define TELEMETRY_INTERVAL_MS 2000UL
static unsigned long lastTelemetryMs = 0;

// =============================================================
//  OBJETOS
// =============================================================
U8G2_SSD1306_128X64_NONAME_F_SW_I2C
    display(U8G2_R0, PIN_OLED_SCL, PIN_OLED_SDA, PIN_OLED_RST);

SX1262        radio = new Module(PIN_LORA_CS, PIN_LORA_DIO1,
                                 PIN_LORA_RST, PIN_LORA_BUSY);
TinyGPSPlus    gps;
HardwareSerial gpsSerial(1);
Preferences    prefs;   // NVS — calibración HMC5883L

// =============================================================
//  DECLARACIONES ADELANTADAS
// =============================================================
void motorsStop();
void turnRight(uint8_t spd);
void turnLeft(uint8_t spd);
void moveForward(uint8_t spd);
void oledShow(const char *l1, const char *l2 = "", const char *l3 = "");
void driveStraight(float distanceM, uint8_t baseSpd);
void returnToOrigin(uint32_t advanceMs, uint8_t spd);
void detectAndSaveTurnSign();
static bool processTelnetCmd(const char *cmd);

// =============================================================
//  HMC5883L — IMPLEMENTACIÓN DIRECTA I2C
// =============================================================

// Guarda la calibración actual en NVS (flash persistente)
static void hmcSaveCalibration() {
    prefs.begin("hmc", false);           // namespace "hmc", modo lectura-escritura
    prefs.putInt  ("offX",   hmcOffX);
    prefs.putInt  ("offY",   hmcOffY);
    prefs.putFloat("scaleX", hmcScaleX);
    prefs.putFloat("scaleY", hmcScaleY);
    prefs.putBool ("valid",  true);
    prefs.end();
    wlogf("[HMC] Calibración guardada en flash (NVS)");
}

// Carga la calibración desde NVS. Devuelve true si había datos válidos.
static bool hmcLoadCalibration() {
    prefs.begin("hmc", true);            // namespace "hmc", modo solo lectura
    bool valid = prefs.getBool("valid", false);
    if (valid) {
        hmcOffX   = (int16_t)prefs.getInt  ("offX",   0);
        hmcOffY   = (int16_t)prefs.getInt  ("offY",   0);
        hmcScaleX = prefs.getFloat("scaleX", 1.0f);
        hmcScaleY = prefs.getFloat("scaleY", 1.0f);
    }
    prefs.end();
    if (valid) {
        wlogf("[HMC] Calibración cargada: offX=%d offY=%d scX=%.3f scY=%.3f", 
                      hmcOffX, hmcOffY, hmcScaleX, hmcScaleY);
        // Resetear filtro para que parta de cero con los nuevos offsets
        headingReady = false;
        filteredHeading = 0.0f;
    } else {
        wlogf("[HMC] No hay calibración guardada en flash");
    }
    return valid;
}

static void hmcWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t hmcReadByte(uint8_t reg) {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)HMC_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

bool hmcInit() {
    // Verificar ID del chip: registros 0x0A..0x0C deben devolver 'H','4','3'
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(HMC_REG_ID_A);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)HMC_ADDR, (uint8_t)3);
    if (Wire.available() < 3) {
        wlogf("[HMC] ERROR: no responde en 0x1E — revisa SDA/SCL");
        return false;
    }
    uint8_t idA = Wire.read();  // 'H' = 0x48
    uint8_t idB = Wire.read();  // '4' = 0x34
    uint8_t idC = Wire.read();  // '3' = 0x33
    if (idA != 0x48 || idB != 0x34 || idC != 0x33) {
        wlogf("[HMC] ERROR ID: 0x%02X 0x%02X 0x%02X (esperado 0x48 0x34 0x33)", 
                      idA, idB, idC);
        return false;
    }

    // Config A: 8 muestras promedio | 75 Hz | modo normal
    //   Bits 6-5 (MA)  = 11  → 8 muestras
    //   Bits 4-2 (DO)  = 110 → 75 Hz
    //   Bits 1-0 (MS)  = 00  → normal
    hmcWrite(HMC_REG_CFG_A, 0x78);

    // Config B: ganancia 1 → ±1.3 Gauss, 1090 LSB/G
    //   Bits 7-5 (GN) = 001
    hmcWrite(HMC_REG_CFG_B, 0x20);

    // Modo: medición continua
    hmcWrite(HMC_REG_MODE, 0x00);

    delay(10);
    wlogf("[HMC] HMC5883L inicializado OK");
    return true;
}

// Lee los tres ejes crudos sin verificar DRDY (llamar solo cuando DRDY ya fue confirmado).
// ATENCIÓN: el HMC5883L envía los bytes en orden X, Z, Y (¡Z antes que Y!)
static bool hmcReadRaw(int16_t &x, int16_t &y, int16_t &z) {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(HMC_REG_DATA);   // 0x03 → X_MSB
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)HMC_ADDR, (uint8_t)6);
    if (Wire.available() < 6) return false;

    uint8_t xh = Wire.read(), xl = Wire.read();  // 0x03, 0x04
    uint8_t zh = Wire.read(), zl = Wire.read();  // 0x05, 0x06  ← Z primero (no Y!)
    uint8_t yh = Wire.read(), yl = Wire.read();  // 0x07, 0x08

    x = (int16_t)((xh << 8) | xl);
    z = (int16_t)((zh << 8) | zl);
    y = (int16_t)((yh << 8) | yl);
    return true;
}

// Lee los tres ejes crudos verificando DRDY primero.
bool hmcRead(int16_t &x, int16_t &y, int16_t &z) {
    if (!(hmcReadByte(HMC_REG_STATUS) & 0x01)) return false;  // DRDY no activo
    return hmcReadRaw(x, y, z);
}

// Devuelve el rumbo magnético filtrado en grados (0-360°)
float hmcHeading() {
    int16_t mx, my, mz;
    if (!hmcRead(mx, my, mz)) return filteredHeading;

    // Remapeo físico corregido: sensor montado 180° respecto al original.
    // Se invierten los signos para que Norte = frente real del robot.
    float fx = (float)(my - hmcOffY) * hmcScaleY;
    float fy = (float)(mx - hmcOffX) * hmcScaleX;

    float h = atan2f(fy, fx) * 180.0f / (float)M_PI;
    if (h < 0.0f) h += 360.0f;

    // Filtro paso-bajo circular (α = 0.2)
    if (!headingReady) {
        filteredHeading = h;
        headingReady    = true;
    } else {
        float diff = h - filteredHeading;
        if (diff >  180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        filteredHeading += diff * 0.2f;
        if (filteredHeading <   0.0f) filteredHeading += 360.0f;
        if (filteredHeading >= 360.0f) filteredHeading -= 360.0f;
    }
    return filteredHeading;
}

// Calibración MANUAL sin motores — evita interferencia electromagnética.
// El operario gira el robot a mano durante 20s describiendo varios 360°.
// Motores deben estar parados durante toda la calibración.
void hmcCalibrate() {
    motorsStop();  // garantizar motores parados

    wlogf("[HMC] Calibrando — GIRA EL ROBOT A MANO 360 durante 20s");
    oledShow("CALIBRA HMC", "GIRA A MANO", "20 segundos...");

    int16_t xMin = 32767, xMax = -32768;
    int16_t yMin = 32767, yMax = -32768;
    uint32_t samples = 0;

    // Cuenta atrás visual en OLED
    for (int seg = 20; seg > 0; seg--) {
        unsigned long t0 = millis();
        while (millis() - t0 < 1000) {
            int16_t x, y, z;
            if (hmcRead(x, y, z)) {
                if (x < xMin) xMin = x;  if (x > xMax) xMax = x;
                if (y < yMin) yMin = y;  if (y > yMax) yMax = y;
                samples++;
            }
            delay(14);
        }
        char buf[22];
        snprintf(buf, sizeof(buf), "%d segundos...", seg - 1);
        oledShow("CALIBRA HMC", "GIRA A MANO", buf);
        wlogf("[HMC] Calibrando... %d s restantes\n", seg - 1);
    }

    // Calcular offsets (centro elipse)
    hmcOffX = (xMax + xMin) / 2;
    hmcOffY = (yMax + yMin) / 2;

    // Factores de escala (normalización esférica)
    float rX = (xMax - xMin) / 2.0f;
    float rY = (yMax - yMin) / 2.0f;
    float avg = (rX + rY) / 2.0f;
    hmcScaleX = (rX > 0.0f) ? avg / rX : 1.0f;
    hmcScaleY = (rY > 0.0f) ? avg / rY : 1.0f;

    wlogf("[HMC] Calibración OK — %lu muestras\n", (unsigned long)samples);
    wlogf("[HMC]   xMin=%d xMax=%d rX=%.0f\n", xMin, xMax, rX);
    wlogf("[HMC]   yMin=%d yMax=%d rY=%.0f\n", yMin, yMax, rY);
    wlogf("[HMC]   offX=%d offY=%d  scX=%.3f scY=%.3f\n",
                  hmcOffX, hmcOffY, hmcScaleX, hmcScaleY);

    // Advertir si el rango es demasiado pequeño (calibración insuficiente)
    if (rX < 80.0f || rY < 80.0f) {
        wlogf("[HMC] WARN: rango muy pequeño (rX=%.0f rY=%.0f) — gira mas despacio y mas vueltas!\n", rX, rY);
        oledShow("Cal DUDOSA", "Rango pequeño", "Repite calibrar");
        delay(3000);
    } else {
        oledShow("Cal guardada", "en Flash OK", "");
    }

    // Resetear filtro paso-bajo para que parta de cero con los nuevos offsets
    headingReady = false;
    filteredHeading = 0.0f;

    hmcSaveCalibration();  // persistir en NVS
    delay(1500);
}

// Verificación: gira 4×90° y comprueba que el heading varía > 200° en total
bool hmcVerify() {
    wlogf("[HMC] Verificando magnetómetro...");
    float h[4];
    for (int i = 0; i < 4; i++) {
        turnRight(200);
        delay(700);        // ≈ 90° a velocidad 200
        motorsStop();
        delay(500);        // esperar que se estabilice
        h[i] = hmcHeading();
        wlogf("[HMC]   Heading[%d] = %.1f\n", i, h[i]);
        delay(200);
    }
    // Calcular variación acumulada (circular)
    float totalChange = 0.0f;
    for (int i = 1; i < 4; i++) {
        float diff = h[i] - h[i - 1];
        if (diff >  180.0f) diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        totalChange += fabsf(diff);
    }
    wlogf("[HMC] Variación total: %.1f° (%s)\n",
                  totalChange, totalChange > 200.0f ? "OK" : "FALLO");
    return totalChange > 200.0f;
}

// Heading instantáneo sin filtro paso-bajo: promedia N lecturas crudas
// usando media circular para evitar el error en el cruce 0°/360°.
// Espera activa hasta que DRDY esté activo, con timeout en ms.
// Devuelve true si hay dato disponible.
static bool hmcWaitDRDY(uint16_t timeoutMs = 30) {
    unsigned long t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (hmcReadByte(HMC_REG_STATUS) & 0x01) return true;
        delay(2);
    }
    return false;
}

float hmcHeadingInstant(uint8_t n = 6) {
    float sinSum = 0.0f, cosSum = 0.0f;
    uint8_t valid = 0;
    for (uint8_t i = 0; i < n; i++) {
        // Espera activa hasta que haya dato nuevo (máx 30ms por muestra a 75Hz)
        if (!hmcWaitDRDY(30)) continue;
        int16_t mx, my, mz;
        // Usar hmcReadRaw: DRDY ya fue confirmado justo arriba, no re-leer status
        if (!hmcReadRaw(mx, my, mz)) continue;
        // Remapeo físico corregido (sensor montado 180°)
        float fx = (float)(my - hmcOffY) * hmcScaleY;
        float fy = (float)(mx - hmcOffX) * hmcScaleX;
        float angle = atan2f(fy, fx);
        sinSum += sinf(angle);
        cosSum += cosf(angle);
        valid++;
    }
    if (valid == 0) {
        // Sin lecturas válidas: reinicializar el HMC y devolver último valor conocido
        wlogf("[HMC] WARN: sin lecturas validas, reiniciando...\r\n");
        hmcInit();
        return filteredHeading;
    }
    float h = atan2f(sinSum, cosSum) * 180.0f / (float)M_PI;
    if (h < 0.0f) h += 360.0f;
    return h;
}

// Gira el robot hasta apuntar al Norte (0°), timeout 20s.
// Usa control proporcional + pulsos cortos cerca del objetivo
// para evitar sobrepasar el Norte.
// Detecta si turnRight() aumenta o disminuye el heading y guarda en gTurnSign.
// Llama una sola vez desde setup() antes de faceNorth().
void detectAndSaveTurnSign() {
    // Precalentar filtro con 10 lecturas
    for (int i = 0; i < 10; i++) { hmcHeading(); delay(15); }

    uint8_t spd = 180;
    float delta = 0.0f;

    // Reintentar con más fuerza si el robot no se mueve
    for (int attempt = 0; attempt < 4; attempt++) {
        float h0 = hmcHeadingInstant(12);
        wlogf("[HMC] TurnSign intento %d — SPD=%u h0=%.1f\n", attempt + 1, spd, h0);
        turnRight(spd);
        delay(600);
        motorsStop();
        delay(1200);
        for (int i = 0; i < 8; i++) { hmcHeading(); delay(15); }
        float h1 = hmcHeadingInstant(12);
        delta = h1 - h0;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;
        wlogf("[HMC] TurnSign: h0=%.1f h1=%.1f delta=%.1f\n", h0, h1, delta);
        if (fabsf(delta) >= 2.0f) break;  // movió suficiente — fiable
        spd = (uint8_t)min((int)spd + 25, 255);
        wlogf("[HMC] Delta insuficiente — subiendo a SPD=%u\n", spd);
    }

    gTurnSign = (delta >= 0.0f) ? +1 : -1;
    wlogf("[HMC] TurnSign final: %+d\n", gTurnSign);
}

bool faceNorth() {
    const float    TOL_DEG   =  5.0f;
    const uint32_t TIMEOUT   = 60000;

    // Velocidad adaptativa para faceNorth — sube si el robot no mueve
    uint8_t SPD_CONT  = 165;
    uint8_t SPD_VSLOW = 100;
    const uint8_t SPD_MAX = 255;
    const uint8_t SPD_STEP = 15;          // cuánto subir por pulso sin movimiento
    const float   MIN_DELTA_DEG = 1.5f;   // mínimo cambio de heading para considerarse "movido"

    wlogf("[HMC] Orientando al Norte (turnSign=%+d)...\n", gTurnSign);

    // Precalentar el filtro: leer 10 veces para estabilizar filteredHeading
    for (int i = 0; i < 10; i++) {
        hmcHeading();
        delay(15);
    }

    unsigned long t0 = millis();
    while (millis() - t0 < TIMEOUT) {
        // Asegurar motores parados antes de leer la posición actual
        motorsStop();
        delay(50);
        // Usar heading filtrado (paso-bajo α=0.2) para evitar saltos en 0°/360°
        float h   = hmcHeading();
        float err = -h;
        if (err >  180.0f) err -= 360.0f;
        if (err < -180.0f) err += 360.0f;
        float absErr = fabsf(err);

        char buf[22];
        snprintf(buf, sizeof(buf), "H:%.0f  err:%.0f", h, err);
        oledShow("-> Norte", buf, "");
        wlogf("[HMC] H=%.1f err=%.1f\n", h, err);

        if (absErr <= TOL_DEG) {
            motorsStop();
            // Confirmar con 5 lecturas seguidas que el error sigue dentro de tolerancia
            bool stable = true;
            for (int i = 0; i < 5; i++) {
                delay(20);
                float hc = hmcHeading();
                float ec = -hc;
                if (ec >  180.0f) ec -= 360.0f;
                if (ec < -180.0f) ec += 360.0f;
                if (fabsf(ec) > TOL_DEG * 1.5f) { stable = false; break; }
            }
            if (stable) {
                wlogf("[HMC] Norte alcanzado y estable: %.1f°\n", h);
                return true;
            }
            // No estable — continuar corrigiendo
        }

        bool goRight = (err * gTurnSign) > 0.0f;

        if (absErr > 10.0f) {
            // Lejos: giro continuo 250ms — medir delta para detectar si mueve
            float hBefore = hmcHeadingInstant(6);
            if (goRight) turnRight(SPD_CONT); else turnLeft(SPD_CONT);
            delay(250);
            motorsStop();
            delay(1200);
            for (int i = 0; i < 5; i++) { hmcHeading(); delay(15); }
            float hAfter = hmcHeadingInstant(6);
            float delta = fabsf(hAfter - hBefore);
            if (delta > 180.0f) delta = 360.0f - delta;
            if (delta < MIN_DELTA_DEG) {
                // El robot no giró — subir fuerza
                if (SPD_CONT < SPD_MAX) {
                    SPD_CONT = (uint8_t)min((int)SPD_CONT + SPD_STEP, (int)SPD_MAX);
                    wlogf("[HMC] Sin movimiento — SPD_CONT sube a %u\n", SPD_CONT);
                }
            }
        } else {
            // Muy cerca (≤10°): pulso muy corto y muy lento
            uint16_t pulseMs = (uint16_t)(absErr * 4.0f);
            if (pulseMs < 30)  pulseMs = 30;
            if (pulseMs > 80)  pulseMs = 80;
            float hBefore = hmcHeadingInstant(4);
            if (goRight) turnRight(SPD_VSLOW); else turnLeft(SPD_VSLOW);
            delay(pulseMs);
            motorsStop();
            delay(1800);
            for (int i = 0; i < 10; i++) { hmcHeading(); delay(15); }
            float hAfter = hmcHeadingInstant(4);
            float delta = fabsf(hAfter - hBefore);
            if (delta > 180.0f) delta = 360.0f - delta;
            if (delta < MIN_DELTA_DEG) {
                if (SPD_VSLOW < SPD_MAX) {
                    SPD_VSLOW = (uint8_t)min((int)SPD_VSLOW + SPD_STEP, (int)SPD_MAX);
                    wlogf("[HMC] Sin movimiento fino — SPD_VSLOW sube a %u\n", SPD_VSLOW);
                }
            }
        }
    }

    motorsStop();
    wlogf("[HMC] Timeout — heading final: %.1f°\n", hmcHeading());
    return false;
}

// =============================================================
//  OLED
// =============================================================
void oledInit() {
    pinMode(PIN_VEXT, OUTPUT);
    digitalWrite(PIN_VEXT, LOW);   // LOW = alimentar pantalla
    delay(100);
    display.begin();
    display.setFont(u8g2_font_6x10_tf);
    display.clearBuffer();
    display.sendBuffer();
}

void oledShow(const char *l1, const char *l2, const char *l3) {
    display.clearBuffer();
    display.drawStr(0, 10, l1);
    display.drawStr(0, 24, l2);
    display.drawStr(0, 38, l3);
    display.sendBuffer();
}

// =============================================================
//  MOTORES — LEDC API v2.x (Arduino Core para ESP32)
//  PWM en IN1/IN2; ENA/ENB fijos a HIGH
// =============================================================
#define MOT_FREQ  5000
#define MOT_RES      8   // 8 bits → duty 0-255
#define CH_A_IN1     0
#define CH_A_IN2     1
#define CH_B_IN1     2
#define CH_B_IN2     3

void motorsInit() {
    ledcSetup(CH_A_IN1, MOT_FREQ, MOT_RES);
    ledcSetup(CH_A_IN2, MOT_FREQ, MOT_RES);
    ledcSetup(CH_B_IN1, MOT_FREQ, MOT_RES);
    ledcSetup(CH_B_IN2, MOT_FREQ, MOT_RES);
    ledcAttachPin(PIN_A_IN1, CH_A_IN1);
    ledcAttachPin(PIN_A_IN2, CH_A_IN2);
    ledcAttachPin(PIN_B_IN1, CH_B_IN1);
    ledcAttachPin(PIN_B_IN2, CH_B_IN2);

    // ENA/ENB fijos HIGH
    pinMode(PIN_A_ENA, OUTPUT); digitalWrite(PIN_A_ENA, HIGH);
    pinMode(PIN_B_ENB, OUTPUT); digitalWrite(PIN_B_ENB, HIGH);

    // Parar
    ledcWrite(CH_A_IN1, 0); ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_B_IN1, 0); ledcWrite(CH_B_IN2, 0);
}

void motorsStop() {
    ledcWrite(CH_A_IN1, 0); ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_B_IN1, 0); ledcWrite(CH_B_IN2, 0);
}
void moveForward(uint8_t spd) {
    ledcWrite(CH_A_IN1, spd); ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_B_IN1, spd); ledcWrite(CH_B_IN2, 0);
}
void moveBackward(uint8_t spd) {
    ledcWrite(CH_A_IN1, 0); ledcWrite(CH_A_IN2, spd);
    ledcWrite(CH_B_IN1, 0); ledcWrite(CH_B_IN2, spd);
}
void turnRight(uint8_t spd) {   // Motor A adelante, Motor B atrás
    ledcWrite(CH_A_IN1, spd); ledcWrite(CH_A_IN2, 0);
    ledcWrite(CH_B_IN1, 0);   ledcWrite(CH_B_IN2, spd);
}
void turnLeft(uint8_t spd) {    // Motor A atrás, Motor B adelante
    ledcWrite(CH_A_IN1, 0);   ledcWrite(CH_A_IN2, spd);
    ledcWrite(CH_B_IN1, spd); ledcWrite(CH_B_IN2, 0);
}

// =============================================================
//  MPU-6500 — DRIVER DIRECTO I2C
// =============================================================

static void mpuWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// Inicializa el MPU-6500: despierta, configura giroscopio ±250°/s, 1kHz
bool mpuInit() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    uint8_t who = Wire.available() ? Wire.read() : 0x00;
    if (who != 0x70) {
        wlogf("[MPU] WHO_AM_I = 0x%02X (esperado 0x70)\n", who);
        return false;
    }
    mpuWrite(MPU_REG_PWR, 0x00);      // despertar, clock interno
    delay(10);
    mpuWrite(MPU_REG_GYRO_CFG, 0x00); // ±250 °/s → 131 LSB/(°/s)
    mpuWrite(MPU_REG_SMPLRT, 0x07);   // sample rate = 1kHz / (1+7) = 125 Hz
    delay(10);
    wlogf("[MPU] MPU-6500 inicializado OK (WHO_AM_I=0x70)");
    return true;
}

// Lee el eje Z del giroscopio y devuelve velocidad angular en °/s (ya con bias)
float mpuReadGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_GYRO_Z);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0.0f;
    int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
    return (raw / GYRO_SCALE) - gyroZbias;
}

// Calibra el bias del giroscopio Z promediando N muestras en reposo.
// Robot debe estar completamente quieto durante la calibración.
void mpuCalibrateGyro(uint16_t samples = 500) {
    wlogf("[MPU] Calibrando bias giroscopio (%u muestras)...\n", samples);
    double sum = 0.0;
    for (uint16_t i = 0; i < samples; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(MPU_REG_GYRO_Z);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
        if (Wire.available() >= 2) {
            int16_t raw = (int16_t)((Wire.read() << 8) | Wire.read());
            sum += raw / GYRO_SCALE;
        }
        delay(2);  // 125 Hz → muestra cada 8ms; 2ms de margen
    }
    gyroZbias = (float)(sum / samples);
    wlogf("[MPU] Bias giroscopio Z: %.4f °/s\n", gyroZbias);
}

// Control diferencial directo: spdA = Motor A (derecho), spdB = Motor B (izquierdo)
// Valores positivos = adelante, negativos = atrás. Rango: -255..255
static void driveMotors(int16_t spdA, int16_t spdB) {
    spdA = constrain(spdA, -255, 255);
    spdB = constrain(spdB, -255, 255);
    if (spdA >= 0) { ledcWrite(CH_A_IN1, spdA); ledcWrite(CH_A_IN2, 0); }
    else            { ledcWrite(CH_A_IN1, 0);    ledcWrite(CH_A_IN2, -spdA); }
    if (spdB >= 0) { ledcWrite(CH_B_IN1, spdB); ledcWrite(CH_B_IN2, 0); }
    else            { ledcWrite(CH_B_IN1, 0);    ledcWrite(CH_B_IN2, -spdB); }
}

// Avanza en línea recta durante 'distanceM' metros usando PID sobre el heading.
// El rumbo objetivo es el heading al arrancar (lo que haya en ese momento).
// El tiempo de recorrido se estima con velocidad conocida del robot.
//
//  PID: error = heading_actual - heading_objetivo (normalizado ±180°)
//       corrección = Kp*err + Ki*integral + Kd*derivada
//       Motor A (derecho) recibe baseSpd + corrección
//       Motor B (izquierdo)   recibe baseSpd - corrección
//
//  Para ~6 m a velocidad 200 PWM el tiempo depende del robot —
//  ajusta SPEED_MS_PER_M según tus pruebas reales.
// Diagnóstico de signos de sensores — ejecutar UNA VEZ antes de ajustar el PID.
// Con el robot quieto: imprime gz en reposo (debe ser ~0).
// Luego pide girar a mano a la DERECHA: gz debe ser POSITIVO, HMC heading sube.
// Luego pide girar a mano a la IZQUIERDA: gz debe ser NEGATIVO, HMC heading baja.
// Si los signos son al revés → cambiar GYRO_SIGN a -1 o invertir spdA/spdB.
void sensorDiagnose() {
    motorsStop();
    wlogf("=== DIAGNOSTICO SENSORES ===");
    wlogf("Robot quieto 3s — gz debe ser ~0:");
    oledShow("Diagnostico", "Robot QUIETO", "3 segundos...");
    for (int i = 0; i < 30; i++) {
        delay(100);
        float gz = mpuReadGyroZ();
        int16_t mx, my, mz;
        hmcRead(mx, my, mz);
        float fx = -(float)(my - hmcOffY) * hmcScaleY;   // Norte = -sensor_Y
        float fy = -(float)(mx - hmcOffX) * hmcScaleX;   // Este  = -sensor_X
        float h = atan2f(fy, fx) * 180.0f / (float)M_PI;
        if (h < 0.0f) h += 360.0f;
        wlogf("  gz=%+.3f  heading=%.1f\n", gz, h);
    }
    wlogf("Ahora gira el robot a la DERECHA a mano — gz debe ser POSITIVO:");
    oledShow("Diagnostico", "Gira DERECHA", "a mano 3s");
    for (int i = 0; i < 30; i++) {
        delay(100);
        float gz = mpuReadGyroZ();
        int16_t mx, my, mz;
        hmcRead(mx, my, mz);
        float fx = -(float)(my - hmcOffY) * hmcScaleY;
        float fy = -(float)(mx - hmcOffX) * hmcScaleX;
        float h = atan2f(fy, fx) * 180.0f / (float)M_PI;
        if (h < 0.0f) h += 360.0f;
        wlogf("  gz=%+.3f  heading=%.1f\n", gz, h);
    }
    wlogf("Ahora gira el robot a la IZQUIERDA a mano — gz debe ser NEGATIVO:");
    oledShow("Diagnostico", "Gira IZQUIERDA", "a mano 3s");
    for (int i = 0; i < 30; i++) {
        delay(100);
        float gz = mpuReadGyroZ();
        int16_t mx, my, mz;
        hmcRead(mx, my, mz);
        float fx = -(float)(my - hmcOffY) * hmcScaleY;
        float fy = -(float)(mx - hmcOffX) * hmcScaleX;
        float h = atan2f(fy, fx) * 180.0f / (float)M_PI;
        if (h < 0.0f) h += 360.0f;
        wlogf("  gz=%+.3f  heading=%.1f\n", gz, h);
    }
    wlogf("=== FIN DIAGNOSTICO ===");
    oledShow("Diagnostico OK", "Ver Serial", "");
    delay(2000);
}

// Avanza en línea recta 'distanceM' metros usando fusión complementaria
// giroscopio MPU-6500 + magnetómetro HMC5883L.
//
//  Fusión complementaria (cada iteración, sin parar):
//    fusedAngle = α × (fusedAngle + gyroZ × dt) + (1-α) × hmcDelta
//
//  PID sobre fusedAngle: error = desviación acumulada respecto al arranque.
//    err > 0 → girado a la derecha → motor A más lento, motor B más rápido
//    err < 0 → girado a la izquierda → motor A más rápido, motor B más lento
//
//  GYRO_SIGN: +1 si girar derecha da gz positivo, -1 si es al revés.
//  Verificar con sensorDiagnose() antes de ajustar.
void driveStraight(float distanceM, uint8_t baseSpd = 200) {
    // ── Signo del giroscopio ──────────────────────────────────
    // +1: girar derecha da gz > 0  (normal, MPU mirando arriba)
    // -1: girar derecha da gz < 0  (MPU montado al revés o eje invertido)
    // Verificar con sensorDiagnose() leyendo el Serial.
    const float GYRO_SIGN = +1.0f;

    // ── Parámetros fusión ─────────────────────────────────────
    // gz tiene mucho ruido de vibración de motores (picos de ±25°/s).
    // ALPHA bajo = más peso al HMC que es más estable a largo plazo.
    // GYRO_CLAMP: descarta picos de gz mayores que este umbral (vibración pura).
    const float ALPHA      = 0.20f;   // 20% gyro, 80% HMC
    const float GYRO_CLAMP = 8.0f;    // °/s — picos mayores = vibración, ignorar

    // ── Parámetros PID ────────────────────────────────────────
    const float Kp             = 2.5f;
    const float Ki             = 0.05f;
    const float Kd             = 0.0f;
    const float MAX_CORRECTION = 40.0f;
    const float DEAD_BAND      =  2.0f;  // °

    // Tiempo de recorrido: ajusta SPEED_MS_PER_M midiendo 1 m real a baseSpd=200
    const float SPEED_MS_PER_M = 2500.0f;
    uint32_t durationMs = (uint32_t)(distanceM * SPEED_MS_PER_M);

    // ── Referencia inicial — motores parados para HMC limpio ──
    motorsStop();
    delay(600);
    float hmcRef = hmcHeadingInstant(20);
    wlogf("[STR] HMC ref=%.1f°  dur=%lu ms  alpha=%.2f\n",
                  hmcRef, (unsigned long)durationMs, ALPHA);
    char buf[22];
    snprintf(buf, sizeof(buf), "Tgt:%.0f  %.1fm", hmcRef, distanceM);
    oledShow("Recta fusion", buf, "Arrancando...");
    delay(300);

    // ── Estado inicial del ángulo fusionado ───────────────────
    // Arranca en 0 (sin desviación). fusedAngle mide la desviación
    // acumulada respecto al heading de arranque (positivo = girado derecha).
    float fusedAngle = 0.0f;
    float integral   = 0.0f;
    float prevErr    = 0.0f;

    unsigned long tPrev = millis();
    unsigned long tEnd  = millis() + durationMs;

    // ── Test de signos en OLED — 4s antes de arrancar ────────
    // Gira el robot a mano a la DERECHA y mira la OLED:
    //   gz > 0 y hmcD sube → GYRO_SIGN correcto (+1)
    //   gz < 0 y hmcD sube → cambia GYRO_SIGN a -1
    oledShow("Test signos", "Gira DERECHA", "a mano 4s");
    {
        unsigned long tTest = millis();
        while (millis() - tTest < 4000) {
            telnetHandle();
            float gz = mpuReadGyroZ() * GYRO_SIGN;
            int16_t mx, my, mz;
            float hmcNow = hmcRef;
            if (hmcRead(mx, my, mz)) {
                float fx = -(float)(my - hmcOffY) * hmcScaleY;   // Norte = -sensor_Y
                float fy = -(float)(mx - hmcOffX) * hmcScaleX;   // Este  = -sensor_X
                hmcNow = atan2f(fy, fx) * 180.0f / (float)M_PI;
                if (hmcNow < 0.0f) hmcNow += 360.0f;
            }
            float hmcD = hmcNow - hmcRef;
            if (hmcD >  180.0f) hmcD -= 360.0f;
            if (hmcD < -180.0f) hmcD += 360.0f;
            char l2[22], l3[22];
            snprintf(l2, sizeof(l2), "gz:%+.2f", gz);
            snprintf(l3, sizeof(l3), "hmcD:%+.1f", hmcD);
            oledShow("gz+ si giras D", l2, l3);
            wlogf("[SIGN] gz=%+.3f  hmcD=%+.1f\n", gz, hmcD);
            delay(100);
        }
    }
    oledShow("Arrancando...", "", "");
    delay(500);

    // Reset del ángulo fusionado justo antes de arrancar —
    // el test de signos acumula gz, hay que partir de cero aquí.
    fusedAngle = 0.0f;
    integral   = 0.0f;
    prevErr    = 0.0f;
    tPrev      = millis();
    tEnd       = millis() + durationMs;

    driveMotors(baseSpd, baseSpd);

    while (millis() < tEnd) {
        // ── Telnet — aceptar conexiones incluso dentro del bucle ──
        telnetHandle();

        unsigned long now = millis();
        float dt = (now - tPrev) / 1000.0f;
        if (dt < 0.001f) { delay(1); continue; }
        tPrev = now;

        // ── Giroscopio Z: predicción ──────────────────────────
        float gz = mpuReadGyroZ() * GYRO_SIGN;
        // Clamp: picos mayores que GYRO_CLAMP son vibración de motores, no rotación real
        float gzClamped = constrain(gz, -GYRO_CLAMP, GYRO_CLAMP);
        float gyroPredict = fusedAngle + gzClamped * dt;

        // ── HMC: delta respecto a referencia inicial ──────────
        int16_t mx, my, mz;
        float hmcDelta = fusedAngle;  // fallback si falla lectura
        if (hmcRead(mx, my, mz)) {
            float fx = -(float)(my - hmcOffY) * hmcScaleY;   // Norte = -sensor_Y
            float fy = -(float)(mx - hmcOffX) * hmcScaleX;   // Este  = -sensor_X
            float hmcNow = atan2f(fy, fx) * 180.0f / (float)M_PI;
            if (hmcNow < 0.0f) hmcNow += 360.0f;
            hmcDelta = hmcNow - hmcRef;
            if (hmcDelta >  180.0f) hmcDelta -= 360.0f;
            if (hmcDelta < -180.0f) hmcDelta += 360.0f;
        }

        // ── Fusión complementaria ─────────────────────────────
        fusedAngle = ALPHA * gyroPredict + (1.0f - ALPHA) * hmcDelta;

        // ── PID ───────────────────────────────────────────────
        float err = fusedAngle;

        float correction = 0.0f;
        if (fabsf(err) > DEAD_BAND) {
            integral += err * dt;
            integral  = constrain(integral, -50.0f, 50.0f);
            float derivative = (err - prevErr) / dt;
            correction = Kp * err + Ki * integral + Kd * derivative;
            correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
        } else {
            integral = 0.0f;
        }
        prevErr = err;

        // err > 0 → girado derecha → acelerar A, frenar B (corrige girando izquierda)
        // err < 0 → girado izquierda → frenar A, acelerar B (corrige girando derecha)
        int16_t spdA = (int16_t)((float)baseSpd - correction);
        int16_t spdB = (int16_t)((float)baseSpd + correction);
        driveMotors(spdA, spdB);

        // ── Log cada 300 ms — OLED + Telnet ──────────────────
        static unsigned long lastLog = 0;
        if (millis() - lastLog >= 300) {
            lastLog = millis();
            uint32_t rem = (tEnd > millis()) ? (uint32_t)((tEnd - millis()) / 1000) : 0;
            wlogf("[PID] gz=%+.2f fused=%+.1f hmcD=%+.1f err=%+.1f corr=%+.1f A=%d B=%d %lus\n",
                  gz, fusedAngle, hmcDelta, err, correction,
                  (int)spdA, (int)spdB, (unsigned long)rem);
            char oledL2[22], oledL3[22];
            snprintf(oledL2, sizeof(oledL2), "f:%+.1f e:%+.1f", fusedAngle, err);
            snprintf(oledL3, sizeof(oledL3), "c:%+.0f  %lus", correction, (unsigned long)rem);
            oledShow("Recta PID", oledL2, oledL3);
        }
    }

    motorsStop();
    delay(500);
    float finalHmc = hmcHeadingInstant(10);
    float dev = finalHmc - hmcRef;
    if (dev >  180.0f) dev -= 360.0f;
    if (dev < -180.0f) dev += 360.0f;
    wlogf("[STR] OK — fused=%.1f  dev_hmc=%+.1f°\n", fusedAngle, dev);
    snprintf(buf, sizeof(buf), "fused:%.1f dev:%+.0f", fusedAngle, dev);
    oledShow("Recta OK", buf, "");
    delay(2000);
}

// =============================================================
//  GPS — DETECCIÓN AUTO BAUDRATE + CÁLCULOS DE NAVEGACIÓN
// =============================================================
void gpsAutoDetect() {
    const uint32_t bauds[] = {9600, 4800, 38400, 57600, 115200};
    for (uint32_t baud : bauds) {
        gpsSerial.begin(baud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        delay(300);
        unsigned long t0 = millis();
        while (millis() - t0 < 600) {
            if (gpsSerial.available()) {
                char c = (char)gpsSerial.read();
                gps.encode(c);
                if (c == '$') {
                    wlogf("[GPS] Detectado a %lu bps\n", (unsigned long)baud);
                    return;
                }
            }
        }
        gpsSerial.end();
    }
    // Fallback
    gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    wlogf("[GPS] No detectado — usando 9600 bps");
}

static float gpsDistanceTo(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;
    float dLat = (lat2 - lat1) * (float)M_PI / 180.0f;
    float dLon = (lon2 - lon1) * (float)M_PI / 180.0f;
    float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f)
            + cosf(lat1 * (float)M_PI / 180.0f) * cosf(lat2 * (float)M_PI / 180.0f)
            * sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
    return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

static float gpsCourseTo(float lat1, float lon1, float lat2, float lon2) {
    float dLon = (lon2 - lon1) * (float)M_PI / 180.0f;
    float y = sinf(dLon) * cosf(lat2 * (float)M_PI / 180.0f);
    float x = cosf(lat1 * (float)M_PI / 180.0f) * sinf(lat2 * (float)M_PI / 180.0f)
             - sinf(lat1 * (float)M_PI / 180.0f) * cosf(lat2 * (float)M_PI / 180.0f) * cosf(dLon);
    float c = atan2f(y, x) * 180.0f / (float)M_PI;
    if (c < 0.0f) c += 360.0f;
    return c;
}

// =============================================================
//  LORA
// =============================================================
bool loraInit() {
    SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_CS);
    // SF7, BW=125, CR=5, sync=PRIVATE(0x12) → escribe [0x14,0x24] en registros → 0x14 OTA
    // La estación SX1276 debe usar setSyncWord(0x14) para coincidir OTA.
    int st = radio.begin(868.0f, 125.0f, 7, 5,
                         RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14, 8);
    if (st != RADIOLIB_ERR_NONE) {
        wlogf("[LoRa] ERROR init: %d\n", st);
        return false;
    }
    radio.setCRC(false);   // SX1262 y SX1276 tienen CRC incompatible; usamos checksum propio
    radio.startReceive();  // dejar siempre en RX al arrancar
    wlogf("[LoRa] Init OK 868MHz SF7 BW125 CRC=off\r\n");
    return true;
}

// Envía datos de forma bloqueante; deja el radio en STANDBY al terminar
static bool loraSend(uint8_t *data, size_t len) {
    int st = radio.transmit(data, len);
    if (st != RADIOLIB_ERR_NONE) {
        wlogf("[LoRa] TX err=%d\r\n", st);
        return false;
    }
    return true;
}

// Escucha hasta recibir un paquete o agotar timeoutMs.
// Detecta paquete via digitalRead(DIO1) O available() (el SX1262 requiere esto).
static int loraReceive(uint8_t *buf, size_t maxLen, uint32_t timeoutMs) {
    radio.startReceive();
    unsigned long t0 = millis();
    while (millis() - t0 < timeoutMs) {
        telnetHandle();
        while (gpsSerial.available()) gps.encode(gpsSerial.read());
        if (digitalRead(PIN_LORA_DIO1) == HIGH || radio.available()) {
            size_t pktLen = radio.getPacketLength();
            if (pktLen == 0) pktLen = maxLen;
            if (pktLen > maxLen) pktLen = maxLen;
            int st = radio.readData(buf, pktLen);
            if (st == RADIOLIB_ERR_NONE) {
                wlogf("[LoRa] RX %u bytes msg=0x%02X RSSI=%.0f\r\n",
                      (unsigned)pktLen, buf[0], radio.getRSSI());
                return (int)pktLen;
            }
            wlogf("[LoRa] readData err=%d\r\n", st);
            radio.startReceive();   // recuperar RX
        }
        delay(5);
    }
    return -1;
}

// =============================================================
//  WAYPOINTS
// =============================================================
static uint8_t xorChecksum(const uint8_t *data, size_t len) {
    uint8_t chk = 0;
    for (size_t i = 0; i < len; i++) chk ^= data[i];
    return chk;
}

bool requestWaypoints() {
    uint8_t buf[64];

    for (int attempt = 1; attempt <= 5; attempt++) {
        wlogf("[WP] === Intento %d/5 ===\r\n", attempt);

        // 1) Enviar REQUEST (1 byte)
        uint8_t req[1] = {MSG_REQUEST_WP};
        bool txOk = loraSend(req, 1);
        wlogf("[WP] TX REQUEST %s\r\n", txOk ? "OK" : "FAIL");
        if (!txOk) { delay(1000); continue; }

        // 2) Esperar START_TX (la estación tarda ~100ms en procesar y responder)
        //    Damos 10 s de margen.
        int len = loraReceive(buf, sizeof(buf), 10000);
        wlogf("[WP] START_TX: ret=%d buf[0]=0x%02X\r\n", len, len > 0 ? buf[0] : 0xFF);
        if (len < 2 || buf[0] != MSG_START_TX) {
            delay(2000);
            continue;
        }

        uint8_t total = buf[1];
        wlogf("[WP] Total waypoints: %u\r\n", total);
        waypointCount = 0;
        bool done = false;

        // 3) Recibir waypoints uno a uno
        while (!done && waypointCount < total) {
            // Esperar WP_DATA o END_TX (timeout generoso 12 s)
            len = loraReceive(buf, sizeof(buf), 12000);
            wlogf("[WP] pkt: ret=%d buf[0]=0x%02X\r\n", len, len > 0 ? buf[0] : 0xFF);

            if (len <= 0) {
                wlogf("[WP] Timeout esperando WP — abortando\r\n");
                break;
            }

            if (buf[0] == MSG_WP_DATA && len >= (int)sizeof(WaypointPacket)) {
                WaypointPacket *pkt = (WaypointPacket *)buf;
                uint8_t cs = xorChecksum(buf, sizeof(WaypointPacket) - 1);
                if (cs == pkt->checksum) {
                    uint8_t idx = pkt->waypointId - 1;
                    if (idx < MAX_WAYPOINTS) {
                        waypoints[idx].lat = pkt->latitude;
                        waypoints[idx].lon = pkt->longitude;
                        waypointCount++;
                        wlogf("[WP] WP%u lat=%.6f lon=%.6f\r\n",
                              pkt->waypointId, pkt->latitude, pkt->longitude);
                    }
                } else {
                    wlogf("[WP] Checksum BAD (got=0x%02X exp=0x%02X)\r\n", cs, pkt->checksum);
                }

                // La estación espera POST_SEND_DELAY(800ms) antes de abrir RX para ACK.
                // Esperamos 900ms para asegurarnos de que ya escucha antes de transmitir.
                delay(900);
                uint8_t ack[2] = {MSG_ACK, pkt->waypointId};
                loraSend(ack, 2);
                wlogf("[WP] ACK enviado para WP%u\r\n", pkt->waypointId);
                radio.startReceive();

            } else if (buf[0] == MSG_END_TX) {
                wlogf("[WP] END_TX recibido\r\n");
                done = true;
            } else {
                wlogf("[WP] pkt desconocido 0x%02X — ignorando\r\n", buf[0]);
            }
        }

        if (done || waypointCount == total) {
            wlogf("[WP] Recibidos %u/%u waypoints OK\r\n", waypointCount, total);
            return waypointCount > 0;
        }
    }
    wlogf("[WP] No se recibieron waypoints tras 5 intentos\r\n");
    return false;
}

// =============================================================
//  TELEMETRÍA
// =============================================================
void sendTelemetry() {
    TelemetryPacket pkt = {};
    pkt.msgType          = MSG_TELEMETRY;
    pkt.latitude         = gps.location.isValid() ? (float)gps.location.lat()  : 0.0f;
    pkt.longitude        = gps.location.isValid() ? (float)gps.location.lng()  : 0.0f;
    pkt.satellites       = gps.satellites.isValid() ? gps.satellites.value()   : 0;
    pkt.gpsFixed         = gps.location.isValid() ? 1 : 0;
    pkt.currentWaypoint  = currentWaypoint;

    if (navActive && gps.location.isValid() && waypointCount > 0) {
        float lat = (float)gps.location.lat();
        float lon = (float)gps.location.lng();
        pkt.distanceToWaypoint = gpsDistanceTo(lat, lon,
            waypoints[currentWaypoint].lat, waypoints[currentWaypoint].lon);
        pkt.headingToWaypoint  = gpsCourseTo(lat, lon,
            waypoints[currentWaypoint].lat, waypoints[currentWaypoint].lon);
    }

    pkt.rssi         = (int16_t)radio.getRSSI();
    pkt.snr          = radio.getSNR();
    // Heltec V3: GPIO37 activa el ADC de batería, GPIO1 lee VBAT (divisor 1:2)
    pinMode(37, OUTPUT);
    digitalWrite(37, LOW);          // LOW = habilitar lectura
    delay(5);
    int raw = analogRead(1);        // 12-bit: 0-4095
    digitalWrite(37, HIGH);         // HIGH = deshabilitar (ahorra energía)
    // El divisor 1:2 con Vref 3.3V: Vbat = raw * 3.3 * 2 / 4095
    float vbat = raw * (3.3f * 2.0f / 4095.0f);
    // LiPo: 4.2V = 100%, 3.0V = 0%
    int pct = (int)((vbat - 3.0f) / (4.2f - 3.0f) * 100.0f);
    if (pct > 100) pct = 100;
    if (pct < 0)   pct = 0;
    pkt.batteryLevel = (uint8_t)pct;

    // Checksum = suma de todos los bytes excepto el último
    uint8_t *ptr = (uint8_t *)&pkt;
    uint8_t  chk = 0;
    for (size_t i = 0; i < sizeof(TelemetryPacket) - 1; i++) chk += ptr[i];
    pkt.checksum = chk;

    loraSend(ptr, sizeof(TelemetryPacket));
}

// =============================================================
//  NAVEGACIÓN GPS
// =============================================================
#define WAYPOINT_REACH_M  2.0f   // metros para considerar waypoint alcanzado
#define BASE_SPEED          200   // velocidad normal (punto de partida)
#define TURN_SPEED          160   // velocidad al girar
#define HEADING_TOL_DEG    15.0f  // ±15° de tolerancia de rumbo
// Velocidad adaptativa
#define NAV_MIN_SPEED       130   // PWM mínimo (nunca bajar de aquí)
#define NAV_MAX_SPEED       255   // PWM máximo
#define NAV_TARGET_MPS     0.30f  // velocidad objetivo en m/s (calibrar en llano)
#define NAV_KP             30.0f  // ganancia proporcional (PWM por m/s de error)

static uint8_t  gAdaptiveSpeed = BASE_SPEED;  // PWM actual — adaptado al terreno

void navigate() {
    if (!navActive) return;

    // Sin waypoints cargados
    if (waypointCount == 0 || currentWaypoint >= waypointCount) {
        motorsStop();
        navActive = false;
        wlogf("[NAV] Sin waypoints — usa 'navegar' por telnet\r\n");
        return;
    }

    // Sin GPS — parar motores y esperar
    if (!gps.location.isValid()) {
        motorsStop();
        static unsigned long lastGpsWarn = 0;
        if (millis() - lastGpsWarn > 5000) {
            lastGpsWarn = millis();
            wlogf("[NAV] Sin GPS fix — motores parados\r\n");
        }
        return;
    }

    float lat  = (float)gps.location.lat();
    float lon  = (float)gps.location.lng();
    float tLat = waypoints[currentWaypoint].lat;
    float tLon = waypoints[currentWaypoint].lon;

    float dist = gpsDistanceTo(lat, lon, tLat, tLon);

    // ── Waypoint alcanzado ──
    if (dist < WAYPOINT_REACH_M) {
        motorsStop();
        wlogf("[NAV] WP%u/%u alcanzado (dist=%.1fm)\r\n",
              currentWaypoint + 1, waypointCount, dist);
        oledShow("WP ALCANZADO!", "", "");
        currentWaypoint++;
        if (currentWaypoint >= waypointCount) {
            navActive = false;
            wlogf("[NAV] === RUTA COMPLETADA (%u WPs) ===\r\n", waypointCount);
            oledShow("RUTA COMPLETADA", "Todos los WPs!", "Buen trabajo :)");
        }
        return;
    }

    float targetCourse  = gpsCourseTo(lat, lon, tLat, tLon);
    float compassCourse = hmcHeading();

    float err = targetCourse - compassCourse;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;

    // ── Velocidad adaptativa (solo mientras avanza en línea recta) ──
    {
        static double   prevLat = 0, prevLon = 0;
        static uint32_t prevAge = UINT32_MAX;
        static unsigned long prevMs = 0;

        uint32_t age = (uint32_t)gps.location.age();
        // Detectar nueva muestra GPS: age retrocede (dato fresco)
        if (age < prevAge && prevAge != UINT32_MAX && prevMs != 0) {
            unsigned long dt_ms = millis() - prevMs;
            if (dt_ms > 200 && dt_ms < 5000) {
                float moved = gpsDistanceTo((float)prevLat, (float)prevLon, lat, lon);
                float actualMps = moved / (dt_ms / 1000.0f);

                // Solo ajustar si vamos en dirección correcta (rumbo OK)
                if (fabsf(err) <= HEADING_TOL_DEG && actualMps < 3.0f) {
                    float speedErr = NAV_TARGET_MPS - actualMps;  // >0 → más lento de lo esperado
                    int16_t adj = (int16_t)(speedErr * NAV_KP);
                    int16_t newSpd = (int16_t)gAdaptiveSpeed + adj;
                    if (newSpd < NAV_MIN_SPEED) newSpd = NAV_MIN_SPEED;
                    if (newSpd > NAV_MAX_SPEED) newSpd = NAV_MAX_SPEED;
                    gAdaptiveSpeed = (uint8_t)newSpd;
                    wlogf("[NAV] Velocidad adaptada: %.2fm/s → PWM=%u (adj=%+d)\r\n",
                          actualMps, gAdaptiveSpeed, adj);
                }
            }
            prevMs = millis();
        } else if (age < prevAge) {
            prevMs = millis();
        }
        prevAge = age;
        prevLat = lat;
        prevLon = lon;
    }

    // Log periódico cada 2 s para no saturar telnet
    static unsigned long lastNavLog = 0;
    if (millis() - lastNavLog > 2000) {
        lastNavLog = millis();
        wlogf("[NAV] WP%u/%u  dist=%.1fm  tgt=%.0f  hdg=%.0f  err=%+.0f  PWM=%u\r\n",
              currentWaypoint + 1, waypointCount,
              dist, targetCourse, compassCourse, err, gAdaptiveSpeed);
    }

    // ── Giro: parar → medir → pulso → repetir ────────────────────────────
    // Los motores interfieren electromagnéticamente con el HMC5883L.
    // La única lectura fiable es con motores PARADOS y campo disipado.
    // Patrón: stop 300ms → leer heading → si necesita girar → pulso → repetir.
    // Para GPS (1 Hz) esta cadencia es más que suficiente.
    static uint8_t  turnSpeedNav  = TURN_SPEED;
    static uint32_t turnCooldownEnd = 0;
    const  float    TURN_ENTER_DEG = 20.0f;
    const  float    TURN_EXIT_DEG  =  8.0f;
    const  uint32_t TURN_COOLDOWN  = 500;      // ms sin girar tras alinearse
    const  uint8_t  TURN_ADAPT_STEP = 20;

    if (fabsf(err) > TURN_ENTER_DEG && millis() >= turnCooldownEnd) {
        // ── Necesita girar: pulso bloqueante con medición limpia ──
        // 1. Parar y dejar disipar el campo magnético de los motores
        motorsStop();
        delay(300);

        // 2. Leer heading con motores parados (lectura fiable)
        float cleanH = hmcHeadingInstant(8);
        float cleanErr = targetCourse - cleanH;
        if (cleanErr >  180.0f) cleanErr -= 360.0f;
        if (cleanErr < -180.0f) cleanErr += 360.0f;

        wlogf("[NAV] GIRO: hdg=%.0f tgt=%.0f err=%+.0f spd=%u\r\n",
              cleanH, targetCourse, cleanErr, turnSpeedNav);

        if (fabsf(cleanErr) > TURN_EXIT_DEG) {
            // 3. Aplicar pulso proporcional al error
            uint16_t pulseMs = (uint16_t)constrain(fabsf(cleanErr) * 5.0f, 80.0f, 350.0f);
            if ((cleanErr * gTurnSign) > 0.0f) turnRight(turnSpeedNav);
            else                                turnLeft(turnSpeedNav);
            delay(pulseMs);
            motorsStop();

            // 4. Verificar que giró (campo disipado: esperar 300ms)
            delay(300);
            float h2 = hmcHeadingInstant(6);
            float moved = fabsf(h2 - cleanH);
            if (moved > 180.0f) moved = 360.0f - moved;
            if (moved < 2.0f && turnSpeedNav < NAV_MAX_SPEED) {
                turnSpeedNav = (uint8_t)min((int)turnSpeedNav + TURN_ADAPT_STEP,
                                            (int)NAV_MAX_SPEED);
                wlogf("[NAV] Giro estancado — TurnSpd sube a %u\r\n", turnSpeedNav);
            }
        } else {
            // Ya alineado tras la medición limpia
            turnSpeedNav   = TURN_SPEED;
            turnCooldownEnd = millis() + TURN_COOLDOWN;
            wlogf("[NAV] Alineado: hdg=%.0f\r\n", cleanH);
        }
    } else {
        // Rumbo correcto — avanzar
        if (fabsf(err) < TURN_EXIT_DEG) {
            turnSpeedNav   = TURN_SPEED;   // reset para el próximo giro
            turnCooldownEnd = millis() + TURN_COOLDOWN;
        }
        uint8_t spd = (dist < 3.0f) ? NAV_MIN_SPEED : gAdaptiveSpeed;
        moveForward(spd);
    }
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // ── OLED ──
    oledInit();
    oledShow("VEHICULO AUTONOMO", "Iniciando...");
    wlogf("\n=== AgroMqueen — Arranque ===");

    // ── WiFi AP + Telnet ──
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    telnetServer.begin();
    telnetServer.setNoDelay(true);
    telnetCmdCallback = processTelnetCmd;
    {
        IPAddress ip = WiFi.softAPIP();
        char ipbuf[22];
        snprintf(ipbuf, sizeof(ipbuf), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
        oledShow("WiFi AP listo", WIFI_SSID, ipbuf);
        wlogf("[WiFi] AP '%s' en %s puerto %d\n", WIFI_SSID, ipbuf, TELNET_PORT);
    }
    delay(1500);

    // ── I2C hardware ──
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);
    delay(100);

    // ── GPS — auto-detect baudrate ──
    oledShow("GPS", "Detectando baud...");
    gpsAutoDetect();

    // ── HMC5883L ──
    if (hmcInit()) {
        bool calLoaded = hmcLoadCalibration();
        gTurnSign = -1;  // turnRight() baja el heading en este robot.
        wlogf("[HMC] TurnSign hardcoded = %+d\r\n", gTurnSign);
        if (calLoaded) {
            wlogf("[HMC] Calibracion cargada — buscando Norte...\r\n");
            oledShow("HMC OK", "Buscando Norte", "");
            motorsInit();
            faceNorth();
            motorsStop();
            wlogf("[HMC] Orientado al Norte\r\n");
        } else {
            wlogf("[HMC] Sin calibracion — usa 'calibrar' por telnet\r\n");
            oledShow("HMC OK", "Sin calibracion", "Escribe: calibrar");
        }
    }

    mpuInit(); mpuCalibrateGyro(500);

    // ── LoRa ──
    oledShow("LoRa init", "868 MHz SF7...", "");
    if (!loraInit()) {
        oledShow("ERROR LoRa", "Fallo init!", "");
        delay(3000);
    }
    delay(1000);
    wlogf("[SYS] Setup completado — entrando en loop");
}

// =============================================================
//  LOOP
// =============================================================
// Prueba de retorno al origen:
//  1. Guarda posición GPS actual como "origen"
//  2. Avanza con driveStraight() durante advanceMs milisegundos
//  3. Gira 180° usando el magnetómetro
//  4. Avanza de vuelta hacia el origen usando GPS (navigate hacia un waypoint temporal)
void returnToOrigin(uint32_t advanceMs = 8000, uint8_t spd = 200) {
    // ── 1. Guardar origen ──────────────────────────────────────
    if (!gps.location.isValid()) {
        wlogf("[RTO] Sin fix GPS — espera a tener satelites\r\n");
        oledShow("Sin GPS fix", "No puedo hacer", "retorno");
        delay(2000);
        return;
    }
    float originLat = (float)gps.location.lat();
    float originLon = (float)gps.location.lng();
    wlogf("[RTO] Origen: %.6f, %.6f\r\n", originLat, originLon);
    oledShow("Retorno", "Origen guardado", "Avanzando...");
    delay(500);

    // ── 2. Avanzar con PID de heading ─────────────────────────
    // Capturamos heading inicial para el retorno (heading + 180°)
    motorsStop();
    delay(500);
    float headingOut = hmcHeadingInstant(20);  // heading al salir
    wlogf("[RTO] Heading salida: %.1f deg\r\n", headingOut);

    // Avanzar el tiempo indicado con driveStraight
    driveStraight(advanceMs / 2500.0f, spd);  // convierte ms a metros aprox.

    // ── 3. Girar 180° ─────────────────────────────────────────
    float targetReturn = headingOut + 180.0f;
    if (targetReturn >= 360.0f) targetReturn -= 360.0f;
    wlogf("[RTO] Girando a %.1f deg para volver...\r\n", targetReturn);
    oledShow("Retorno", "Girando 180...", "");

    // Girar hasta apuntar al heading de retorno
    const uint32_t TURN_TIMEOUT = 20000;
    unsigned long tTurn = millis();
    while (millis() - tTurn < TURN_TIMEOUT) {
        float h = hmcHeadingInstant(8);
        float err = targetReturn - h;
        if (err >  180.0f) err -= 360.0f;
        if (err < -180.0f) err += 360.0f;
        char buf[22];
        snprintf(buf, sizeof(buf), "H:%.0f tgt:%.0f", h, targetReturn);
        oledShow("Girando", buf, "");
        wlogf("[RTO] Giro H=%.1f tgt=%.1f err=%.1f\r\n", h, targetReturn, err);
        if (fabsf(err) <= 5.0f) break;
        if ((err * gTurnSign) > 0.0f) turnRight(160); else turnLeft(160);
        delay(150);
        motorsStop();
        delay(1000);  // esperar disipación campo motor
    }
    motorsStop();
    delay(500);

    // ── 4. Volver al origen guiado por GPS ────────────────────
    wlogf("[RTO] Volviendo al origen GPS...\r\n");
    oledShow("Retorno", "Volviendo...", "GPS guiando");

    const float REACH_M      = 2.5f;   // metros para dar por llegado
    const uint32_t NAV_TIMEOUT = 60000;
    unsigned long tNav = millis();

    while (millis() - tNav < NAV_TIMEOUT) {
        // Alimentar GPS
        while (gpsSerial.available()) gps.encode(gpsSerial.read());
        telnetHandle();

        if (!gps.location.isValid()) { delay(100); continue; }

        float lat = (float)gps.location.lat();
        float lon = (float)gps.location.lng();
        float dist = gpsDistanceTo(lat, lon, originLat, originLon);
        float course = gpsCourseTo(lat, lon, originLat, originLon);
        float heading = hmcHeading();
        float err = course - heading;
        if (err >  180.0f) err -= 360.0f;
        if (err < -180.0f) err += 360.0f;

        char l2[22], l3[22];
        snprintf(l2, sizeof(l2), "Dist: %.1fm", dist);
        snprintf(l3, sizeof(l3), "err: %+.0f", err);
        oledShow("Volviendo", l2, l3);
        wlogf("[RTO] dist=%.1fm course=%.0f head=%.0f err=%+.0f\r\n",
              dist, course, heading, err);

        if (dist <= REACH_M) {
            motorsStop();
            wlogf("[RTO] Origen alcanzado! dist=%.1fm\r\n", dist);
            oledShow("ORIGEN OK", "Llegado!", "");
            delay(3000);
            return;
        }

        // Giro adaptativo en retorno: igual lógica que navigate()
        if (fabsf(err) > 15.0f) {
            static uint8_t  rtoTurnSpd      = TURN_SPEED;
            static float    rtoHdgPrev      = -1.0f;
            static uint32_t rtoLastCheckMs  = 0;
            uint32_t nowMs = millis();
            if (rtoHdgPrev < 0.0f) {
                rtoHdgPrev = heading; rtoLastCheckMs = nowMs;
            } else if (nowMs - rtoLastCheckMs >= 400) {
                float d = fabsf(heading - rtoHdgPrev);
                if (d > 180.0f) d = 360.0f - d;
                if (d < 2.0f && rtoTurnSpd < NAV_MAX_SPEED) {
                    rtoTurnSpd = (uint8_t)min((int)rtoTurnSpd + 20, (int)NAV_MAX_SPEED);
                    wlogf("[RTO] Giro estancado — TurnSpd sube a %u\r\n", rtoTurnSpd);
                }
                rtoHdgPrev = heading; rtoLastCheckMs = nowMs;
            }
            if ((err * gTurnSign) > 0.0f) turnRight(rtoTurnSpd); else turnLeft(rtoTurnSpd);
        } else {
            moveForward(BASE_SPEED);
        }
        delay(100);
    }

    motorsStop();
    wlogf("[RTO] Timeout — no se alcanzo el origen\r\n");
    oledShow("Timeout retorno", "Para manual", "");
    delay(2000);
}

// Procesa un comando recibido por telnet.
// Se llama desde telnetHandle() cuando llega una línea completa.
static bool processTelnetCmd(const char *cmd) {
    // Filtrar bytes de negociación Telnet (IAC=0xFF + 2 bytes) y conservar solo ASCII imprimible
    static char buf[64];
    int out = 0;
    for (int i = 0; cmd[i] && out < 63; i++) {
        uint8_t b = (uint8_t)cmd[i];
        if (b == 0xFF) { i += 2; continue; }       // saltar secuencia IAC (3 bytes)
        if (b >= 0x20 && b < 0x7F) buf[out++] = (char)b;  // solo ASCII imprimible
    }
    buf[out] = '\0';
    while (out > 0 && buf[out-1] == ' ') buf[--out] = '\0';  // trim espacios finales
    cmd = buf;

    if (strlen(cmd) == 0) return false;

    wlogf("[CMD] '%s'\r\n", cmd);

    if (strcmp(cmd, "ayuda") == 0 || strcmp(cmd, "help") == 0) {
        wlogf("Comandos disponibles:\r\n");
        wlogf("  navegar   — pide waypoints al base y arranca navegacion\r\n");
        wlogf("  wp        — lista waypoints almacenados\r\n");
        wlogf("  saltar    — salta al siguiente waypoint\r\n");
        wlogf("  calibrar  — recalibrar magnetometro HMC5883L\r\n");
        wlogf("  retorno   — prueba: avanza 8s y vuelve al origen por GPS\r\n");
        wlogf("  gps       — debug GPS: chars, sentencias, volcado 3s crudo\r\n");
        wlogf("  estado    — muestra GPS, heading y estado actual\r\n");
        wlogf("  parar     — para motores y desactiva navegacion\r\n");

    } else if (strcmp(cmd, "navegar") == 0) {
        wlogf("[CMD] Solicitando waypoints via LoRa...\r\n");
        oledShow("Pidiendo WP", "via LoRa...", "");
        motorsStop();
        navActive = false;
        currentWaypoint = 0;
        waypointCount   = 0;
        if (requestWaypoints()) {
            navActive = true;
            gAdaptiveSpeed = BASE_SPEED;  // reset velocidad adaptativa
            char buf[22];
            snprintf(buf, sizeof(buf), "%u waypoints OK", waypointCount);
            wlogf("[CMD] %s — navegacion iniciada\r\n", buf);
            oledShow("Waypoints OK", buf, "Navegando!");
        } else {
            wlogf("[CMD] Sin respuesta del base station\r\n");
            oledShow("Sin waypoints", "Reintenta", "mas tarde");
        }

    } else if (strcmp(cmd, "wp") == 0) {
        wlogf("[WP] %u waypoints cargados (WP actual: %u, navActivo: %s)\r\n",
              waypointCount, currentWaypoint + 1, navActive ? "si" : "no");
        for (uint8_t i = 0; i < waypointCount; i++) {
            wlogf("  WP%u: %.6f, %.6f%s\r\n",
                  i + 1, (double)waypoints[i].lat, (double)waypoints[i].lon,
                  (i == currentWaypoint && navActive) ? "  <-- actual" : "");
        }
        if (waypointCount == 0) wlogf("  (ninguno — usa 'navegar')\r\n");

    } else if (strcmp(cmd, "saltar") == 0) {
        if (!navActive || waypointCount == 0) {
            wlogf("[CMD] Navegacion no activa\r\n");
        } else if (currentWaypoint + 1 >= waypointCount) {
            wlogf("[CMD] Ya estamos en el ultimo waypoint\r\n");
        } else {
            motorsStop();
            currentWaypoint++;
            wlogf("[CMD] Saltado a WP%u/%u\r\n", currentWaypoint + 1, waypointCount);
        }

    } else if (strcmp(cmd, "calibrar") == 0) {
        wlogf("[CMD] Iniciando calibracion HMC...\r\n");
        motorsStop();
        hmcCalibrate();
        wlogf("[CMD] Calibracion completada — buscando Norte...\r\n");
        oledShow("Calibrado!", "Buscando Norte", "");
        motorsInit();
        faceNorth();
        motorsStop();
        wlogf("[CMD] Orientado al Norte\r\n");

    } else if (strcmp(cmd, "retorno") == 0) {
        wlogf("[CMD] Iniciando prueba de retorno al origen...\r\n");
        returnToOrigin(8000, 200);
        wlogf("[CMD] Retorno completado\r\n");
//hola
    } else if (strcmp(cmd, "gps") == 0) {
        // Debug GPS: muestra cuántos chars/sentencias ha procesado TinyGPS++
        // y vuelca 3 segundos de datos crudos del serial del GPS
        wlogf("[GPS] chars=%lu sentencias=%lu fallidas=%lu\r\n",
              (unsigned long)gps.charsProcessed(),
              (unsigned long)gps.sentencesWithFix(),
              (unsigned long)gps.failedChecksum());
        wlogf("[GPS] fix=%s  sats=%u  age=%lums\r\n",
              gps.location.isValid() ? "SI" : "NO",
              gps.satellites.isValid() ? gps.satellites.value() : 0,
              (unsigned long)gps.location.age());
        wlogf("[GPS] Volcando 3s de datos crudos GPS:\r\n");
        unsigned long tDump = millis();
        while (millis() - tDump < 3000) {
            while (gpsSerial.available()) {
                char c = (char)gpsSerial.read();
                gps.encode(c);
                telnetClient.write((uint8_t)c);
            }
        }
        wlogf("\r\n[GPS] Fin volcado\r\n");

    } else if (strcmp(cmd, "estado") == 0) {
        float h = hmcHeadingInstant(6);
        wlogf("[EST] Heading: %.1f deg\r\n", h);
        wlogf("[EST] GPS fix: %s  sats=%u  age=%lums\r\n",
              gps.location.isValid() ? "SI" : "NO",
              gps.satellites.isValid() ? gps.satellites.value() : 0,
              (unsigned long)gps.location.age());
        wlogf("[EST] chars=%lu sentencias=%lu\r\n",
              (unsigned long)gps.charsProcessed(),
              (unsigned long)gps.sentencesWithFix());
        if (gps.location.isValid()) {
            wlogf("[EST] Pos: %.6f, %.6f  sats=%u\r\n",
                  gps.location.lat(), gps.location.lng(),
                  gps.satellites.value());
        }
        wlogf("[EST] navActive: %s  WP: %u/%u\r\n",
              navActive ? "si" : "no", currentWaypoint, waypointCount);

    } else if (strcmp(cmd, "parar") == 0) {
        motorsStop();
        navActive = false;
        wlogf("[CMD] Motores parados — navActive=false\r\n");
        oledShow("PARADO", "Cmd telnet", "");

    } else {
        wlogf("[CMD] Comando desconocido: '%s'  (escribe 'ayuda')\r\n", cmd);
    }
    return false;
}
//hola
void loop() {
    telnetHandle();

    static bool wpRequested = false;
    if (!wpRequested) {
        wpRequested = true;
        wlogf("[SYS] Pidiendo waypoints...\r\n");
        oledShow("Pidiendo WP", "868MHz SF7...", "");
        if (requestWaypoints()) {
            navActive = true;
            gAdaptiveSpeed = BASE_SPEED;  // reset velocidad adaptativa
            wlogf("[SYS] Waypoints recibidos OK\r\n");
        } else {
            wlogf("[SYS] No se recibieron waypoints\r\n");
            wpRequested = false;
        }
    }

    // ── GPS — alimentar el parser ──
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    // ── Navegación ──
    if (navActive) navigate();

    // ── Telemetría cada TELEMETRY_INTERVAL_MS ──
    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = millis();
        sendTelemetry();
    }

    // ── OLED cada 500 ms ──
    static unsigned long lastOled = 0;
    if (millis() - lastOled >= 500) {
        lastOled = millis();
        float heading = hmcHeading();
        char l1[22], l2[22], l3[22];
        if (navActive) {
            snprintf(l1, sizeof(l1), "WP%u/%u H:%.0f",
                     currentWaypoint + 1, waypointCount, heading);
        } else if (waypointCount > 0) {
            snprintf(l1, sizeof(l1), "PARADO WP%u/%u",
                     currentWaypoint + 1, waypointCount);
        } else {
            snprintf(l1, sizeof(l1), "H:%.0f  sin ruta", heading);
        }
        if (gps.location.isValid()) {
            snprintf(l2, sizeof(l2), "%.5f", gps.location.lat());
            char satBuf[8];
            snprintf(satBuf, sizeof(satBuf), "%us",
                     gps.satellites.isValid() ? gps.satellites.value() : 0);
            snprintf(l3, sizeof(l3), "%.5f %s", gps.location.lng(), satBuf);
        } else {
            snprintf(l2, sizeof(l2), "Sin GPS fix");
            snprintf(l3, sizeof(l3), navActive ? "Navegando" : "Espera WP");
        }
        oledShow(l1, l2, l3);
    }

    delay(50);
}
