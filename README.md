# AgroMqueen — Vehiculo Autonomo

Firmware del robot agricola autonomo **AgroMqueen**, basado en **Heltec WiFi LoRa 32 V3** (ESP32-S3). El vehiculo recibe waypoints por LoRa, navega de forma autonoma combinando GPS, magnetometro y giroscopio, y envia telemetria en tiempo real a la estacion base.

> Trabajo de Fin de Grado — Jose Fornes Jimenez.
> Repositorio: <https://github.com/JoseFornesJimenez/TFGAgrooMqueen>

## Proyecto AgroMqueen

El sistema completo se compone de tres repositorios:

- **AgroMqueen (este repo)** — vehiculo autonomo (Heltec V3).
- **[Estacion Base](https://github.com/JoseFornesJimenez/TFGEstacionBase)** — pasarela LoRa-USB (Heltec V2).
- **[GCS web (mapa)](https://github.com/JoseFornesJimenez/TFGGCS)** — interfaz Flask + Leaflet para planificar y monitorizar misiones.
- **[Sensor Humedad](https://github.com/JoseFornesJimenez/TFGModuloMoisture))** — Primer sensor modular.
```
GCS web  <-- USB -->  Estacion base  <-- LoRa 868 MHz -->  Vehiculo (este repo)
```

## Hardware

| Componente | Modelo | Funcion |
|---|---|---|
| MCU | Heltec WiFi LoRa 32 V3 (ESP32-S3) | Control principal + LoRa SX1262 |
| GPS | NEO-6M | Posicionamiento |
| Magnetometro | HMC5883L | Brujula (I2C, 0x1E) |
| IMU | MPU-6500 | Giroscopio Z para fusion (I2C, 0x68) |
| Driver motores | L298N | Dos motores DC con traccion diferencial |
| Pantalla | OLED SSD1306 128x64 | Estado en tiempo real |

## Funcionalidad

- **Navegacion GPS autonoma** waypoint a waypoint, con deteccion de llegada por distancia.
- **Avance recto con PID** que fusiona magnetometro (HMC5883L) y giroscopio (MPU-6500).
- **Calibracion de brujula** persistente en flash NVS, con recalibracion manual bajo demanda.
- **Enlace LoRa** bidireccional a 868 MHz con la estacion base (ACK por waypoint, reintentos, telemetria cada 2 s).
- **Punto de acceso WiFi + Telnet** (`AgroMqueen` / `agro1234`, `192.168.4.1:23`) para depuracion inalambrica y comandos en caliente (`navegar`, `calibrar`, `retorno`, `estado`, `parar`...).
- **Medida real de bateria** del paquete LiPo por ADC.

## Compilar y subir

```bash
pio run
pio run --target upload --upload-port COM20
pio device monitor --port COM20 --baud 115200
# o por WiFi:
telnet 192.168.4.1
```

## Dependencias

- `olikraus/U8g2` — pantalla OLED.
- `jgromes/RadioLib` — LoRa SX1262.
- `mikalhart/TinyGPSPlus` — parser NMEA.

El HMC5883L y el MPU-6500 se controlan directamente por registros I2C.
