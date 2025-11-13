// Test de control de motor para PlatformIO (Unity)
// Placa: Heltec WiFi LoRa 32 V3 (ESP32, Arduino framework)
// Pines usados (según petición): GPIO19, GPIO20, GPIO21, GPIO26
// Asumimos un puente H donde cada motor se controla con dos pines (IN1/IN2).
// En este ejemplo se muestra cómo rotorizar adelante, atrás y detener.
// Este test es principalmente manual/educativo: las aserciones automáticas
// no pueden verificar movimiento físico. Por eso las aserciones comprueban
// que las operaciones de pin se pueden ejecutar sin errores.

#include <Arduino.h>
#include <unity.h>

// Pines del puente H
const int M1_IN1 = 19; // Motor 1, entrada A
const int M1_IN2 = 20; // Motor 1, entrada B
const int M2_IN1 = 21; // Motor 2, entrada A (si hay segundo motor o controle alternativo)
const int M2_IN2 = 26; // Motor 2, entrada B

// Duraciones (ms)
const unsigned long DURATION_MOVE = 1000; // tiempo para mover en cada dirección
const unsigned long DURATION_STOP = 500;

// Utilidades de control
void motorForward(int in1, int in2) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void motorBackward(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

void motorStop(int in1, int in2) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void setUp(void) {
  // Se ejecuta antes de cada test
}

void tearDown(void) {
  // Se ejecuta después de cada test
}

void test_motor_cycle(void) {
  // Inicializar pines
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // Parar por seguridad
  motorStop(M1_IN1, M1_IN2);
  motorStop(M2_IN1, M2_IN2);
  delay(200);

  // Mover motor 1 hacia adelante
  motorForward(M1_IN1, M1_IN2);
  delay(DURATION_MOVE);
  motorStop(M1_IN1, M1_IN2);
  delay(DURATION_STOP);

  // Mover motor 1 hacia atras
  motorBackward(M1_IN1, M1_IN2);
  delay(DURATION_MOVE);
  motorStop(M1_IN1, M1_IN2);
  delay(DURATION_STOP);

  // Mover motor 2 (alternativo) adelante
  motorForward(M2_IN1, M2_IN2);
  delay(DURATION_MOVE);
  motorStop(M2_IN1, M2_IN2);
  delay(DURATION_STOP);

  // Mover motor 2 atras
  motorBackward(M2_IN1, M2_IN2);
  delay(DURATION_MOVE);
  motorStop(M2_IN1, M2_IN2);
  delay(DURATION_STOP);

  // Assert trivial: si llegamos hasta aquí, los comandos de GPIO se ejecutaron
  TEST_ASSERT_TRUE_MESSAGE(true, "Secuencia de control completada (verificar movimiento fisico)");
}

int main(int argc, char **argv) {
  init(); // inicializa Arduino
  UNITY_BEGIN();
  RUN_TEST(test_motor_cycle);
  UNITY_END();
  return 0;
}

/* Notas de cableado y seguridad (en español):
 - Asegúrate de conectar la alimentación del motor y del puente H correctamente.
 - No alimentar motores desde la placa ESP32; usa una fuente externa con masa común.
 - Si el puente H tiene pines de enable (ENA), configúralos adecuadamente o
   asegúrate de que estén en HIGH para permitir la salida.
 - Verifica la corriente y la protección (fusible) para evitar daños.
 - Este test no puede comprobar físicamente el movimiento; observa el motor
   mientras corre para verificar la operación.
*/
