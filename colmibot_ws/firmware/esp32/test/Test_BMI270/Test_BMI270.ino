
#include <Wire.h>
#include <math.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// ---------------- CONFIGURACION ----------------

#define SDA_PIN 21
#define SCL_PIN 22

// Direccion I2C: SDO a GND = 0x68
// Direccion I2C: SDO a 3.3 V = 0x69
#define BMI270_ADDRESS BMI2_I2C_PRIM_ADDR

BMI270 imu;

// Muestreo nominal: 100 Hz
const uint32_t SAMPLE_PERIOD_US = 10000;

// Filtro complementario para roll y pitch
const float ALPHA = 0.98f;

// Calibracion del giroscopio
const int CALIBRATION_SAMPLES = 500;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// Angulos estimados en grados
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;

// Tiempo de la ultima lectura
uint32_t lastTimeUs = 0;

// ---------------- FUNCIONES ----------------

// Mantiene yaw en el intervalo [0, 360)
float wrap360(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

// Lee el acelerometro y calcula la inclinacion
void initializeTilt() {
  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  roll = atan2f(ay, az) * RAD_TO_DEG;

  pitch = atan2f(
    -ax,
    sqrtf(ay * ay + az * az)
  ) * RAD_TO_DEG;
}

// Calibracion: la IMU debe estar inmovil
bool calibrateGyroscope() {
  Serial.println(
    "Calibrando giroscopio: no mover el robot..."
  );

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    if (imu.getSensorData() != BMI2_OK) {
      Serial.println("Error leyendo IMU.");
      return false;
    }

    sumX += imu.data.gyroX;
    sumY += imu.data.gyroY;
    sumZ += imu.data.gyroZ;

    delay(5);
  }

  gyroBiasX = sumX / CALIBRATION_SAMPLES;
  gyroBiasY = sumY / CALIBRATION_SAMPLES;
  gyroBiasZ = sumZ / CALIBRATION_SAMPLES;

  Serial.println("Calibracion terminada.");
  Serial.print("Bias Z: ");
  Serial.println(gyroBiasZ, 5);

  return true;
}

// ---------------- INICIALIZACION ----------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("Iniciando BMI270...");

  while (imu.beginI2C(BMI270_ADDRESS) != BMI2_OK) {
    Serial.println(
      "No se detecta el BMI270. Revisar I2C."
    );
    delay(1000);
  }

  Serial.println("BMI270 detectado.");

  // Obtener una lectura inicial valida
  if (imu.getSensorData() != BMI2_OK) {
    Serial.println("Error en lectura inicial.");
    while (true) delay(1000);
  }

  initializeTilt();

  // El robot debe permanecer quieto durante esta etapa
  if (!calibrateGyroscope()) {
    while (true) delay(1000);
  }

  // El yaw comienza en cero; no es norte absoluto
  yaw = 0.0f;

  lastTimeUs = micros();

  Serial.println("roll_deg,pitch_deg,yaw_deg,gz_dps");
}

// ---------------- BUCLE PRINCIPAL ----------------

void loop() {
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastTimeUs) < SAMPLE_PERIOD_US) {
    return;
  }

  // Medir el tiempo real transcurrido
  float dt = (nowUs - lastTimeUs) / 1000000.0f;
  lastTimeUs = nowUs;

  if (imu.getSensorData() != BMI2_OK) {
    return;
  }

  // Aceleracion en g
  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  // Velocidad angular en grados por segundo
  float gx = imu.data.gyroX - gyroBiasX;
  float gy = imu.data.gyroY - gyroBiasY;
  float gz = imu.data.gyroZ - gyroBiasZ;

  // Angulos obtenidos con el acelerometro
  float rollAcc = atan2f(ay, az) * RAD_TO_DEG;

  float pitchAcc = atan2f(
    -ax,
    sqrtf(ay * ay + az * az)
  ) * RAD_TO_DEG;

  // Filtro complementario: roll
  roll = ALPHA * (roll + gx * dt)
       + (1.0f - ALPHA) * rollAcc;

  // Filtro complementario: pitch
  pitch = ALPHA * (pitch + gy * dt)
        + (1.0f - ALPHA) * pitchAcc;

  // Yaw: integracion de la velocidad angular Z
  yaw = wrap360(yaw + gz * dt);

  // Salida CSV
  Serial.print(roll, 2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(yaw, 2);
  Serial.print(",");
  Serial.println(gz, 3);
}
