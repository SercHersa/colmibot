#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_VL53L0X.h>
#include "SparkFun_BMI270_Arduino_Library.h"

// Bus I2C compartido por la IMU y los tres sensores ToF.
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t BMI270_ADDRESS = BMI2_I2C_PRIM_ADDR;  // 0x68 con SDO a GND

// Pines de los encoders y de apagado de los sensores ToF.
constexpr uint8_t L_EncA = 35;
constexpr uint8_t L_EncB = 34;
constexpr uint8_t R_EncA = 39;
constexpr uint8_t R_EncB = 36;
constexpr uint8_t XSHUT_LEFT = 18;
constexpr uint8_t XSHUT_CENTER = 19;
constexpr uint8_t XSHUT_RIGHT = 23;

constexpr uint32_t SAMPLE_PERIOD_US = 10000;  // IMU a 100 Hz nominales
constexpr uint32_t PRINT_PERIOD_MS = 200;
constexpr uint16_t TOF_PERIOD_MS = 50;
constexpr int CALIBRATION_SAMPLES = 500;
constexpr float ALPHA = 0.98f;

volatile long L_pulses = 0;
volatile long R_pulses = 0;

BMI270 imu;
Adafruit_VL53L0X leftSensor;
Adafruit_VL53L0X centerSensor;
Adafruit_VL53L0X rightSensor;

bool imuOk = false;
bool imuReadingOk = false;
bool leftOk = false;
bool centerOk = false;
bool rightOk = false;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float roll = 0.0f;
float pitch = 0.0f;
float yaw = 0.0f;
float gz = 0.0f;
uint32_t lastTimeUs = 0;
uint32_t lastPrintMs = 0;

// El canal A dispara la interrupcion y el canal B determina el signo.
void IRAM_ATTR L_PulsesCounter() {
  if (digitalRead(L_EncB) == HIGH) {
    L_pulses++;
  } else {
    L_pulses--;
  }
}

void IRAM_ATTR R_PulsesCounter() {
  if (digitalRead(R_EncB) == HIGH) {
    R_pulses--;
  } else {
    R_pulses++;
  }
}

bool iniciarSensor(Adafruit_VL53L0X &sensor, uint8_t xshut,
                   uint8_t direccion, const char *nombre) {
  // El pull-up del modulo habilita el sensor al liberar XSHUT.
  pinMode(xshut, INPUT);
  delay(10);

  if (sensor.begin(direccion, false, &Wire) &&
      sensor.startRangeContinuous(TOF_PERIOD_MS)) {
    return true;
  }

  // Aislar el sensor antes de activar el siguiente.
  pinMode(xshut, OUTPUT);
  digitalWrite(xshut, LOW);
  Serial.print("Error al inicializar ToF ");
  Serial.println(nombre);
  return false;
}

// Lee el resultado disponible sin esperar a que termine una medicion.
int distanciaMm(Adafruit_VL53L0X &sensor, bool disponible) {
  if (!disponible || !sensor.isRangeComplete()) {
    return -1;
  }

  const uint16_t distancia = sensor.readRangeResult();
  if (distancia == 0xFFFF || sensor.readRangeStatus() != 0) {
    return -1;
  }
  return distancia;
}

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
    "Calibrando giroscopio. Mantener el robot quieto..."
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

void actualizarIMU() {
  uint32_t nowUs = micros();

  if ((uint32_t)(nowUs - lastTimeUs) < SAMPLE_PERIOD_US) {
    return;
  }

  // Medir el tiempo real transcurrido
  float dt = (nowUs - lastTimeUs) / 1000000.0f;
  lastTimeUs = nowUs;

  imuReadingOk = imu.getSensorData() == BMI2_OK;
  if (!imuReadingOk) {
    return;
  }

  // Aceleracion en g
  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  // Velocidad angular en grados por segundo
  float gx = imu.data.gyroX - gyroBiasX;
  float gy = imu.data.gyroY - gyroBiasY;
  gz = imu.data.gyroZ - gyroBiasZ;

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
}

void imprimirDistancia(int distancia) {
  if (distancia < 0) {
    Serial.print("N/D");
  } else {
    Serial.print(distancia);
  }
}

void imprimirSensores() {
  const int izquierda = distanciaMm(leftSensor, leftOk);
  const int centro = distanciaMm(centerSensor, centerOk);
  const int derecha = distanciaMm(rightSensor, rightOk);

  noInterrupts();
  const long pulsosIzquierda = L_pulses;
  const long pulsosDerecha = R_pulses;
  interrupts();

  Serial.printf("Encoders (pulsos) Izq=%ld Der=%ld", pulsosIzquierda, pulsosDerecha);
  if (imuReadingOk) {
    Serial.printf(" | IMU (deg) Roll=%.2f Pitch=%.2f Yaw=%.2f Gz=%.3f deg/s",
                  roll, pitch, yaw, gz);
  } else {
    Serial.print(" | IMU N/D");
  }
  Serial.print(" | ToF (mm) Izq=");
  imprimirDistancia(izquierda);
  Serial.print(" Centro=");
  imprimirDistancia(centro);
  Serial.print(" Der=");
  imprimirDistancia(derecha);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(L_EncA, INPUT);
  pinMode(L_EncB, INPUT);
  pinMode(R_EncA, INPUT);
  pinMode(R_EncB, INPUT);
  attachInterrupt(L_EncA, L_PulsesCounter, RISING);
  attachInterrupt(R_EncA, R_PulsesCounter, RISING);

  // Apagar los tres ToF antes de asignar sus direcciones.
  const uint8_t pinesXshut[] = {XSHUT_LEFT, XSHUT_CENTER, XSHUT_RIGHT};
  for (uint8_t pin : pinesXshut) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  delay(10);

  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("Error al iniciar I2C.");
    return;
  }

  if (imu.beginI2C(BMI270_ADDRESS) == BMI2_OK &&
      imu.getSensorData() == BMI2_OK) {
    initializeTilt();
    imuOk = calibrateGyroscope();
  } else {
    Serial.println("Error al iniciar BMI270.");
  }

  leftOk = iniciarSensor(leftSensor, XSHUT_LEFT, 0x30, "izquierdo");
  centerOk = iniciarSensor(centerSensor, XSHUT_CENTER, 0x31, "central");
  rightOk = iniciarSensor(rightSensor, XSHUT_RIGHT, 0x32, "derecho");

  lastTimeUs = micros();
  lastPrintMs = millis();
}

void loop() {
  if (imuOk) {
    actualizarIMU();
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = nowMs;
    imprimirSensores();
  }
}
