#include <Wire.h>
#include <Adafruit_VL53L0X.h>

constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t XSHUT_LEFT = 18;
constexpr uint8_t XSHUT_CENTER = 19;
constexpr uint8_t XSHUT_RIGHT = 23;

constexpr uint8_t ADDRESS_LEFT = 0x30;
constexpr uint8_t ADDRESS_CENTER = 0x31;
constexpr uint8_t ADDRESS_RIGHT = 0x32;

Adafruit_VL53L0X leftSensor;
Adafruit_VL53L0X centerSensor;
Adafruit_VL53L0X rightSensor;

bool iniciarSensor(Adafruit_VL53L0X &sensor, uint8_t xshut,
                   uint8_t direccion, const char *nombre) {
  // Liberar XSHUT. La resistencia pull-up del módulo habilita el sensor.
  pinMode(xshut, INPUT);
  delay(10);

  if (sensor.begin(direccion, false, &Wire)) {
    return true;
  }

  // Aislar el sensor antes de activar el siguiente, por si sigue en 0x29.
  pinMode(xshut, OUTPUT);
  digitalWrite(xshut, LOW);
  Serial.print("# Error al inicializar el sensor ");
  Serial.println(nombre);
  return false;
}

int distanciaMm(Adafruit_VL53L0X &sensor, bool disponible) {
  if (!disponible) {
    return -1;
  }

  VL53L0X_RangingMeasurementData_t medida{};
  const VL53L0X_Error resultado = sensor.rangingTest(&medida, false);
  if (resultado != VL53L0X_ERROR_NONE || medida.RangeStatus != 0) {
    return -1;
  }
  return medida.RangeMilliMeter;
}

bool leftOk = false;
bool centerOk = false;
bool rightOk = false;

void setup() {
  Serial.begin(115200);

  // Apagar los tres sensores antes de asignar direcciones.
  const uint8_t pinesXshut[] = {XSHUT_LEFT, XSHUT_CENTER, XSHUT_RIGHT};
  for (uint8_t pin : pinesXshut) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  delay(10);

  if (!Wire.begin(SDA_PIN, SCL_PIN, 100000)) {
    Serial.println("# Error al iniciar el bus I2C");
  } else {
    leftOk = iniciarSensor(leftSensor, XSHUT_LEFT, ADDRESS_LEFT, "izquierdo");
    centerOk = iniciarSensor(centerSensor, XSHUT_CENTER, ADDRESS_CENTER, "central");
    rightOk = iniciarSensor(rightSensor, XSHUT_RIGHT, ADDRESS_RIGHT, "derecho");
  }

  Serial.println("izquierda_mm,centro_mm,derecha_mm");
}

void loop() {
  // Medir de forma secuencial y emitir una fila CSV.
  const int izquierda = distanciaMm(leftSensor, leftOk);
  const int centro = distanciaMm(centerSensor, centerOk);
  const int derecha = distanciaMm(rightSensor, rightOk);

  Serial.print(izquierda);
  Serial.print(',');
  Serial.print(centro);
  Serial.print(',');
  Serial.println(derecha);

  delay(50);
}
