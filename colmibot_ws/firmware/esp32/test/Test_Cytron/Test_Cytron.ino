/* Programa de prueba para el movimiento de un robot diferencial utilizando ESP32 y un Cytron de 2 canales
 el cual realiza en una rutina infinita con los 4 movimientos en el robot básico y detenido */

// Pines de conexión de los motores
#define LMot_dir 33
#define LMot_pwm 32
#define RMot_dir 26
#define RMot_pwm 25


// ------------- Setup ----------------

void setup() {
  // Configuración de pines como salida o entrada
  pinMode(LMot_dir, OUTPUT);
  pinMode(RMot_dir, OUTPUT);
  Serial.begin(115200);
}


// ------------- Function ----------------

void moveRobot(float linealVel, float angularVel) {
//Se supondrá el que valor máximo de ambos valores es de 1.0

// Variables para calcular las señales PWM de cada motor
  int pwmIzquierdo = 0;
  int pwmDerecho = 0;

  pwmIzquierdo = linealVel*100 - angularVel*100;        // Se multiplican los valores en m/s y rad/s por un factor de 100 cada uno para obtener el valor de la rueda izq
  pwmIzquierdo = constrain(pwmIzquierdo, -150, 150);    // Restringuiendo límites para rueda izquierda para no salturarla
  pwmIzquierdo = map(pwmIzquierdo,-150,150, -255, 255); // Se escala para tener el valor de PWM de la rueda izquierda
  
  pwmDerecho = linealVel*100 + angularVel*100;          // Se multiplican los valores en m/s y rad/s por un factor de 100 cada uno para obtener el valor de la rueda der
  pwmDerecho = constrain(pwmDerecho, -150, 150);        // Restringuiendo límites para rueda derecha para no salturarla
  pwmDerecho = map(pwmDerecho,-150,150, -255, 255);     // Se escala para tener el calor de PWM de la rueda derecha

// Condiciones para establecer los 4 movimientos del robot
  if (pwmIzquierdo > 10 && pwmDerecho > 10){
    // Movimiento recto al frente
    digitalWrite(LMot_dir,HIGH);
    digitalWrite(RMot_dir,HIGH);
  }
  
  else if (pwmIzquierdo < -10 && pwmDerecho < -10){
    // Movimiento recto atrás
    digitalWrite(LMot_dir,LOW);
    digitalWrite(RMot_dir,LOW);
  }

  else if (pwmIzquierdo < -10 && pwmDerecho > 10){
    // Giro a la izquierda
    digitalWrite(LMot_dir,LOW);
    digitalWrite(RMot_dir,HIGH);
  } 
  else if (pwmIzquierdo > 10 && pwmDerecho < -10){
    // Giro a la derecha
    digitalWrite(LMot_dir,HIGH);
    digitalWrite(RMot_dir,LOW);
  }

  // Aplicar señales PWM a los motores
  analogWrite(LMot_pwm, abs(pwmIzquierdo));
  analogWrite(RMot_pwm, abs(pwmDerecho));
}


// ------------- Loop ----------------

void loop() {
  // Ejemplo de uso de la función moverRobot
  moveRobot(1.0, 0.0);  // Mover recto hacia adelante 
  Serial.println("Forward");
  delay(2000);         // Esperar 2 segundos
  moveRobot(-1.0, 0);  // Mover recto hacia atrás 
  Serial.println("Back");
  delay(2000);         // Esperar 2 segundos
  moveRobot(0.0, 1.0); // Girar hacia la izquierda
  Serial.println("Left");
  delay(2000);         // Esperar 2 segundos
  moveRobot(0.0, -1.0);  // Girar hacia la derecha
  Serial.println("Right");
  delay(2000);         // Esperar 2 segundos
  moveRobot(0, 0);    // Detenido
  Serial.println("Stop");
  delay(2000);         // Esperar 2 segundos
}

