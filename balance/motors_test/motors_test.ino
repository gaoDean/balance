#define MOTOR_PIN_ENA 10
#define MOTOR_PIN_IN1 9
#define MOTOR_PIN_IN2 8
#define MOTOR_PIN_IN3 7
#define MOTOR_PIN_IN4 6
#define MOTOR_PIN_ENB 5

int output;
unsigned long timer = 0;

void moveMotors() {
  if (output == 0) return;
  if (output > 0) {
    digitalWrite(MOTOR_PIN_IN1, LOW);
    digitalWrite(MOTOR_PIN_IN2, HIGH);
    digitalWrite(MOTOR_PIN_IN3, LOW);
    digitalWrite(MOTOR_PIN_IN4, HIGH);
  } else {
    digitalWrite(MOTOR_PIN_IN1, HIGH);
    digitalWrite(MOTOR_PIN_IN2, LOW);
    digitalWrite(MOTOR_PIN_IN3, HIGH);
    digitalWrite(MOTOR_PIN_IN4, LOW);
  }

  analogWrite(MOTOR_PIN_ENA, abs(output));
  analogWrite(MOTOR_PIN_ENB, abs(output));
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  // set output to a value between -255 and 255 every 1000ms
  if (millis() - timer > 1000) {
    timer = millis();
    output = random(-255, 255);
  }

  moveMotors();
}
