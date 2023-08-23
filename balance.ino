#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define MOTOR_PIN_ENA 10
#define MOTOR_PIN_IN1 9
#define MOTOR_PIN_IN2 8
#define MOTOR_PIN_IN3 7
#define MOTOR_PIN_IN4 6
#define MOTOR_PIN_ENB 5

unsigned long timer = 0;

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr_radians[3];

int pid_p = 2;
int pid_i = 5;
int pid_d = 1;
double setpoint, input, output;
PID pid(&input, &output, &setpoint, pid_p, pid_i, pid_d, DIRECT);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU() {
  // https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_DMP6_Ethernet/MPU6050_DMP6_Ethernet.ino
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    // mpu.CalibrateAccel(6);
    // mpu.CalibrateGyro(6);
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setupPID() {
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(200);
  pid.SetControllerDirection(REVERSE);
}

void setupMotors() {
  pinMode(MOTOR_PIN_ENA, OUTPUT);
  pinMode(MOTOR_PIN_ENB, OUTPUT);
  pinMode(MOTOR_PIN_IN1, OUTPUT);
  pinMode(MOTOR_PIN_IN2, OUTPUT);
  pinMode(MOTOR_PIN_IN3, OUTPUT);
  pinMode(MOTOR_PIN_IN4, OUTPUT);
}

void setupLog() {
  Serial.print("input,setpoint,error,output");
}

void moveMotors() {
  if (output == 0) return;

  digitalWrite(MOTOR_PIN_IN1, output > 0 ? LOW  : HIGH);
  digitalWrite(MOTOR_PIN_IN2, output > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_PIN_IN3, output > 0 ? LOW  : HIGH);
  digitalWrite(MOTOR_PIN_IN4, output > 0 ? HIGH : LOW);

  analogWrite(MOTOR_PIN_ENA, abs(output));
  analogWrite(MOTOR_PIN_ENB, abs(output));
}

void updateMPU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr_radians, &q, &gravity);
    input = ypr_radians[1] * 180/M_PI; // pitch
  }
}

void logPID() {
  if (millis() - timer > 10) {
    Serial.print(input);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(setpoint - input); // error = setpoint - input
    Serial.print(",");
    Serial.print(output);
  }
}

void checkFallen() {
  if (abs(input) > 60) {
    output = 0;
  }
}

void setup() {
  Serial.begin(38400);

  setupMPU();
  setupPID();
  setupLog();
  setupMotors();
}

void loop() {
  if (!dmpReady) return;

  setpoint = 0;

  updateMPU();
  pid.Compute();
  logPID();
  checkFallen();
  moveMotors();
}
