#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define MOTOR_PIN_L_DIRECTION1 x
#define MOTOR_PIN_L_DIRECTION2 x
#define MOTOR_PIN_L_SPEED x
#define MOTOR_PIN_R_DIRECTION1 x
#define MOTOR_PIN_R_DIRECTION2 x
#define MOTOR_PIN_R_SPEED x


// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr_radians[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// pid
int pid_p = 2;
int pid_i = 5;
int pid_d = 1;
double setpoint, input, output;
PID pid(&input, &output, &setpoint, pid_p, pid_i, pid_d, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setupPID() {
  input = analogRead(0);
  setpoint = 0;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
}

void moveMotors() {
  if (output == 0) return;
  if (output > 0) {
    analogWrite(MOTOR_PIN_L_DIRECTION1, LOW);
    analogWrite(MOTOR_PIN_L_DIRECTION2, HIGH);
    analogWrite(MOTOR_PIN_R_DIRECTION1, LOW);
    analogWrite(MOTOR_PIN_R_DIRECTION2, HIGH);
  } else {
    analogWrite(MOTOR_PIN_L_DIRECTION1, HIGH);
    analogWrite(MOTOR_PIN_L_DIRECTION2, LOW);
    analogWrite(MOTOR_PIN_R_DIRECTION1, HIGH);
    analogWrite(MOTOR_PIN_R_DIRECTION2, LOW);
  }

  analogWrite(MOTOR_PIN_L_SPEED, abs(output));
  analogWrite(MOTOR_PIN_R_SPEED, abs(output));
}

void printMPU() {
  if((millis() - timer) > 10) {
    Serial.print("X : ");
    Serial.print(mpu.getAngleX());
    Serial.print("\tY : ");
    Serial.print(mpu.getAngleY());
    Serial.print("\tZ : ");
    Serial.println(mpu.getAngleZ());
    timer = millis();
  }
}

void updateYPR() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr_radians, &q, &gravity);
    Serial.print("ypr_radians\t");
    Serial.print(ypr_radians[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr_radians[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr_radians[2] * 180/M_PI);
  }
}

void setup() {
  Serial.begin(9600);

  setupMPU();
  setupPID();
}

void loop() {
  if (!dmpReady) return;

  updateYPR()

  input = ypr_radians[1] * 180/M_PI; // pitch

  pid.Compute();

  moveMotors()
}
