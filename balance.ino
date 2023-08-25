#include "PID_v1.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "WiFi.h"
#include "ESPAsyncWebSrv.h"
#include "ArduinoJson.h"

#define WEB_PORT 80
#define PID_SAMPLE_TIME 200
#define INTERRUPT_PIN 35
#define MOTOR_PIN_ENA 14
#define MOTOR_PIN_IN1 27
#define MOTOR_PIN_IN2 26
#define MOTOR_PIN_IN3 25
#define MOTOR_PIN_IN4 33
#define MOTOR_PIN_ENB 32

const char* ssid = "Carnegie";
const char* pass = "greatchina";
const char htmlPage[] PROGMEM = R"rawliteral(
<html><head><script>
function sendData(button) {
  var xhr = new XMLHttpRequest();
  xhr.open("POST", "/comms", true);
  xhr.setRequestHeader("Content-Type", "application/json;charset=UTF-8");

  var data = {
    button: button
  };

  xhr.send(JSON.stringify(data));
}
</script></head>
<body>
  <h1>ESP32 Button Control</h1>
  <button onclick="sendData(1)">Button 1</button>
  <button onclick="sendData(2)">Button 2</button>
</body>
</html>
)rawliteral";

AsyncWebServer server(WEB_PORT);

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

    pinMode(INTERRUPT_PIN, INPUT_PULLDOWN);
    attachInterrupt(INTERRUPT_PIN, dmpDataReady, RISING);

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
  pid.SetSampleTime(PID_SAMPLE_TIME);
  pid.SetControllerDirection(REVERSE);
}

void setupWiFi() {
  Serial.print("Connecting to: ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address of ESP32 module: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", htmlPage);
  });

  server.on("/comms", HTTP_POST, [](AsyncWebServerRequest *request){
    DynamicJsonDocument jsonBuffer(1024);
    /* deserializeJson(jsonBuffer, request->getParam("plain")->value()); */
    /**/
    /* int button = jsonBuffer["button"]; */
    /**/
    /* // Handle the button press */
    /* if (button == 1) { */
    /*   Serial.println("Button 1 pressed"); */
    /*   // Add your action for Button 1 here */
    /* } else if (button == 2) { */
    /*   Serial.println("Button 2 pressed"); */
    /*   // Add your action for Button 2 here */
    /* } */
    /**/
    Serial.println("Button press handled");
    request->send(200, "application/json", "{\"message\":\"Button press handled\"}");
  });

  server.begin();
}

void setupMotors() {
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_PIN_ENA, 0);
  ledcAttachPin(MOTOR_PIN_ENB, 1);
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  pinMode(MOTOR_PIN_IN1, OUTPUT);
  pinMode(MOTOR_PIN_IN2, OUTPUT);
  pinMode(MOTOR_PIN_IN3, OUTPUT);
  pinMode(MOTOR_PIN_IN4, OUTPUT);
}

void moveMotors() {
  if (output == 0) return;

  digitalWrite(MOTOR_PIN_IN1, output > 0 ? LOW  : HIGH);
  digitalWrite(MOTOR_PIN_IN2, output > 0 ? HIGH : LOW);
  digitalWrite(MOTOR_PIN_IN3, output > 0 ? LOW  : HIGH);
  digitalWrite(MOTOR_PIN_IN4, output > 0 ? HIGH : LOW);

  ledcWrite(0, abs(output));
  ledcWrite(1, abs(output));
}

void updateMPU() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr_radians, &q, &gravity);
    input = ypr_radians[1] * 180/M_PI; // pitch
  }
}

void updateWiFi() {
}

void logPID() {
  if (millis() - timer > PID_SAMPLE_TIME) {
    Serial.print("input:");
    Serial.print(input);
    Serial.print(",setpoint:");
    Serial.print(setpoint);
    Serial.print(",error:");
    Serial.print(setpoint - input); // error = setpoint - input
    Serial.print(",output:");
    Serial.println(output);
  }
}

void checkFallen() {
  if (abs(input) > 60) {
    output = 0;
  }
}

void setup() {
  Serial.begin(9600);

  setupMPU();
  setupPID();
  setupMotors();
  setupWiFi();
}

void loop() {
  updateWiFi();

  if (!dmpReady) return;

  setpoint = 0;

  updateMPU();
  pid.Compute();
  logPID();
  checkFallen();
  moveMotors();
}
