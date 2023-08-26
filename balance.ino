#include "PID_v1.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "WiFi.h"
#include "WebServer.h"

#define WEB_PORT 80
#define PID_SAMPLE_TIME 200
#define MAX_PITCH_SETPOINT 20
#define INTERRUPT_PIN 35
#define MOTOR_PIN_ENA 14
#define MOTOR_PIN_IN1 27
#define MOTOR_PIN_IN2 26
#define MOTOR_PIN_IN3 25
#define MOTOR_PIN_IN4 33
#define MOTOR_PIN_ENB 32
#define PID_PITCH_P 3
#define PID_PITCH_I 1
#define PID_PITCH_D 2

String logBuffer = "input,setpoint,output,error\n";
const char* ssid = "Carnegie";
const char* pass = "greatchina";
/* const char* ssid = "CGS_WiFi"; */
/* const char* pass = "cgswifi4321"; */
const char htmlPage[] PROGMEM = R"rawliteral(
<html>
<head>
<style>
#joystick {
    width: 100%;
    height: 100vh;
    position: relative;
}
p {
  white-space: pre-wrap;
  font-family: monospace;
}
</style>
</head>
<body>
  <p>PID</p>
  <input type="number" id="p" value="PID_P" />
  <input type="number" id="i" value="PID_I" />
  <input type="number" id="d" value="PID_D" />
  <div id="joystick"></div>
  <p>LOG</p>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js"></script>
  <script>
  let lastUpdate = Date.now();
  nipplejs.create({
    zone: document.getElementById('joystick'),
    mode: 'dynamic',
    size: 500,
    color: 'red',
  }).on('move', function (evt, data) {
    if (Date.now() - lastUpdate <= 200) {
      return;
    }
    lastUpdate = Date.now();
    console.log('move', data);
    var xhr = new XMLHttpRequest();
    xhr.open("GET", `/?x=${data.vector.x}&y=${data.vector.y}&p=${document.getElementById('p').value}&i=${document.getElementById('i').value}&d=${document.getElementById('d').value}`, true);
    xhr.send();
  }).on('end', function () {
    console.log('stop');
    var xhr = new XMLHttpRequest();
    xhr.open("GET", `/?x=0&y=0&p=${document.getElementById('p').value}&i=${document.getElementById('i').value}&d=${document.getElementById('d').value}`, true);
    xhr.send();
  });
  </script>
</body>
</html>
)rawliteral";

WebServer server(WEB_PORT);

unsigned long logTimer = 0;

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

double setpoint, input, output;
PID pid_pitch(
  &input,
  &output,
  &setpoint,
  PID_PITCH_P,
  PID_PITCH_I,
  PID_PITCH_D,
  DIRECT
);

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void handleRoot() {
  if (server.hasArg("x")) {
    pid_pitch.SetTunings(
      server.arg("p").toFloat(),
      server.arg("i").toFloat(),
      server.arg("d").toFloat()
    );
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    if (abs(x) < 0.1) {
      x = 0;
    }
    setpoint = y * MAX_PITCH_SETPOINT;
  }

  // replace LOG in htmlPage with log
  String htmlPageEdited = String(htmlPage);
  htmlPageEdited.replace("LOG", logBuffer);
  htmlPageEdited.replace("PID_D", String(PID_PITCH_D));
  htmlPageEdited.replace("PID_I", String(PID_PITCH_I));
  htmlPageEdited.replace("PID_P", String(PID_PITCH_P));

  server.send(200, "text/html", htmlPageEdited);
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

  mpu.setXAccelOffset(-3722);
  mpu.setYAccelOffset(2137);
  mpu.setZAccelOffset(1166);
  mpu.setXGyroOffset(44);
  mpu.setYGyroOffset(14);
  mpu.setZGyroOffset(7);

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
  pid_pitch.SetMode(AUTOMATIC);
  pid_pitch.SetOutputLimits(-255, 255);
  pid_pitch.SetSampleTime(PID_SAMPLE_TIME);
  pid_pitch.SetControllerDirection(REVERSE);
}

void setupWiFi() {
  Serial.print("Connecting to: ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP-Address of ESP32 module: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);

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

void logPID() {
  if (millis() - logTimer > PID_SAMPLE_TIME) {
    Serial.printf(
      "input:%.2f, setpoint:%.2f, error:%.2f, output:%.2f, p:%.2f, i:%.2f, d:%.2f\n",
      input,
      setpoint,
      setpoint - input,
      output,
      pid_pitch.GetKp(),
      pid_pitch.GetKi(),
      pid_pitch.GetKd()
    );

    // excel Paste > Paste Special
    logBuffer += String(input) + "," + String(setpoint) + "," + String(output) + "," + String(setpoint - input) + "\n";

    logTimer = millis();
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
  if (dmpReady) {
    updateMPU();
  };

  server.handleClient();
  pid_pitch.Compute();
  logPID();
  checkFallen();
  moveMotors();
}
