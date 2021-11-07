#include "WiFi.h"
#include <HTTPClient.h>
#include <Arduino_JSON.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

const char* ssid = "DormWifi";
const char* password =  "497addfd33";

//Your Domain name with URL path or IP address with path
String serverLocation = "http://192.168.0.100:5000";

unsigned long hapticRefresh = 1000;
const unsigned long rateRefresh = 60000;
const unsigned long angleRefresh = 2000;

unsigned long currentTime = 0;
unsigned long lastHapticUpdate = 0;
unsigned long lastRateUpdate = 0;
unsigned long lastAngleUpdate = 0;

short motorStatus[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int motorPins[8] = {13, 12, 14, 27, 26, 25, 33, 32};
MPU6050 mpu;

uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_lastSent[3] = {0, -375, -375};           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void setup() {
  for (int i = 0; i < 8; i++) {
    pinMode(motorPins[i], OUTPUT);
  }
  
  Serial.begin(115200);
  Serial.println("Starting Connection Process");

  Wire.begin();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");

  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverPath = serverLocation + "/glove/refreshrate";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String payload = http.getString();

      JSONVar myObject = JSON.parse(payload);

      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        return;
      }
      hapticRefresh = long(myObject["rate"]);
    }
    else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }

  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(motorPins[i], motorStatus[i]);
  }
  currentTime = millis();
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    if (abs(aaReal.x) > 1500) {
      WiFiClient client;
      HTTPClient http;
      
      String serverPath = serverLocation + "/glove/movement";

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      http.addHeader("Content-Type", "application/json");

      // Send HTTP GET request
      int httpResponseCode = http.POST("{\"move\": \"x\"}");

      http.end();
      return;
    }
    else if (abs(aaReal.y) > 1500) {
      WiFiClient client;
      HTTPClient http;
      
      String serverPath = serverLocation + "/glove/movement";

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      http.addHeader("Content-Type", "application/json");

      // Send HTTP GET request
      int httpResponseCode = http.POST("{\"move\": \"y\"}");

      http.end();
      return;
    }
    else if ((abs(ypr[1] - ypr_lastSent[1]) > 0.0348*2.5 || abs(ypr[2] - ypr_lastSent[2]) > 0.0348*2.5 ) && currentTime - lastAngleUpdate > angleRefresh) {
      ypr_lastSent[1] = ypr[1];
      ypr_lastSent[2] = ypr[2];

      WiFiClient client;
      HTTPClient http;

      String serverPath = serverLocation + "/glove/updateAngle";

      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      http.addHeader("Content-Type", "application/json");

      // Send HTTP GET request
      int httpResponseCode = http.POST("{\"pitch\": " + String(ypr[1]) + ", \"roll\": " + String(ypr[2]) + "}");

      http.end();
      lastAngleUpdate = millis();
      return;
    }
  }

  if (currentTime - lastHapticUpdate > hapticRefresh  ) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;

      String serverPath = serverLocation + "/glove";

      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        String payload = http.getString();

        JSONVar myObject = JSON.parse(payload);

        // JSON.typeof(jsonVar) can be used to get the type of the var
        if (JSON.typeof(myObject) == "undefined") {
          Serial.println("Parsing input failed!");
          lastHapticUpdate = millis();
          return;
        }

        for (int i = 0; i < 8; i++) {
          motorStatus[i] = (int)myObject["motors"][i];
        }
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    lastHapticUpdate = millis();
  }
}
