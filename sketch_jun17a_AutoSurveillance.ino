// ESP32 Remote Surveillance

// Blynk template information
// #define BLYNK_TEMPLATE_ID "TMPL2uqpPdwaF"
// #define BLYNK_TEMPLATE_NAME "Surveillance"

#define BLYNK_TEMPLATE_ID "TMPL2sj-eG4NU"
#define BLYNK_TEMPLATE_NAME "AutoSurvaillance"

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WebServer.h>
#include <BlynkSimpleEsp32.h>
#include <Stepper.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "esp_camera.h"

// Blynk Auth Token
char auth[] = "cU1rEJAP1acbwevy6Mhta4mV1ZJKC2eN";

// Wi-Fi credentials
const char *ssid = "jacob";
const char *pass = "123456789";

// Camera Configuration 
#define CAMERA_MODEL_AI_THINKER  

// Define camera pins 
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// Blynk virtual pins
#define VPIN_BUTTON_LOCK V1
#define VPIN_BUTTON_MOTOR V2
#define VPIN_BUTTON_SIREN_SPRAY V3
#define VPIN_GPS_LAT V4
#define VPIN_GPS_LNG V4
#define VPIN_BUTTON_CAMERA_AUDIO V5

// Stepper Motor Configuration
const int stepsPerRevolution = 200;
Stepper stepper(stepsPerRevolution, 14, 12, 13, 15); // GPIO14, GPIO12, GPIO13, GPIO15

// DC Motor Configuration
const int motorPin1 = 2;
const int motorPin2 = 4;
const int enablePin = 16;  // GPIO16 for enabling the motor driver

// Relay Pins
const int relayLock = 5;       // GPIO5
const int relayUnlock = 18;    // GPIO18
const int relaySpray = 19;     // GPIO19
const int relaySiren = 21;     // GPIO21
const int relayCamera = 22;    // GPIO22
const int relayAudio = 23;     // GPIO23

// Limit Switches
const int limitSwitchOpen = 32;
const int limitSwitchClose = 33;

// GPS Configuration
TinyGPSPlus gps;
HardwareSerial ss(1); // Use UART1 (TX: GPIO12, RX: GPIO13)

// Force Sensor
const int forceSensorPin = 34;
int forceSensorThreshold = 500;
bool forceDetected = false;

// Function Prototypes
void lockDoor();
void unlockDoor();
void startSirenAndSpray();
void stopSirenAndSpray();
void startCameraAndAudio();
void stopCameraAndAudio();
void checkForceSensor();
void setupGPS();
void updateGPS();
void startCameraServer();
void runDCMotor();
void stopDCMotor();

BlynkTimer timer;
WebServer server(80);

void setup() {
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 13, 12); // Use GPIO pins 13 (RX) and 12 (TX) for GPS

  Blynk.begin(auth, ssid, pass);

  // Initialize GPIOs
  pinMode(relayLock, OUTPUT);
  pinMode(relayUnlock, OUTPUT);
  pinMode(relaySpray, OUTPUT);
  pinMode(relaySiren, OUTPUT);
  pinMode(relayCamera, OUTPUT);
  pinMode(relayAudio, OUTPUT);
  pinMode(limitSwitchOpen, INPUT_PULLUP);
  pinMode(limitSwitchClose, INPUT_PULLUP);
  pinMode(forceSensorPin, INPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Initialize camera and GPS
  startCameraServer();
  setupGPS();

  // Setup Blynk handlers
  Blynk.setProperty(VPIN_BUTTON_LOCK, "label", "Lock/Unlock Door");
  Blynk.setProperty(VPIN_BUTTON_MOTOR, "label", "Run Motor");
  Blynk.setProperty(VPIN_BUTTON_SIREN_SPRAY, "label", "Siren and Spray");
  Blynk.setProperty(VPIN_BUTTON_CAMERA_AUDIO, "label", "Camera and Audio");

  // Timer to check force sensor regularly
  timer.setInterval(1000L, checkForceSensor);

  Serial.println("Setup complete");
}

void loop() {
  Blynk.run();
  timer.run();
  updateGPS();
  server.handleClient();
}

// Lock door
BLYNK_WRITE(VPIN_BUTTON_LOCK) {
  if (param.asInt()) {
    unlockDoor();
  } else {
    lockDoor();
  }
}

void lockDoor() {
  digitalWrite(relayLock, HIGH);
  stepper.step(stepsPerRevolution / 2); // Rotate stepper +90°
  digitalWrite(relayLock, LOW);
  Blynk.virtualWrite(VPIN_BUTTON_LOCK, 0); // Reset button state
}

void unlockDoor() {
  digitalWrite(relayUnlock, HIGH);
  stepper.step(-stepsPerRevolution / 2); // Rotate stepper -90°
  digitalWrite(relayUnlock, LOW);
  Blynk.virtualWrite(VPIN_BUTTON_LOCK, 0); // Reset button state
}

// Run DC motor
BLYNK_WRITE(VPIN_BUTTON_MOTOR) {
  if (param.asInt()) {
    runDCMotor();
  } else {
    stopDCMotor();
  }
}

void runDCMotor() {
  digitalWrite(enablePin, HIGH);
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  while (digitalRead(limitSwitchOpen) == HIGH) {
    // Motor Keep running until limit switch is triggered
  }
  stopDCMotor();
}

void stopDCMotor() {
  digitalWrite(enablePin, LOW);
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  Blynk.virtualWrite(VPIN_BUTTON_MOTOR, 0); // Reset button state
  while (digitalRead(limitSwitchClose) == HIGH) {
    // Run motor in reverse until limit switch is triggered
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  lockDoor(); // Lock the door after returning to original position
}

// Start siren and spray
BLYNK_WRITE(VPIN_BUTTON_SIREN_SPRAY) {
  if (param.asInt()) {
    startSirenAndSpray();
  } else {
    stopSirenAndSpray();
  }
}

void startSirenAndSpray() {
  digitalWrite(relaySiren, HIGH);
  digitalWrite(relaySpray, HIGH);
  Blynk.virtualWrite(VPIN_BUTTON_SIREN_SPRAY, 1);
}

void stopSirenAndSpray() {
  digitalWrite(relaySiren, LOW);
  digitalWrite(relaySpray, LOW);
  Blynk.virtualWrite(VPIN_BUTTON_SIREN_SPRAY, 0);
}

// Start camera and audio
BLYNK_WRITE(VPIN_BUTTON_CAMERA_AUDIO) {
  if (param.asInt()) {
    startCameraAndAudio();
  } else {
    stopCameraAndAudio();
  }
}

void startCameraAndAudio() {
  digitalWrite(relayCamera, HIGH);
  digitalWrite(relayAudio, HIGH);
  Blynk.virtualWrite(VPIN_BUTTON_CAMERA_AUDIO, 1);
  Blynk.virtualWrite(VPIN_BUTTON_CAMERA_AUDIO, "http://" + WiFi.localIP().toString() + "/jpg");
}

void stopCameraAndAudio() {
  digitalWrite(relayCamera, LOW);
  digitalWrite(relayAudio, LOW);
  Blynk.virtualWrite(VPIN_BUTTON_CAMERA_AUDIO, 0);
}

// Check force sensor
void checkForceSensor() {
  int forceValue = analogRead(forceSensorPin);
  if (forceValue > forceSensorThreshold && !forceDetected) {
    forceDetected = true;
    Blynk.virtualWrite(VPIN_BUTTON_SIREN_SPRAY, "Forceful entry detected!");
    startSirenAndSpray();
    startCameraAndAudio();
    timer.setTimeout(60000L, []() { forceDetected = false; }); // Reset after 1 minute
  }
}

// GPS setup
void setupGPS() {
  ss.begin(9600);
}

// Update GPS location
void updateGPS() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      Blynk.virtualWrite(VPIN_GPS_LAT, gps.location.lat());
      Blynk.virtualWrite(VPIN_GPS_LNG, gps.location.lng());
    }
  }
}

// Start camera server
void startCameraServer() {
  server.on("/jpg", HTTP_GET, []() {
    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb) {
      server.send(503, "text/plain", "Camera capture failed");
      return;
    }
    server.send_P(200, "image/jpeg", (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
  });

  server.begin();
}
