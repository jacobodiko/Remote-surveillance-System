// Blynk template and Wi-Fi credentials
#define BLYNK_TEMPLATE_NAME "RemoteAutoSurvaillance"
#define BLYNK_TEMPLATE_ID "TMPL2AB9Cknqi"
#define BLYNK_AUTH_TOKEN "hnU9XV3h-kkPok6ReLVoH3ffd5YZlufi"

// Include necessary libraries
#include <BlynkSimpleEsp32.h>
#include <Stepper.h>
#include <TinyGPS++.h>
#include <WiFi.h>

#define BLYNK_PRINT Serial

const char *ssid = "EMTECH";
const char *pass = "E&m2024#";

// DC Motor Configuration (L298 Driver)
#define MOTOR_IN1 2
#define MOTOR_IN2 4
#define ENABLE_PIN 16

// Stepper Motor Configuration (L298 Driver)
#define STEPPER_IN1 14
#define STEPPER_IN2 12
#define STEPPER_IN3 13
#define STEPPER_IN4 15
const int stepsPerRevolution = 200;
Stepper stepper(stepsPerRevolution, STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);

// Limit Switches
#define LIMIT_SWITCH_OPEN 32
#define LIMIT_SWITCH_CLOSE 33

// Force Sensor and Pump
#define FORCE_SENSOR_PIN 34
#define PUMP_IN1 19
#define PUMP_IN2 18
int forceSensorThreshold = 500;
bool forceDetected = false;

// GPS Configuration
TinyGPSPlus gps;
HardwareSerial ss(1); // Use UART1 (TX: GPIO12, RX: GPIO13)

// Blynk Virtual Pins
#define VPIN_MOTOR V1          // DC Motor ON/OFF
#define VPIN_LOCK_DOOR V2      // Lock Door
#define VPIN_UNLOCK_DOOR V3    // Unlock Door
#define VPIN_SIREN_SPRAY V4    // Siren and Spray
#define VPIN_GPS_LAT_LONG V5   // GPS Latitude and Longitude

BlynkTimer timer;

void setup() {
  // Start serial communication
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 13, 12); // GPS

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize GPIOs
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_SWITCH_OPEN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_CLOSE, INPUT_PULLUP);
  pinMode(FORCE_SENSOR_PIN, INPUT);
  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);

  // Timer to check force sensor
  timer.setInterval(1000L, checkForceSensor);

  // Set labels in Blynk for easier UI control
  Blynk.setProperty(VPIN_MOTOR, "label", "DC Motor (ON/OFF)");
  Blynk.setProperty(VPIN_LOCK_DOOR, "label", "Lock Door");
  Blynk.setProperty(VPIN_UNLOCK_DOOR, "label", "Unlock Door");
  Blynk.setProperty(VPIN_SIREN_SPRAY, "label", "Siren and Spray");
  Blynk.setProperty(VPIN_GPS_LAT_LONG, "label", "GPS Lat/Long");

  Serial.println(" Blynk Online");
}

void loop() {
  Blynk.run();
  timer.run();
  updateGPS();
}

// Run DC Motor for door control (open/close)
BLYNK_WRITE(VPIN_MOTOR) {
  int value = param.asInt();
  if (value) {
    runDCMotorOpen();  // Open the door
  } else {
    runDCMotorClose();  // Close the door
  }
}

// Function to run motor to open door
void runDCMotorOpen() {
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);  // Clockwise rotation
  while (digitalRead(LIMIT_SWITCH_OPEN) == HIGH) {
    // Motor runs until the open limit switch is hit
  }
  stopDCMotor();  // Stop motor once limit switch is hit
}

// Function to run motor to close door
void runDCMotorClose() {
  digitalWrite(ENABLE_PIN, HIGH);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);  // Counter-clockwise rotation
  while (digitalRead(LIMIT_SWITCH_CLOSE) == HIGH) {
    // Motor runs until the close limit switch is hit
  }
  stopDCMotor();  // Stop motor once limit switch is hit
}

// Stop motor function
void stopDCMotor() {
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

// Lock the Door (-90° rotation)
BLYNK_WRITE(VPIN_LOCK_DOOR) {
  lockDoor();
}

void lockDoor() {
  stepper.step(-stepsPerRevolution / 2);  // -90° to lock
}

// Unlock the Door (+90° rotation)
BLYNK_WRITE(VPIN_UNLOCK_DOOR) {
  unlockDoor();
}

void unlockDoor() {
  stepper.step(stepsPerRevolution / 2);  // +90° to unlock
}

// Force sensor detection and alert
void checkForceSensor() {
  int forceValue = analogRead(FORCE_SENSOR_PIN);
  if (forceValue > forceSensorThreshold && !forceDetected) {
    forceDetected = true;
    Blynk.logEvent("force_detected", "Force detected on door!");  // Log the event for notification
    activatePumpAndSiren();
    timer.setTimeout(60000L, deactivatePumpAndSiren);  // Stop after 1 minute
  }
}

// Activate pump and siren
void activatePumpAndSiren() {
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);  // Activate pump
}

// Deactivate pump and siren after 1 minute
void deactivatePumpAndSiren() {
  digitalWrite(PUMP_IN1, LOW);
  digitalWrite(PUMP_IN2, LOW);  // Deactivate pump
  forceDetected = false;
}

// Manual activation of pump and siren via Blynk
BLYNK_WRITE(VPIN_SIREN_SPRAY) {
  int value = param.asInt();
  if (value) {
    activatePumpAndSiren();  // Manually activate pump via Blynk
    timer.setTimeout(60000L, deactivatePumpAndSiren);  // Stop after 1 minute
  }
}

// GPS setup and update location
void updateGPS() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      String gps_data = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      Blynk.virtualWrite(VPIN_GPS_LAT_LONG, gps_data);  // Send Lat/Long to Blynk
    }
  }
}
