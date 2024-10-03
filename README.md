# RemoteAuto-surveillance-System

This code provides a comprehensive implementation of a **Remote Surveillance System** using an ESP32, controlled via the **Blynk app**. The system manages various components, including motors, sensors, and a GPS module. Here's a detailed explanation:

### **Components and Setup**

1. **Blynk Configuration and Wi-Fi Setup:**
   - **Blynk Template and Auth Token:** The template and authentication token allow the ESP32 to connect and communicate with the Blynk app, where users can control the system remotely.
   - **Wi-Fi Credentials:** The ESP32 connects to the local Wi-Fi network using the specified SSID and password.

2. **Motor and Sensor Configuration:**
   - **DC Motor (L298 Driver):**  
     The DC motor is connected to pins 2, 4, and 16, which control the door opening/closing mechanism. It uses the L298 motor driver.
   - **Stepper Motor (L298 Driver):**  
     The stepper motor (connected to pins 12, 13, 14, and 15) controls the locking and unlocking of the door by rotating -90° to lock and +90° to unlock.
   - **Limit Switches:**  
     These switches ensure that the DC motor stops when the door is fully opened or closed by detecting the limits.
   - **Force Sensor and Pump:**  
     A force sensor (connected to pin 34) detects pressure or force on the door, which may indicate tampering. The pump (connected to pins 18 and 19) and siren are activated if a force is detected.
   - **GPS Module:**  
     The GPS module uses the TinyGPS++ library and UART communication to get real-time location data, which is sent to the Blynk app.

### **Code Functions**

1. **`setup()`:**
   - Initializes serial communication for debugging.
   - Connects the ESP32 to Wi-Fi and Blynk.
   - Configures the GPIO pins for motor control, limit switches, force sensor, and pump.
   - Sets up a timer to regularly check the force sensor.
   - Initializes the Blynk properties for the virtual pins used in the mobile app.

2. **Motor Control:**
   - **DC Motor Control (`runDCMotorOpen()` and `runDCMotorClose()`):**  
     These functions control the DC motor to either open or close the door. The motor stops when the respective limit switch is triggered.
   - **Stepper Motor Control (`lockDoor()` and `unlockDoor()`):**  
     The stepper motor is used to lock or unlock the door by rotating it 90°.

3. **Force Detection:**
   - **`checkForceSensor()`:**  
     Reads the force sensor value and checks if it exceeds a certain threshold. If force is detected, it logs an event in Blynk and activates the pump and siren for one minute.
   - **`activatePumpAndSiren()` and `deactivatePumpAndSiren()`:**  
     These functions control the pump and siren, which can be triggered either automatically (upon force detection) or manually via the Blynk app.

4. **GPS Tracking:**
   - **`updateGPS()`:**  
     Retrieves the GPS location from the GPS module and updates the Blynk app with the current latitude and longitude.

5. **Blynk Virtual Pin Handlers:**
   - **`BLYNK_WRITE(VPIN_MOTOR)`:**  
     Controls the DC motor (to open/close the door) based on user input from the Blynk app.
   - **`BLYNK_WRITE(VPIN_LOCK_DOOR)` and `BLYNK_WRITE(VPIN_UNLOCK_DOOR)`:**  
     Lock and unlock the door using the stepper motor based on user input.
   - **`BLYNK_WRITE(VPIN_SIREN_SPRAY)`:**  
     Manually activates the pump and siren via the Blynk app.

### **Overall Functionality:**

- The **DC motor** handles the opening and closing of the door.
- The **stepper motor** locks and unlocks the door.
- The **force sensor** detects forced entry, triggering an alarm (pump and siren) and logging the event in the Blynk app.
- The **GPS module** continuously updates the app with the current location, allowing remote tracking.
- All these features can be monitored and controlled via the **Blynk mobile app**, making it a powerful IoT-based security system.

This project combines IoT, motor control, sensor integration, and remote surveillance into one cohesive system that is ideal for securing homes or small premises.
