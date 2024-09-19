# Remote-surveillance-System 

### Overview

This ESP32-based project is a **remote surveillance system** that integrates several functionalities such as **GPS tracking**, **camera streaming**, **DC and stepper motor control**, and various **relay-driven mechanisms** (siren, spray, lock). The system is connected to **Blynk**, a platform for IoT device control via a mobile app. The project is designed to perform multiple tasks including:

- **Lock/unlock doors**
- **Control motors**
- **Activate siren and spray**
- **Stream video and audio from a camera**
- **Monitor GPS data for real-time location tracking**
- **Detect forced entry using a force sensor**

### Key Components

- **Blynk**: Used to interface with a mobile app that controls various devices.
- **ESP32 Camera**: Used for streaming video.
- **GPS Module**: Used for tracking location.
- **Stepper and DC Motors**: Used for door control and other mechanisms.
- **Relays**: Used to switch various components like siren, spray, camera, and audio.

### Code Explanation

1. **Includes and Defines**:
   - `WiFi.h`, `WebServer.h`, `BlynkSimpleEsp32.h`, `Stepper.h`, `Wire.h`, `TinyGPS++.h`, and `esp_camera.h`: Include necessary libraries.
   - `#define BLYNK_TEMPLATE_ID`, `BLYNK_TEMPLATE_NAME`: Used to identify the Blynk template for your project.
   - GPIO pins for the ESP32 Camera, Stepper Motor, and relays are defined here for easy configuration.

2. **Wi-Fi and Blynk Credentials**:
   - The ESP32 connects to a Wi-Fi network using credentials provided (`ssid`, `pass`) and communicates with Blynk using the **authentication token** `auth[]`.

3. **Camera Configuration**:
   - Pins for the camera module are defined to correctly interface with the ESP32. This is for the **AI Thinker Camera Module**.

4. **Motor Configurations**:
   - **Stepper Motor**: Configured to control the door lock mechanism using the GPIO pins 14, 12, 13, and 15.
   - **DC Motor**: Controlled using an **H-Bridge** motor driver connected to GPIO pins 2, 4, and an enable pin 16.

5. **Relays**:
   - Control components like the lock (`relayLock`), spray (`relaySpray`), siren (`relaySiren`), camera (`relayCamera`), and audio system (`relayAudio`).

6. **Force Sensor**:
   - Connected to GPIO pin 34, it detects if force is applied beyond a certain threshold (`forceSensorThreshold = 500`), which might indicate a forced entry.

7. **GPS Setup**:
   - Configured using `TinyGPS++` library, with UART communication on GPIO pins 13 and 12 for receiving GPS data.

8. **Blynk Virtual Pins**:
   - Mapped to different Blynk interface buttons for controlling the device. For example:
     - **V1**: Door lock/unlock
     - **V2**: Motor control
     - **V3**: Siren and spray control
     - **V4**: GPS coordinates
     - **V5**: Camera and audio

### Functional Explanation

#### 1. **`setup()` Function**
   - Initializes serial communication, Blynk, and the GPIO pins for various components (relays, motors, etc.).
   - **Starts the camera server** to handle requests for video streaming.
   - **Initializes GPS** and sets up Blynk properties for different virtual pins.

#### 2. **Main Loop: `loop()`**
   - Runs the core functionalities:
     - **Blynk.run()**: Maintains communication with the Blynk server.
     - **timer.run()**: Executes timed tasks like checking the force sensor every second.
     - **updateGPS()**: Updates GPS location data.
     - **server.handleClient()**: Handles HTTP requests to stream camera feed.

#### 3. **Lock/Unlock Door**
   - Blynk virtual pin **V1** controls door locking/unlocking via a stepper motor.
   - The motor rotates **90°** to lock and **-90°** to unlock the door.

#### 4. **DC Motor Control**
   - Controlled by **V2** in the Blynk app. The motor runs until a limit switch (`limitSwitchOpen`) is triggered.
   - When the motor reaches the end of its path, it reverses and returns to the starting position, locking the door.

#### 5. **Siren and Spray**
   - Controlled by **V3**. Activates both the siren and spray when triggered.

#### 6. **Camera and Audio Control**
   - Controlled by **V5**. Starts streaming the camera feed and audio to the Blynk app. The stream is accessed via the local IP of the ESP32.

#### 7. **Force Sensor Detection**
   - The force sensor is checked every second. If a forceful entry is detected (force exceeds the threshold), the system automatically activates the **siren and spray**, and starts the **camera and audio** for a minute.

#### 8. **GPS Tracking**
   - Updates the GPS coordinates via **UART** communication. The data is regularly sent to the Blynk app on virtual pins **V4**.

#### 9. **Camera Server**
   - A **web server** is set up to handle HTTP GET requests at the `/jpg` endpoint. When accessed, the camera takes a snapshot and sends the image as a JPEG.

### Key Functionalities:
- **Remote Door Control**: Control locking/unlocking via the Blynk app.
- **Siren and Spray Activation**: Triggered manually or automatically by force detection.
- **Live Camera Stream**: Accessible via the local web server.
- **GPS Monitoring**: Provides real-time location tracking.
- **Force Detection**: Triggers emergency protocols.

---
