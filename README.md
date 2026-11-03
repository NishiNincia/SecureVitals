# SecureVitals

SecureVitals is an ESP32-based wearable health monitoring prototype designed to track physiological signals and detect emergency events, with a focus on fall detection and activity-aware vitals monitoring.

This repository contains the Arduino firmware for a prototype system built around an ESP32, MAX30100 pulse oximeter, and MPU6050 inertial measurement unit. The design aims to combine heart-rate and SpO2 estimation with motion analysis and an embedded machine-learning fall classifier for real-time risk assessment.

## Project overview

The system monitors:
- Heart rate from photoplethysmography (PPG)
- Blood oxygen estimate (SpO2 proxy) from the MAX30100
- Motion and acceleration from the MPU6050
- Activity state and motion variance
- Fall likelihood using a lightweight on-device ML model
- Visual status updates on a 16x2 LCD display

The firmware is intended as a prototype and research platform for health monitoring and emergency response workflows. It is not a certified medical device and should not be used for clinical diagnosis or treatment decisions.

## Hardware

The prototype uses:
- ESP32 WROOM-32
- MAX30100 pulse oximeter / heart-rate sensor
- MPU6050 accelerometer + gyroscope
- 16x2 LCD with I2C backpack (PCF8574)

### Recommended wiring

I2C bus:
- SDA -> GPIO 21
- SCL -> GPIO 22

Sensors:
- MAX30100 -> 0x57
- MPU6050 -> 0x68
- LCD -> 0x27 (common backpack address; verify with I2C scan if needed)

## Repository contents

- `SecureVitals_v5_ML_Prototype_f.ino` — main ESP32 firmware
- `README.md` — project overview and setup notes

## Features

### Signal monitoring
- Continuous heart-rate estimation using a motion-aware filtering strategy
- Pulse signal processing with rolling IBI / beat interval logic
- SpO2 estimation from red and IR photoplethysmography data
- Sensor health checks for MAX30100 and MPU6050

### Motion and activity awareness
- Real-time acceleration magnitude tracking
- Activity detection with hysteresis to reduce flicker
- Rolling motion variance used to estimate confidence in heart-rate readings
- Adaptive Kalman-style filtering to resist noisy motion-induced updates

### Fall detection
- Motion-gated fall candidate detection
- Lightweight ML classifier based on IMU features only
- Threshold-based gating to avoid false positives
- LCD emergency-screen override when a fall is confirmed

### User interface
- Serial console diagnostics for sensor values and health state
- LCD display cycling through health and status screens
- Visual status reporting for activity, confidence, and fall warning states

## Firmware behavior

The firmware runs a main loop that periodically:
1. Reads IMU sensor data
2. Updates motion/activity metrics
3. Samples PPG signals from MAX30100
4. Estimates heart rate and SpO2
5. Evaluates fall candidates using activity and ML logic
6. Refreshes LCD and serial output

The code includes safety and reliability measures such as:
- sensor communication validation
- I2C scanning support
- motion-gated confidence updates
- filtering to suppress extreme outliers
- a non-blocking confirmation process for fall events

## Getting started

### Tools required
- Arduino IDE or VS Code with Arduino support
- ESP32 board package installed
- Required Arduino libraries for core ESP32 support

### Setup steps
1. Open `SecureVitals_v5_ML_Prototype_f.ino` in the Arduino IDE.
2. Select the correct ESP32 board and COM port.
3. Ensure the ESP32 board package is installed.
4. Connect the sensors exactly as wired above.
5. Upload the sketch to the device.
6. Open the serial monitor at 115200 baud to inspect readings and diagnostics.

## Notes on the ML model

The implemented fall classifier uses a lightweight logistic-regression style model stored in a generated header file referenced by the firmware. The code comments note that the provided model is intended for prototype evaluation and is not clinically validated.

This means the model should be treated as an experimental proof of concept, and the system should be refined using real labeled fall and non-fall data before deployment in any real-world safety setting.

## Safety and limitations

- This is a prototype for research and development.
- It is not a certified medical or personal-safety product.
- The fall detection logic is designed for experimental evaluation, not guaranteed emergency response behavior.
- Sensor readings may be affected by motion artifact, placement, or device configuration.
- Always validate the system with realistic testing before relying on it for monitoring or alerting.

## License

This project is provided as-is for educational and prototype development use. Please check the repository for any licensing notices or adapt the project to your own terms as needed.

## Future improvements

Potential next steps include:
- data logging for real-world validation
- BLE / Wi-Fi connectivity for remote alerts
- improved real-time filtering and signal quality assessment
- collection of labeled fall datasets for model retraining
- power optimization for battery-powered wearable operation
- integration with a more robust clinical or safety monitoring workflow
