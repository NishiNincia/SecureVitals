// ============================================================
// SecureVitals.ino - ESP32 Prototype (v4 - ML Fall)
//
// Hardware:
//   ESP32 WROOM-32
//   MAX30100  -> I2C 0x57
//   MPU6050   -> I2C 0x68
//
// I2C:
//   SDA -> GPIO 21
//   SCL -> GPIO 22
//
// Prototype goals:
//   1. Monitor heart rate
//   2. Estimate SpO2
//   3. Detect activity using MPU6050
//   4. Run fall/impact screening only when motion occurs
//   5. Reduce unnecessary IMU/ML processing
//   6. Reduce false alerts using confirmation logic
//
// CHANGES FROM v2 (all marked with "// [v3]" comments below):
//   - Fixed a blocking delay(50) inside detectFall() that stalled
//     the whole loop during exactly the moments fast sampling
//     matters most. Fall confirmation is now a non-blocking
//     two-stage timer using FALL_CONFIRM_TIME.
//   - Added a rolling accelerometer-variance window. High motion
//     variance now flags the HR reading as low-confidence AND
//     dynamically inflates the Kalman filter's measurement noise,
//     so the estimate leans on the predicted trend instead of a
//     noisy PPG reading during movement.
//   - Replaced the unstable 2-state HR Kalman filter with a
//     conservative scalar Kalman filter. Motion increases measurement
//     noise, extreme beat outliers are gated, and the displayed HR
//     trend is diagnostic only so it cannot drive the estimate toward
//     the 180 BPM safety limit.
//   - printStatus() now reports accel variance, HR trust state,
//     and both raw/filtered HR/current measurement-noise R value.
//
// NO external libraries required.
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "securevitals_fall_model_prototype.h"

// [fix] Forward declaration moved here, above every function in the
// file. evaluateMLFall() (defined early, around where the ML window
// closes) calls updateLCD() to force an immediate emergency screen
// override the moment a fall is confirmed - but updateLCD() itself
// isn't defined until much later, near setup(). C++ requires a
// function to be declared before anything that calls it, so the
// declaration needs to live above the earliest caller, not just
// above detectFall() like in v2.
void updateLCD();

// ============================================================
// I2C
// ============================================================

#define SDA_PIN 21
#define SCL_PIN 22

#define MAX30100_ADDR 0x57
#define MPU6050_ADDR  0x68
#define LCD_ADDR      0x27   // change to 0x3F if your backpack uses that address

// ============================================================
// MPU6050 REGISTERS
// ============================================================

#define MPU_PWR_MGMT_1   0x6B
#define MPU_WHO_AM_I     0x75
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_XOUT_H 0x3B

// ============================================================
// MAX30100 REGISTERS
// ============================================================

#define MAX_MODE_CONFIG      0x06
#define MAX_SPO2_CONFIG      0x07
#define MAX_LED_CONFIG       0x09
#define MAX_FIFO_WR_PTR      0x02
#define MAX_FIFO_RD_PTR      0x04
#define MAX_FIFO_DATA        0x05
#define MAX_REV_ID           0xFE
#define MAX_PART_ID          0xFF

// ============================================================
// SYSTEM SETTINGS
// ============================================================

// MAX30100 sample rate
#define PPG_INTERVAL_MS 40       // ~25 Hz

// IMU sampling
#define IMU_INTERVAL_MS 50       // ~20 Hz

// Print status
#define DISPLAY_INTERVAL_MS 1000

// LCD screen rotation interval
#define LCD_INTERVAL_MS 1500
#define LCD_SCREEN_COUNT 4

// Activity detection - hysteresis to avoid flicker from sensor
// noise sitting near a single threshold (see updateActivity)
#define ACTIVITY_THRESHOLD_HIGH 1.15f
#define ACTIVITY_THRESHOLD_LOW  1.05f

// Strong motion
#define FALL_ACTIVITY_THRESHOLD 2.20f

// Impact threshold
#define IMPACT_THRESHOLD 2.80f

// Fall confirmation window (now actually used - see detectFall) [v2]
#define FALL_CONFIRM_TIME 1200

// HR limits
#define MIN_HR 40.0f
#define MAX_HR 180.0f

// [v2] Motion-gated HR trust / Kalman R scaling
#define ACCEL_VAR_WINDOW 20          // samples in rolling variance window
#define MOTION_TRUST_LIMIT 0.05f     // variance above this -> HR untrusted
#define KALMAN_R_BASE 4.0f           // base measurement noise (bpm^2) when still
#define KALMAN_R_MOTION_GAIN 400.0f  // how strongly motion inflates R
#define KALMAN_Q_HR  0.05f           // process noise: HR state
#define KALMAN_Q_DHR 0.02f           // process noise: HR rate-of-change state

// ============================================================
// STATE
// ============================================================

bool max30100OK = false;
bool mpuOK = false;
bool lcdOK = false;

int lcdScreen = 0; // which LCD screen is showing

float heartRate = 0.0f;   // raw moving-average HR (from IBI buffer, as in v1)
float spo2 = 0.0f;

float ax = 0;
float ay = 0;
float az = 0;

float gx = 0;
float gy = 0;
float gz = 0;

float svm = 1.0f;

bool activeState = false;
bool fallDetected = false;
bool impactDetected = false;

unsigned long lastPPG = 0;
unsigned long lastIMU = 0;
unsigned long lastDisplay = 0;
unsigned long lastLCD = 0;

unsigned long activityStart = 0;

// ============================================================
// PPG VARIABLES
// ============================================================

float previousIR = 0;

bool pulseState = false;

unsigned long lastBeatTime = 0;

float ibiBuffer[8];
int ibiIndex = 0;
int ibiCount = 0;

float redDC = 0;
float irDC = 0;

float redAC = 0;
float irAC = 0;

float filteredIR = 0;

float previousFilteredIR = 0;

// ============================================================
// [v2] MOTION VARIANCE (drives HR trust + Kalman R)
// ============================================================

float svmHistory[ACCEL_VAR_WINDOW];
int svmHistIndex = 0;
bool svmHistFilled = false;
float accelVariance = 0.0f;
bool hrTrusted = true;

void updateAccelVariance()
{
    svmHistory[svmHistIndex] = svm;
    svmHistIndex = (svmHistIndex + 1) % ACCEL_VAR_WINDOW;
    if (svmHistIndex == 0) svmHistFilled = true;

    int count = svmHistFilled ? ACCEL_VAR_WINDOW : svmHistIndex;
    if (count < 2) return;

    float mean = 0;
    for (int i = 0; i < count; i++) mean += svmHistory[i];
    mean /= count;

    float variance = 0;
    for (int i = 0; i < count; i++)
    {
        float d = svmHistory[i] - mean;
        variance += d * d;
    }
    variance /= count;

    accelVariance = variance;
    hrTrusted = (accelVariance <= MOTION_TRUST_LIMIT);
}

// ============================================================
// [v3] STABLE SCALAR KALMAN FILTER FOR HR
//
// v2 used a 2-state [HR, dHR/dt] filter. On the ESP32 prototype,
// a noisy beat could push the trend state too hard, after which
// repeated predict() calls could drive HR toward the 180 BPM clamp.
//
// v3 intentionally uses a scalar Kalman filter for HR only.
// Motion changes the measurement noise R:
//   - still      -> trust the PPG measurement more
//   - moving     -> trust it less, but DO NOT let a trend state
//                   run away from the measured HR.
//
// A separate, heavily limited trend value is kept only as a
// diagnostic display. It is NOT used to predict HR.
// ============================================================

float kX1 = 75.0f;       // filtered HR estimate (bpm)
float kP11 = 16.0f;      // HR estimate variance
float kX2 = 0.0f;        // diagnostic trend only (bpm/s)

unsigned long kalmanLastPredict = 0;
unsigned long kalmanLastUpdate = 0;

float currentR = KALMAN_R_BASE;

void kalmanPredict(float dt)
{
    if (dt <= 0.0f) return;

    // Scalar Kalman: prediction is intentionally conservative.
    // There is no trend-driven HR prediction in v3.
    kP11 += KALMAN_Q_HR * dt;

    // Keep covariance bounded on embedded hardware.
    if (kP11 < 0.01f) kP11 = 0.01f;
    if (kP11 > 100.0f) kP11 = 100.0f;
}

void kalmanUpdate(float measuredHR)
{
    if (measuredHR < MIN_HR || measuredHR > MAX_HR)
        return;

    // Motion-dependent measurement noise.
    // Cap the motion contribution so R cannot become excessively large.
    float motionVariance = accelVariance;
    if (motionVariance < 0.0f) motionVariance = 0.0f;
    if (motionVariance > 0.10f) motionVariance = 0.10f;

    currentR =
        KALMAN_R_BASE +
        KALMAN_R_MOTION_GAIN * motionVariance;

    // Keep R in a useful range for this prototype.
    if (currentR < 4.0f)  currentR = 4.0f;
    if (currentR > 44.0f) currentR = 44.0f;

    // ------------------------------------------------------------
    // Innovation gating
    //
    // A single impossible/noisy beat should not move HR by a huge
    // amount. Reject only extreme jumps; normal physiological
    // changes remain accepted.
    // ------------------------------------------------------------

    float innovation = measuredHR - kX1;

    if (fabs(innovation) > 35.0f)
    {
        // Ignore an extreme outlier, but keep the filter alive.
        return;
    }

    // Scalar Kalman gain.
    float S = kP11 + currentR;
    if (S < 0.0001f) S = 0.0001f;

    float K = kP11 / S;

    // Update HR.
    float previousHR = kX1;
    kX1 += K * innovation;

    // Update covariance.
    kP11 = (1.0f - K) * kP11;

    if (kP11 < 0.01f) kP11 = 0.01f;

    // ------------------------------------------------------------
    // Diagnostic trend only.
    //
    // This value is deliberately smoothed and bounded. It is NOT
    // fed back into kX1, preventing the runaway trend seen in v2.
    // ------------------------------------------------------------

    unsigned long now = millis();

    if (kalmanLastUpdate > 0)
    {
        float dt = (now - kalmanLastUpdate) / 1000.0f;

        if (dt >= 0.20f && dt <= 5.0f)
        {
            float measuredTrend =
                (measuredHR - previousHR) / dt;

            // Reject absurd beat-to-beat trend estimates.
            if (measuredTrend > 5.0f)  measuredTrend = 5.0f;
            if (measuredTrend < -5.0f) measuredTrend = -5.0f;

            // Strong smoothing for diagnostic display.
            kX2 = 0.85f * kX2 + 0.15f * measuredTrend;
        }
    }

    kalmanLastUpdate = now;

    // Final physiological safety limits.
    if (kX1 < MIN_HR) kX1 = MIN_HR;
    if (kX1 > MAX_HR) kX1 = MAX_HR;

    if (kX2 < -5.0f) kX2 = -5.0f;
    if (kX2 > 5.0f)  kX2 = 5.0f;
}

// ============================================================
// [v5] LIGHTWEIGHT ML FALL CLASSIFIER
//
// Logistic regression runs locally on the ESP32.
// The model uses ONLY MPU6050-derived features so unreliable
// motion-corrupted PPG/SpO2 values do not drive fall detection.
//
// Feature order:
//   0 accel_mean_g
//   1 accel_variance_g2
//   2 peak_accel_g
//   3 peak_jerk_g_s
//   4 gyro_magnitude_dps
//   5 post_impact_g
//
// IMPORTANT:
// The supplied header is trained on REAL-READING-ANCHORED SYNTHETIC
// data for prototype testing. It is NOT clinically validated.
// Replace it after collecting and labeling real fall/non-fall data.
// ============================================================

#define ML_FALL_THRESHOLD       0.85f
#define ML_WINDOW_SAMPLES       20
#define ML_POST_IMPACT_WINDOW   1200UL

// ML gate: require meaningful motion before allowing a fall decision.
#define ML_MIN_PEAK_ACCEL       2.00f
#define ML_MIN_PEAK_GYRO        70.0f

// Set to 1 only if you want the old deterministic rule as a fallback.
// Default 0 keeps the ML model as the decision layer.
#define USE_RULE_FALL_FALLBACK  0

float mlFallProbability = 0.0f;
bool mlFallCandidate = false;
bool mlFallConfirmed = false;

float mlAccelMean = 1.0f;
float mlPeakAccel = 1.0f;
float mlPeakJerk = 0.0f;
float mlPeakGyro = 0.0f;

float mlPrevSVM = 1.0f;
unsigned long mlPrevIMUTime = 0;

float mlImpactPeakAccel = 1.0f;
float mlImpactPeakJerk = 0.0f;
float mlImpactPeakGyro = 0.0f;
float mlPostImpactMin = 1.0f;

unsigned long mlImpactTime = 0;
bool mlMLWindowActive = false;

void updateMLFeatures()
{
    unsigned long now = millis();

    float dt = 0.0f;
    if (mlPrevIMUTime > 0)
        dt = (now - mlPrevIMUTime) / 1000.0f;

    float jerk = 0.0f;

    if (dt > 0.001f && dt < 1.0f)
        jerk = fabsf(svm - mlPrevSVM) / dt;

    float gyroMagnitude = sqrtf(
        gx * gx +
        gy * gy +
        gz * gz
    );

    mlPeakAccel = max(mlPeakAccel, svm);
    mlPeakJerk = max(mlPeakJerk, jerk);
    mlPeakGyro = max(mlPeakGyro, gyroMagnitude);

    static float accelBuffer[ML_WINDOW_SAMPLES];
    static int accelIndex = 0;
    static int accelCount = 0;

    accelBuffer[accelIndex] = svm;
    accelIndex = (accelIndex + 1) % ML_WINDOW_SAMPLES;

    if (accelCount < ML_WINDOW_SAMPLES)
        accelCount++;

    if (accelCount >= 2)
    {
        float mean = 0.0f;

        for (int i = 0; i < accelCount; i++)
            mean += accelBuffer[i];

        mean /= accelCount;

        float variance = 0.0f;

        for (int i = 0; i < accelCount; i++)
        {
            float d = accelBuffer[i] - mean;
            variance += d * d;
        }

        mlAccelMean = mean;
        accelVariance = variance / accelCount;
    }

    if (mlMLWindowActive)
        mlPostImpactMin = min(mlPostImpactMin, svm);

    mlPrevSVM = svm;
    mlPrevIMUTime = now;
}

void resetMLFeatures()
{
    mlPeakAccel = svm;
    mlPeakJerk = 0.0f;
    mlPeakGyro = 0.0f;
}

void startMLFallWindow()
{
    mlImpactTime = millis();

    mlImpactPeakAccel = max(mlPeakAccel, svm);
    mlImpactPeakJerk = mlPeakJerk;
    mlImpactPeakGyro = mlPeakGyro;
    mlPostImpactMin = svm;

    mlMLWindowActive = true;
    mlFallCandidate = true;

    Serial.println();
    Serial.println("[ML] Fall candidate window started.");
}

void evaluateMLFall()
{
    if (!mlMLWindowActive)
        return;

    unsigned long elapsed = millis() - mlImpactTime;

    if (elapsed < ML_POST_IMPACT_WINDOW)
        return;

    float features[SV_N_FEATURES];

    features[0] = mlAccelMean;
    features[1] = accelVariance;
    features[2] = mlImpactPeakAccel;
    features[3] = mlImpactPeakJerk;
    features[4] = mlImpactPeakGyro;
    features[5] = mlPostImpactMin;

    mlFallProbability = svFallProbability(features);

    bool motionGate =
        (mlImpactPeakAccel >= ML_MIN_PEAK_ACCEL) ||
        (mlImpactPeakGyro >= ML_MIN_PEAK_GYRO);

    Serial.println();
    Serial.println("[ML] FALL CLASSIFIER");
    Serial.print("Probability : ");
    Serial.print(mlFallProbability * 100.0f, 1);
    Serial.println(" %");

    Serial.print("Peak accel  : ");
    Serial.print(mlImpactPeakAccel, 2);
    Serial.println(" g");

    Serial.print("Peak gyro   : ");
    Serial.print(mlImpactPeakGyro, 1);
    Serial.println(" dps");

    Serial.print("Post min G  : ");
    Serial.println(mlPostImpactMin, 2);

    if (motionGate &&
        mlFallProbability >= ML_FALL_THRESHOLD)
    {
        mlFallConfirmed = true;
        fallDetected = true;

        Serial.println("[ML] FALL CONFIRMED");
        updateLCD();
    }
    else
    {
        Serial.println("[ML] NORMAL / LOW FALL PROBABILITY");
    }

    mlMLWindowActive = false;
    mlFallCandidate = false;
    resetMLFeatures();
}

// ============================================================
// I2C HELPERS
// ============================================================

bool writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool readRegister8(uint8_t address, uint8_t reg, uint8_t &value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(address, (uint8_t)1) != 1)
        return false;

    value = Wire.read();

    return true;
}

bool readRegisters(
    uint8_t address,
    uint8_t reg,
    uint8_t *buffer,
    uint8_t length)
{
    Wire.beginTransmission(address);
    Wire.write(reg);

    if (Wire.endTransmission(false) != 0)
        return false;

    uint8_t received = Wire.requestFrom(address, length);

    if (received != length)
        return false;

    for (uint8_t i = 0; i < length; i++)
        buffer[i] = Wire.read();

    return true;
}

// ============================================================
// I2C SCANNER
// ============================================================

void scanI2C()
{
    Serial.println();
    Serial.println("=== I2C SCANNER ===");

    int devices = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0)
        {
            Serial.print("Found device at 0x");

            if (address < 16)
                Serial.print("0");

            Serial.println(address, HEX);

            devices++;
        }
    }

    if (devices == 0)
        Serial.println("No I2C devices found.");

    Serial.println("Scan complete.");
    Serial.println();
}

// ============================================================
// [v2] LCD DRIVER (16x2, PCF8574 I2C backpack)
//
// No external LiquidCrystal library used - this talks to the
// PCF8574 I/O expander directly over I2C, consistent with the
// rest of this file. Standard PCF8574 backpack wiring:
//   P0 -> RS   P1 -> RW   P2 -> Enable   P3 -> Backlight
//   P4-P7 -> D4-D7 (4-bit mode)
// ============================================================

#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04
#define LCD_RW        0x02
#define LCD_RS        0x01

void lcdWriteI2C(uint8_t data)
{
    Wire.beginTransmission(LCD_ADDR);
    Wire.write(data | LCD_BACKLIGHT);
    Wire.endTransmission();
}

void lcdPulseEnable(uint8_t data)
{
    lcdWriteI2C(data | LCD_ENABLE);
    delayMicroseconds(1);
    lcdWriteI2C(data & ~LCD_ENABLE);
    delayMicroseconds(50);
}

void lcdSend4Bits(uint8_t nibble, uint8_t mode)
{
    lcdPulseEnable((nibble & 0xF0) | mode);
}

void lcdSendByte(uint8_t value, uint8_t mode)
{
    lcdSend4Bits(value & 0xF0, mode);
    lcdSend4Bits((uint8_t)(value << 4) & 0xF0, mode);
}

void lcdCommand(uint8_t cmd) { lcdSendByte(cmd, 0); }
void lcdData(uint8_t data)   { lcdSendByte(data, LCD_RS); }

void lcdClear()
{
    lcdCommand(0x01);
    delay(2);
}

void lcdSetCursor(uint8_t col, uint8_t row)
{
    uint8_t rowOffsets[] = { 0x00, 0x40 };
    lcdCommand(0x80 | (col + rowOffsets[row]));
}

void lcdPrint(const char *str)
{
    while (*str) lcdData(*str++);
}

bool initLCD()
{
    Wire.beginTransmission(LCD_ADDR);
    if (Wire.endTransmission() != 0)
    {
        Serial.println("[LCD] Communication failed.");
        return false;
    }

    delay(50);

    // HD44780 4-bit init sequence
    lcdSend4Bits(0x30, 0);
    delay(5);
    lcdSend4Bits(0x30, 0);
    delayMicroseconds(150);
    lcdSend4Bits(0x30, 0);
    lcdSend4Bits(0x20, 0); // switch to 4-bit mode

    lcdCommand(0x28); // function set: 4-bit, 2 line, 5x8 font
    lcdCommand(0x0C); // display on, cursor off, blink off
    lcdCommand(0x06); // entry mode: increment, no shift
    lcdClear();

    Serial.println("[LCD] OK.");
    return true;
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

bool initMPU6050()
{
    uint8_t whoAmI = 0;

    if (!readRegister8(MPU6050_ADDR, MPU_WHO_AM_I, whoAmI))
    {
        Serial.println("[MPU6050] Communication failed.");
        return false;
    }

    Serial.print("[MPU6050] WHO_AM_I = 0x");
    Serial.println(whoAmI, HEX);

    if (whoAmI != 0x68 &&
        whoAmI != 0x69 &&
        whoAmI != 0x70)
    {
        Serial.println("[MPU6050] Unexpected device ID.");
        return false;
    }

    // Wake sensor
    if (!writeRegister8(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x00))
    {
        Serial.println("[MPU6050] Cannot wake sensor.");
        return false;
    }

    delay(100);

    // Accelerometer +/-2g
    writeRegister8(MPU6050_ADDR, MPU_ACCEL_CONFIG, 0x00);

    // Gyroscope +/-250 dps
    writeRegister8(MPU6050_ADDR, MPU_GYRO_CONFIG, 0x00);

    Serial.println("[MPU6050] OK.");

    return true;
}

// ============================================================
// READ MPU6050
// ============================================================

bool readMPU6050()
{
    uint8_t data[14];

    if (!readRegisters(
            MPU6050_ADDR,
            MPU_ACCEL_XOUT_H,
            data,
            14))
    {
        return false;
    }

    int16_t rawAx =
        ((int16_t)data[0] << 8) | data[1];

    int16_t rawAy =
        ((int16_t)data[2] << 8) | data[3];

    int16_t rawAz =
        ((int16_t)data[4] << 8) | data[5];

    int16_t rawGx =
        ((int16_t)data[8] << 8) | data[9];

    int16_t rawGy =
        ((int16_t)data[10] << 8) | data[11];

    int16_t rawGz =
        ((int16_t)data[12] << 8) | data[13];

    // +/-2g -> 16384 LSB/g
    ax = rawAx / 16384.0f;
    ay = rawAy / 16384.0f;
    az = rawAz / 16384.0f;

    // +/-250 dps -> 131 LSB/dps
    gx = rawGx / 131.0f;
    gy = rawGy / 131.0f;
    gz = rawGz / 131.0f;

    svm = sqrt(
        ax * ax +
        ay * ay +
        az * az
    );

    return true;
}

// ============================================================
// MAX30100 INITIALIZATION
// ============================================================

bool initMAX30100()
{
    uint8_t partID = 0;

    if (!readRegister8(
            MAX30100_ADDR,
            MAX_PART_ID,
            partID))
    {
        Serial.println("[MAX30100] Communication failed.");
        return false;
    }

    Serial.print("[MAX30100] PART ID = 0x");
    Serial.println(partID, HEX);

    // Reset
    writeRegister8(
        MAX30100_ADDR,
        MAX_MODE_CONFIG,
        0x40);

    delay(100);

    // SpO2 mode
    writeRegister8(
        MAX30100_ADDR,
        MAX_MODE_CONFIG,
        0x03);

    // SpO2 configuration:
    //
    // ADC range = 2048 nA
    // Sample rate = 100 Hz
    // LED pulse width = 1600 us
    //
    writeRegister8(
        MAX30100_ADDR,
        MAX_SPO2_CONFIG,
        0x47);

    // LED currents
    // Red + IR
    writeRegister8(
        MAX30100_ADDR,
        MAX_LED_CONFIG,
        0x77);

    // Clear FIFO
    writeRegister8(
        MAX30100_ADDR,
        MAX_FIFO_WR_PTR,
        0x00);

    writeRegister8(
        MAX30100_ADDR,
        MAX_FIFO_RD_PTR,
        0x00);

    Serial.println("[MAX30100] OK.");

    return true;
}

// ============================================================
// READ MAX30100 SAMPLE
// ============================================================

bool readMAX30100(uint16_t &ir, uint16_t &red)
{
    uint8_t data[4];

    if (!readRegisters(
            MAX30100_ADDR,
            MAX_FIFO_DATA,
            data,
            4))
    {
        return false;
    }

    // FIFO format:
    // IR high
    // IR low
    // Red high
    // Red low

    ir =
        ((uint16_t)data[0] << 8) |
        data[1];

    red =
        ((uint16_t)data[2] << 8) |
        data[3];

    return true;
}

// ============================================================
// HEART RATE PROCESSING
// ============================================================

void processPPG(uint16_t irRaw, uint16_t redRaw)
{
    float ir = (float)irRaw;
    float red = (float)redRaw;

    // DC tracking
    if (irDC == 0)
    {
        irDC = ir;
        redDC = red;
    }

    irDC =
        0.95f * irDC +
        0.05f * ir;

    redDC =
        0.95f * redDC +
        0.05f * red;

    float irACValue =
        ir - irDC;

    float redACValue =
        red - redDC;

    irAC =
        0.90f * irAC +
        0.10f * fabs(irACValue);

    redAC =
        0.90f * redAC +
        0.10f * fabs(redACValue);

    // Simple low-pass filter
    filteredIR =
        0.8f * filteredIR +
        0.2f * irACValue;

    // Adaptive threshold
    float threshold =
        max(50.0f, irAC * 0.65f);

    unsigned long now = millis();

    // Detect positive pulse crossing
    if (!pulseState &&
        filteredIR > threshold &&
        filteredIR > previousFilteredIR)
    {
        pulseState = true;

        if (lastBeatTime > 0)
        {
            unsigned long ibi =
                now - lastBeatTime;

            if (ibi >= 333 &&
                ibi <= 1500)
            {
                float bpm =
                    60000.0f /
                    (float)ibi;

                if (bpm >= MIN_HR &&
                    bpm <= MAX_HR)
                {
                    ibiBuffer[
                        ibiIndex
                    ] = ibi;

                    ibiIndex =
                        (ibiIndex + 1) % 8;

                    if (ibiCount < 8)
                        ibiCount++;

                    // Moving HR (raw, unfiltered baseline - kept
                    // from v1 for comparison against Kalman output)
                    float total = 0;

                    for (int i = 0;
                         i < ibiCount;
                         i++)
                    {
                        total +=
                            60000.0f /
                            ibiBuffer[i];
                    }

                    heartRate =
                        total /
                        ibiCount;

                    // [v2] Feed this beat-to-beat estimate into the
                    // Kalman filter. R is scaled by motion at update
                    // time, inside kalmanUpdate().
                    kalmanUpdate(bpm);
                }
            }
        }

        lastBeatTime = now;
    }

    // Reset pulse state
    if (pulseState &&
        filteredIR < threshold * 0.25f)
    {
        pulseState = false;
    }

    previousFilteredIR = filteredIR;

    // ========================================================
    // Basic SpO2 estimation
    // ========================================================

    if (irDC > 100 &&
        redDC > 100 &&
        irAC > 1 &&
        redAC > 1)
    {
        float ratio =
            (redAC / redDC) /
            (irAC / irDC);

        float calculatedSpO2 =
            110.0f -
            25.0f * ratio;

        // Prototype bounds
        // NOTE: this is the generic linear R-ratio approximation,
        // not a calibrated curve. Treat as a demo estimate only.
        if (calculatedSpO2 > 100)
            calculatedSpO2 = 100;

        if (calculatedSpO2 < 70)
            calculatedSpO2 = 70;

        spo2 =
            0.90f * spo2 +
            0.10f * calculatedSpO2;
    }
}

// ============================================================
// ACTIVITY DETECTION
// ============================================================

void updateActivity()
{
    static bool previousActive = false;
    static float svmSmoothed = 1.0f;

    // [v2 fix] Light smoothing dedicated to activity classification
    // only - fall/impact detection below still uses raw svm so it
    // stays fast and doesn't miss a real spike.
    svmSmoothed = 0.8f * svmSmoothed + 0.2f * svm;

    // [v2 fix] Hysteresis instead of a single threshold. Enter ACTIVE
    // only above the high threshold; don't return to REST until
    // dropping below the low threshold. Without this gap, noise
    // sitting near one threshold flickers the state every read and
    // prints "Motion detected" repeatedly - which is what you saw.
    if (!activeState && svmSmoothed > ACTIVITY_THRESHOLD_HIGH)
    {
        activeState = true;
    }
    else if (activeState && svmSmoothed < ACTIVITY_THRESHOLD_LOW)
    {
        activeState = false;
    }

    if (activeState && !previousActive)
    {
        activityStart = millis();

        Serial.println();
        Serial.println(
            "[ACTIVITY] Motion detected"
        );
    }

    previousActive = activeState;
}

// ============================================================
// [v2] FALL / IMPACT DETECTION - now non-blocking
//
// v1 called delay(50) here, stalling the whole loop right when a
// possible fall needed continuous sampling the most. This version
// replaces that with a small state machine driven by millis():
//
//   IDLE -> (impact seen) -> CONFIRMING -> (checked after
//   FALL_CONFIRM_TIME has elapsed, no delay() involved) -> IDLE
// ============================================================

enum FallCheckState
{
    FALLCHECK_IDLE,
    FALLCHECK_CONFIRMING
};

FallCheckState fallCheckState = FALLCHECK_IDLE;
unsigned long fallCheckStartTime = 0;
float fallCheckInitialSVM = 0.0f;

void detectFall()
{
    // Update ML features on every IMU sample.
    updateMLFeatures();

    // Keep evaluating a previously triggered event even if the
    // activity classifier has already returned to REST.
    evaluateMLFall();

    // A strong impact is itself sufficient to start the ML window.
    // Do this BEFORE the activity gate so a fast impact is not missed
    // because the smoothed activity flag has not changed yet.
    if (svm >= IMPACT_THRESHOLD)
    {
        impactDetected = true;

        if (!mlMLWindowActive)
            startMLFallWindow();

        Serial.println();
        Serial.println("!!! IMPACT DETECTED !!!");
        return;
    }

    // For non-impact candidates, require meaningful activity.
    if (!activeState)
        return;

    // Strong movement can also start a candidate window even if the
    // instantaneous impact threshold was missed.
    if (!mlMLWindowActive &&
        svm >= FALL_ACTIVITY_THRESHOLD)
    {
        startMLFallWindow();
        return;
    }

#if USE_RULE_FALL_FALLBACK
    // Optional legacy rule-based fallback.
    if (fallCheckState == FALLCHECK_IDLE)
    {
        if (svm >= FALL_ACTIVITY_THRESHOLD)
        {
            fallCheckState = FALLCHECK_CONFIRMING;
            fallCheckStartTime = millis();
            fallCheckInitialSVM = svm;
        }
        return;
    }

    if (fallCheckState == FALLCHECK_CONFIRMING)
    {
        unsigned long elapsed = millis() - fallCheckStartTime;

        if (elapsed < FALL_CONFIRM_TIME)
            return;

        if (svm < 0.75f || svm > FALL_ACTIVITY_THRESHOLD)
        {
            fallDetected = true;
            Serial.println("!!! RULE-BASED FALL DETECTED !!!");
            updateLCD();
        }

        fallCheckState = FALLCHECK_IDLE;
    }
#endif
}

// ============================================================
// CLEAR EVENTS
// ============================================================

void clearEvents()
{
    // Keep event visible for one display cycle.
    static unsigned long lastClear = 0;

    if (millis() - lastClear > 3000)
    {
        fallDetected = false;
        impactDetected = false;
        mlFallConfirmed = false;

        lastClear = millis();
    }
}

// ============================================================
// STATUS DISPLAY
// ============================================================

void printStatus()
{
    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.print("MAX30100 : ");

    if (max30100OK)
        Serial.println("OK");
    else
        Serial.println("ERROR");

    Serial.print("MPU6050  : ");

    if (mpuOK)
        Serial.println("OK");
    else
        Serial.println("ERROR");

    Serial.print("LCD      : ");

    if (lcdOK)
        Serial.println("OK");
    else
        Serial.println("ERROR");

    Serial.print("HR (raw) : ");

    if (heartRate > 0)
    {
        Serial.print(heartRate, 1);
        Serial.println(" BPM");
    }
    else
    {
        Serial.println("-- BPM");
    }

    // [v2] Kalman-filtered HR + trust/diagnostic info
    Serial.print("HR (Filtered): ");
    Serial.print(kX1, 1);
    Serial.print(" BPM  (trend=");
    Serial.print(kX2, 2);
    Serial.print(" bpm/s, R=");
    Serial.print(currentR, 1);
    Serial.print(", ");
    Serial.print(hrTrusted ? "TRUSTED" : "LOW CONFIDENCE - MOTION");
    Serial.println(")");

    Serial.print("ML Fall  : ");
    Serial.print(mlFallProbability * 100.0f, 1);
    Serial.print(" %");
    if (mlMLWindowActive)
        Serial.print(" (ANALYZING)");
    else if (mlFallConfirmed)
        Serial.print(" (CONFIRMED)");
    Serial.println();

    Serial.print("SpO2     : ");

    if (spo2 > 0)
    {
        Serial.print(spo2, 1);
        Serial.println(" %");
    }
    else
    {
        Serial.println("-- %");
    }

    Serial.print("Accel    : ");
    Serial.print(svm, 2);
    Serial.print(" g   (variance=");
    Serial.print(accelVariance, 4);
    Serial.println(")");

    Serial.print("Activity : ");

    if (activeState)
        Serial.println("ACTIVE");
    else
        Serial.println("REST");

    Serial.print("Fall check state: ");
    Serial.println(
        fallCheckState == FALLCHECK_IDLE ? "IDLE" : "CONFIRMING"
    );

    Serial.print("Fall     : ");

    if (fallDetected)
        Serial.println("DETECTED");
    else
        Serial.println("NO");

    Serial.print("Impact   : ");

    if (impactDetected)
        Serial.println("DETECTED");
    else
        Serial.println("NO");

    Serial.println(
        "================================"
    );
}

// ============================================================
// [v2] LCD SCREEN ROTATION + EMERGENCY OVERRIDE
//
// Rotates through 3 screens every LCD_INTERVAL_MS (loop() drives
// the timer). A confirmed fall immediately overrides whatever is
// showing - called directly from detectFall() the moment a fall
// is confirmed, in addition to the normal periodic refresh.
// ============================================================

void updateLCD()
{
    if (!lcdOK) return;

    char line1[17];
    char line2[17];

    if (fallDetected)
    {
        lcdClear();
        lcdSetCursor(0, 0);
        lcdPrint("FALL DETECTED!");
        lcdSetCursor(0, 1);
        lcdPrint("CHECK PATIENT");
        return;
    }

    lcdClear();

    switch (lcdScreen)
    {
        case 0:
            snprintf(line1, sizeof(line1), "HR:%3d BPM", (int)(kX1 + 0.5f));
            snprintf(line2, sizeof(line2), "SpO2:%3d %%", (int)(spo2 + 0.5f));
            break;

        case 1:
            snprintf(line1, sizeof(line1), "FALL:%s", fallDetected ? "YES" : "NO");
            snprintf(line2, sizeof(line2), "STAT:%s", activeState ? "ACTIVE" : "REST");
            break;

        case 2:
            snprintf(line1, sizeof(line1), "ACC:%.2fg", svm);
            snprintf(line2, sizeof(line2), "VAR:%.3f", accelVariance);
            break;

        default:
            snprintf(line1, sizeof(line1), "ML:%3d%%",
                     (int)(mlFallProbability * 100.0f + 0.5f));
            snprintf(line2, sizeof(line2), "FALL:%s",
                     mlMLWindowActive ? "CHECK" :
                     (mlFallConfirmed ? "YES" : "NO"));
            break;
    }

    lcdSetCursor(0, 0);
    lcdPrint(line1);
    lcdSetCursor(0, 1);
    lcdPrint(line2);
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "   SecureVitals-Edge Prototype v2"
    );

    Serial.println(
        "================================"
    );

    Serial.println();

    // Start I2C
    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );

    Wire.setClock(100000);

    delay(100);

    // --------------------------------------------------------
    // Scan bus
    // --------------------------------------------------------

    scanI2C();

    // --------------------------------------------------------
    // Initialize MAX30100
    // --------------------------------------------------------

    max30100OK =
        initMAX30100();

    // --------------------------------------------------------
    // Initialize MPU6050
    // --------------------------------------------------------

    mpuOK =
        initMPU6050();

    // [v2] Initialize LCD
    lcdOK =
        initLCD();

    Serial.println();

    Serial.println(
        "=== SENSOR STATUS ==="
    );

    Serial.print(
        "MAX30100: "
    );

    Serial.println(
        max30100OK
            ? "READY"
            : "ERROR"
    );

    Serial.print(
        "MPU6050: "
    );

    Serial.println(
        mpuOK
            ? "READY"
            : "ERROR"
    );

    Serial.println();

    if (!max30100OK ||
        !mpuOK)
    {
        Serial.println(
            "WARNING: One or more sensors"
        );

        Serial.println(
            "are unavailable."
        );

        Serial.println(
            "Check wiring and I2C addresses."
        );
    }

    // [v2] init motion history + Kalman timer
    for (int i = 0; i < ACCEL_VAR_WINDOW; i++) svmHistory[i] = 1.0f;

    kalmanLastPredict = millis();
    mlPrevIMUTime = millis();
    kalmanLastUpdate = 0;

    Serial.println();

    Serial.println(
        "Starting monitoring..."
    );

    Serial.println();
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    unsigned long now =
        millis();

    // ========================================================
    // MAX30100
    // ========================================================

    if (max30100OK &&
        now - lastPPG >= PPG_INTERVAL_MS)
    {
        lastPPG = now;

        uint16_t ir;
        uint16_t red;

        if (readMAX30100(
                ir,
                red))
        {
            processPPG(
                ir,
                red
            );
        }
    }

    // ========================================================
    // MPU6050
    // ========================================================

    if (mpuOK &&
        now - lastIMU >= IMU_INTERVAL_MS)
    {
        lastIMU = now;

        if (readMPU6050())
        {
            updateAccelVariance(); // [v2]
            updateActivity();

            // Important:
            // Only process possible falls when
            // meaningful movement is detected.
            detectFall();
        }
    }

    // ========================================================
    // [v2] Kalman predict step - runs every loop iteration so the
    // HR estimate keeps advancing smoothly between beats, not just
    // when a new beat is detected.
    // ========================================================

    if (max30100OK)
    {
        float dt = (now - kalmanLastPredict) / 1000.0f;
        if (dt > 0.001f)
        {
            kalmanPredict(dt);
            kalmanLastPredict = now;
        }
    }

    // ========================================================
    // [v2] LCD - non-blocking rotation every LCD_INTERVAL_MS.
    // Emergency override on a confirmed fall already happens
    // instantly from inside detectFall(); this just keeps the
    // screen refreshed/rotating the rest of the time.
    // ========================================================

    if (lcdOK &&
        now - lastLCD >= LCD_INTERVAL_MS)
    {
        lastLCD = now;

        if (!fallDetected)
        {
            lcdScreen = (lcdScreen + 1) % LCD_SCREEN_COUNT;
        }

        updateLCD();
    }

    // ========================================================
    // STATUS
    // ========================================================

    if (now - lastDisplay >=
        DISPLAY_INTERVAL_MS)
    {
        lastDisplay = now;

        printStatus();

        clearEvents();
    }

    delay(1);
}
