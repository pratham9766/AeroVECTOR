/*
 * ============================================================
 * ESP32 ROCKET FLIGHT COMPUTER
 * ============================================================
 *
 * Architecture
 * -------------
 * CORE 0:
 *   - I2C sensor acquisition
 *   - MPU6050 service (complementary filter)
 *   - BMP388 altitude
 *   - MPU6050 gyro calibration
 *   - BMP388 pressure calibration
 *   - 2-state altitude/velocity Kalman filter
 *   - flight state machine
 *   - Pitch/Yaw PID -> TVC servo output
 *
 * CORE 1:
 *   - MicroSD logging
 *
 * CORE 0 -> CORE 1:
 *   - FreeRTOS queue
 *
 * IMPORTANT:
 *   TVC servo output is now driven by this program during
 *   STATE_ASCENT only. In every other state the servos are
 *   held at home position (90 deg) and both PID controllers
 *   are reset to avoid integral windup while unpowered.
 *
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h>

#include <Adafruit_BMP3XX.h>

// ============================================================
// PIN CONFIGURATION
// ============================================================

// ---------------- I2C ----------------

#define I2C_SDA                 21
#define I2C_SCL                 22

#define BMP388_ADDR             0x76
#define MPU6050_ADDR             0x68   // AD0 pin low; use 0x69 if AD0 is tied high

// ---------------- MicroSD ----------------

#define SD_MOSI                 23
#define SD_MISO                 19
#define SD_SCK                  18
#define SD_CS                   5

// ---------------- Servo pins ----------------
// SERVO_1 -> pitch actuator
// SERVO_2 -> yaw actuator

#define SERVO_1_PIN             14
#define SERVO_2_PIN             26

// ---------------- Pyro ----------------

#define PYRO_PIN                27     // not a strapping pin, not used elsewhere

constexpr uint32_t PYRO_FIRE_DURATION_MS = 1000;   // tune for your igniter/charge
constexpr float    PYRO_MIN_ALTITUDE_M   = 5.0f;   // sanity gate: never fire near the ground

// ============================================================
// GROUND TEST MODE
// ============================================================
//
// When true, the TVC PID loops run and the servos move according
// to live orientation (tiltPitchDeg / tiltYawDeg) REGARDLESS of
// flightState. This lets you pick the rocket up on the bench,
// tilt it by hand, and watch the mount correct, with no need to
// force the state machine into STATE_ASCENT.
//
// IMPORTANT: set this back to false before an actual flight.
// With it true, the state machine no longer gates the servos, so
// the mount will also move on the pad / during descent, and the
// pyro-min-altitude / ascent logic is unaffected (this flag only
// touches updateTVC()).
// ============================================================

bool GROUND_TEST_MODE = true;

// ============================================================
// SERIAL DEBUG PRINT
// ============================================================

constexpr uint32_t SERIAL_PRINT_PERIOD_MS = 100;   // 10 Hz cap on the print rate (sensor loop itself now runs at ~14 Hz)

// ============================================================
// SENSOR OBJECTS
// ============================================================

Adafruit_BMP3XX bmp388;

// ============================================================
// SERVO OBJECTS
// ============================================================

Servo servoPitch;
Servo servoYaw;

constexpr float SERVO_HOME_DEG   = 90.0f;
constexpr float SERVO_RANGE_DEG  = 12.0f;   // +/- range around home
constexpr float SERVO_MIN_DEG    = SERVO_HOME_DEG - SERVO_RANGE_DEG;
constexpr float SERVO_MAX_DEG    = SERVO_HOME_DEG + SERVO_RANGE_DEG;

// ============================================================
// FLIGHT STATES
// ============================================================

enum FlightState
{
    STATE_BOOT = 0,
    STATE_ON_PAD,
    STATE_ASCENT,
    STATE_APOGEE,
    STATE_DESCENT,
    STATE_LANDED
};

FlightState flightState = STATE_BOOT;

// ============================================================
// STATE MACHINE PARAMETERS
// ============================================================

constexpr float ASCENT_ALTITUDE_THRESHOLD = 3.0f;   // matches the "3 m AGL" comment below; was -6.0f for bench testing
constexpr int ASCENT_REQUIRED_READINGS = 5;

constexpr int APOGEE_REQUIRED_READINGS = 5;

constexpr float LANDED_ALTITUDE_BAND = 1.0f;
constexpr int LANDED_REQUIRED_READINGS = 20;

// ============================================================
// BMP388
// ============================================================

float bmp388ReferencePressure = 1013.25f;

float bmp388Altitude = 0.0f;
float bmp388Pressure = 0.0f;
float bmp388Temperature = 0.0f;

bool bmp388OK = false;

// ============================================================
// MPU6050 DATA
// ============================================================
//
// mpuAccelX/Y/Z, mpuGyroX/Y/Z are stored ALREADY REMAPPED into the
// rocket body frame (Xr/Yr/Zr below) -- not the MPU's native axes.
// Units: accel in g, gyro in deg/s.
//
// ============================================================

float mpuAccelX = 0.0f;
float mpuAccelY = 0.0f;
float mpuAccelZ = 0.0f;

float mpuGyroX = 0.0f;
float mpuGyroY = 0.0f;
float mpuGyroZ = 0.0f;

// ------------------------------------------------------------
// MPU6050 MOUNTING (rocket frame)
//
// As given: the MPU's native X arrow points toward the tail (i.e.
// opposite the nosecone) along the longitudinal axis, its native Y
// arrow points to the right (perpendicular to the longitudinal
// axis), and Z is the remaining axis perpendicular to X and lying
// in the same transverse plane as Y.
//
// We define a right-handed rocket body frame (Xr, Yr, Zr):
//
//   Xr (forward/nose) = -MPU_X   (native X is reversed: tailward)
//   Yr (right)         =  MPU_Y
//   Zr (down)          = -MPU_Z  (completes a right-handed
//                                  forward/right/down frame,
//                                  i.e. a 180 deg rotation about
//                                  the shared Y axis)
//
//   PITCH -> rotation about Yr (nose tips within the Xr-Zr plane)
//   YAW   -> rotation about Zr (nose swings within the Xr-Yr plane)
//   ROLL  -> spin about Xr (longitudinal, not controlled)
//
// This is the standard forward/right/down aerospace body-axis
// convention, chosen because the pitch/yaw <-> which physical servo
// mapping wasn't specified. If a servo moves the wrong way on the
// bench, flip the matching *_CONTROL_SIGN below rather than
// re-deriving the axes.
//
// Control angles (tiltPitchDeg / tiltYawDeg) are tilt of the rocket
// away from vertical, computed via a complementary filter (gyro
// integration fused with the accelerometer's gravity-direction
// estimate) rather than a full attitude/quaternion solution. Like
// the previous BNO-based scheme, 0 deg = nose pointing straight up,
// and the definition is free of gimbal lock at vertical, valid up
// to +-90 deg tilt.
// ------------------------------------------------------------

float tiltPitchDeg = 0.0f;   // rotation about rocket Yr axis
float tiltYawDeg   = 0.0f;   // rotation about rocket Zr axis

// Flip to -1.0f if a servo moves the wrong way when bench testing.
constexpr float PITCH_CONTROL_SIGN = 1.0f;
constexpr float YAW_CONTROL_SIGN   = 1.0f;

// Complementary filter blend: weight given to the gyro-integrated
// angle vs. the accelerometer's instantaneous tilt estimate.
// Higher = smoother but drifts more between accel corrections;
// lower = noisier but tracks gravity more tightly. TUNE for your
// vehicle.
constexpr float COMP_FILTER_ALPHA = 0.98f;

// Gyro zero-rate bias, in the MPU's NATIVE axes (subtracted before
// the axis remap above). Captured by calibrateMPU6050().
//
// NOTE: unlike the old BNO "linear acceleration" offset, we do NOT
// remove an accelerometer bias here -- raw accel must keep gravity
// in it for the tilt estimate to mean anything.

float mpuGyroOffsetX = 0.0f;
float mpuGyroOffsetY = 0.0f;
float mpuGyroOffsetZ = 0.0f;

// ============================================================
// KALMAN FILTER
// ============================================================
//
// State:
//   x[0] = altitude
//   x[1] = vertical velocity
//
// Model:
//   altitude(k+1) = altitude + velocity * dt
//   velocity(k+1) = velocity
//
// Measurement:
//   altitude from BMP388
//
// ============================================================

class AltitudeKalman{
public:

    float altitude = 0.0f;
    float velocity = 0.0f;

    void begin(float initialAltitude)
    {
        altitude = initialAltitude;
        velocity = 0.0f;

        P00 = 2.0f;
        P01 = 0.0f;
        P10 = 0.0f;
        P11 = 2.0f;
    }

    void update(float measuredAltitude, float dt)
    {
        if (dt <= 0.0f)
            return;

        if (dt > 0.2f)
            dt = 0.2f;

        // ----------------------------------------------------
        // Prediction
        // ----------------------------------------------------

        altitude =
            altitude +
            velocity * dt;

        // Process noise
        //
        // These values are intentionally conservative starting
        // values and should be characterized experimentally.

        constexpr float qAltitude = 0.02f;
        constexpr float qVelocity = 1.0f;

        P00 =
            P00 +
            dt * (P10 + P01) +
            dt * dt * P11 +
            qAltitude * dt;

        P01 =
            P01 +
            dt * P11;

        P10 = P01;

        P11 =
            P11 +
            qVelocity * dt;

        // ----------------------------------------------------
        // Measurement update
        // ----------------------------------------------------

        constexpr float measurementNoise = 0.5f;

        float innovation =
            measuredAltitude - altitude;

        float S =
            P00 + measurementNoise;

        if (S <= 0.0f)
            return;

        float K0 = P00 / S;
        float K1 = P10 / S;

        altitude =
            altitude +
            K0 * innovation;

        velocity =
            velocity +
            K1 * innovation;

        // ----------------------------------------------------
        // Covariance update
        // ----------------------------------------------------

        float oldP00 = P00;
        float oldP01 = P01;

        P00 =
            (1.0f - K0) * oldP00;

        P01 =
            (1.0f - K0) * oldP01;

        P10 = P01;

        P11 =
            P11 -
            K1 * oldP01;
    }

private:

    float P00 = 2.0f;
    float P01 = 0.0f;
    float P10 = 0.0f;
    float P11 = 2.0f;
};

AltitudeKalman altitudeKalman;

// ============================================================
// PD CONTROLLER
// ============================================================
//
// This was previously a PID; the integral term has been removed
// entirely (it was reaching the output rail too easily and taking
// a long time to unwind - see the removed integral-separation /
// anti-windup comments in git history if you ever want them back).
// It is now a straight PD with:
//   - low-pass filtered derivative term
//   - error deadband: if |error| <= PID_ERROR_DEADBAND_DEG, the
//     whole update is skipped for that reading. No
//     derivative/previousError update, no new output computed -
//     the previously commanded output (and therefore servo
//     position) is simply held.
//
// ============================================================

constexpr float PID_ERROR_DEADBAND_DEG = 0.500f;

class PID
{
private:

    float Kp, Kd;

    float previousError = 0.0f;
    float filteredDerivative = 0.0f;

    float derivativeFilter = 0.90f;

    float outputMin = -12.0f;
    float outputMax =  12.0f;

    float pTerm = 0.0f;
    float dTerm = 0.0f;

    float output = 0.0f;

public:

    PID(float kp, float kd, float filter = 0.90f)
        : Kp(kp),
          Kd(kd),
          derivativeFilter(filter) {}

    float update(float setpoint, float measured, float dt)
    {
        if (dt <= 0.0f)
            return output;

        float error = setpoint - measured;

        // ------------------------------------------------------
        // Error deadband
        //
        // Within this band the reading is considered "close enough"
        // to the target state: skip the loop entirely for this
        // reading. Integral is not touched, and no new output is
        // computed/actuated - the last commanded output is held.
        // ------------------------------------------------------

        if (fabsf(error) <= PID_ERROR_DEADBAND_DEG)
        {
            return output;
        }

        pTerm = Kp * error;

        float rawDerivative =
            (error - previousError) / dt;

        // Low-pass filter derivative
        filteredDerivative =
            derivativeFilter * filteredDerivative +
            (1.0f - derivativeFilter) * rawDerivative;

        dTerm = Kd * filteredDerivative;

        float rawOutput =
            pTerm + dTerm;

        output = constrain(
            rawOutput,
            outputMin,
            outputMax);

        previousError = error;

        return output;
    }

    float getP() const { return pTerm; }
    float getD() const { return dTerm; }
    float getOutput() const { return output; }

    void reset()
    {
        previousError = 0.0f;
        filteredDerivative = 0.0f;

        pTerm = 0.0f;
        dTerm = 0.0f;
        output = 0.0f;
    }
};

// ============================================================
// PD INSTANCES: PITCH / YAW TVC
// ============================================================
//
// Both loops stabilize the vehicle about the axes perpendicular
// to the longitudinal axis (rocket Xr): pitch = about Yr, yaw =
// about Zr, driving the two TVC servos. Measured values are
// tiltPitchDeg / tiltYawDeg. Setpoints are captured from actual
// orientation right after MPU6050 calibration (see pitchSetpointDeg /
// yawSetpointDeg), not hardcoded to 0 deg. Gains below are
// placeholder starting values -- TUNE THESE for your vehicle's
// mass properties, actuator dynamics, and thrust curve before
// flight. No integral term: any steady-state bias (e.g. a slight
// mounting/mass offset) is left uncorrected rather than integrated
// out - tune Kp/Kd, or trim the setpoint capture, if that turns
// out to matter for your vehicle.
//
// ============================================================

PID pitchPID(
    1.5f,
    5.2f,
    0.90f);

PID yawPID(
    1.5f,
    5.2f,
    0.90f);

// Setpoints: NOT hardcoded to 0. They are captured from the actual
// orientation at the end of MPU6050 calibration (see sensorTask, right
// after calibrateMPU6050() succeeds), so "correct" is whatever attitude
// the airframe was in on the pad when calibration finished, not a
// perfect mathematical vertical.
float pitchSetpointDeg = 0.0f;
float yawSetpointDeg   = 0.0f;

float motor1Angle = SERVO_HOME_DEG; // pitch servo, mirrors packet.motor1Angle
float motor2Angle = SERVO_HOME_DEG; // yaw servo,   mirrors packet.motor2Angle

// ============================================================
// TIMING
// ============================================================

uint32_t lastSensorUpdate = 0;

// Sensor loop target.
// BMP388 configured for 50 Hz.

constexpr uint32_t SENSOR_PERIOD_US = 71429;   // ~14 Hz sensor read / state machine / PID / TVC update rate

// ============================================================
// STATE MACHINE COUNTERS
// ============================================================

int ascentCounter = 0;
int apogeeCounter = 0;
int landedCounter = 0;

float previousAltitude = 0.0f;

float landingReferenceAltitude = 0.0f;

// ============================================================
// TELEMETRY PACKET
// ============================================================

struct TelemetryPacket
{
    uint32_t timestamp;

    FlightState state;

    float kalmanAltitude;
    float altitude;
    float verticalVelocity;

    float pressure;
    float temperature;

    float mpuAccelX;
    float mpuAccelY;
    float mpuAccelZ;

    float mpuGyroX;
    float mpuGyroY;
    float mpuGyroZ;

    float pitch;
    float yaw;

    // --------------------------------------------------------
    // TVC servo telemetry
    // --------------------------------------------------------

    float motor1Angle; // pitch servo commanded angle (deg)
    float motor2Angle; // yaw servo commanded angle (deg)

    // Pitch loop
    float pitchPTerm;
    float pitchDTerm;
    float pitchOutput;   // raw PD output (deg, pre servo-mapping), clamped +-12

    // Yaw loop
    float yawPTerm;
    float yawDTerm;
    float yawOutput;     // raw PD output (deg, pre servo-mapping), clamped +-12

    // Pyro
    bool pyroActive;
    bool pyroFired;
};

// ============================================================
// QUEUE
// ============================================================

QueueHandle_t telemetryQueue = nullptr;
constexpr int TELEMETRY_QUEUE_LENGTH = 32;

// ============================================================
// GENERIC I2C FUNCTIONS
// ============================================================

bool i2cWriteByte(
    uint8_t address,
    uint8_t reg,
    uint8_t value){
    Wire.beginTransmission(address);

    Wire.write(reg);
    Wire.write(value);

    return Wire.endTransmission() == 0;
}

bool i2cReadByte(
    uint8_t address,
    uint8_t reg,
    uint8_t &value)
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

// ============================================================
// MPU6050 LOW-LEVEL I2C
// ============================================================

bool readMPU6050Raw(
    int16_t &ax,
    int16_t &ay,
    int16_t &az,
    int16_t &gx,
    int16_t &gy,
    int16_t &gz)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write((uint8_t)0x3B);   // ACCEL_XOUT_H

    if (Wire.endTransmission(false) != 0)
        return false;

    constexpr uint8_t bytesExpected = 14;   // accel(6) + temp(2) + gyro(6)

    if (Wire.requestFrom((int)MPU6050_ADDR, (int)bytesExpected) != bytesExpected)
        return false;

    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());

    Wire.read();   // TEMP_OUT_H (discarded)
    Wire.read();   // TEMP_OUT_L (discarded)

    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());

    return true;
}

// ============================================================
// MPU6050 INITIALIZATION
// ============================================================

bool initMPU6050()
{
    Serial.println("Initializing MPU6050...");

    uint8_t whoAmI = 0;

    if (!i2cReadByte(MPU6050_ADDR, 0x75, whoAmI))
    {
        Serial.println("MPU6050 not responding.");
        return false;
    }

    if (whoAmI != 0x68)
    {
        // Some clone boards report a slightly different value;
        // warn instead of hard-failing on this alone.
        Serial.print("MPU6050 WHO_AM_I unexpected: 0x");
        Serial.println(whoAmI, HEX);
    }

    // Wake the device up, use the gyro X PLL as the clock source.
    if (!i2cWriteByte(MPU6050_ADDR, 0x6B, 0x01))
    {
        Serial.println("MPU6050 wake failed.");
        return false;
    }

    delay(50);

    // DLPF: ~44 Hz accel / ~42 Hz gyro bandwidth.
    i2cWriteByte(MPU6050_ADDR, 0x1A, 0x03);

    // Sample rate divider (with DLPF enabled, base rate is 1 kHz).
    i2cWriteByte(MPU6050_ADDR, 0x19, 0x09);   // 1kHz / (1+9) = 100 Hz

    // Gyro full-scale range: +-500 deg/s -> 65.5 LSB / (deg/s).
    i2cWriteByte(MPU6050_ADDR, 0x1B, 0x08);

    // Accel full-scale range: +-8g -> 4096 LSB / g.
    i2cWriteByte(MPU6050_ADDR, 0x1C, 0x08);

    Serial.println("MPU6050 OK");

    return true;
}

// ============================================================
// MPU6050 SERVICE (raw read -> rocket frame -> complementary filter)
// ============================================================
//
// Call this every loop tick (same spot serviceBNO085() used to be
// called). Computes its own dt from micros() so it stays correct
// no matter how often it's invoked.
//
// ============================================================

uint32_t lastMPUUpdate = 0;

void updateMPU6050()
{
    int16_t rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;

    if (!readMPU6050Raw(
            rawAx, rawAy, rawAz,
            rawGx, rawGy, rawGz))
    {
        return;   // I2C read failed this tick; keep prior estimate
    }

    // --------------------------------------------------------
    // Convert to physical units (native MPU axes).
    // --------------------------------------------------------

    float axG = rawAx / 4096.0f;
    float ayG = rawAy / 4096.0f;
    float azG = rawAz / 4096.0f;

    float gxDps = (rawGx / 65.5f) - mpuGyroOffsetX;
    float gyDps = (rawGy / 65.5f) - mpuGyroOffsetY;
    float gzDps = (rawGz / 65.5f) - mpuGyroOffsetZ;

    // --------------------------------------------------------
    // Remap native MPU axes -> rocket body frame (see mounting
    // comment above mpuAccelX/etc.):
    //   Xr (forward/nose) = -MPU_X
    //   Yr (right)         =  MPU_Y
    //   Zr (down)          = -MPU_Z
    // --------------------------------------------------------

    mpuAccelX = -axG;
    mpuAccelY =  ayG;
    mpuAccelZ = -azG;

    mpuGyroX  = -gxDps;   // roll rate about Xr (not used for control)
    mpuGyroY  =  gyDps;   // pitch rate about Yr
    mpuGyroZ  = -gzDps;   // yaw rate about Zr

    // --------------------------------------------------------
    // Complementary filter.
    //
    // Accel-only tilt-from-vertical estimate, same definition the
    // old quaternion code used (0 deg = nose straight up, no
    // gimbal lock there): with the rocket vertical, gravity's
    // reaction reads (approx) +1g along Xr, 0 along Yr/Zr.
    //
    //   pitch (about Yr) = atan2(Zr component, Xr component)
    //   yaw   (about Zr) = atan2(-Yr component, Xr component)
    //
    // *_CONTROL_SIGN is folded in here (both into the accel
    // estimate and the integrated rate) so the filter state stays
    // self-consistent across iterations.
    // --------------------------------------------------------

    float pitchAccelDeg =
        PITCH_CONTROL_SIGN *
        atan2f(mpuAccelZ, mpuAccelX) * RAD_TO_DEG;

    float yawAccelDeg =
        YAW_CONTROL_SIGN *
        atan2f(-mpuAccelY, mpuAccelX) * RAD_TO_DEG;

    float pitchRateSigned = PITCH_CONTROL_SIGN * mpuGyroY;
    float yawRateSigned   = YAW_CONTROL_SIGN   * mpuGyroZ;

    uint32_t now = micros();

    float dt =
        (lastMPUUpdate == 0)
            ? 0.0f
            : (now - lastMPUUpdate) / 1000000.0f;

    lastMPUUpdate = now;

    if (dt <= 0.0f || dt > 0.5f)
    {
        // First sample after boot, or a stale/garbage dt: snap to
        // the accel estimate instead of integrating garbage.
        tiltPitchDeg = pitchAccelDeg;
        tiltYawDeg   = yawAccelDeg;
    }
    else
    {
        tiltPitchDeg =
            COMP_FILTER_ALPHA * (tiltPitchDeg + pitchRateSigned * dt) +
            (1.0f - COMP_FILTER_ALPHA) * pitchAccelDeg;

        tiltYawDeg =
            COMP_FILTER_ALPHA * (tiltYawDeg + yawRateSigned * dt) +
            (1.0f - COMP_FILTER_ALPHA) * yawAccelDeg;
    }
}

// ============================================================
// MPU6050 CALIBRATION (gyro zero-rate bias only)
// ============================================================
//
// 200 gyro readings, averaged, in the MPU's NATIVE axes. The MPU
// must remain completely stationary.
//
// Unlike the old BNO calibration, this does NOT touch the
// accelerometer: raw accel must keep gravity in it for the
// complementary filter's tilt estimate to mean anything.
//
// ============================================================

bool calibrateMPU6050()
{
    Serial.println();
    Serial.println(
        "MPU6050 gyro calibration starting.");

    Serial.println(
        "Keep rocket completely stationary.");

    delay(1000);

    double gyroSumX = 0.0;
    double gyroSumY = 0.0;
    double gyroSumZ = 0.0;

    int gyroSamples = 0;

    uint32_t startTime = millis();

    while (gyroSamples < 200)
    {
        int16_t ax, ay, az, gx, gy, gz;

        if (readMPU6050Raw(ax, ay, az, gx, gy, gz))
        {
            gyroSumX += gx / 65.5;
            gyroSumY += gy / 65.5;
            gyroSumZ += gz / 65.5;

            gyroSamples++;
        }

        delay(5);

        if (millis() - startTime > 10000)
        {
            Serial.println(
                "MPU6050 calibration timeout.");

            return false;
        }
    }

    mpuGyroOffsetX =
        gyroSumX / 200.0f;

    mpuGyroOffsetY =
        gyroSumY / 200.0f;

    mpuGyroOffsetZ =
        gyroSumZ / 200.0f;

    Serial.println(
        "MPU6050 gyro calibration complete.");

    return true;
}

// ============================================================
// BMP388 INITIALIZATION
// ============================================================

bool initBMP388()
{
    Serial.println(
        "Initializing BMP388...");

    if (!bmp388.begin_I2C(
            BMP388_ADDR,
            &Wire))
    {
        Serial.println(
            "BMP388 failed.");

        return false;
    }

    // --------------------------------------------------------
    // Configuration for flight
    // --------------------------------------------------------

    bmp388.setTemperatureOversampling(
        BMP3_OVERSAMPLING_2X);

    bmp388.setPressureOversampling(
        BMP3_OVERSAMPLING_8X);

    bmp388.setIIRFilterCoeff(
        BMP3_IIR_FILTER_COEFF_3);

    bmp388.setOutputDataRate(
        BMP3_ODR_50_HZ);

    Serial.println(
        "BMP388 OK");

    return true;
}

// ============================================================
// BMP388 PRESSURE CALIBRATION
// ============================================================
//
// 150 readings discarded.
// 300 valid readings averaged.
//
// ============================================================

bool calibrateBMP388()
{
    Serial.println();
    Serial.println(
        "BMP388 calibration starting.");

    Serial.println(
        "Discarding first 150 readings...");

    int discarded = 0;

    while (discarded < 150)
    {
        if (bmp388.performReading())
        {
            discarded++;
        }

        delay(20);
    }

    Serial.println(
        "First 150 readings discarded.");

    Serial.println(
        "Collecting 300 reference readings...");

    double pressureSum = 0.0;

    int validSamples = 0;

    uint32_t startTime = millis();

    while (validSamples < 300)
    {
        if (bmp388.performReading())
        {
            pressureSum +=
                bmp388.pressure / 100.0f;

            validSamples++;
        }

        delay(20);

        if (millis() - startTime > 15000)
        {
            Serial.println(
                "BMP388 calibration timeout.");

            return false;
        }
    }

    bmp388ReferencePressure =
        pressureSum / 300.0;

    Serial.print(
        "Reference pressure = ");

    Serial.print(
        bmp388ReferencePressure,
        3);

    Serial.println(
        " hPa");

    return true;
}

// ============================================================
// BMP388 READ
// ============================================================

bool readBMP388()
{
    if (!bmp388.performReading())
        return false;

    bmp388Temperature =
        bmp388.temperature;

    bmp388Pressure =
        bmp388.pressure / 100.0f;

    bmp388Altitude =
        44330.0f *
        (
            1.0f -
            pow(
                bmp388Pressure /
                bmp388ReferencePressure,
                0.19029495f)
        );

    return true;
}

// ============================================================
// SERVO INITIALIZATION
// ============================================================

bool initServos()
{
    Serial.println(
        "Initializing TVC servos...");

    // ESP32Servo needs the PWM timers allocated before attach.
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    servoPitch.setPeriodHertz(50);
    servoYaw.setPeriodHertz(50);

    servoPitch.attach(
        SERVO_1_PIN,
        500,
        2400);

    servoYaw.attach(
        SERVO_2_PIN,
        500,
        2400);

    servoPitch.write(SERVO_HOME_DEG);
    servoYaw.write(SERVO_HOME_DEG);

    motor1Angle = SERVO_HOME_DEG;
    motor2Angle = SERVO_HOME_DEG;

    Serial.println(
        "TVC servos OK, homed to 90 deg.");

    return true;
}

// ============================================================
// TVC CONTROL (PITCH / YAW PID -> SERVOS)
// ============================================================
//
// Only actuates during STATE_ASCENT (i.e. while under power).
// In every other state the servos are held at home and both
// PID loops are reset, so integral windup does not accumulate
// on the pad or during unpowered descent.
//
// ============================================================

void updateTVC(float dt)
{
    if (flightState != STATE_ASCENT && !GROUND_TEST_MODE)
    {
        pitchPID.reset();
        yawPID.reset();

        motor1Angle = SERVO_HOME_DEG;
        motor2Angle = SERVO_HOME_DEG;

        servoPitch.write(motor1Angle);
        servoYaw.write(motor2Angle);

        return;
    }

    float pitchOut =
        pitchPID.update(
            pitchSetpointDeg,
            tiltPitchDeg,
            dt);

    float yawOut =
        yawPID.update(
            yawSetpointDeg,
            tiltYawDeg,
            dt);

    motor1Angle =
        constrain(
            SERVO_HOME_DEG + pitchOut,
            SERVO_MIN_DEG,
            SERVO_MAX_DEG);

    motor2Angle =
        constrain(
            SERVO_HOME_DEG + yawOut,
            SERVO_MIN_DEG,
            SERVO_MAX_DEG);

    servoPitch.write(motor1Angle);
    servoYaw.write(motor2Angle);
}

// ============================================================
// STATE NAME
// ============================================================

const char *stateToString(
    FlightState state)
{
    switch (state)
    {
        case STATE_BOOT:
            return "BOOT";

        case STATE_ON_PAD:
            return "ON_PAD";

        case STATE_ASCENT:
            return "ASCENT";

        case STATE_APOGEE:
            return "APOGEE";

        case STATE_DESCENT:
            return "DESCENT";

        case STATE_LANDED:
            return "LANDED";

        default:
            return "UNKNOWN";
    }
}

// ============================================================
// PYRO CHANNEL
// ============================================================

bool     pyroFired   = false;   // latched: can only ever fire once
bool     pyroActive  = false;   // true while output is HIGH
uint32_t pyroStartMs = 0;

void initPyro()
{
    digitalWrite(PYRO_PIN, LOW);   // preload LOW so there is no glitch when switching to output
    pinMode(PYRO_PIN, OUTPUT);
    digitalWrite(PYRO_PIN, LOW);
}

void firePyro()
{
    if (pyroFired)
        return;

    pyroFired   = true;
    pyroActive  = true;
    pyroStartMs = millis();

    digitalWrite(PYRO_PIN, HIGH);

    Serial.println("PYRO FIRED");
}

// Call every loop iteration. Non-blocking.
void servicePyro()
{
    if (pyroActive &&
        (millis() - pyroStartMs) >= PYRO_FIRE_DURATION_MS)
    {
        digitalWrite(PYRO_PIN, LOW);
        pyroActive = false;
    }
}

// ============================================================
// STATE MACHINE
// ============================================================

void updateFlightState(
    float altitude)
{
    switch (flightState)
    {
        // ----------------------------------------------------
        // BOOT
        // ----------------------------------------------------

        case STATE_BOOT:

            // This transition occurs after all initialization
            // and calibration have completed.

            flightState =
                STATE_ON_PAD;

            ascentCounter = 0;
            apogeeCounter = 0;
            landedCounter = 0;

            previousAltitude =
                altitude;

            landingReferenceAltitude =
                altitude;

            Serial.println(
                "STATE -> ON_PAD");

            break;

        // ----------------------------------------------------
        // ON PAD
        // ----------------------------------------------------

        case STATE_ON_PAD:

            // Require altitude to exceed 3 m AGL for five
            // consecutive readings.

            if (altitude >
                ASCENT_ALTITUDE_THRESHOLD)
            {
                ascentCounter++;
            }
            else
            {
                ascentCounter = 0;
            }

            if (ascentCounter >=
                ASCENT_REQUIRED_READINGS)
            {
                flightState =
                    STATE_ASCENT;

                apogeeCounter = 0;

                Serial.println(
                    "STATE -> ASCENT");
            }

            break;

        // ----------------------------------------------------
        // ASCENT
        // ----------------------------------------------------

        case STATE_ASCENT:

            // Five consecutive decreasing altitude readings.

            if (altitude < previousAltitude)
            {
                apogeeCounter++;
            }
            else
            {
                apogeeCounter = 0;
            }

            if (apogeeCounter >=
                APOGEE_REQUIRED_READINGS)
            {
                flightState =
                    STATE_APOGEE;

                landingReferenceAltitude =
                    altitude;

                landedCounter = 0;

                if (altitude >= PYRO_MIN_ALTITUDE_M)
                    firePyro();

                Serial.println(
                    "STATE -> APOGEE");
            }

            break;

        // ----------------------------------------------------
        // APOGEE
        // ----------------------------------------------------

        case STATE_APOGEE:

            // Immediately transition to descent.

            flightState =
                STATE_DESCENT;

            Serial.println(
                "STATE -> DESCENT");

            break;

        // ----------------------------------------------------
        // DESCENT
        // ----------------------------------------------------

        case STATE_DESCENT:

            // Use the current altitude as the landing
            // reference once the rocket approaches the ground.
            //
            // The criterion here is:
            // altitude remains within 1 m of a reference
            // altitude for 20 consecutive readings.

            if (fabs(
                    altitude -
                    landingReferenceAltitude)
                < LANDED_ALTITUDE_BAND)
            {
                landedCounter++;
            }
            else
            {
                landedCounter = 0;

                // Continuously move the reference toward
                // the current altitude so the test follows
                // the low-altitude region.

                landingReferenceAltitude =
                    altitude;
            }

            if (landedCounter >=
                LANDED_REQUIRED_READINGS)
            {
                flightState =
                    STATE_LANDED;

                Serial.println(
                    "STATE -> LANDED");
            }

            break;

        // ----------------------------------------------------
        // LANDED
        // ----------------------------------------------------

        case STATE_LANDED:

            // Terminal state.

            break;
    }

    previousAltitude =
        altitude;
}

// ============================================================
// CREATE TELEMETRY PACKET
// ============================================================

TelemetryPacket createTelemetry()
{
    TelemetryPacket packet;

    packet.timestamp =
        millis();

    packet.state =
        flightState;

    packet.kalmanAltitude =
        altitudeKalman.altitude;

    packet.altitude =
        bmp388Altitude;

    packet.verticalVelocity =
        altitudeKalman.velocity;

    packet.pressure =
        bmp388Pressure;

    packet.temperature =
        bmp388Temperature;

    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    packet.mpuAccelX =
        mpuAccelX;

    packet.mpuAccelY =
        mpuAccelY;

    packet.mpuAccelZ =
        mpuAccelZ;

    packet.mpuGyroX =
        mpuGyroX;

    packet.mpuGyroY =
        mpuGyroY;

    packet.mpuGyroZ =
        mpuGyroZ;

    // --------------------------------------------------------
    // Orientation
    // --------------------------------------------------------

    // Control angles (tilt about Yr / Zr), as seen by the PIDs.

    packet.pitch =
        tiltPitchDeg;

    packet.yaw =
        tiltYawDeg;

    // --------------------------------------------------------
    // TVC servo + PID telemetry
    // --------------------------------------------------------

    packet.motor1Angle =
        motor1Angle;

    packet.motor2Angle =
        motor2Angle;

    packet.pitchPTerm =
        pitchPID.getP();

    packet.pitchDTerm =
        pitchPID.getD();

    packet.pitchOutput =
        pitchPID.getOutput();

    packet.yawPTerm =
        yawPID.getP();

    packet.yawDTerm =
        yawPID.getD();

    packet.yawOutput =
        yawPID.getOutput();

    packet.pyroActive =
        pyroActive;

    packet.pyroFired =
        pyroFired;

    return packet;
}

// ============================================================
// SERIAL DEBUG PRINT
// ============================================================
//
// pitch,yaw,pitch_P,pitch_D,pitch_output,yaw_P,yaw_D,yaw_output
//
// Throttled to SERIAL_PRINT_PERIOD_MS so it doesn't flood the
// port at the full sensor-loop rate.
// ============================================================

uint32_t lastSerialPrintMs = 0;

void printSerialTelemetry(const TelemetryPacket &p)
{
    uint32_t now = millis();

    if (now - lastSerialPrintMs < SERIAL_PRINT_PERIOD_MS)
        return;

    lastSerialPrintMs = now;

    Serial.print(p.pitch, 3);
    Serial.print(",");

    Serial.print(p.yaw, 3);
    Serial.print(",");

    Serial.print(p.pitchPTerm, 4);
    Serial.print(",");

    Serial.print(p.pitchDTerm, 4);
    Serial.print(",");

    Serial.print(p.pitchOutput, 4);
    Serial.print(",");

    Serial.print(p.yawPTerm, 4);
    Serial.print(",");

    Serial.print(p.yawDTerm, 4);
    Serial.print(",");

    Serial.println(p.yawOutput, 4);
}

// ============================================================
// CORE 0 SENSOR TASK
// ============================================================

void sensorTask(
    void *parameter)
{
    (void)parameter;

    // --------------------------------------------------------
    // BOOT
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "CORE 0: BOOT");

    // --------------------------------------------------------
    // I2C
    // --------------------------------------------------------

    Wire.begin(
        I2C_SDA,
        I2C_SCL);

    Wire.setClock(400000);
    Wire.setTimeOut(20);

    Serial.println(
        "I2C initialized.");

    // --------------------------------------------------------
    // Sensors
    // --------------------------------------------------------

    bool bmpOK =
        initBMP388();

    bool mpuOK =
        initMPU6050();

    bool servoOK =
        initServos();

    if (!bmpOK ||
        !mpuOK ||
        !servoOK)
    {
        Serial.println();
        Serial.println(
            "CRITICAL SENSOR INITIALIZATION FAILURE.");

        // Remain in BOOT.
        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Calibration
    // --------------------------------------------------------

    if (!calibrateMPU6050())
    {
        Serial.println(
            "CRITICAL MPU6050 CALIBRATION FAILURE.");

        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Attitude setpoint
    //
    // Whatever tilt the airframe is reading right now (at rest,
    // right after calibration) is taken as "correct" / vertical,
    // rather than assuming a mathematical 0.0 deg. This matters
    // if the rocket sits on a slightly tilted launch rail/rod, or
    // the MPU isn't perfectly aligned with the airframe axis -
    // the PIDs will hold THIS attitude instead of fighting a
    // built-in offset for the whole burn.
    //
    // A few updateMPU6050() calls are run first so the
    // complementary filter has settled onto the accel-derived
    // estimate instead of still sitting at its dt==0 startup value.
    // --------------------------------------------------------

    for (int i = 0; i < 10; i++)
    {
        updateMPU6050();
        delay(10);
    }
#warning we changed this?
    pitchSetpointDeg =0;
        //tiltPitchDeg;

    yawSetpointDeg =0;
       // tiltYawDeg;

    Serial.print(
        "Attitude setpoint captured -> pitch: ");
    Serial.print(
        pitchSetpointDeg, 3);
    Serial.print(
        " deg, yaw: ");
    Serial.print(
        yawSetpointDeg, 3);
    Serial.println(
        " deg");

    if (!calibrateBMP388())
    {
        Serial.println(
            "CRITICAL BMP388 CALIBRATION FAILURE.");

        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Initial altitude
    // --------------------------------------------------------

    if (!readBMP388())
    {
        Serial.println(
            "Initial BMP388 reading failed.");

        while (true)
        {
            delay(1000);
        }
    }

    altitudeKalman.begin(
        bmp388Altitude);

    previousAltitude =
        bmp388Altitude;

    landingReferenceAltitude =
        bmp388Altitude;

    // --------------------------------------------------------
    // All boot configuration/calibration completed.
    // --------------------------------------------------------

    flightState =
        STATE_ON_PAD;

    Serial.println();
    Serial.println(
        "================================");
    Serial.println(
        "SYSTEM READY");
    Serial.println(
        "STATE -> ON_PAD");
    Serial.println(
        "================================");

    if (GROUND_TEST_MODE)
    {
        Serial.println(
            "GROUND_TEST_MODE is ON: TVC will actuate on orientation now, ignoring flight state.");
    }

    Serial.println(
        "pitch,yaw,pitch_P,pitch_D,pitch_output,yaw_P,yaw_D,yaw_output");

    lastSensorUpdate =
        micros();

    // --------------------------------------------------------
    // MAIN SENSOR LOOP
    // --------------------------------------------------------

    while (true)
    {
        // Pyro shutoff check first, so it still happens even if a
        // BMP388 read fails below and hits `continue`.
        servicePyro();

        uint32_t now =
            micros();

        if ((uint32_t)(
                now -
                lastSensorUpdate)
            < SENSOR_PERIOD_US)
        {
            updateMPU6050();
            taskYIELD();
            continue;
        }

        float dt =
            (now -
             lastSensorUpdate)
            / 1000000.0f;

        lastSensorUpdate =
            now;

        // ----------------------------------------------------
        // MPU6050
        // ----------------------------------------------------

        updateMPU6050();

        // ----------------------------------------------------
        // BMP388
        // ----------------------------------------------------

        if (!readBMP388())
        {
            taskYIELD();
            continue;
        }

        // ----------------------------------------------------
        // Kalman filter
        // ----------------------------------------------------

        altitudeKalman.update(
            bmp388Altitude,
            dt);

        // ----------------------------------------------------
        // State machine
        // ----------------------------------------------------

        updateFlightState(
            bmp388Altitude);

        // ----------------------------------------------------
        // Pitch/Yaw TVC (PID -> servos)
        //
        // Uses complementary-filter tilt about Yr (pitch) and Zr
        // (yaw), with the rocket Xr axis along the longitudinal
        // axis. See updateMPU6050(). Only actuates during
        // STATE_ASCENT.
        // ----------------------------------------------------

        updateTVC(dt);

        // ----------------------------------------------------
        // Telemetry
        // ----------------------------------------------------

        TelemetryPacket packet =
            createTelemetry();

        // ----------------------------------------------------
        // Serial debug print
        // ----------------------------------------------------

        printSerialTelemetry(packet);

        // ----------------------------------------------------
        // Queue
        // ----------------------------------------------------

        if (xQueueSend(
                telemetryQueue,
                &packet,
                0) != pdTRUE)
        {
            // Queue full.
            // Drop this sample rather than blocking the
            // sensor/control task.
        }

        taskYIELD();
    }
}

// ============================================================
// SD CARD INITIALIZATION
// ============================================================

bool initSD()
{
    Serial.println(
        "Initializing MicroSD...");

    SPI.begin(
        SD_SCK,
        SD_MISO,
        SD_MOSI,
        SD_CS);

    if (!SD.begin(
            SD_CS,
            SPI,
            20000000))
    {
        Serial.println(
            "SD initialization failed.");

        return false;
    }

    uint8_t cardType =
        SD.cardType();

    if (cardType == CARD_NONE)
    {
        Serial.println(
            "No SD card detected.");

        return false;
    }

    Serial.println(
        "SD card OK.");

    return true;
}

// ============================================================
// CSV HEADER
// ============================================================

void writeCSVHeader(
    File &file)
{
    file.println(
        "timestamp,"
        "state,"
        "kalman_altitude,"
        "altitude,"
        "vertical_velocity_altitude,"
        "pressure,"
        "temperature,"
        "acc_gyro_mpu6050,"
        "pitch,"
        "yaw,"
        "motor_1_angle,"
        "motor_2_angle,"
        "pitch_p_term,"
        "pitch_d_term,"
        "pitch_pd_output,"
        "yaw_p_term,"
        "yaw_d_term,"
        "yaw_pd_output,"
        "pyro_active,"
        "pyro_fired");
}

// ============================================================
// WRITE TELEMETRY
// ============================================================

void writeTelemetry(
    File &file,
    const TelemetryPacket &p)
{
    // --------------------------------------------------------
    // Basic values
    // --------------------------------------------------------

    file.print(p.timestamp);
    file.print(",");

    file.print(
        stateToString(p.state));
    file.print(",");

    file.print(
        p.kalmanAltitude,
        4);
    file.print(",");

    file.print(
        p.altitude,
        4);
    file.print(",");

    file.print(
        p.verticalVelocity,
        4);
    file.print(",");

    file.print(
        p.pressure,
        3);
    file.print(",");

    file.print(
        p.temperature,
        3);
    file.print(",");

    // --------------------------------------------------------
    // MPU6050 ACC/GYRO (rocket frame)
    //
    // Stored as:
    // ax|ay|az|gx|gy|gz
    // --------------------------------------------------------

    file.print(
        p.mpuAccelX,
        4);
    file.print("|");

    file.print(
        p.mpuAccelY,
        4);
    file.print("|");

    file.print(
        p.mpuAccelZ,
        4);
    file.print("|");

    file.print(
        p.mpuGyroX,
        4);
    file.print("|");

    file.print(
        p.mpuGyroY,
        4);
    file.print("|");

    file.print(
        p.mpuGyroZ,
        4);

    file.print(",");

    // --------------------------------------------------------
    // Pitch/Yaw attitude
    // --------------------------------------------------------

    file.print(
        p.pitch,
        3);
    file.print(",");

    file.print(
        p.yaw,
        3);
    file.print(",");

    // --------------------------------------------------------
    // TVC servo angles
    // --------------------------------------------------------

    file.print(
        p.motor1Angle,
        3);
    file.print(",");

    file.print(
        p.motor2Angle,
        3);
    file.print(",");

    // --------------------------------------------------------
    // Pitch PID
    // --------------------------------------------------------

    file.print(
        p.pitchPTerm,
        4);
    file.print(",");

    file.print(
        p.pitchDTerm,
        4);
    file.print(",");

    file.print(
        p.pitchOutput,
        4);
    file.print(",");

    // --------------------------------------------------------
    // Yaw PID
    // --------------------------------------------------------

    file.print(
        p.yawPTerm,
        4);
    file.print(",");

    file.print(
        p.yawDTerm,
        4);
    file.print(",");

    file.print(
        p.yawOutput,
        4);
    file.print(",");

    file.print(
        p.pyroActive ? 1 : 0);
    file.print(",");

    file.println(
        p.pyroFired ? 1 : 0);
}

// ============================================================
// CORE 1 LOGGER TASK
// ============================================================

void loggerTask(
    void *parameter)
{
    (void)parameter;

    Serial.println(
        "CORE 1: Logger starting.");

    if (!initSD())
    {
        Serial.println(
            "CRITICAL SD FAILURE.");

        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Create flight log
    // --------------------------------------------------------

    const char *filename =
        "/flight.csv";

    File logFile =
        SD.open(
            filename,
            FILE_WRITE);

    if (!logFile)
    {
        Serial.println(
            "Could not open flight.csv.");

        while (true)
        {
            delay(1000);
        }
    }

    // Write header if file is empty.

    if (logFile.size() == 0)
    {
        writeCSVHeader(
            logFile);

        logFile.flush();
    }

    Serial.println(
        "CSV logging started.");

    TelemetryPacket packet;

    uint32_t samplesSinceFlush = 0;

    while (true)
    {
        if (xQueueReceive(
                telemetryQueue,
                &packet,
                portMAX_DELAY)
            == pdTRUE)
        {
            writeTelemetry(
                logFile,
                packet);

            samplesSinceFlush++;

            // Flush periodically rather than every line.
            //
            // This significantly reduces SD write overhead.

            if (samplesSinceFlush >= 25)
            {
                logFile.flush();

                samplesSinceFlush = 0;
            }
        }
    }
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // Drive pyro pin LOW before any sensor init/calibration.
    initPyro();

    delay(1000);

    Serial.println();
    Serial.println(
        "==============================================");
    Serial.println(
        " ESP32 FLIGHT COMPUTER");
    Serial.println(
        "==============================================");

    // --------------------------------------------------------
    // Queue
    // --------------------------------------------------------

    telemetryQueue =
        xQueueCreate(
            TELEMETRY_QUEUE_LENGTH,
            sizeof(TelemetryPacket));

    if (telemetryQueue == nullptr)
    {
        Serial.println(
            "ERROR: Queue creation failed.");

        while (true)
        {
            delay(1000);
        }
    }

    // --------------------------------------------------------
    // Sensor task -> CORE 0
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorTask",
        12000,
        nullptr,
        5,
        nullptr,
        0);

    // --------------------------------------------------------
    // Logger task -> CORE 1
    // --------------------------------------------------------

    xTaskCreatePinnedToCore(
        loggerTask,
        "LoggerTask",
        12000,
        nullptr,
        2,
        nullptr,
        1);

    Serial.println(
        "Tasks started.");
}

// ============================================================
// LOOP
// ============================================================
//
// Nothing is performed here.
// Both cores are occupied by explicit FreeRTOS tasks.
//
// ============================================================

void loop(){vTaskDelay(pdMS_TO_TICKS(20));}