#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

const uint16_t SAMPLE_PERIOD_MS = 10;
const float DT_S = SAMPLE_PERIOD_MS / 1000.0f;
const float EMA_ALPHA = 0.10f;

const float MOTION_ACCEL_THRESHOLD = 0.5f;
const uint32_t STILL_DURATION_MS = 200;
const float MIN_REP_VELOCITY = 0.08f;
const uint32_t MAX_REP_DURATION_MS = 3000;

float gravity_x = 0, gravity_y = 0, gravity_z = 0;
float gravity_mag = 9.81f;
float filt_x = 0, filt_y = 0, filt_z = 0;
float vel_x = 0, vel_y = 0, vel_z = 0;

enum MotionState { STILL, MOVING };
MotionState motion_state = STILL;

uint32_t still_since_ms = 0;
float peak_vert_vel = 0;
uint32_t rep_start_ms = 0;
uint32_t rep_count = 0;

uint32_t last_sample_ms = 0;

void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg); Wire.write(value);
    Wire.endTransmission();
}

void readAccel(float* ax, float* ay, float* az) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (int)6);
    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read();
    const float G = 9.81f;
    *ax = (raw_x / 4096.0f) * G;
    *ay = (raw_y / 4096.0f) * G;
    *az = (raw_z / 4096.0f) * G;
}

void calibrateGravity() {
    Serial.println("CALIBRATING. Hold chip STILL for 2 seconds...");
    delay(500);
    const int N = 200;
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < N; i++) {
        float ax, ay, az;
        readAccel(&ax, &ay, &az);
        sx += ax; sy += ay; sz += az;
        delay(10);
    }
    gravity_x = sx / N; gravity_y = sy / N; gravity_z = sz / N;
    gravity_mag = sqrt(gravity_x*gravity_x + gravity_y*gravity_y + gravity_z*gravity_z);
    Serial.print("Calibrated |g|=");
    Serial.println(gravity_mag, 2);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== BARBELL TRACKER ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);
    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);
    delay(100);

    calibrateGravity();

    float ax, ay, az;
    readAccel(&ax, &ay, &az);
    filt_x = ax; filt_y = ay; filt_z = az;

    last_sample_ms = millis();
    Serial.println("Ready. Do reps.");
}

void loop() {
    uint32_t now = millis();

    if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
        last_sample_ms = now;

        float ax, ay, az;
        readAccel(&ax, &ay, &az);

        filt_x = EMA_ALPHA * ax + (1.0f - EMA_ALPHA) * filt_x;
        filt_y = EMA_ALPHA * ay + (1.0f - EMA_ALPHA) * filt_y;
        filt_z = EMA_ALPHA * az + (1.0f - EMA_ALPHA) * filt_z;

        float ax_net = filt_x - gravity_x;
        float ay_net = filt_y - gravity_y;
        float az_net = filt_z - gravity_z;

        float accel_dev = sqrt(ax_net*ax_net + ay_net*ay_net + az_net*az_net);
        bool instant_moving = accel_dev > MOTION_ACCEL_THRESHOLD;

        vel_x += ax_net * DT_S;
        vel_y += ay_net * DT_S;
        vel_z += az_net * DT_S;

        float vert_vel = -(vel_x * gravity_x + vel_y * gravity_y + vel_z * gravity_z) / gravity_mag;
        float vert_vel_abs = fabs(vert_vel);

        if (motion_state == STILL) {
            vel_x = 0; vel_y = 0; vel_z = 0;
            if (instant_moving) {
                motion_state = MOVING;
                still_since_ms = 0;
                rep_start_ms = now;
                peak_vert_vel = 0;
            }
        } else {
            if (vert_vel_abs > peak_vert_vel) peak_vert_vel = vert_vel_abs;
            uint32_t rep_duration = now - rep_start_ms;
            bool too_long = rep_duration > MAX_REP_DURATION_MS;

            if (!instant_moving) {
                if (still_since_ms == 0) {
                    still_since_ms = now;
                } else if (now - still_since_ms >= STILL_DURATION_MS) {
                    if (peak_vert_vel >= MIN_REP_VELOCITY && !too_long) {
                        rep_count++;
                        Serial.print(">>> REP ");
                        Serial.print(rep_count);
                        Serial.print("   peak vertical: ~");
                        Serial.print(peak_vert_vel, 2);
                        Serial.println(" m/s (rough estimate)");
                    } else if (too_long) {
                        Serial.println("    [skipped: motion too long]");
                    }
                    motion_state = STILL;
                    vel_x = 0; vel_y = 0; vel_z = 0;
                }
            } else {
                still_since_ms = 0;
            }

            if (too_long && (now - rep_start_ms) > MAX_REP_DURATION_MS + 500) {
                motion_state = STILL;
                vel_x = 0; vel_y = 0; vel_z = 0;
            }
        }
    }
}
