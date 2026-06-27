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
const uint16_t PRINT_PERIOD_MS = 200;

const float EMA_ALPHA = 0.10f;

// Gravity assumption: chip held flat, Z is up.
// Day 5 will replace this with real calibration.
const float GRAVITY_X = 9.81f;
const float GRAVITY_Y = 0.0f;
const float GRAVITY_Z = 0.0f;

float filt_x = 0;
float filt_y = 0;
float filt_z = 0;

float vel_x = 0;
float vel_y = 0;
float vel_z = 0;

uint32_t last_sample_ms = 0;
uint32_t last_print_ms = 0;

void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void readAccel(float* ax, float* ay, float* az) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6);

    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read();

    const float G = 9.81f;
    *ax = (raw_x / 4096.0f) * G;
    *ay = (raw_y / 4096.0f) * G;
    *az = (raw_z / 4096.0f) * G;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DAY 4: VELOCITY INTEGRATION ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);
    delay(100);

    // Prime the EMA filter
    float ax, ay, az;
    readAccel(&ax, &ay, &az);
    filt_x = ax;
    filt_y = ay;
    filt_z = az;

    last_sample_ms = millis();
    last_print_ms = millis();

    Serial.println("Hold chip flat. Then lift slowly upward.");
    Serial.println("Format: az_net (m/s²) | vz (m/s) | |v| (m/s)");
}

void loop() {
    uint32_t now = millis();

    if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
        last_sample_ms = now;

        float ax, ay, az;
        readAccel(&ax, &ay, &az);

        // EMA filter
        filt_x = EMA_ALPHA * ax + (1.0f - EMA_ALPHA) * filt_x;
        filt_y = EMA_ALPHA * ay + (1.0f - EMA_ALPHA) * filt_y;
        filt_z = EMA_ALPHA * az + (1.0f - EMA_ALPHA) * filt_z;

        // Subtract gravity (chip assumed flat)
        float ax_net = filt_x - GRAVITY_X;
        float ay_net = filt_y - GRAVITY_Y;
        float az_net = filt_z - GRAVITY_Z;

        // Euler integration: v += a * dt
        vel_x += ax_net * DT_S;
        vel_y += ay_net * DT_S;
        vel_z += az_net * DT_S;
    }

    if (now - last_print_ms >= PRINT_PERIOD_MS) {
        last_print_ms = now;

        float az_net = filt_z - GRAVITY_Z;
        float v_mag = sqrt(vel_x*vel_x + vel_y*vel_y + vel_z*vel_z);

        Serial.print("az_net: ");
        Serial.print(az_net, 3);
        Serial.print("  vz: ");
        Serial.print(vel_z, 3);
        Serial.print("  |v|: ");
        Serial.println(v_mag, 3);
    }
}
