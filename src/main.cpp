#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Sampling
const uint16_t SAMPLE_PERIOD_MS = 10;
const uint16_t BUFFER_SIZE = 100;
const uint16_t PRINT_PERIOD_MS = 1000;

// EMA filter coefficient: smaller = more smoothing
const float EMA_ALPHA = 0.10f;

// Filtered axis values (persist across loop iterations)
float filt_x = 0;
float filt_y = 0;
float filt_z = 0;

// Buffers for both raw and filtered magnitudes
float raw_mag_buf[BUFFER_SIZE];
float filt_mag_buf[BUFFER_SIZE];
uint16_t buf_idx = 0;
bool buf_filled = false;

uint32_t last_sample_ms = 0;
uint32_t last_print_ms = 0;
uint32_t samples_this_second = 0;

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

void computeStats(const float* buf, uint16_t count, float* mean, float* min_v, float* max_v) {
    if (count == 0) {
        *mean = 0; *min_v = 0; *max_v = 0;
        return;
    }
    float sum = 0;
    float mn = buf[0];
    float mx = buf[0];
    for (uint16_t i = 0; i < count; i++) {
        sum += buf[i];
        if (buf[i] < mn) mn = buf[i];
        if (buf[i] > mx) mx = buf[i];
    }
    *mean = sum / count;
    *min_v = mn;
    *max_v = mx;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== DAY 3: EMA FILTER ===");
    Serial.print("EMA alpha = ");
    Serial.println(EMA_ALPHA, 3);

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);
    delay(100);

    // Prime the filter with one read so we don't start at zero
    // (otherwise filtered value spends ~30 samples climbing to gravity)
    float ax, ay, az;
    readAccel(&ax, &ay, &az);
    filt_x = ax;
    filt_y = ay;
    filt_z = az;

    last_sample_ms = millis();
    last_print_ms = millis();

    Serial.println("Format: samples/s | RAW: mean spread | FILT: mean spread");
}

void loop() {
    uint32_t now = millis();

    if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
        last_sample_ms = now;

        float ax, ay, az;
        readAccel(&ax, &ay, &az);

        // Raw magnitude
        float raw_mag = sqrt(ax*ax + ay*ay + az*az);

        // EMA filter on each axis
        filt_x = EMA_ALPHA * ax + (1.0f - EMA_ALPHA) * filt_x;
        filt_y = EMA_ALPHA * ay + (1.0f - EMA_ALPHA) * filt_y;
        filt_z = EMA_ALPHA * az + (1.0f - EMA_ALPHA) * filt_z;
        float filt_mag = sqrt(filt_x*filt_x + filt_y*filt_y + filt_z*filt_z);

        raw_mag_buf[buf_idx] = raw_mag;
        filt_mag_buf[buf_idx] = filt_mag;
        buf_idx = (buf_idx + 1) % BUFFER_SIZE;
        if (buf_idx == 0) buf_filled = true;

        samples_this_second++;
    }

    if (now - last_print_ms >= PRINT_PERIOD_MS) {
        last_print_ms = now;

        uint16_t count = buf_filled ? BUFFER_SIZE : buf_idx;
        float raw_mean, raw_min, raw_max;
        float filt_mean, filt_min, filt_max;
        computeStats(raw_mag_buf, count, &raw_mean, &raw_min, &raw_max);
        computeStats(filt_mag_buf, count, &filt_mean, &filt_min, &filt_max);

        Serial.print("samples/s: ");
        Serial.print(samples_this_second);
        Serial.print("  RAW mean=");
        Serial.print(raw_mean, 3);
        Serial.print(" spread=");
        Serial.print(raw_max - raw_min, 3);
        Serial.print("  FILT mean=");
        Serial.print(filt_mean, 3);
        Serial.print(" spread=");
        Serial.println(filt_max - filt_min, 3);

        samples_this_second = 0;
    }
}
