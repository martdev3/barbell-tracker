#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

// Sampling configuration
const uint16_t SAMPLE_PERIOD_MS = 10;      // 100 Hz
const uint16_t BUFFER_SIZE = 100;          // 1 second of samples
const uint16_t PRINT_PERIOD_MS = 1000;     // print stats every 1 sec

// Circular buffer for magnitude readings
float mag_buf[BUFFER_SIZE];
uint16_t buf_idx = 0;
bool buf_filled = false;

// Timing
uint32_t last_sample_ms = 0;
uint32_t last_print_ms = 0;
uint32_t samples_this_second = 0;

//Max spread
float lifetime_max_spread = 0;


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
    Serial.println("=== DAY 2: 100 Hz SAMPLING + BUFFER ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    writeReg(REG_PWR_MGMT_1, 0x00);  // wake up
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);  // ±8g range
    delay(100);

    Serial.println("Streaming summary stats once per second:");
}

void loop() {
    uint32_t now = millis();

    // Sample at 100 Hz
    if (now - last_sample_ms >= SAMPLE_PERIOD_MS) {
        last_sample_ms = now;

        float ax, ay, az;
        readAccel(&ax, &ay, &az);
        float mag = sqrt(ax*ax + ay*ay + az*az);

        mag_buf[buf_idx] = mag;
        buf_idx = (buf_idx + 1) % BUFFER_SIZE;
        if (buf_idx == 0) buf_filled = true;

        samples_this_second++;
    }

    // Print stats once per second
    if (now - last_print_ms >= PRINT_PERIOD_MS) {
        last_print_ms = now;

        uint16_t count = buf_filled ? BUFFER_SIZE : buf_idx;
        float mean_mag, min_mag, max_mag;
        computeStats(mag_buf, count, &mean_mag, &min_mag, &max_mag);
        float current_spread = max_mag - min_mag;
        if (current_spread > lifetime_max_spread) {
            lifetime_max_spread = current_spread;
        }

        Serial.print("samples/sec: ");
        Serial.print(samples_this_second);
        Serial.print("   samples in buffer: ");
        Serial.print(count);
        Serial.print("   |a| mean: ");
        Serial.print(mean_mag, 3);
        Serial.print("   min: ");
        Serial.print(min_mag, 3);
        Serial.print("   max: ");
        Serial.print(max_mag, 3);
        Serial.print("   spread: ");
        Serial.println(current_spread);
        Serial.print("   max spread: ");
        Serial.print(lifetime_max_spread, 3);

        samples_this_second = 0;
    }
}
