#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define SDA_PIN 4
#define SCL_PIN 5
#define MPU_ADDR 0x68

#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_WHO_AM_I     0x75

void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    return Wire.read();
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== RAW MPU6050 DRIVER (ESP32-S3) ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    delay(100);

    uint8_t who = readReg(REG_WHO_AM_I);
    Serial.print("WHO_AM_I = 0x");
    Serial.println(who, HEX);

    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);  // ±8g
    delay(100);

    Serial.println("Streaming:");
}

void loop() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)6);

    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read();

    const float G = 9.81f;
    float ax = (raw_x / 4096.0f) * G;
    float ay = (raw_y / 4096.0f) * G;
    float az = (raw_z / 4096.0f) * G;
    float mag = sqrt(ax*ax + ay*ay + az*az);

    Serial.print("Accel X: ");
    Serial.print(ax, 2);
    Serial.print("  Y: ");
    Serial.print(ay, 2);
    Serial.print("  Z: ");
    Serial.print(az, 2);
    Serial.print("  |a|: ");
    Serial.print(mag, 2);
    Serial.println(" m/s^2");

    delay(200);
}
