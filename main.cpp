#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

Adafruit_MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {
        delay(10);
    }

    Wire.begin(9, 8);  // SDA = GPIO 8, SCL = GPIO 9

    if (!mpu.begin()) {
        Serial.println("ERROR: Could not find MPU6050. Check wiring!");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 initialized.");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(100);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    float magnitude = sqrt(ax*ax + ay*ay + az*az);

    Serial.print("Accel X: ");
    Serial.print(ax, 2);
    Serial.print("  Y: ");
    Serial.print(ay, 2);
    Serial.print("  Z: ");
    Serial.print(az, 2);
    Serial.print("  |a|: ");
    Serial.print(magnitude, 2);
    Serial.println(" m/s^2");

    delay(200);
}