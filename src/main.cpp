#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
const float MAX_REP_VELOCITY = 3.0f;
const uint32_t MAX_REP_DURATION_MS = 2500;

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

BLECharacteristic* pRepCharacteristic = nullptr;
bool ble_connected = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        ble_connected = true;
        Serial.println("BLE: central connected");
    }
    void onDisconnect(BLEServer* pServer) override {
        ble_connected = false;
        Serial.println("BLE: central disconnected");
        pServer->getAdvertising()->start();
    }
};

void writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

bool readAccel(float* ax, float* ay, float* az) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(MPU_ADDR, (int)6) != 6) return false;

    int16_t raw_x = (Wire.read() << 8) | Wire.read();
    int16_t raw_y = (Wire.read() << 8) | Wire.read();
    int16_t raw_z = (Wire.read() << 8) | Wire.read();

    const float G = 9.81f;
    *ax = (raw_x / 4096.0f) * G;
    *ay = (raw_y / 4096.0f) * G;
    *az = (raw_z / 4096.0f) * G;
    return true;
}

void calibrateGravity() {
    Serial.println("Calibrating gravity - hold STILL 2 seconds...");
    delay(500);
    const int N = 200;
    float sx = 0, sy = 0, sz = 0;
    int good = 0;
    for (int i = 0; i < N; i++) {
        float ax, ay, az;
        if (readAccel(&ax, &ay, &az)) {
            sx += ax; sy += ay; sz += az;
            good++;
        }
        delay(10);
    }
    if (good > 0) {
        gravity_x = sx / good;
        gravity_y = sy / good;
        gravity_z = sz / good;
    }
    gravity_mag = sqrt(gravity_x*gravity_x + gravity_y*gravity_y + gravity_z*gravity_z);
    Serial.print("Calibrated |g|="); Serial.println(gravity_mag, 2);
}

void initBLE() {
    BLEDevice::init("BarbellTracker");
    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService("12345678-1234-5678-1234-56789abcdef0");
    pRepCharacteristic = pService->createCharacteristic(
        "abcd1234-5678-1234-5678-1234567890ab",
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pRepCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    Serial.println("BLE ready: advertising as 'BarbellTracker'");
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("=== BARBELL TRACKER ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);
    delay(100);
    writeReg(REG_PWR_MGMT_1, 0x00);
    delay(100);
    writeReg(REG_ACCEL_CONFIG, 0x10);
    delay(100);

    calibrateGravity();

    float ax, ay, az;
    if (readAccel(&ax, &ay, &az)) {
        filt_x = ax; filt_y = ay; filt_z = az;
    }

    last_sample_ms = millis();
    initBLE();
    Serial.println("Ready. Do reps.");
}

void loop() {
    uint32_t now = millis();

    if (now - last_sample_ms < SAMPLE_PERIOD_MS) return;
    last_sample_ms = now;

    float ax, ay, az;
    if (!readAccel(&ax, &ay, &az)) return;

    filt_x = EMA_ALPHA * ax + (1.0f - EMA_ALPHA) * filt_x;
    filt_y = EMA_ALPHA * ay + (1.0f - EMA_ALPHA) * filt_y;
    filt_z = EMA_ALPHA * az + (1.0f - EMA_ALPHA) * filt_z;

    float ax_net = filt_x - gravity_x;
    float ay_net = filt_y - gravity_y;
    float az_net = filt_z - gravity_z;

    float raw_mag = sqrt(ax*ax + ay*ay + az*az);
    bool instant_moving = fabs(raw_mag - 9.81f) > MOTION_ACCEL_THRESHOLD;

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
        return;
    }

    if (vert_vel_abs > peak_vert_vel) peak_vert_vel = vert_vel_abs;
    uint32_t rep_duration = now - rep_start_ms;
    bool too_long = rep_duration > MAX_REP_DURATION_MS;

    if (!instant_moving) {
        if (still_since_ms == 0) {
            still_since_ms = now;
        } else if (now - still_since_ms >= STILL_DURATION_MS) {
            bool valid = peak_vert_vel >= MIN_REP_VELOCITY
                      && peak_vert_vel <= MAX_REP_VELOCITY
                      && !too_long;
            if (valid) {
                rep_count++;
                Serial.print(">>> REP ");
                Serial.print(rep_count);
                Serial.print("   peak: ");
                Serial.print(peak_vert_vel, 2);
                Serial.print(" m/s   duration: ");
                Serial.print(rep_duration / 1000.0f, 1);
                Serial.println(" s");

                if (pRepCharacteristic != nullptr) {
                    float value = peak_vert_vel;
                    pRepCharacteristic->setValue((uint8_t*)&value, sizeof(value));
                    pRepCharacteristic->notify();
                }
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
