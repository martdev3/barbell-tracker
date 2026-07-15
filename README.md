# Barbell Velocity Tracker
This is a wireless device that is placed onto a barbell and senses when a rep is completed. After the rep is completed, it provides the maximum velocity of the rep. This device is for weight lifters to know when their reps are slowing down, as this is a good way to measure fatigue within a workout.

## Image
<img width="768" height="1024" alt="618D94D6-9975-4062-84E4-2744F479BF2A_1_105_c" src="https://github.com/user-attachments/assets/4743dca9-1ca3-49aa-8ee4-a9e666cf25f2" />


## Hardware
- Materials: ESP32-S3, MPU6050, jumper wires, breadboard, USB type C wire that supports data transmission. 
- Wiring Table
| MPU6050 Pin | ESP32-S3 Pin |
|-------------|--------------|
| VCC         | 3V3          |
| GND         | GND          |
| SDA         | IO4          |
| SCL         | IO5          |

## Software Setup
1. Install VS Code and the PlatformIO extension
2. Clone this repo, open the folder in VS Code
3. Wire the MPU6050 per the table above; connect ESP32 via USB-C
4. In PlatformIO: click Build (checkmark), then Upload (right arrow)
5. Open Serial Monitor at 115200 baud
6. Hold chip still during 2-second calibration; then do reps

## How It Works
- Sensor sampling at 100 Hz
- EMA filter for noise
- Static gravity calibration at boot
- Rep state machine (still → moving → still)
- BLE peripheral broadcasts per-rep peak velocity

## BLE Protocol
- Service UUID: 12345678-1234-5678-1234-56789abcdef0
- Characteristic UUID: abcd1234-5678-1234-5678-1234567890ab
- Properties: READ, NOTIFY
- Data format: 32-bit float, little-endian, units of m/s (peak velocity of most recent rep)
- How to connect (Web Bluetooth example page in /webtest/)

## Known Limitations
- Prototype Hardware: breadboard-mounted MPU6050 with jumper wires. I2C reliability during rapid morion is limited by physical wiring. A PCB build would solve this.
- Velocity accuracy is approximate (~±30-50%)
- Bar rotation during motion not compensated (accel-only, no full sensor fusion)
- Designed for non-rotational lifts (squat, bench, deadlift)
- Static gravity calibration — must hold still at boot
- Reps > 3.0 m/s are auto-rejected, assuming they are products of drift.

## Future Work
- 9-axis IMU (BNO085) upgrade from MPU6050
- Use the orientation of the chip to determine the gravitational acceleration vectors to cancel
- 3D-printed enclosure and bar attachment
- PCB build with soldered connections

## What I Learned
- Exponential Moving Average (EMA) can be used to smooth out noise by dictating how fast the response is to movement. This is memory-advantageous, as it only requires 1 float of memory.
- Drift can be a problem when integrating input from a sensor, so it is often necessary to find ways to attenuate or work around that. In this instance, the MPU6050 senses acceleration on the sensor, which at rest includes gravity's contribution (9.81 m/s² pointing away from earth's center in the sensor's frame). Calibration upon boot can help find the orientation of the sensor to start, but it is not exact, and slight rotation changed the gravitational acceleration vectors, which cause drift. The MPU6050 has a 3 axis gyroscope, so if the device's orientation can be calculated at each sample, it can more precisely remove gravitational acceleration and improve accuracy.
- Bluetooth Low Energy (BLE) includes Generic Access Profile (GAP), which advertises a device's name, MAC address. and basic connectability. Within BLE, the peripheral device (in this case, the ESP32) pushes the value to the central device, instead of the central device searching for the value every few milliseconds. 
- When building a hardware project, it is a good rule of thumb to start basic by setting up the hardware in a way that works, and then building features on top of that. Only after the devices returns the desired information should software be built to handle and process the information. This is mainly for a one-person team who is unable to allocate themselves to multiple things at once.

## License
MIT License. See LICENSE file.
