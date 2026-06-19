# Embedded-RC-Flight-Control
A high-performance **Embedded RC Flight Controller** for RC race bots and robotic platforms. This project combines **RadioMaster ELRS receiver inputs**, **dual brushed ESCs**, and an **MPU-9250 IMU** to provide smooth motor control, real-time yaw stabilization, and reliable failsafe protection.

The controller is designed to improve driving stability by compensating for motor speed mismatch, gearbox tolerances, and external disturbances while preserving responsive manual steering.

---

## Features

* ⚡ High-speed interrupt-based RC signal decoding
* 🎮 Compatible with RadioMaster ELRS receivers
* 🚗 Differential drive motor control
* 🧭 Real-time gyroscope-based yaw stabilization
* ⚙️ PD feedback controller for smooth correction
* 🔄 Dual brushed ESC support
* 🛡️ Watchdog failsafe for signal loss
* 📉 Slew-rate limiting for smoother acceleration
* 📡 50 Hz real-time control loop
* 🧪 Automatic gyroscope bias calibration during startup
* 📊 Serial telemetry for debugging and tuning

---

## Hardware Used

| Component                      | Description                        |
| ------------------------------ | ---------------------------------- |
| ESP32 Dev Module               | Main controller                    |
| MPU-9250                       | 3-axis gyroscope and accelerometer |
| RadioMaster Transmitter        | RC controller                      |
| ELRS Micro Receiver            | PWM signal receiver                |
| Dual Brushed ESC               | Motor driver                       |
| Johnson Grade A 400 RPM Motors | Differential drive motors          |
| Li-Po Battery                  | Power source                       |

---

## Pin Configuration

| ESP32 Pin | Function                          |
| --------- | --------------------------------- |
| GPIO 25   | Receiver CH3 (Throttle)           |
| GPIO 26   | Receiver CH4 (Steering)           |
| GPIO 27   | Receiver CH5 (Gyro Enable Switch) |
| GPIO 18   | Left ESC Output                   |
| GPIO 19   | Right ESC Output                  |
| GPIO 21   | I2C SDA                           |
| GPIO 22   | I2C SCL                           |

---

## Working Principle

1. The ELRS receiver sends PWM signals to the ESP32.
2. Hardware interrupts measure each PWM pulse with minimal latency.
3. The MPU-9250 continuously measures the robot's yaw rate.
4. A PD controller compares the desired turning rate with the measured yaw rate.
5. The controller adjusts the left and right motor outputs to maintain stable motion.
6. ESC outputs are updated every control cycle while slew-rate limiting ensures smooth acceleration.
7. If receiver signals are lost, the watchdog immediately returns both ESCs to the neutral position.

---

## Software Architecture

```
RadioMaster Transmitter
          │
          ▼
    ELRS Receiver
          │
          ▼
ESP32 Interrupt-Based PWM Decoder
          │
          ├─────────────► Throttle & Steering Inputs
          │
          ▼
     MPU-9250 Gyroscope
          │
          ▼
      PD Stabilization
          │
          ▼
 Differential Motor Mixer
          │
          ▼
 Dual Brushed ESC Outputs
          │
          ▼
     Left & Right Motors
```

---

## Control Modes

### Manual Mode

* Direct RC control
* Gyroscope stabilization disabled
* Suitable for testing and tuning

### Stabilized Mode

* Gyroscope feedback enabled
* Automatically compensates for unwanted yaw motion
* Helps reduce drift caused by:

  * Motor speed mismatch
  * Gearbox tolerances
  * Uneven traction
  * Battery voltage variations
  * External disturbances

---

## Safety Features

* Receiver signal watchdog
* Automatic neutral output during signal loss
* ESC slew-rate limiting
* Gyroscope deadband filtering
* Startup gyro calibration
* Invalid sensor reading rejection

---

## Serial Monitor Output

Example:

```
MODE: STABILIZED | GYRO: -1.4 || L_OUT: 1648 | R_OUT: 1532
```

Information displayed:

* Current control mode
* Measured yaw rate
* Left ESC output
* Right ESC output

---

## Future Improvements

* Heading-hold controller
* PID auto-tuning
* Wheel encoder feedback
* Adaptive motor trim
* Sensor fusion using complementary/Kalman filtering
* Data logging to SD card
* Bluetooth tuning interface
* OLED status display

---

## Applications

* RC race bots
* Differential drive robots
* Autonomous robotic platforms
* Educational robotics
* Research and control system development
* Embedded systems projects

---

## Repository Structure

```
├── Gyro_Controller.ino
├── README.md
├── images/
├── docs/
└── LICENSE
```

---

## License

This project is released under the MIT License.

---

## Author

**Thayanithy S**

Electronics and Communication Engineering
Sri Manakula Vinayagar Engineering College

---

## Acknowledgements

Special thanks to the open-source Arduino, ESP32, and robotics communities for their excellent libraries and development tools that made this project possible.
