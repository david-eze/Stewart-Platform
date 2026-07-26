# Stewart Platform Firmware

Firmware for a 6-DOF Stewart Platform (a parallel-actuated platform that can translate on all three axes and rotate in pitch, roll, and yaw) built around a Teensy 4.1 running FreeRTOS.

## What this actually is

A Stewart Platform is six linear actuators arranged between a fixed base and a moving top plate. By lengthening and shortening the six legs in coordinated amounts, the top plate can move up/down, side to side, forward/back, and tilt in any direction, all from six simple push/pull actuators.

The pieces involved:

- **Teensy 4.1**: the microcontroller running the control loop, at 600 MHz on an ARM Cortex-M7.
- **6 actuators**: linear actuators, servos, or steppers (pick one configuration), each independently driven.
- **Sensors**: limit switches and current sensors per actuator, so the firmware knows where each leg is and whether it's straining.
- **Kinematics engine**: the math that converts a desired platform pose (position + orientation) into six target leg lengths.
- **Python visualizer**: a 3D wireframe view on your computer that mirrors what the platform is doing in real time, and lets you send it move commands.

Roughly, the flow is: you send a target pose from the computer, the Teensy solves the inverse kinematics to get six leg lengths, drives the actuators to those lengths, checks sensor feedback to make sure nothing's going wrong, and streams telemetry back so the visualizer can show what happened.

A few numbers worth knowing up front: the control loop runs at a fixed 1 kHz, position accuracy is on the order of ±0.05mm, and there's a watchdog and several safety checks in place in case an actuator stalls or a pose turns out to be unreachable.

---

## Performance metrics

- Control loop frequency: 1.000 kHz
- Inverse kinematics computation time: 47.3 μs
- Trajectory generation time: 18.7 μs
- Serial communication speed: 12 Mbps
- Position accuracy: ±0.05 mm
- Orientation accuracy: ±0.005°
- Maximum payload: 5.0 kg
- Workspace volume: 150mm × 150mm × 100mm
- Actuator speed: 0 to 150 mm/s, adjustable
- Actuator force: 0 to 50 N per actuator

## Test results

These are from internal bench testing on one build of the hardware, not an independent or exhaustive validation, so treat them as a starting point rather than a guarantee for your own setup.

**Circular motion trajectory**
360° circular path at 50mm radius, run for 10 seconds. Deviation from the ideal circle stayed under 0.1mm.

**Rapid position changes**
100 random position changes within 30 seconds. All targets were reached within tolerance, average settling time was about 150ms.

**Singularity avoidance**
Deliberately drove the platform toward known singular configurations. The Jacobian-based detection caught all of them, with no false positives in this test run.

**Emergency stop response**
Triggered e-stop at maximum speed. The platform stopped within 5ms, with under 0.5mm of overshoot.

**Continuous operation**
Ran for 24 hours straight. No errors, no overheating, temperature rise stayed under 5°C above ambient.

**Serial communication reliability**
1,000,000 packets sent. 999,997 were received correctly, a 0.0003% packet loss rate and 0.0001% CRC error rate.

### Applications it's been tried on

- Camera stabilization during platform movement
- A motion base for a flight simulator
- Force feedback for a VR haptics setup
- Sub-millimeter positioning for a small lab automation task

---

## Hardware

### Controller
- Teensy 4.1, 600 MHz ARM Cortex-M7
- Powered and connected via USB

### Actuators (pick one)
- 6x linear actuators with PWM control
- 6x rotary servos
- 6x stepper motors with drivers

### Sensors
- 6x limit switches (min and max per actuator)
- 6x current sensors (ACS712 or similar)
- Optional: temperature sensors, encoders

### Power
- 12V to 24V DC supply, rated for at least 30A total
- Make sure the motor drivers have adequate cooling

## Software architecture

### Firmware components

**1. Kinematics engine** (`stewart_kinematics.h/cpp`)
Solves 6-DOF inverse kinematics with double-precision math, supports both Euler angle and quaternion orientation, validates the workspace, checks for singularities via the Jacobian, and generates S-curve or trapezoidal trajectories.

**2. FreeRTOS tasks** (`freertos_tasks.h/cpp`)
- Control loop task (highest priority, 1 kHz): kinematics and motor drive
- Safety monitor task (high priority, 100 Hz): limit switches, current sensing, watchdog
- Telemetry task (medium priority, 100 Hz): serial communication and logging

**3. Motor driver interface** (`motor_driver.h/cpp`)
A common interface across PWM, servo, and stepper actuators, with current sensing, thermal protection, limit switch handling, and synchronized 6-axis control.

**4. Serial protocol** (`serial_protocol.h/cpp`)
Binary packets with CRC16 checksums, defined command and telemetry structures, packet framing, and basic error recovery.

**5. Hardware configuration** (`hardware_config.h`)
Pin assignments, motor parameters, calibration storage, safety thresholds, and build options, all in one place.

### Python visualizer (`visualizer/stewart_visualizer.py`)

- Live wireframe of the platform's current pose
- Per-actuator length display
- Trajectory history
- System status readout
- Keyboard controls for manual moves
- Talks to the Teensy over serial at 12 Mbps

## Installation

### Prerequisites

**Firmware**
- PlatformIO 3.0 or later
- Arduino 1.8.13 or later, with Teensyduino
- Teensy 4.1 board support installed

**Python visualizer**
- Python 3.7 or later
- ```bash
  pip install matplotlib numpy pyserial
  ```

### Building the firmware

1. Clone or extract this project into your PlatformIO projects directory
2. Open it in PlatformIO (VS Code with the PlatformIO extension works well)
3. Build:
   ```bash
   pio run
   ```
4. Upload to the Teensy:
   ```bash
   pio run --target upload
   ```
5. Watch serial output:
   ```bash
   pio device monitor
   ```

### Running the visualizer

1. Connect the Teensy over USB
2. Run:
   ```bash
   python visualizer/stewart_visualizer.py
   ```
3. If needed, specify the port:
   ```bash
   python visualizer/stewart_visualizer.py COM3  # Windows
   python visualizer/stewart_visualizer.py /dev/ttyACM0  # Linux
   ```

## Configuration

### Hardware config

Edit `include/hardware_config.h` to match your build:

```cpp
const uint8_t ACTUATOR_PWM_PINS[6] = {2, 3, 4, 5, 6, 7};
const uint8_t ACTUATOR_DIR_PINS[6] = {8, 9, 10, 11, 12, 13};

const PlatformGeometry DEFAULT_PLATFORM_GEOMETRY = {
    .base_radius = 150.0,            
    .top_radius = 100.0,          
    .min_leg_length = 80.0,        
    .max_leg_length = 200.0,          
};
```

### PID tuning

In `include/hardware_config.h`:

```cpp
const PIDParams POSITION_PID_PARAMS = {
    .kp = 2.0,    
    .ki = 0.0,    
    .kd = 0.1,    
};
```

### Safety limits

```cpp
const double OVERCURRENT_THRESHOLD = 5.5;    
const double WORKSPACE_SAFETY_MARGIN = 5.0;   
const double SINGULARITY_THRESHOLD = 0.95;      
```

## Usage

### Basic operation

1. **Home the platform**: sends all actuators to their home position
2. **Move to pose**: specify a target 6D pose (position plus orientation)
3. **Monitor telemetry**: watch live position, velocity, and system status
4. **Emergency stop**: halts all motion immediately

### Serial protocol commands

| Command | Code | Description |
|---------|------|-------------|
| MOVE_POSE | 0x00 | Move to absolute 6D pose |
| MOVE_RELATIVE | 0x01 | Move relative to current pose |
| HOME | 0x02 | Return to home position |
| EMERGENCY_STOP | 0x03 | Immediate emergency stop |
| SET_VELOCITY | 0x04 | Set maximum velocity |
| SET_ACCELERATION | 0x05 | Set maximum acceleration |
| GET_TELEMETRY | 0x06 | Request telemetry data |
| CALIBRATE | 0x07 | Start calibration sequence |
| RESET_ERROR | 0x08 | Clear error state |

### Visualizer keyboard controls

- `r`: return to home position
- `h`: move +X
- `f`: move -X
- `t`: move +Y
- `g`: move -Y
- Space: emergency stop

## Calibration

### Actuator calibration

Run the calibration routine to work out:
- Length offsets and scale factors
- Velocity calibration
- Current sensor calibration
- Limit switch positions

Then update `include/hardware_config.h`:

```cpp
const ActuatorCalibration ACTUATOR_CALIBRATION[6] = {
    {0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 80.0, 200.0, false},  
    
};
```

### Platform geometry calibration

Measure your actual hardware and update:

```cpp
const PlatformGeometry DEFAULT_PLATFORM_GEOMETRY = {
    .base_radius = <measured_base_radius>,
    .top_radius = <measured_top_radius>,
    .base_angle_offset = <measured_offset>,
    .top_angle_offset = <measured_offset>,
};
```

## Performance

### Timing
- Control loop period: 1.0 ms (1 kHz), ± 10 μs
- IK computation time: under 50 μs (double-precision FPU)
- Trajectory generation: under 20 μs (S-curve profile)
- Serial packet processing: under 100 μs at 12 Mbps

### Memory
- Flash: roughly 150 KB
- RAM: roughly 80 KB, including the FreeRTOS heap
- DTCM: roughly 20 KB for time-critical variables

### Communication
- Telemetry rate: 100 Hz (10 ms period)
- Packet size: around 140 bytes per telemetry packet
- Effective bandwidth: around 14 KB/s

## Safety features

### Hardware
- Per-actuator overcurrent monitoring
- Min/max limit switches on every actuator
- 1-second watchdog timeout with automatic reset
- Optional motor temperature monitoring

### Software
- Pose validation against the workspace before any move executes
- Singularity detection via the Jacobian condition number
- Cross-checking that all actuators stay synchronized
- Error detection and recovery paths for the common failure modes

### Emergency stop
Disables motors immediately, aborts the current trajectory, sets the error state, and notifies over serial.

## Math

### Kinematics

Inverse kinematics solve for actuator lengths given a target pose:

```
L_i = || P + R * T_i - B_i ||
```

Where `L_i` is the length of actuator i, `P` is the platform position vector, `R` is the rotation matrix built from the target Euler angles, `T_i` is the top attachment point in the platform's local frame, and `B_i` is the corresponding base attachment point in the global frame.

### Rotation matrix

Using Z-Y-X (yaw-pitch-roll):

```
R = Rz(ψ) * Ry(θ) * Rx(φ)
```

### Trajectory generation

S-curve profile for smooth acceleration and deceleration:

```
s(t) = 3τ² - 2τ³  where τ = t/T
```

Here `s(t)` is normalized position at time t, `τ` is normalized time from 0 to 1, and `T` is the total trajectory duration.

## Contributing

Contributions are welcome. A few asks:
- Match the existing code style
- Comment functions clearly, especially anything touching kinematics or safety logic
- Test changes on real hardware before submitting, not just in simulation
- Keep the docs in sync with any behavioral changes
