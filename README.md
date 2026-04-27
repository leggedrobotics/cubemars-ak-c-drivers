# ak-c-drivers

C driver library for CubeMars AK series motors over CAN bus.
Supports both MIT mode and Servo mode communication protocols.

---

## Overview

* Provides message generation and decoding for CubeMars AK brushless motors
* Intended for robotics engineers and embedded developers using Linux with SocketCAN
* Fills the gap between the manufacturer PDF and working code — no dependencies, no bloat

---

## Features

* MIT mode: position/velocity/torque control with configurable gains (kp, kd, t_ff)
* Servo mode: duty cycle, current, RPM, position, and position+speed control
* CAN feedback decoding (position, speed, current, temperature, error)
* Supports 10 motor models out of the box (AK10-9, AK40-10, AK60-6, AK70-10, AK80-6/8/9/64, AK45-10/36)
* Interactive ncurses TUI demo with live CAN monitoring

---

## Tech Stack

* Language: C17
* Infrastructure: Linux SocketCAN
* Other tools: CMake 3.28+, ncurses (demo only)

---

## Installation

```bash
# Build library and demo
cmake -B build
cmake --build build
```

The static library `libcubemars-drivers.a` and the `demo` binary will be placed in `build/`.

To link the library in your own project:

```cmake
add_subdirectory(ak-c-drivers)
target_link_libraries(your_target PRIVATE cubemars-drivers)
```

---

## Configuration

No environment variables are required. The CAN interface name and motor ID are set at runtime (either in code or via the demo TUI).

To bring up a CAN interface before use:

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

## Usage

### Servo mode — send a position command

```c
#include "ak_servo.h"
#include <linux/can.h>

uint8_t motor_id = 1;
struct can_frame frame;
AKMotorServoMessage msg;

msg = generate_position_message(motor_id, 90.0f);  // 90 degrees
frame.can_id  = msg.id | CAN_EFF_FLAG;
frame.can_dlc = 8;
memcpy(frame.data, msg.data, 8);
// write(can_socket, &frame, sizeof(frame));
```

### MIT mode — torque-controlled command

```c
#include "ak_mit.h"
#include <linux/can.h>

uint8_t motor_id = 1;
MotorModel model = AK80_9;   // pick your motor
struct can_frame frame;
AKMotorMITMessage msg;

// Enter MIT mode first
msg = generate_mit_enter_message(motor_id);
// ... write to socket ...

// Send control command: hold position 0 with soft gains
msg = generate_mit_command_message(motor_id, model,
        /*p_des=*/0.0f, /*v_des=*/0.0f,
        /*kp=*/5.0f,    /*kd=*/0.5f,
        /*t_ff=*/0.0f);
frame.can_id  = msg.id;
frame.can_dlc = 8;
memcpy(frame.data, msg.data, 8);
// write(can_socket, &frame, sizeof(frame));
```

### Decoding feedback (Servo mode)

```c
#include "ak_servo.h"

struct can_frame rx;
// read(can_socket, &rx, sizeof(rx));

ServoCANFeedback fb = decode_servo_can_feedback(rx.data);
printf("pos=%.2f deg  speed=%d ERPM  current=%.2f A  temp=%d°C\n",
       fb.position, fb.speed, fb.current, fb.temperature);
```

---

