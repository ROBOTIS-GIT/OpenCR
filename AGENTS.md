# OpenCR TurtleBot3 Firmware Project Guide

## Project Overview

This is an OpenCR board firmware project for TurtleBot3 robots. The firmware is built using the Arduino framework and provides ROS2 communication capabilities through a DYNAMIXEL serial protocol.

## Project Structure

### Key Directories

- **`arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/`** - Main TurtleBot3 ROS2 library
- **`arduino/opencr_arduino/opencr/libraries/`** - All Arduino libraries including OpenCR, sensors, etc.

### Core Files You'll Work With

#### Main Sketch File
- **`arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/examples/turtlebot3_burger/turtlebot3_burger.ino`**
  - Minimal Arduino sketch that just calls TurtleBot3Core::begin() and TurtleBot3Core::run()
  - This is what gets uploaded to the OpenCR board

#### Core Implementation Files
- **`arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3.cpp`** - Main firmware logic
- **`arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/include/turtlebot3/turtlebot3.h`** - Main header

#### Component Files (Header + Implementation pairs)
- **`turtlebot3_sensor.*`** - IMU, battery, buttons, analog pins, sound
- **`turtlebot3_motor_driver.*`** - DYNAMIXEL wheel motor control  
- **`turtlebot3_diagnosis.*`** - LED status, voltage monitoring
- **`turtlebot3_controller.*`** - RC100 remote controller interface
- **`open_manipulator_driver.*`** - Manipulator arm support (Waffle model)

### How The Code Works

1. **Arduino Entry Point**: The `.ino` file includes `TurtleBot3_ROS2.h` which includes `turtlebot3.h`
2. **Initialization**: `TurtleBot3Core::begin()` initializes all subsystems (motors, sensors, communication)  
3. **Main Loop**: `TurtleBot3Core::run()` handles all periodic tasks:
   - ROS2 communication via DYNAMIXEL protocol
   - Sensor reading (IMU, analog pins, buttons) 
   - Motor control
   - Status updates

### DYNAMIXEL Communication Protocol

The OpenCR acts as a DYNAMIXEL slave device (ID 200) that communicates with ROS2 nodes. Key addresses in the control table:

- **Analog Pins**: `ADDR_ANALOG_A0` (30) through `ADDR_ANALOG_A5` (40) - 2 bytes each, 12-bit ADC values
- **IMU Data**: `ADDR_ANGULAR_VELOCITY_X` (60) through `ADDR_ORIENTATION_Z` (108) - 4 bytes each, float values
- **Motor Control**: `ADDR_CMD_VEL_LINEAR_X` (150) through `ADDR_CMD_VEL_ANGULAR_Z` (170) - 4 bytes each
- **Status**: `ADDR_DEVICE_READY` (17), `ADDR_CONNECT_ROS2` (15), etc.

### Timing and Updates

- **Motor Control**: 20ms intervals (`INTERVAL_MS_TO_CONTROL_MOTOR`)
- **Sensor Updates**: 20ms intervals (`INTERVAL_MS_TO_UPDATE_CONTROL_ITEM`) 
- **Analog Pins**: 20ms intervals (`INTERVAL_MS_TO_UPDATE_APINS`)

## Development Guidelines

### Adding New Sensors

1. **Update Headers**: Add sensor declarations to appropriate header file (usually `turtlebot3_sensor.h`)
2. **Add Control Table Entries**: Define new `ADDR_*` constants in `turtlebot3.cpp`
3. **Initialize in begin()**: Add sensor initialization code in `TurtleBot3Core::begin()`
4. **Add Update Function**: Create `update_*()` function following existing patterns
5. **Call from run()**: Add your update function call in `TurtleBot3Core::run()`

### Sensor Reading Patterns

All sensor update functions follow this pattern:
```cpp
void update_sensor_name(uint32_t interval_ms)
{
  static uint32_t pre_time = 0;
  
  if(millis() - pre_time >= interval_ms){
    pre_time = millis();
    // Read sensor and update control_items
  }
}
```

### Memory Layout

- Use `control_items` struct to store all data shared with ROS2
- Follow existing naming: `control_items.sensor_name` or `control_items.sensor_array[index]`
- 2-byte values for integers, 4-byte for floats

## Build and Deploy Commands

```bash
# Navigate to project root directory
cd /home/trav/ez-tb3-project/OpenCR

# Compile and upload to OpenCR board
arduino-cli compile --upload -v -p /dev/ttyACM0 --fqbn OpenCR:OpenCR:OpenCR --libraries=$(pwd)/arduino/opencr_arduino/opencr/libraries arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/examples/turtlebot3_burger/turtlebot3_burger.ino

# Alternative: Just compile (no upload)
arduino-cli compile -v --fqbn OpenCR:OpenCR:OpenCR --libraries=$(pwd)/arduino/opencr_arduino/opencr/libraries arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/examples/turtlebot3_burger/turtlebot3_burger.ino
```

### Command Explanation
- `--fqbn OpenCR:OpenCR:OpenCR` -> Fully Qualified Board Name Package:Architecture:Board. Specifies OpenCR board (not standard Arduino)
- `--libraries=$(pwd)/arduino/...`: Points to custom OpenCR libraries
- `-v`: Verbose output for debugging compilation issues
- `-p /dev/ttyACM0`: Upload port (may vary, check with `ls /dev/ttyACM*`)

## Debug Output

- Debug output goes to `Serial` (USB serial - viewable in Arduino IDE Serial Monitor)
- Use `DEBUG_PRINTLN()` macros (enabled when `DEBUG_ENABLE = 1`)
- Monitor via Arduino IDE Serial Monitor at 57600 baud
- **Note**: Original firmware used `SerialBT2` (Bluetooth), modified for easier debugging

## Key Dependencies

- **DynamixelSDK**: For DYNAMIXEL communication protocol
- **IMU**: For inertial measurement unit support  
- **OLLO**: For OLLO sensor/actuator support
- **OpenCR**: For OpenCR board-specific functions

## Troubleshooting

- **Motor Connection**: Check `is_connected()` status in debug output
- **ROS2 Communication**: Monitor `ADDR_CONNECT_ROS2` and heartbeat
- **Sensor Issues**: Check power supply and wiring connections
