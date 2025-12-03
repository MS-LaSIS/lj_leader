# LJ Leader - LabJack Steering Reader

This ROS2 package reads the current steering angle from a LabJack T7 device and publishes it to a ROS2 topic.

## Overview

The `lj_leader` node reads analog voltages from the LabJack T7 DACs that control the steering system and converts them back to steering angles in degrees. This allows monitoring the actual steering position of the vehicle.

## Hardware Requirements

- LabJack T7 device
- Steering control system connected to LabJack DACs:
  - TDAC0: Steering Master 1
  - TDAC1: Steering Master 2
  - TDAC2: Steering Slave 1
  - TDAC3: Steering Slave 2
  - AIN10: Nominal voltage reference for steering master
  - AIN12: Nominal voltage reference for steering slave

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select lj_leader
source install/setup.bash
```

## Usage

### Run with default parameters

```bash
ros2 launch lj_leader lj_leader.launch.py
```

### Run with custom parameters

```bash
ros2 launch lj_leader lj_leader.launch.py \
  steering_topic:=/leader/steering \
  publish_rate:=50.0 \
  max_steering_angle:=29.74
```

### Run node directly

```bash
ros2 run lj_leader lj_leader
```

## Published Topics

- `/leader/steering` (std_msgs/Float32) - Current steering angle in degrees
  - Positive values = right turn
  - Negative values = left turn
  - Range: -29.74° to +29.74° (default max angle for Melex vehicle)

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steering_topic` | string | `/leader/steering` | Topic name for publishing steering angle |
| `publish_rate` | double | 50.0 | Publishing frequency in Hz |
| `max_steering_angle` | double | 29.74 | Maximum steering angle in degrees |
| `steering_master1_pin` | string | `TDAC0` | LabJack pin for steering master 1 |
| `steering_master2_pin` | string | `TDAC1` | LabJack pin for steering master 2 |
| `steering_slave1_pin` | string | `TDAC2` | LabJack pin for steering slave 1 |
| `steering_slave2_pin` | string | `TDAC3` | LabJack pin for steering slave 2 |
| `nominal_vs_steer_master_pin` | string | `AIN10` | Nominal voltage reference for master |
| `nominal_vs_steer_slave_pin` | string | `AIN12` | Nominal voltage reference for slave |
| `input_voltage_min` | double | 0.1 | Minimum input voltage |
| `input_voltage_max` | double | 5.26 | Maximum input voltage |

## Monitoring

### View steering angle

```bash
ros2 topic echo /leader/steering
```

### Plot steering angle

```bash
ros2 run rqt_plot rqt_plot /leader/steering/data
```

### Check publish rate

```bash
ros2 topic hz /leader/steering
```

## How It Works

1. **Reads DAC voltages**: The node reads the current output voltages from the LabJack DACs that control the steering
2. **Reads nominal voltages**: Reads reference voltages to normalize the readings
3. **Calculates ratio**: Computes the voltage ratio to determine steering position
4. **Maps to angle**: Converts the voltage ratio back to steering angle in degrees using the inverse of the mapping used in `lj_handler`
5. **Publishes**: Sends the angle to the ROS2 topic at the specified rate

## Coordinate System

The steering angle follows the same convention as `lj_handler`:
- **Positive angle**: Right turn
- **Negative angle**: Left turn
- **Zero**: Straight ahead

## Debug Mode

Enable detailed logging:

```bash
DEBUG=true ros2 launch lj_leader lj_leader.launch.py
```

## Troubleshooting

### LabJack not found

```
Failed to open LabJack T7
```

**Solution**: Check USB connection and ensure LabJack drivers are installed.

### Incorrect readings

Check that:
1. Pin configurations match your hardware setup
2. Nominal voltage references are correctly connected
3. Voltage range parameters match your system

### Permissions

If you get permission errors accessing the LabJack:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Integration with lj_handler

This package is designed to work alongside `lj_handler_pkg`:
- `lj_handler` writes steering commands (follower mode)
- `lj_leader` reads actual steering position (leader mode)

They can run simultaneously on the same LabJack device.

## Example: Leader-Follower System

Terminal 1 (Leader - reads steering):
```bash
ros2 launch lj_leader lj_leader.launch.py
```

Terminal 2 (Follower - writes steering):
```bash
ros2 launch lj_handler_pkg lj_handler.launch.py
```

Terminal 3 (Monitor):
```bash
ros2 topic echo /leader/steering
```
