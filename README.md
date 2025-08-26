# joy2crsf_ros2
---

##  Description
`joy2crsf_ros2` is a ROS 2 package that allows you to control a drone through **ELRS (Crossfire/CRSF protocol)** using a **joystick** connected to your PC.  
It converts joystick inputs into CRSF channel packets and sends them via serial to an ELRS transmitter module.

---

## ✨ Features
- Joystick → CRSF channel mapping (roll, pitch, throttle, yaw, arm, angle, aux)
- Serial communication with ELRS at 115200 baud
- Telemetry reading (RSSI, Link statistics, GPS, Attitude, etc.)
- ROS 2 `sensor_msgs/msg/Joy` subscription
- Toggle arming and flight mode with joystick buttons

---
## 🛠 Requirements
- ROS 2 Humble (or newer)
- Python 3.8+
- Packages:
  - `rclpy`
  - `sensor_msgs`
  - `pyserial`

---
## ⚙️ Build
Clone into your workspace and build with `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/lantacisaza/joy2crsf_ros2.git
cd ~/ros2_ws
colcon build 
source install/setup.bash
ros2 run crsf_joystick joy_to_crsf
```
---

## Usage

### 1. Start the joystick driver. Logitech joystick was used.
In one terminal, run the standard ROS2 joystick driver:
```bash
ros2 run joy joy_node
```
### 2. (Optional) Monitor joystick messages
In another terminal, you can check raw joystick data:
```bash
ros2 topic echo /joy
```
### 3. Run the joystick → CRSF node
```bash
colcon build 
source install/setup.bash
ros2 run crsf_joystick joy_to_crsf
```

## License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.

Portions of the CRSF parsing logic are adapted from the [CRSF Working Group](https://github.com/crsf-wg) repository (MIT License).
