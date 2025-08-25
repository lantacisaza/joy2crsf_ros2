# joy2crsf_ros2 ## 📖 Description joy2crsf_ros2 is a ROS 2 package that allows you to control a drone through **ELRS (Crossfire/CRSF protocol)** using a **joystick** connected to your PC. It converts joystick inputs into CRSF channel packets and sends them via serial to an ELRS transmitter module. ## ✨ Features - Joystick → CRSF channel mapping (roll, pitch, throttle, yaw, arm, angle, aux) - Serial communication with ELRS at 115200 baud - Telemetry reading (RSSI, Link statistics, GPS, Attitude, etc.) - ROS 2 sensor_msgs/msg/Joy subscription - Toggle arming and flight mode with joystick buttons ## 🛠 Requirements - ROS 2 Humble (or newer) - Python 3.8+ - Packages: - rclpy - sensor_msgs - pyserial ## ⚙️ Build & Usage Clone into your workspace and build with colcon:
bash
cd ~/ros2_ws/src
git clone https://github.com/lantacisaza/joy2crsf_ros2.git
cd ~/ros2_ws
colcon build
source install/setup.bash
Now you can use the joystick and run the node:

### 1. Start the joystick driver  
Tested with a **Logitech joystick**. In one terminal, run the standard ROS 2 joystick driver:

```bash
ros2 run joy joy_node
2. (Optional) Monitor joystick messages

In another terminal, you can check raw joystick data:

ros2 topic echo /joy

3. Run the joystick → CRSF node

In a new terminal, after sourcing your workspace:

ros2 run crsf_joystick joy_to_crsf