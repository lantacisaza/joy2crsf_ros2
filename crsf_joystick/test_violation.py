import rclpy
import csv

from docutils.nodes import target
from rclpy.node import Node
from sensor_msgs.msg import Joy
import serial
import time
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from enum import IntEnum
from crsf_joystick.cbf_safeguard import cbf_filter
from crsf_joystick.PID import PID_controller
from crsf_joystick.PID import PID_controller
import csv
CRSF_SYNC = 0xC8
PACKET_TYPE = 0x16
class PacketsTypes(IntEnum):
   GPS = 0x02
   VARIO = 0x07
   BATTERY_SENSOR = 0x08
   BARO_ALT = 0x09
   HEARTBEAT = 0x0B
   VIDEO_TRANSMITTER = 0x0F
   LINK_STATISTICS = 0x14
   RC_CHANNELS_PACKED = 0x16
   ATTITUDE = 0x1E
   FLIGHT_MODE = 0x21
   DEVICE_INFO = 0x29
   CONFIG_READ = 0x2C
   CONFIG_WRITE = 0x2D
   RADIO_ID = 0x3A
def pwm_to_crsf(pwm_us: int) -> int:
   pwm_us = max(1000, min(2000, pwm_us))
   return int((pwm_us - 1000) * (1792 - 191) / (2000 - 1000) + 191)
def crsf_validate_frame(frame) -> bool:
   return crc8_data(frame[2:-1]) == frame[-1]
def signed_byte(b):
   return b - 256 if b >= 128 else b
def crc8_dvb_s2(crc, a):
   crc ^= a
   for _ in range(8):
       if crc & 0x80:
           crc = (crc << 1) ^ 0xD5
       else:
           crc <<= 1
   return crc & 0xFF
def crc8_data(data):
   crc = 0
   for b in data:
       crc = crc8_dvb_s2(crc, b)
   return crc
def pack_crsf_channels(channels):
   b = bytearray(22)
   bits = 0
   bitpos = 0
   bytepos = 0
   for i, ch in enumerate(channels):
       bits |= (ch & 0x7FF) << bitpos


       bitpos += 11
       while bitpos >= 8:
           b[bytepos] = bits & 0xFF


           bits >>= 8
           bitpos -= 8
           bytepos += 1
   return b
def build_frame(channels):
   payload = bytearray()
   payload.append(PACKET_TYPE)
   payload += pack_crsf_channels(channels)
   frame = bytearray()
   frame.append(CRSF_SYNC)
   frame.append(len(payload) + 1)  # +1 for CRC
   frame += payload
   frame.append(crc8_data(payload))
   return frame
def packCrsfToBytes(channels) -> bytes:
   # channels is in CRSF format! (0-1984)
   # Values are packed little-endianish such that bits BA987654321 -> 87654321, 00000BA9
   # 11 bits per channel x 16 channels = 22 bytes
   if len(channels) != 16:
       raise ValueError('CRSF must have 16 channels')
   result = bytearray()
   destShift = 0
   newVal = 0
   for ch in channels:
       # Put the low bits in any remaining dest capacity
       newVal |= (ch << destShift) & 0xff
       result.append(newVal)


       # Shift the high bits down and place them into the next dest byte
       srcBitsLeft = 11 - 8 + destShift
       newVal = ch >> (11 - srcBitsLeft)
       # When there's at least a full byte remaining, consume that as well
       if srcBitsLeft >= 8:
           result.append(newVal & 0xff)
           newVal >>= 8
           srcBitsLeft -= 8


       # Next dest should be shifted up by the bits consumed
       destShift = srcBitsLeft


   return result
def channelsCrsfToChannelsPacket(channels) -> bytes:
   result = bytearray([CRSF_SYNC, 24, PacketsTypes.RC_CHANNELS_PACKED]) # 24 is packet length
   result += packCrsfToBytes(channels)
   result.append(crc8_data(result[2:]))
   return result
def handleCrsfPacket(ptype, data):
   global latest_yaw
   global current_yaw
   if ptype == PacketsTypes.RADIO_ID and data[5] == 0x10:
       #print(f"OTX sync")
       pass
   elif ptype == PacketsTypes.LINK_STATISTICS:
       rssi1 = signed_byte(data[3])
       rssi2 = signed_byte(data[4])
       lq = data[5]
       snr = signed_byte(data[6])
       antenna = data[7]
       mode = data[8]
       power = data[9]
       # telemetry strength
       downlink_rssi = signed_byte(data[10])
       downlink_lq = data[11]
       downlink_snr = signed_byte(data[12])
       #print(f"RSSI={rssi1}/{rssi2}dBm LQ={lq:03} mode={mode}") # ant={antenna} snr={snr} power={power} drssi={downlink_rssi} dlq={downlink_lq} dsnr={downlink_snr}")
   elif ptype == PacketsTypes.ATTITUDE:
       pitch = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10000.0
       roll = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10000.0
       yaw = int.from_bytes(data[7:9], byteorder='big', signed=True) / 10000.0

       #print(f"Attitude: Pitch={pitch:0.2f} Roll={roll:0.2f} Yaw={yaw:0.2f} (rad)")
   elif ptype == PacketsTypes.FLIGHT_MODE:
       packet = ''.join(map(chr, data[3:-2]))
       print(f"Flight Mode: {packet}")
   elif ptype == PacketsTypes.BATTERY_SENSOR:
       vbat = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
       curr = int.from_bytes(data[5:7], byteorder='big', signed=True) / 10.0
       mah = data[7] << 16 | data[8] << 7 | data[9]
       pct = data[10]
       print(f"Battery: {vbat:0.2f}V {curr:0.1f}A {mah}mAh {pct}%")
   elif ptype == PacketsTypes.BARO_ALT:
       print(f"BaroAlt: ")
   elif ptype == PacketsTypes.DEVICE_INFO:
       packet = ' '.join(map(hex, data))
       print(f"Device Info: {packet}")
   elif data[2] == PacketsTypes.GPS:
       lat = int.from_bytes(data[3:7], byteorder='big', signed=True) / 1e7
       lon = int.from_bytes(data[7:11], byteorder='big', signed=True) / 1e7
       gspd = int.from_bytes(data[11:13], byteorder='big', signed=True) / 36.0
       hdg =  int.from_bytes(data[13:15], byteorder='big', signed=True) / 100.0
       alt = int.from_bytes(data[15:17], byteorder='big', signed=True) - 1000
       sats = data[17]
       print(f"GPS: Pos={lat} {lon} GSpd={gspd:0.1f}m/s Hdg={hdg:0.1f} Alt={alt}m Sats={sats}")
   elif ptype == PacketsTypes.VARIO:
       vspd = int.from_bytes(data[3:5], byteorder='big', signed=True) / 10.0
       print(f"VSpd: {vspd:0.1f}m/s")
   elif ptype == PacketsTypes.RC_CHANNELS_PACKED:
       #print(f"Channels: (data)")
       pass
   else:
       packet = ' '.join(map(hex, data))
       print(f"Unknown 0x{ptype:02x}: {packet}")


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class PID_control(Node):
   def __init__(self):
       self.traj_file = "/home/lantacis/ros2_ws/src/crsf_joystick/crsf_joystick/trajectory_log.csv"
       with open(self.traj_file, mode='w', newline='') as f:
           writer = csv.writer(f)
           writer.writerow(['x', 'y', 'z',"roll", "pitch","throttle", "yaw"])
       super().__init__('joy_to_crsf')
       qos = QoSProfile(depth=10)
       qos.reliability = ReliabilityPolicy.BEST_EFFORT
       self.subscription_pose = self.create_subscription(
           PoseStamped,
           '/vrpn_mocap/AzamatDrone/pose',
           self.pose_callback,
           qos)

       self.subscription_joy = self.create_subscription(
           Joy,
           'joy',
           self.joy_callback,
           10)
       self.ser = serial.Serial("/dev/ttyACM0", 115200)
       time.sleep(2)
       self.input_buffer = bytearray()
       threading.Thread(target=self.read_telemetry_loop, daemon=True).start()
       self.get_logger().info("Serial connection opened and ready")
       self.arm_state = 1000
       self.angle_state = 1000
       self.prev_button_4 = None
       self.prev_button_5 = None
       self.armed_flag = False
       self.last_time = time.time()


       # State
       self.x_actual = 0.0
       self.y_actual = 0.0
       self.z_actual = 0.0
       self.yaw_actual = 0.0
       self.pitch_actual = 0.0
       self.roll_actual = 0.0
       self.y_prev = 0.0
       self.pid = PID_controller()
       self.was_armed = False
       self.prev_cbf_bool = None
       self.return_home = False

       self.create_timer(0.02, self.control_loop)  # 50 Hz

   def read_telemetry_loop(self):
       while True:
           if self.ser.in_waiting > 0:
               self.input_buffer.extend(self.ser.read(self.ser.in_waiting))


           while len(self.input_buffer) > 2:
               expected_len = self.input_buffer[1] + 2
               if expected_len > 64 or expected_len < 4:
                   self.input_buffer = bytearray()
                   continue


               if len(self.input_buffer) >= expected_len:
                   packet = self.input_buffer[:expected_len]
                   self.input_buffer = self.input_buffer[expected_len:]


                   if crsf_validate_frame(packet):
                       acm=0
                       handleCrsfPacket(packet[2], packet)
                   else:
                       print(f"crc error: {' '.join(hex(b) for b in packet)}")
               else:
                   break
           time.sleep(0.001)

   def pose_callback(self, msg):
       self.x_actual = msg.pose.position.x
       self.y_actual = msg.pose.position.y
       self.z_actual = msg.pose.position.z
       q = msg.pose.orientation
       r = R.from_quat([q.x, q.y, q.z, q.w])
       self.roll_actual, self.pitch_actual, self.yaw_actual = r.as_euler('xyz', degrees=False)

   def joy_callback(self, msg: Joy):

       current_button_4 = msg.buttons[4]
       current_button_5 = msg.buttons[5]
       # Only set prev on first message and return
       if self.prev_button_4 is None:
           self.prev_button_4 = current_button_4
           return
       if self.prev_button_5 is None:
           self.prev_button_5 = current_button_5
           return
       # Now do toggle logic
       if current_button_4 == 1 and self.prev_button_4 == 0:
           self.arm_state = 2000 if self.arm_state == 1000 else 1000
           print(f"Toggled arm: {self.arm_state}")
       if current_button_5 == 1 and self.prev_button_5 == 0:
           self.angle_state = 1500 if self.angle_state == 1000 else 1000
           print(f"Toggled angle: {self.angle_state}")


       self.prev_button_4 = current_button_4
       self.prev_button_5 = current_button_5

   def control_loop(self):
       current_position= [self.x_actual, self.y_actual, self.z_actual]
       roll = pwm_to_crsf(1500)
       pitch = pwm_to_crsf(1500)
       yaw = pwm_to_crsf(1500)
       throttle = pwm_to_crsf(1000)
       if self.arm_state == 2000:
           if not self.armed_flag:
               throttle = pwm_to_crsf(1000)
               self.armed_flag = True
               self.pid.update_state(self.x_actual, self.y_actual, self.z_actual,
                                     self.roll_actual, self.pitch_actual, self.yaw_actual)
               self.pid.reset()  # <<< flush on arming
               self.was_armed = True
               print("Armed, holding throttle min & PID reset")
           else:

               cbf_violation = cbf_filter(current_position,alpha=1.0)
               current_cbf_bool= cbf_violation
               if self.prev_cbf_bool == None:
                   self.prev_cbf_bool = current_cbf_bool
                   return
               if current_cbf_bool == True and self.prev_cbf_bool == False:
                   self.return_home = True
                   print("home", self.x_actual, self.y_actual, self.z_actual)
               if self.return_home == True: #went out of range
                   target = [0,0.6,0]
                   #print("goes to origin",current_position)
               else:
                   target = [3.0, 0.6, 0.0]
                   #print("main target",current_position)
               self.prev_cbf_bool=current_cbf_bool
               self.pid.update_state(self.x_actual, self.y_actual, self.z_actual, self.roll_actual, self.pitch_actual,self.yaw_actual)
               roll, pitch, throttle, yaw = self.pid.main(target)

       else:
           roll = 1500
           pitch = 1500
           yaw = 1500
           throttle = 1000
           if self.was_armed:
               # only reset once on disarm edge
               self.pid.reset()  # <<< flush on disarm
               self.was_armed = False
           self.armed_flag = False

       aux = [pwm_to_crsf(1000)] * 10
       channels = [pwm_to_crsf(roll), pwm_to_crsf(pitch),pwm_to_crsf(throttle), pwm_to_crsf(yaw), pwm_to_crsf(self.arm_state), pwm_to_crsf(self.angle_state)] + aux
       frame = build_frame(channels)
       self.ser.write(frame)





def main(args=None):
   rclpy.init(args=args)
   node = PID_control()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
