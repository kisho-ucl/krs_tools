import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String, Bool
import numpy as np
import tf2_ros
import serial
import time

STRETCH = 0x01
SPEED = 0x02
CURRENT = 0x03
TEMPERATURE = 0x04

def pos2deg(pos):
    deg = (pos - 7500) * 135.0 / 4000
    return deg

def deg2pos(deg):
    pos = deg * 4000 / 135.0 + 7500
    return int(pos)

class Servo(Node):
    def __init__(self):
        super().__init__('control_servo')
        self.krs = serial.Serial('/dev/ttyUSB0', baudrate=115200, parity=serial.PARITY_EVEN, timeout=0.5)
        self.rot_speed = 30
        self.rot_once = 90.0
        self.r = 0.38
        self.h = 0.15
        self.cntStop = 0
        self.stateMove = False
        reData = self.krs_setValue(0, SPEED, 30)
        print(reData)
        self.publisher = self.create_publisher(Bool, '/servo_state_move', 10)
        self.sub = self.create_subscription(String, '/cmd_rot', self.cmd_rot_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(0.1, self.publish_data)
        self.deg = 0.0
        self.pos = 7500

    def cmd_rot_callback(self,msg):
        data = msg.data
        self.get_logger().info(f'{data}')
        if data == 'right':
            self.turnRight()
        elif data == 'left':
            self.turnLeft()
        elif data == 'init':
            self.setInit()
        
    def publish_data(self):
        try:
            bl, reData = self.krs_getPos36_CMD(0)
            pos=reData
            #self.pos = pos
        except:
            pass

        if self.stateMove == True:
           self.pos = pos
           if self.pos == pos: 
               self.cntStop += 1
               #print(self.cntStop)
        
        if self.cntStop > 10:
            self.stateMove = False
            self.cntStop = 0


        #print(self.pos)
        self.deg = pos2deg(self.pos)
        #print(self.stateMove)
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'        
        t.child_frame_id = 'servo'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        r = R.from_euler('z', -self.deg, degrees=True)
        q = r.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        """

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'        
        t.child_frame_id = 'camera_depth_optical_frame'
        #t.transform.translation.x = 0.0
        #t.transform.translation.y = self.r
        #t.transform.translation.z = self.h
        th = self.deg*3.14/180
        t.transform.translation.x = self.r * np.sin(-th)
        t.transform.translation.y = self.r * np.cos(-th)
        t.transform.translation.z = self.h
        r = R.from_euler('xyz', [np.pi/2, np.pi, th])
        q = r.as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        msg = Bool()
        msg.data = self.stateMove
        self.publisher.publish(msg)
        #self.get_logger().info(f'Published: {msg.data}')
 

    def krs_setPos_CMD(self, servo_id, pos):
        txCmd = [0x80 | servo_id,
                 pos >> 7 & 0x7f,
                 pos & 0x7f]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False
        else:
            return True, (rxCmd[4] << 7) + rxCmd[5]

    def krs_setFree_CMD(self, servo_id):
        bl, Value = self.krs_setPos_CMD(servo_id, 0)
        return bl, Value

    def krs_getValue_CMD(self, servo_id, sc):
        txCmd = [0xA0 | servo_id,
                 sc]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(5)

        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, rxCmd[4]

    def krs_getPos36_CMD(self, servo_id):
        txCmd = [0xA0 | servo_id,
                 0x05]
        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, (rxCmd[4] << 7) + rxCmd[5]

    def krs_getPos35_CMD(self, servo_id):
        bl, Value = self.krs_setPos_CMD(servo_id, 0)
        bl, Value = self.krs_setPos_CMD(servo_id, Value)
        return bl, Value

    def krs_setValue(self, servo_id, sc, Value):
        if sc == 0x00:
            return False

        txCmd = [0xC0 | servo_id,
                 sc,
                 Value]

        self.krs.write(txCmd)
        rxCmd = self.krs.read(6)
        if len(rxCmd) == 0:
            return False
        else:
            if rxCmd[5] == Value:
                return True
            else:
                return False

    def krs_getID_CMD(self):
        txCmd = [0xFF,
                 0x00,
                 0x00,
                 0x00]

        self.krs.write(txCmd)
        rxCmd = self.krs.read(5)

        if len(rxCmd) == 0:
            return False, 0
        else:
            return True, rxCmd[4] & 0x1F

    def setInit(self):
        self.stateMove = True
        target = deg2pos(0)
        bl, reData = self.krs_setPos_CMD(0, target)
        #time.sleep(1.0)
        #bl, reData = self.krs_getPos36_CMD(0)
        #self.deg = pos2deg(reData)
        #print("pos:", self.deg, "deg")

    def turnRight(self):  # Added self parameter
        #bl, reData = self.krs_getPos36_CMD(0)
        #current_pos_deg = pos2deg(reData)
        self.stateMove = True
        target = deg2pos(self.deg + self.rot_once)
        bl, reData = self.krs_setPos_CMD(0, target)
        #for i in range(15):
        #    time.sleep(0.1)
        #    bl, reData = self.krs_getPos36_CMD(0)
        #    self.deg = pos2deg(reData)
        #self.deg += 45 
        #time.sleep(1.0)
        #bl, reData = self.krs_getPos36_CMD(0)
        #self.deg = pos2deg(reData)
        #print("pos:", self.deg, "deg")

    def turnLeft(self):  # Added self parameter
        #bl, reData = self.krs_getPos36_CMD(0)
        #current_pos_deg = pos2deg(reData)
        self.stateMove = True
        target = deg2pos(self.deg - self.rot_once)
        bl, reData = self.krs_setPos_CMD(0, target)
        #for i in range(15):
        #    time.sleep(0.1)
        #    bl, reData = self.krs_getPos36_CMD(0)
        #    self.deg = pos2deg(reData)
        #time.sleep(1.0)
        #bl, reData = self.krs_getPos36_CMD(0)
        #self.deg = pos2deg(reData)
        #print("pos:", self.deg, "deg")

    def reviDeg(self):
        time.sleep(1.0)
        bl, reData = self.krs_getPos36_CMD(0)
        self.pos = reData
        self.deg = pos2deg(reData)
        print("servo:", self.deg, "deg")


def main():
    rclpy.init()
    node = Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.krs.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

