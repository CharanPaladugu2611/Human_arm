#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
import math

finger_dangle = math.pi/180
other_dangle = math.pi/180

class ArmController(Node):

    def __init__(self):
        super().__init__('arm_control_node')
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.joint_velocity_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.settings = termios.tcgetattr(sys.stdin)
        self.gripangle = float(0)
        self.elbowpitch = float(0)
        self.wristpitch = float(0)
        self.wristroll = float(0)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        minute_dangle = other_dangle/100
        joint_positions = Float64MultiArray()
        self.msg = """
        Control Your Hand!
        ---------------------------          
            curl arm :          w
          uncurl arm :          s
        rotate wrist : <--- a      d --->
        
                GRIP : spacebar

                   q : snap to starting state

        Esc to quit
        """

        self.get_logger().info(self.msg)

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  
                    self.gripangle = 0
                    self.elbowpitch = 0
                    self.wristpitch = 0
                    self.wristroll = 0
                elif key == 'w':  # 
                    self.elbowpitch += other_dangle
                    self.wristpitch += other_dangle
                elif key == 's':  # 
                    self.elbowpitch -= other_dangle
                    self.wristpitch -= other_dangle
                elif key == 'a':  # 
                    self.wristroll += other_dangle
                elif key == 'd':  # 
                    self.wristroll -= other_dangle
                elif key == ' ':  # 
                    self.gripangle += finger_dangle
                else:
                    if(self.gripangle > 0):
                        self.gripangle -= finger_dangle/2

                self.elbowpitch = float(self.elbowpitch)
                self.wristpitch = float(self.wristpitch)
                self.wristroll = float(self.wristroll)
                self.gripangle = float(self.gripangle)
                joint_positions.data = [self.elbowpitch,self.wristpitch,-self.wristroll,self.gripangle,self.gripangle,-self.gripangle,self.gripangle,
                                        self.gripangle,-self.gripangle,self.gripangle,self.gripangle,-self.gripangle,self.gripangle,self.gripangle,
                                        -self.gripangle,self.gripangle,self.gripangle]
                self.joint_position_pub.publish(joint_positions)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()