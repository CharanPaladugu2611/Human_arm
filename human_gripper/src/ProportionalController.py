import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
# from sympy import Matrix
from math import atan2, sqrt, tan, cos, sin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from time import time, sleep

class ProportionalController(Node):

    def __init__(self):
        super().__init__('proportional_control_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        # self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.imu_msg = self.create_subscription(Imu,'/imu_plugin/out',self.control,qos_profile)
        self.goalx = 10.0 # in inches, each square is about 20 in
        self.goaly = 10.0
        self.posx = 0.0
        self.posy = 0.0
        self.control_ang = 0.0
        # self.t0 = time()

    #     self.all_rot = Matrix( [[1, 0, 0, 0],
    #                             [0, 1, 0, 0],
    #                             [0, 0, , 0],
    #                             [0, 0, 0, 1]])


    # def quat2rot(self,x,y,z,w):
    #     R = Matrix([[w**2+x**2+y**2+z**2, 2*(x*y-w*z),     2*(w*y+x*z),     0],
    #                 [2*(x*y+w*z),         1-2*(x**2+z**2), 2*(y*z-w*x),     0],
    #                 [2*(x*z-w*y),         2*(w*x+y*z),     1-2*(x**2+y**2), 0],
    #                 [0,                   0,               0,               1]])
    #     return R

    # def quat2euler(self,x,y,z,w):
    #     phi = atan2(2*(w*x+y*z),1-2*(x**2+y**2))
    #     theta = -1.578+2*atan2(sqrt(1+2*(w*y-x*z)),sqrt(1-2*(w*y-x*z)))
    #     psi = atan2(2*(w*z+x*y),1-2*(y**2+z**2))
    #     euler = [phi,theta,psi]
    #     return euler

    def quat2aa(self,qx,qy,qz,qw): # for IMU, the y axis is facing up. The z-axis is behind
        normish = sqrt(qx**2+qy**2+qz**2)
        ax = qx/normish
        ay = qy/normish
        az = qz/normish
        theta = 2*atan2(normish,qw)
        return [ax,ay,az,theta]

    def control(self,imu_msg):
        tstep = .1
        sleep(tstep)
        linear_vel = 0.0
        # t1 = time()
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w 
        # I = Matrix([[1, 0, 0, 0],
        #             [0, 1, 0, 0],
        #             [0, 0, 1, 0],
        #             [0, 0, 0, 1]])

        aav = self.quat2aa(qx,qy,qz,qw)
        real_ang = 3.1416 - aav[3]
        ideal_ang = 3.1416/2 - (self.goaly-self.posy)/(self.goalx-self.posx)
        if(self.goalx - self.posx > .1 and self.goaly - self.posy > .1):
            linear_vel = 200.0
            if(real_ang < ideal_ang): # the car was spawned with an offset of Pi
                self.control_ang = ideal_ang/3.55 # TURNING SCALING FACTOR...3.83
            else:
                linear_vel = 0.0
            position_scaling_val = 1/100 # affects how far the car goes...1/100
            self.posx += cos(real_ang)*tstep*linear_vel*position_scaling_val
            self.posy += sin(real_ang)*tstep*linear_vel*position_scaling_val


        # rot = self.quat2rot(qx,qy,qz,qw)
        # newmat = I * rot
        # self.all_rot = newmat * self.all_rot
        # self.posx += newmat[0,3]
        # self.posy += newmat[1,3]
            
        # euler = self.quat2euler(qx,qy,qz,qw)
        # distx = self.posx + cos(euler[2])*linear_vel*dt
        # distz = self.posz + cos(euler[2])*linear_vel*dt

        # self.get_logger().info("[ x-pos, y-pos ]: [ %s, %s ]" %(self.posx,self.posy))        
        self.get_logger().info("[ actual angle , controller angle ] : [ %s, %s ]" %(real_ang,self.control_ang)) 
        self.get_logger().info("[ linear velocity ] : [ %s ]" %linear_vel)
        joint_positions = Float64MultiArray()
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [linear_vel, linear_vel]
        joint_positions.data = [self.control_ang, -self.control_ang]
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = ProportionalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()