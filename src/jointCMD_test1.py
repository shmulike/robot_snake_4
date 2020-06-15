#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import Twist

_dx         = 0.1
_dTheta     = 0.1
_node_name  = 'joy_listener'
_cmd_topic  = "controlCMD"
_Hz         = 100
_N          = 1000

class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)

    def __init__(self):

        self.pub_vec = [rospy.Publisher('/snake_10/linear_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint1_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint2_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint3_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint4_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint5_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint6_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint7_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint8_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint9_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint10_position_controller/command', Float64, queue_size=10)]

        self.pub = rospy.Publisher(_cmd_topic, Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_update)
        self.angle = Float32MultiArray()
        self.angle_range = np.linspace(-np.pi/2, np.pi, _N)
        self.head = Twist()
        self.head.linear.x = 0
        self.head.angular.z = 0
        self.x_move      = 0
        self.theta_move  = 0

        while not rospy.is_shutdown():
            #self.head.linear.x = 0
            #self.head.angular.z = 0
            
            self.head.linear.x += self.x_move
            self.head.angular.z += self.theta_move
            self.pub.publish(self.head)

            self.pub_vec[0].publish(self.head.linear.x)
            self.pub_vec[1].publish(self.head.angular.z)

            self.rate.sleep()

    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
        """
        
        self.x_move     = _dx * data.axes[1]
        self.theta_move = _dTheta * data.axes[0]        

if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass