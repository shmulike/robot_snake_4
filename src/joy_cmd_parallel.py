#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from math import pi, sqrt
import numpy as np

_dTheta = 0.04
_node_name = 'joy_listener'
_Hz = 300
_N_joints = 4
_joy_dead_zone = 0.12
_joint_limit = 30


class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)

    def __init__(self):
        self.joints = np.zeros((_N_joints, 1))
        self.pub = rospy.Publisher("/robot_snake_1/joint_cmd", Float32MultiArray, queue_size=10)
        rospy.Subscriber('/robot_snake_1/joy', Joy, self.joy_update)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def move(self, rl, up):
        if abs(rl) > _joy_dead_zone:
            self.joints[1] += rl * _dTheta
            self.joints[3] += rl * _dTheta
        if abs(up) > _joy_dead_zone:
            self.joints[0] += up * _dTheta
            self.joints[2] += up * _dTheta

        for i in range(_N_joints):
            if self.joints[i] > 0:
                self.joints[i] = min(self.joints[i], _joint_limit)
            if self.joints[i] < 0:
                self.joints[i] = max(self.joints[i], -_joint_limit)
        self.pub.publish(Float32MultiArray(data=self.joints))

    def resetXaxis(self):
        rospy.loginfo("Reset position button ('X') was pressed")
        self.joints[1] = 0
        self.joints[3] = 0
        self.pub.publish(Float32MultiArray(data=self.joints))

    def resetYaxis(self):
        rospy.loginfo("Reset position button ('X') was pressed")
        self.joints[0] = 0
        self.joints[2] = 0
        self.pub.publish(Float32MultiArray(data=self.joints))

    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
            Left stick - right left - data.axes[0]
            Left stick - up down - data.axes[1]
            Button 'X' - reset X axis - buttons[2]
            Button 'Y' - reset Y axis - buttons[3]
            Button 'A' - reset All axis - buttons[0]
        """
        # print("joy_update: {:.3f}".format(data.axes[1]))
        self.move(-data.axes[0], data.axes[1])


        if data.buttons[2] == 1:
            print("Reset X Axis")
            self.resetXaxis()

        if data.buttons[3] == 1:
            print("Reset X Axis")
            self.resetYaxis()
        if data.buttons[0] == 1:
            print("Reset All Axis")
            self.resetXaxis()
            self.resetYaxis()

if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
