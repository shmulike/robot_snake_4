#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
#from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float32MultiArray
import csv

_node_name  = 'mission_exe_from_CSV'
_cmd_topic  = "/robot_snake_1/joint_cmd"
_Hz         = 100
_N          = 1000
_N_links    = 4

pub = rospy.Publisher(_cmd_topic, Float32MultiArray, queue_size=10)

def myhook():
        print "--> shutdown time!"
        vec_cmd = Float32MultiArray()
        vec_cmd.data = [0]*4
        for i in range(_N_links):
            vec_cmd.data[i] = 0
        pub.publish(vec_cmd)
        #self.file1.close()

class Node:


    rospy.init_node(_node_name, anonymous=True)
    rospy.on_shutdown(myhook)
    rate = rospy.Rate(_Hz)

    

    def __init__(self):

        #self.pub = rospy.Publisher(_cmd_topic, Float32MultiArray, queue_size=10)
        vec_cmd = Float32MultiArray()
        vec_cmd.data = [0]*4
        #rospy.Subscriber('/joy', Joy, self.joy_update)

        fileName = "mission_5_cmd_25.csv"
        self.file1 = open(fileName)
        #reader = csv.reader(self.file1, quoting=csv.QUOTE_NONNUMERIC)
        jointCMD = []
        with open(fileName, 'rb') as csv_file_object:
            file_reader = csv.reader(csv_file_object, quoting=csv.QUOTE_NONNUMERIC)
            for row in file_reader:
                jointCMD.append(row)
        jointCMD = np.array(jointCMD)

        counter = 0

        while not rospy.is_shutdown():

            
            counter+=1
            if counter >= jointCMD.shape[0]:
                counter = 0

            print("line {}: {}".format(counter, jointCMD[counter, :]))
            for i in range(_N_links):
                vec_cmd.data[i] = jointCMD[counter, i]

            pub.publish(vec_cmd)

            rospy.sleep(0.03)
            self.rate.sleep()

    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
        """

    

if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass