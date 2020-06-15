import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

"""Microsoft Xbox 360 Wireless Controller for Linux
-=BUTTONS=-
index | button name                   |in use
``````|```````````````````````````````|``````
  0   |    A                          |
  1   |    B                          |
  2   |    X                          |
  3   |    Y                          |
  4   |    LB                         |
  5   |    RB                         |
  6   |    back                       |  x
  7   |    start                      |  x
  8   |    power                      |  x
  9   |    Button stick left          |
 10   |    Button stick right         |
`````````````````````````````````````````````
-=AXIS=-
index | axis name                     |in use
``````|```````````````````````````````|``````
  0   |    Left/Right Axis stick left |  x
  1   |    Up/Down Axis stick left    |  x
  2   |    Left/Right Axis stick right|
  3   |    Up/Down Axis stick right   |
  4   |    RT                         |  x
  5   |    LT                         |
  6   |    cross key left/right       |  x
  7   |    cross key up/down          |  x
`````````````````````````````````````````````
"""
# Joystick axes and buttons mapping:
linear_axis = 1
steering_axis = 0
hydraulics_axis = 4
loader_axis = 6
brackets_axis = 7

pause_button = 7
model_reset_button = 8
clear_workplace_button = 6


node_name = 'joy_listener'
cmd_topic = "controlCMD"
hydraulics_velocity_multiplier = 1
loader_velocity_multiplier = -1
brackets_velocity_multiplier = -1
linear_velocity_multiplier = 0.05
angular_velocity_multiplier = 0.5


class Node:
    rospy.init_node(node_name)

    Hz = 50
    rate = rospy.Rate(Hz)

    def __init__(self):
        self.pub = rospy.Publisher(cmd_topic, BobcatControl, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_update)

        while not rospy.is_shutdown():
            msg = Twist()
            
            msg.linear.x = 0
            msg.angular.z = 0
            self.pub.publish(msg)

            self.rate.sleep()

    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
        """
        msg = Twist()
        msg.linear.x = data.axes[steering_axis]
        msg.angular.z = 0
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass