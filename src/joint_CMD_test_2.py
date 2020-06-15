#!/usr/bin/env python
# license removed for brevity
import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float32MultiArray
from geometry_msgs.msg import Twist
from math import pi, sqrt
import numpy as np
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev

_dx = 0.1
_dTheta = 0.1
_node_name = 'joy_listener'
_cmd_topic = "controlCMD"
_Hz = 100
_N = 1000
_joy_dead_zone = 0.12
_joint_limit = np.deg2rad(40)

class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)
    def reset_sim(self):
        for i in range(self._link_N+1):
            self.pub_vec[i].publish(Float64(data=0))
        self.theta_move = 0
        self.head_angle = 0
        # self.head_move = 0
        self.A_head = np.asarray([[1, 0, 0],
                                  [0, 1, 0],
                                  [0, 0, 1]])
        self.path = np.asarray([np.linspace(-self._link_L * self._link_N * 2, 0, self._N, endpoint=True),
                                np.zeros(self._N),
                                np.ones(self._N)])

    def __init__(self):

        self._dtheta = 0.05
        self._dx = 5
        self._link_L = 160
        self._link_N = 10
        self._N = 200
        self.Kp_curve = 0.00041


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
        self.reset_sim()
        # self.joint_vals = Float32MultiArray()

        # self.joint_vals.data[0] = [0]
        # for i in range(self._link_N):
        #     self.joint_vals.data[i + 1] = 0

        # self.angle_range = np.linspace(-np.pi / 2, np.pi, _N)
        # self.head = Twist()
        # self.head.linear.x = 0
        # self.head.angular.z = 0
        # self.x_move = 0
        # self.theta_move = 0

        while not rospy.is_shutdown():
            # self.head.linear.x = 0
            # self.head.angular.z = 0

            # self.head.linear.x += self.x_move
            # self.head.angular.z += self.theta_move
            # self.pub.publish(self.head)
            #
            # self.pub_vec[0].publish(self.head.linear.x)
            # self.pub_vec[1].publish(self.head.angular.z)

            self.rate.sleep()



    def split_curve(self, points_x, points_y, link_N, link_L):
        counter = 0
        epsilon = 1e-2
        e = link_L
        px0 = points_x[-1]
        py0 = points_y[-1]
        px1 = py1 = 0
        tck, u = sc.splprep([points_x, points_y], k=2, s=0)
        joints_points = [px0, py0, 1]
        u = 1
        for i in range(link_N):
            e = 1
            while abs(e) > epsilon:
                px1, py1 = sc.splev(u, tck)
                L_current = sqrt((px1 - px0) ** 2 + (py1 - py0) ** 2)
                e = link_L - L_current
                u -= e * self.Kp_curve
                counter += 1

            u -= 0.001
            self.Kp_curve *= 0.99999
            vec = [px1, py1, 1]
            joints_points = np.vstack([joints_points, vec])
            px0 = px1
            py0 = py1

        print("Counter {:.2f}".format(counter/link_N))
        joints_points = joints_points[::-1]

        # First angle
        v0 = [1, 0]
        v1 = joints_points[1, 0:2] - joints_points[0, 0:2]
        # angle = np.math.acos(np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)))
        angle = np.math.atan2(v1[1], v1[0]) - np.math.atan2(v0[1], v0[0])
        joints_angles = np.asarray([angle])

        for i in range(1, link_N):
            v0 = (joints_points[i, 0:2] - joints_points[i - 1, 0:2])
            v1 = (joints_points[i + 1, 0:2] - joints_points[i, 0:2])
            angle = np.arctan2(np.cross(v0, v1), np.dot(v0, v1))
            joints_angles = np.concatenate((joints_angles, [angle]))

        joints_points = (joints_points + 1600)/1000
        return joints_points[:,0], joints_angles, tck

    def stretch(self, x, in_min, in_max, out_min, out_max):
        x, in_min, in_max, out_min, out_max = map(float, (x, in_min, in_max, out_min, out_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def Rz(self, x, theta):
        return np.asarray([[np.cos(theta), -np.sin(theta), np.cos(theta) * x],
                           [np.sin(theta), np.cos(theta), np.sin(theta) * x],
                           [0, 0, 1]])

    def head_turn(self, state):
        if abs(state)>_joy_dead_zone and abs(self.head_angle+self._dtheta*(-state)) < _joint_limit:
            self.theta_move = self._dtheta*(-state)
            self.head_angle += self.theta_move
            vec_view = [[500], [0], [1]]
            # self.R_z = self.Rz(0, self.theta_move)
            self.A_head = np.dot(self.A_head, self.Rz(0, self.theta_move))  # Update A_hed matrix
            #print("Turn: {:.3f}".format(self.theta_move))
            # head_direction = np.dot(self.A_head, vec_view)  # Get new head Direction
            # Start section print and plot of the snake:
            # Remove old plot elements

            # --- For plot:
            # self.ax.lines.remove(self.head_dir)
            # self.head_dir, = plt.plot([self.head_point_new[0], head_direction[0]],
            #                           [self.head_point_new[1], head_direction[1]], 'r-')
            # plt.draw()
            # plt.pause(0.00000000001)
            # plt.axis('equal')
            # plt.axis([-1700, 4000, -20, 20])
            # End section print & Plot
            # print("Head angle: {:.3f}".format(self.head_angle))
            self.pub_vec[self._link_N].publish(Float64(data=self.head_angle))

    def joystick_Left_X(self, state):
        print(state)

    def head_move(self, state):
        if state > _joy_dead_zone:
            # x_move = self._dx * self.stretch(state, 0, 2**15, 0, 1)
            #print("Move: {:.3f}".format(self.theta_move))

            x_move = self._dx * state
            vec0 = [[0], [0], [1]]
            vec_view = [[500], [0], [1]]
            self.R_z = np.asarray([[np.cos(self.theta_move), -np.sin(self.theta_move), np.cos(self.theta_move) * x_move],
                                   [np.sin(self.theta_move), np.cos(self.theta_move), np.sin(self.theta_move) * x_move],
                                   [0, 0, 1]])
            self.A_head = np.dot(self.A_head, self.R_z)             # Update A_hed matrix

            self.head_point_new = np.dot(self.A_head, vec0)              # Get new head position
            # head_direction = np.dot(self.A_head, vec_view)          # Get new head Direction

            self.path = np.concatenate((self.path, self.head_point_new), axis=1)  # Add new head position to path
            x = np.squeeze(np.asarray(self.path[0, :]))
            y = np.squeeze(np.asarray(self.path[1, :]))
            joints_points, joints_angles, tck = self.split_curve(x, y, self._link_N, self._link_L)
            # self.theta_move = 0

            # Start section print and plot of the snake:
            # Remove old plot elements
            # --- For plot:
            # self.ax.lines.remove(self.head_dir)
            # self.ax.lines.remove(self.links_state)

            # Print the angles in console:
            # for angle_i in joints_angles:
            #     print("{:.2f}".format(np.rad2deg(angle_i)), end=" ")
            # for angle_i in range(len(joints_angles)):
            #     print("{:.2f}".format(np.rad2deg(joints_angles[angle_i]))),
                # print("{:.2f}".format(np.rad2deg(joints_angles[angle_i])), end = angle_i == (len(joints_angles)-1) ? " " : "\n";)
                # print("{:.2f}".format(np.rad2deg(joints_angles[angle_i])),
                #       end = " " if angle_i < (len(joints_angles) - 1) else "\n")
            # print('.')

            print("Origin pos: {:.3f}".format(joints_points[0])),
            self.pub_vec[0].publish(Float64(data=joints_points[0]))
            for i in range(self._link_N):
                # self.joint_vals.data[i+1] = joints_angles[i]
                print("{:.2f}".format(joints_angles[i])),
                # print("Joint {} angle: {:.3f}".format(i, joints_angles[i]))
                self.pub_vec[i+1].publish(Float64(data=joints_angles[i]))
            # self.head_angle = self.head_angle - np.deg2rad(joints_angles[-1])
            self.pub_vec[self._link_N].publish(Float64(data=self.head_angle))
            print(".")
            # --- Plot the snake head direction
            #self.head_dir, = plt.plot([self.head_point_new[0], head_direction[0]], [self.head_point_new[1], head_direction[1]], 'r-')
            # Plot the snake links and joints
            # self.links_state, = plt.plot(joints_points[:, 0], joints_points[:, 1], 'og-', markersize=2)
            # plt.plot([self.head_point_new[0]], [self.head_point_new[1]], 'm.', markersize=2)
            ### self.arc_state, = plt.plot([-1600, joints_points[0, 0]], [0, joints_points[0, 1]], 'm--', markersize=2)
            # plt.draw()
            # plt.pause(0.00000000001)
            # plt.axis('equal')
            # plt.axis([-1700, 4000, -20, 20])
            # End section print & Plot

    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
            Left stick - right left - data.axes[0]
            Left stick - up down - data.axes[1]
        """
        # print("joy_update: {:.2f}".format(data.axes[1]))
        self.head_turn(-data.axes[0])
        # self.head_move(data.axes[7])
        self.head_move(data.axes[1])
        if data.buttons[2]==1:
            print("Reset Simulation")
            self.reset_sim()
        # if pygame.joystick.Joystick(joy).get_button(0) == 1:
        #     run = False
        #self.x_move = _dx * data.axes[1]
        #self.theta_move = _dTheta * data.axes[0]


if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass