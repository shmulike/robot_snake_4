from math import sqrt
import numpy as np
# import matplotlib.pyplot as plt
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev
from math import pi, sqrt

epsilon = 1e-6
kp = 0.05

def split_curve(points_x, points_y, link_N, link_L):
    e = link_L
    #points_x = np.flip(points_x)
    #points_y = np.flip(points_y)
    # points_x = points_x[::-1]
    # points_y = points_y[::-1]
    px0 = points_x[-1]
    py0 = points_y[-1]
    tck, u = sc.splprep((points_x, points_y), k=3, s=0)
    # tck, u = sc.splprep((points_x, points_y))
    joints_points = [px0, py0, 1]
    u = 0
    for i in range(link_N):
        e = 1
        # count = 0
        u = 0
        while (abs(e) > epsilon):
            px1, py1 = sc.splev(u, tck)
            L_current = sqrt((px1 - px0) ** 2 + (py1 - py0) ** 2)
            e = link_L - L_current
            u -= e * kp
            # count += 1
        # print("Number of iteretions: {}".format(count))
        # plt.plot(px1, py1, 'yd')
        #u-=0.1
        vec = [px1,py1,1]
        joints_points = np.vstack([joints_points, vec])
        px0 = px1
        py0 = py1
    
    
    joints_points = joints_points[::-1]

    # First angle
    v0 = [1,0]
    v1 = joints_points[1, 0:2] - joints_points[0, 0:2]
    angle = np.math.acos( np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)) )
    joints_angles = np.asarray([angle])

    for i in range(1, link_N):
        v0 = (joints_points[i, 0:2] - joints_points[i - 1, 0:2])
        v1 = (joints_points[i + 1, 0:2] - joints_points[i, 0:2])
        #angle = np.math.acos( np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)) )
        #angle = np.math.acos( np.dot(v0, v1) / (link_L**2) )
        angle = np.arctan2(np.cross(v0, v1), np.dot(v0, v1))
        #joints_angles.append(np.degrees(angle))
        #joints_angles.append(angle)
        joints_angles = np.concatenate( (joints_angles, [angle]) )
    
    return(joints_points, joints_angles, tck)
