import numpy as np
import matplotlib.pyplot as plt
from math import pi, sqrt
# from my_tests import split_curve
# from my_tests import my_fun
from split_curve import *
from scipy.interpolate import splev, splprep

links_N = 8
links_L = 1
x = np.linspace(0, 4*pi, 1000, endpoint=True)
y = np.sin(x)

x = np.append(np.arange(0,5,0.01), x+5)
y = np.append(np.zeros((1,int(5/0.01))), y)

joints_points, joints_angles, tck = split_curve(x, y, links_N, links_L)

# xx, yy = splev(0,tck)
# plt.plot(xx,yy,'rx')

plt.plot(x, y, color='blue', linestyle='--')

u = np.linspace(0,1,1000)
px, py = splev(u, tck, der = 0)
plt.plot(px,py,'-.g')


plt.plot(joints_points[:,0], joints_points[:,1], '-dr')

# plt.plot(x,y,'oy')
plt.grid(True)
plt.axis('square')
plt.xlim((0,x.max()*1.1))
plt.ylim(((-1.5,1.5)))

for i in range(links_N):
    x = joints_points[i+1,0]-joints_points[i,0]
    y = joints_points[i + 1, 1] - joints_points[i, 1]
    L = sqrt(x**2+y**2)
    print(L)

print(joints_angles)

plt.show()
