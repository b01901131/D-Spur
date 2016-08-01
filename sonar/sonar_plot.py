import pdb
import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt

ser = serial.Serial('/dev/cu.wchusbserial1410', 115200, timeout=1)

time.sleep(4)

ser.write('0')
s = ser.read(400*2)

#pdb.set_trace()

radius_data = []

for i in xrange(400):
    dist = struct.unpack('<h', s[2*i : 2*(i+1)])[0]
    #dist = int(raw)
    #pdb.set_trace()
    radius_data.append(dist)

theta = np.arange(0, 360.0, 0.9)
print radius_data
print theta
x_s = []
y_s = []

for r,t in zip(radius_data,theta):
    x_s.append(r*np.cos(np.deg2rad(t)))
    y_s.append(r*np.sin(np.deg2rad(t)))

plt.plot(x_s, y_s, 'ro')
plt.axis([-120, 120, -120, 120])
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

'''
theta * 3.14 / 180
#pdb.set_trace()
#r = 2 * np.pi * theta

ax = plt.subplot(111, projection='polar')
ax.plot(theta, radius_data, marker='ro', color='r', linewidth=3)
ax.set_rmax(100.0)
ax.grid(True)

ax.set_title("A line plot on a polar axis", va='bottom')
plt.show()
'''
