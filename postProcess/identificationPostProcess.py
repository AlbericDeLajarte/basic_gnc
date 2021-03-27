#!/usr/bin/env python

import rospy
from tvc_simulator.msg import FSM
from tvc_simulator.msg import State
from tvc_simulator.msg import Control
from tvc_simulator.msg import Sensor
from tvc_simulator.msg import Trajectory
from tvc_simulator.msg import Waypoint

import numpy as np
import math
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import time

import rosbag
import csv


bag = rosbag.Bag('../log/log.bag')

for topic, msg, t in bag.read_messages(topics=['/sensor_pub']):
  time_init = t.to_sec()
  break
  

# save data to csv file to use in matlab
with open('gyro.csv', mode='w') as out_file:
  csv_writer = csv.writer(out_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  for topic, msg, t in bag.read_messages(topics=['/sensor_pub']):
    csv_writer.writerow([t.to_sec()-time_init, msg.IMU_gyro.z])

with open('input.csv', mode='w') as out_file:
  csv_writer = csv.writer(out_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
  for topic, msg, t in bag.read_messages(topics=['/drone_pub']):
    csv_writer.writerow([t.to_sec()-time_init, msg.bottom, msg.top])


fig, axe = plt.subplots(2,2, figsize=(15,10))

t, gyro = np.loadtxt('gyro.csv', delimiter=',', unpack=True)
l = axe[0][0].plot(t, gyro)
axe[0][0].legend(l, ('Gyro'))

# t, bottom, top = np.loadtxt('input.csv', delimiter=',', unpack=True)
# l = axe[0][0].plot(t, bottom, top)
# axe[0][0].legend(l, ('bottom', 'top'))


plt.show()
