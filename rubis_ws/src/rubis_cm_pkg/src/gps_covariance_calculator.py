#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

max_len = 3

x_list = []
y_list = []
z_list = []

cov_list = []

def cb(msg):
  x = msg.pose.position.x
  y = msg.pose.position.y
  z = msg.pose.position.z
  x_list.append(x)
  y_list.append(y)
  z_list.append(z)

  if(max_len(x_list) == len):
    data = [x_list, y_list, z_list]
    cov_list = np.cov(data)

    print(cov_list)
    exit()

def element_wise_multiplication(data1, data2):
  output = []
  for i in range(len(data1)):
    output.append(data1[i]*data2[i])
  return output

def sum_element(data):
  sum = 0
  for i in range(len(data)):
    sum += data[i]
  return sum




def main():
  rospy.init_node('imu_cm_republisher')
  sub = rospy.Subscriber('gnss_pose', PoseStamped, cb)
  r = rospy.Rate(100)

  rospy.spin()

if __name__=="__main__":
  main()