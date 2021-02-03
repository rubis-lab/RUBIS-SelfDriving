#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
import numpy as np

max_len = 1000

x_list = []
y_list = []
z_list = []
xn_list = []
yn_list = []

def cb(msg):
  entry_x = [msg.header.seq, msg.pose.position.x]
  entry_y = [msg.header.seq, msg.pose.position.y]
  x_list.append(entry_x)
  y_list.append(entry_y)

def cb(msg2):
  entry_nx = [msg.header.seq, msg.pose.position.x]
  entry_ny = [msg.header.seq, msg.pose.position.y]
  nx_list.append(entry_nx)
  ny_list.append(entry_ny)
  if(len(ny_list) == max_len):
    cal_dev()
    exit()

def cal_dev():
  


def element_wise_multiplication(data1, data2):
  output = []
  for i in range(len(data1)):
    output.append(data1[i]*data2[i])
  return output


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
  sub2 = rospy.Subscriber('gnss_pose_noise', PoseStamped, cb2)

  r = rospy.Rate(100)

  rospy.spin()

if __name__=="__main__":
  main()