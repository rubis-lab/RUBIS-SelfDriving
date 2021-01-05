#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl
import json
import time
import subprocess
import rospy
from pathlib import Path
from autoware_msgs.msg import RUBISTrafficSignalArray, RUBISTrafficSignal

## main ##
pub = rospy.Publisher('v2x_traffic_signal', RUBISTrafficSignalArray, queue_size=50)
rospy.init_node('traffic_signal_pub', anonymous=True)
rate = rospy.Rate(10)
remain_time = float(10)

dict_path = os.path.join(str(Path.home()), "RUBIS-SelfDriving/autoware_files/lgsvl/scripts/traffic_signal")
file_path = os.path.join(dict_path, "traffic_signal_policy.json")
subprocess.Popen(["./test_light.py"],cwd=dict_path)
with open(file_path, "r") as read_json:
  light_list = json.load(read_json)

topic_list = []
topic_typelist = []

signal_array_msg = RUBISTrafficSignalArray()

for light in light_list:
  sync_time = 5 + float(light['time'])
  topic = {
    'id': light['id'],
    'type': light['type'],
    'time': sync_time
  }
  topic_list.append(topic)
  topic_typelist.append(light['type_list'])

  signal_msg = RUBISTrafficSignal()
  signal_msg.id = light['id']
  signal_msg.type = light['type']
  signal_msg.time = sync_time
  signal_array_msg.signals.append(signal_msg)

###
# 0 : Red
# 1 : Yellow
# 2 : Green

while not rospy.is_shutdown():
  for (i, topic) in enumerate(topic_list):
    if topic['time'] < 0.05:
      if topic['type'] == 0:
        topic['type'] = 1
        topic['time'] = float(topic_typelist[topic['id']]['yellow'])
      elif topic['type'] == 1:
        topic['type'] = 2
        topic['time'] = float(topic_typelist[topic['id']]['green'])
      elif topic['type'] == 2:
        topic['type'] = 0
        topic['time'] = float(topic_typelist[topic['id']]['red'])
    topic['time'] = topic['time'] - (1/remain_time)

    signal_array_msg.signals[i].type = topic['type']
    signal_array_msg.signals[i].time = topic['time']

  data = json.dumps(topic_list, indent=4)
  # print(data)
  # rospy.loginfo(data)
  pub.publish(signal_array_msg)
  rate.sleep()
