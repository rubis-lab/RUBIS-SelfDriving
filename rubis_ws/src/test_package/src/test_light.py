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
from std_msgs.msg import String



## main ##
pub = rospy.Publisher('traffic_signal', String, queue_size=50)
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

for light in light_list:
  sync_time = 5 + float(light['time'])
  topic = {
    'id': light['id'],
    'type': light['type'],
    'time': sync_time
  }
  topic_list.append(topic)
  topic_typelist.append(light['type_list'])


while True:
  for topic in topic_list:
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
  data = json.dumps(topic_list, indent=4)
  print(data)
  rospy.loginfo(data)
  pub.publish(data)
  rate.sleep()
