#!/usr/bin/env python3
#
# Copyright (c) 2019 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import os
import lgsvl
import random
import time
from pathlib import Path
import json

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)

layer_mask = 0
layer_mask |= 1 << 0  # 0 is the layer for the road (default)

if sim.current_scene == "Testbed":
  sim.reset()
else:
  sim.load("Testbed")

spawns = sim.get_spawn()
forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])
sx = spawns[0].position.x
sz = spawns[0].position.z

spawns = sim.get_spawn()

state = lgsvl.AgentState()
state.transform.position = spawns[0].position
state.transform.rotation = spawns[0].rotation

# ego = sim.add_agent("Lexus2016RXHybrid (Autoware)", lgsvl.AgentType.EGO, state)
# ego = sim.add_agent("DoubleLiDAR (Autoware)", lgsvl.AgentType.EGO, state)
ego = sim.add_agent("TripleLiDAR (Autoware)", lgsvl.AgentType.EGO, state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)



light_list = []
## Get a list of controllable objects
set_control = "green=7;red=7;yellow=3;loop"
signal = sim.get_controllables("signal")
signal[1].control(set_control)
controllables = sim.get_controllables("signal")

# print("\n# List of controllable objects in {} scene:".format(scene_name))
for idx, c in enumerate(controllables):
  light = {
    'id': 0,
    'type_list': {
      'red': 0,
      'yellow': 0,
      'green': 0
    },
    'type': 0,
    'time': 0
  }
  policy_time = []

  temp = c.__repr__()
  _temp = str.split(temp, ',')
  s_temp = str.split(_temp[14], ':')
  light['id'] = idx
  if idx == 1:
    control_policy = str.split(set_control, ';')
  else:
    control_policy = str.split(s_temp[1], ';')
    control_policy[0] = control_policy[0][2:]
  control_policy.pop()

  for color_list in control_policy:
    policy_time.append(str.split(color_list, '='))

  for color in policy_time:
    light['type_list'][color[0]] = float(color[1])
  light['time'] = int(light['type_list']['red'])
  light_list.append(light)


dict_path = os.path.join(str(Path.home()), "autoware.ai/autoware_files/lgsvl/scripts/traffic_signal")
file_path = os.path.join(dict_path, "traffic_signal_policy.json")
config_file = open(file_path, 'w')
json.dump(light_list, config_file, indent=4)
config_file.close()

sim.run()
