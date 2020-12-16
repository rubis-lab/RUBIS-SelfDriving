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

sim = lgsvl.Simulator(os.environ.get("SIMULATOR_HOST", "127.0.0.1"), 8181)

layer_mask = 0
layer_mask |= 1 << 0  # 0 is the layer for the road (default)

if sim.current_scene == "testbed":
  sim.reset()
else:
  sim.load("testbed")

spawns = sim.get_spawn()
forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])
sx = spawns[0].position.x
sz = spawns[0].position.z

spawns = sim.get_spawn()

state = lgsvl.AgentState()
state.transform = spawns[0]

ego = sim.add_agent("Lexus2016RXHybrid (Autoware)", lgsvl.AgentType.EGO, state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)

waypoints = []
speed = 7

#set start waypoint
start = spawns[0].position + 81 * forward + 44 * right

hit = sim.raycast(start, lgsvl.Vector(0, -1, 0), layer_mask)
#you can change trigger_distance what you want
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=5.0, trigger_distance=60, trigger=None)
waypoints.append(wp)


second = spawns[0].position + 81 * forward + 20 * right

hit = sim.raycast(second, lgsvl.Vector(0, -1, 0), layer_mask)
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=3.0, trigger_distance=0, trigger=None)
waypoints.append(wp)

third = spawns[0].position + 110 * forward + 8 * right

hit = sim.raycast(third, lgsvl.Vector(0, -1, 0), layer_mask)
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=None)
waypoints.append(wp)

end = spawns[0].position + 110 * forward - 3 * right

hit = sim.raycast(end, lgsvl.Vector(0, -1, 0), layer_mask)
wp = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=None)
waypoints.append(wp)

#set position of pedestrian
state = lgsvl.AgentState()
state.transform.position = spawns[0].position + 81 * forward + 45 * right
state.transform.rotation = spawns[0].rotation

pedestrian = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, state)
pedestrian.follow(waypoints, False)

def on_waypoint(agent, index):
  print("Waypoint {} reached".format(index))

pedestrian.on_waypoint_reached(on_waypoint)


for sensor in ego.get_sensors():
  if sensor.name == "Lidar":
    sensor.save("lidar.pcd")

print(sim.get_agents())
sim.run()