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
state.transform.position = spawns[0].position + 5 * right
state.transform.rotation = spawns[0].rotation

ego = sim.add_agent("Lexus2016RXHybrid (Autoware)", lgsvl.AgentType.EGO, state)
ego.connect_bridge(os.environ.get("BRIDGE_HOST", "127.0.0.1"), 9090)

set_control = "red=7;yellow=3;green=7;loop"
signal = sim.get_controllables("signal")
signal[1].control(set_control)


#------- Fast Pedestrian -------#
fp_waypoints = []
speed = 7

#set start waypoint
start = spawns[0].position + 81 * forward + 44 * right

hit = sim.raycast(start, lgsvl.Vector(0, -1, 0), layer_mask)
#you can change trigger_distance what you want
fp_wp1 = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=5.0, trigger_distance=60, trigger=None)
fp_waypoints.append(fp_wp1)


second = spawns[0].position + 81 * forward + 20 * right

hit = sim.raycast(second, lgsvl.Vector(0, -1, 0), layer_mask)
fp_wp2 = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=8.0, trigger_distance=0, trigger=None)
fp_waypoints.append(fp_wp2)

third = spawns[0].position + 110 * forward + 8 * right

hit = sim.raycast(third, lgsvl.Vector(0, -1, 0), layer_mask)
fp_wp3 = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=None)
fp_waypoints.append(fp_wp3)

end = spawns[0].position + 110 * forward - 3 * right

hit = sim.raycast(end, lgsvl.Vector(0, -1, 0), layer_mask)
fp_wp4 = lgsvl.WalkWaypoint(position=hit.point, speed=speed, idle=0, trigger_distance=0, trigger=None)
fp_waypoints.append(fp_wp4)

#set position of fast pedestrian
fp_state = lgsvl.AgentState()
fp_state.transform.position = spawns[0].position + 81 * forward + 45 * right
fp_state.transform.rotation = spawns[0].rotation

fast_pedestrian = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, fp_state)
fast_pedestrian.follow(fp_waypoints, False)

def on_waypoint(agent, index):
  print("Waypoint {} reached".format(index))

fast_pedestrian.on_waypoint_reached(on_waypoint)


#------- Stand vehicle -------#
#set stand vehicle's initial position
sv_state = lgsvl.AgentState()
sv_state.transform.position = spawns[0].position + 30 * forward + 6 * right
sv_state.transform.rotation = spawns[0].rotation

stand_vehicle = sim.add_agent("Sedan", lgsvl.AgentType.NPC, sv_state)


#------- Narrow path -------#
#set np vehicle1's initial position
np1_state = lgsvl.AgentState()
np1_state.transform.position = spawns[0].position + 270 * forward + 3.5 * right
np1_state.transform.rotation = spawns[0].rotation

np_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, np1_state)

np2_state = lgsvl.AgentState()
np2_state.transform.position = spawns[0].position + 270 * forward - 5 * right
np2_state.transform.rotation = lgsvl.Vector(0, -180, 0)

np_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, np2_state)


#------- Construction section -------#

#set traffic cone1
cone1_state = lgsvl.ObjectState()
cone1_state.transform.position = spawns[0].position + 385 * forward - right
cone1_state.transform.rotation = lgsvl.Vector(0, 0, 0)

cone1 = sim.controllable_add("TrafficCone", cone1_state)

#set traffic cone2
cone2_state = lgsvl.ObjectState()
cone2_state.transform.position = spawns[0].position + 385 * forward
cone2_state.transform.rotation = lgsvl.Vector(0, 0, 0)

cone2 = sim.controllable_add("TrafficCone", cone2_state)

#set traffic cone3
cone3_state = lgsvl.ObjectState()
cone3_state.transform.position = spawns[0].position + 385 * forward + right
cone3_state.transform.rotation = lgsvl.Vector(0, 0, 0)

cone3 = sim.controllable_add("TrafficCone", cone3_state)


#set worker
worker_state = lgsvl.ObjectState()
worker_state.transform.position = spawns[0].position + 387 * forward
worker_state.transform.rotation = lgsvl.Vector(0, 0, 0)
worker = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, worker_state)


#------- Move pedestrian -------#

mp_waypoints = []

#set start waypoint of cross walk
start = spawns[0].position + 460 * forward + 8 * right

mp_hit1 = sim.raycast(start, lgsvl.Vector(0, -1, 0), layer_mask)

#you can change trigger_distance what you want
mp_wp1 = lgsvl.WalkWaypoint(position=mp_hit1.point, speed=1, idle=0, trigger_distance=30, trigger=None)
mp_waypoints.append(mp_wp1)

#set end waypoint of cross walk
end = start - 23 * right

mp_hit2 = sim.raycast(end, lgsvl.Vector(0, -1, 0), layer_mask)
mp_wp2 = lgsvl.WalkWaypoint(position=mp_hit2.point, speed=1, idle=0, trigger_distance=0, trigger=None)
mp_waypoints.append(mp_wp2)

#set position of move pedestrian
mp_state = lgsvl.AgentState()
mp_state.transform.position = spawns[0].position + 460 * forward + 9 * right
mp_state.transform.rotation = spawns[0].rotation

move_pedestrian = sim.add_agent("Bob", lgsvl.AgentType.PEDESTRIAN, mp_state)
move_pedestrian.follow(mp_waypoints, False)


#------- Congestion section -------#
#set cs_vehicle1's initial position
cs1_state = lgsvl.AgentState()
cs1_state.transform.position = spawns[0].position + 485 * forward - 40 * right
cs1_state.transform.rotation = lgsvl.Vector(0, -90, 0)

cs_angle = cs1_state.transform.rotation
cs_speed = 2

#set cs_vehicle1's start waypoint of congestion section
cs1_waypoints = []
cs1_start = cs1_state.transform.position - 2 * right
cs1_hit1 = sim.raycast(cs1_start, lgsvl.Vector(0, -1, 0), layer_mask)
cs1_wp1 = lgsvl.DriveWaypoint(position=cs1_hit1.point, speed=cs_speed, angle=cs_angle, idle=0,
                              trigger_distance=30, trigger=None)
cs1_waypoints.append(cs1_wp1)

#set cs_vehicle1's end waypoint of congestion section
cs1_end = cs1_start - 100 * right
cs1_hit2 = sim.raycast(cs1_end, lgsvl.Vector(0, -1, 0), layer_mask)
cs1_wp2 = lgsvl.DriveWaypoint(position=cs1_hit2.point, speed=cs_speed, angle=cs_angle, idle=0,
                              trigger_distance=0, trigger=None)
cs1_waypoints.append(cs1_wp2)

cs_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, cs1_state)
cs_vehicle1.follow(cs1_waypoints)

#set cs_vehicle2's initial position
cs2_state = lgsvl.AgentState()
cs2_state.transform.position = spawns[0].position + 481 * forward - 60 * right
cs2_state.transform.rotation = lgsvl.Vector(0, -90, 0)

cs2_waypoints = []

#set cs_vehicle2's start waypoint of congestion section
cs2_start = cs2_state.transform.position - 2 * right
cs2_hit1 = sim.raycast(cs2_start, lgsvl.Vector(0, -1, 0), layer_mask)
cs2_wp1 = lgsvl.DriveWaypoint(position=cs2_hit1.point, speed=cs_speed, angle=cs_angle, idle=0,
                              trigger_distance=50, trigger=None)
cs2_waypoints.append(cs2_wp1)

#set cs_vehicle2's end waypoint of congestion section
cs2_end = cs2_start - 100 * right
cs2_hit2 = sim.raycast(cs2_end, lgsvl.Vector(0, -1, 0), layer_mask)
cs2_wp2 = lgsvl.DriveWaypoint(position=cs2_hit2.point, speed=cs_speed, angle=cs_angle, idle=0,
                              trigger_distance=0, trigger=None)
cs2_waypoints.append(cs2_wp2)

cs_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, cs2_state)
cs_vehicle2.follow(cs2_waypoints)


#------- Cut in scenario 1 -------#

ci1_state = lgsvl.AgentState()
ci1_state.transform.position = spawns[0].position + 483 * forward - 250 * right
ci1_state.transform.rotation = lgsvl.Vector(0, -90, 0)

ci_speed = 6
ci_angle = ci1_state.transform.rotation

ci1_waypoints = []

#set ci_vehicle1's waypoints of Cut in scenario
ci1_start = ci1_state.transform.position - 2 * forward
ci1_wp1 = lgsvl.DriveWaypoint(position=lgsvl.Vector(ci1_start.x, ci1_start.y, ci1_start.z), speed=ci_speed,
                              angle=ci_angle, idle=0, trigger_distance=20, trigger=None)
ci1_waypoints.append(ci1_wp1)

ci1_way1 = ci1_start - 30 * right
ci1_wp2 = lgsvl.DriveWaypoint(position=lgsvl.Vector(ci1_way1.x, ci1_way1.y, ci1_way1.z), speed=ci_speed, angle=ci_angle,
                              idle=0, trigger_distance=0, trigger=None)
ci1_waypoints.append(ci1_wp2)

ci1_way2 = ci1_way1 + 5 * forward - 10 * right
ci1_wp3 = lgsvl.DriveWaypoint(position=lgsvl.Vector(ci1_way2.x, ci1_way2.y, ci1_way2.z), speed=ci_speed, angle=ci_angle,
                              idle=0, trigger_distance=0, trigger=None)
ci1_waypoints.append(ci1_wp3)

ci1_end = ci1_way2 - 30 * right
ci1_wp4 = lgsvl.DriveWaypoint(position=lgsvl.Vector(ci1_end.x, ci1_end.y, ci1_end.z), speed=ci_speed,
                              angle=ci_angle, idle=0, trigger_distance=0, trigger=None)
ci1_waypoints.append(ci1_wp4)

ci_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, ci1_state)
ci_vehicle1.follow(ci1_waypoints)

# #----dump----#
# ci1_state = lgsvl.AgentState()
# ci1_state.transform.position = spawns[0].position + 450 * forward - 615 * right
# ci1_state.transform.rotation = lgsvl.Vector(0, -180, 0)
#
# ci_speed = 6
# ci_angle = ci1_state.transform.rotation
#
# ci1_waypoints = []
#
# #set ci_vehicle1's waypoints of Cut in scenario
# ci1_start = ci1_state.transform.position - 2 * forward
# ci1_hit1 = sim.raycast(ci1_start, lgsvl.Vector(0, -1, 0), layer_mask)
# ci1_wp1 = lgsvl.DriveWaypoint(position=ci1_hit1.point, speed=ci_speed, angle=ci_angle, idle=0,
#                               trigger_distance=20, trigger=None)
# ci1_waypoints.append(ci1_wp1)
#
# ci1_way1 = ci1_start - 30 * forward
# ci1_hit2 = sim.raycast(ci1_way1, lgsvl.Vector(0, -1, 0), layer_mask)
# ci1_wp2 = lgsvl.DriveWaypoint(position=ci1_hit2.point, speed=ci_speed, angle=ci_angle, idle=0,
#                               trigger_distance=0, trigger=None)
# ci1_waypoints.append(ci1_wp2)
#
# ci1_way2 = ci1_way1 - 10 * forward - 5 * right
# ci1_hit3 = sim.raycast(ci1_way2, lgsvl.Vector(0, -1, 0), layer_mask)
# ci1_wp3 = lgsvl.DriveWaypoint(position=ci1_hit3.point, speed=ci_speed, angle=ci_angle, idle=0,
#                               trigger_distance=0, trigger=None)
# ci1_waypoints.append(ci1_wp3)
#
# ci1_end = ci1_way2 - 30 * forward
# ci1_hit4 = sim.raycast(ci1_end, lgsvl.Vector(0, -1, 0), layer_mask)
# ci1_wp4 = lgsvl.DriveWaypoint(position=ci1_hit4.point, speed=ci_speed, angle=ci_angle, idle=0,
#                               trigger_distance=0, trigger=None)
# ci1_waypoints.append(ci1_wp4)
#
# ci_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, ci1_state)
# ci_vehicle1.follow(ci1_waypoints)


#set all of npc_vehicle speed on scenes
npc_speed = 6

#------- NPC car scene 1 -------#
npc1_s1_state = lgsvl.AgentState()
npc1_s1_state.transform.position = spawns[0].position + 90 * forward - 100 * right
npc1_s1_state.transform.rotation = lgsvl.Vector(0, 90, 0)

npc_s1_angle = npc1_s1_state.transform.rotation

npc1_s1_waypoints = []

#set npc_s1_vehicle1's waypoint1
npc1_s1_start = npc1_s1_state.transform.position + 2 * right
npc1_s1_hit1 = sim.raycast(npc1_s1_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_wp1 = lgsvl.DriveWaypoint(position=npc1_s1_hit1.point, speed=npc_speed, angle=npc_s1_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s1_waypoints.append(npc1_wp1)

#set npc_s1_vehicle1's waypoint2
npc1_s1_way1 = npc1_s1_start + 500 * right
npc1_s1_hit2 = sim.raycast(npc1_s1_way1, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s1_wp2 = lgsvl.DriveWaypoint(position=npc1_s1_hit2.point, speed=npc_speed, angle=npc_s1_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s1_waypoints.append(npc1_s1_wp2)

#set npc_s1_vehicle1
npc_s1_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc1_s1_state)
npc_s1_vehicle1.follow(npc1_s1_waypoints)


npc2_s1_state = lgsvl.AgentState()
npc2_s1_state.transform.position = spawns[0].position + 85 * forward - 80 * right
npc2_s1_state.transform.rotation = lgsvl.Vector(0, 90, 0)


npc2_s1_waypoints = []

#set npc_s1_vehicle2's waypoint1
npc2_s1_start = npc2_s1_state.transform.position + 2 * right
npc2_s1_hit1 = sim.raycast(npc2_s1_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s1_wp1 = lgsvl.DriveWaypoint(position=npc2_s1_hit1.point, speed=npc_speed, angle=npc_s1_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s1_waypoints.append(npc2_s1_wp1)

#set npc_s1_vehicle2's waypoint2
npc2_s1_way1 = npc2_s1_start + 500 * right
npc2_s1_hit2 = sim.raycast(npc2_s1_way1, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s1_wp2 = lgsvl.DriveWaypoint(position=npc2_s1_hit2.point, speed=npc_speed, angle=npc_s1_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s1_waypoints.append(npc2_s1_wp2)

#set npc_s1_vehicle2
npc_s1_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc2_s1_state)
npc_s1_vehicle2.follow(npc2_s1_waypoints)

#------- NPC car scene 2 -------#

### scene2 npc 1 ###
npc1_s2_state = lgsvl.AgentState()
npc1_s2_state.transform.position = spawns[0].position + 200 * forward - 10 * right
npc1_s2_state.transform.rotation = lgsvl.Vector(0, 180, 0)

npc_s2_angle = npc1_s2_state.transform.rotation


npc1_s2_waypoints = []

#set npc_s2_vehicle1's waypoint1
npc1_s2_start = npc1_s2_state.transform.position - 2 * forward
npc1_s2_hit1 = sim.raycast(npc1_s2_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s2_wp1 = lgsvl.DriveWaypoint(position=npc1_s2_hit1.point, speed=npc_speed, angle=npc_s2_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s2_waypoints.append(npc1_s2_wp1)

#set npc_s2_vehicle1's waypoint2
npc1_s2_way1 = npc1_s2_start - 98 * forward
npc1_s2_hit2 = sim.raycast(npc1_s2_way1, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s2_wp2 = lgsvl.DriveWaypoint(position=npc1_s2_hit2.point, speed=npc_speed, angle=npc_s2_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s2_waypoints.append(npc1_s2_wp2)

npc_s2_angle1 = lgsvl.Vector(0, 270, 0)

#set npc_s2_vehicle1's waypoint3
npc1_s2_way2 = npc1_s2_way1 - 2 * right
npc1_s2_hit3 = sim.raycast(npc1_s2_way2, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s2_wp3 = lgsvl.DriveWaypoint(position=npc1_s2_hit3.point, speed=npc_speed, angle=npc_s2_angle1, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s2_waypoints.append(npc1_s2_wp3)

#set npc_s2_vehicle1's waypoint4
npc1_s2_way3 = npc1_s2_way2 - 1000 * right
npc1_s2_hit4 = sim.raycast(npc1_s2_way3, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s2_wp4 = lgsvl.DriveWaypoint(position=npc1_s2_hit4.point, speed=npc_speed, angle=npc_s2_angle1, idle=0,
                              trigger_distance=0, trigger=None)
npc1_s2_waypoints.append(npc1_s2_wp4)

#set npc_s2_vehicle1
npc_s2_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc1_s2_state)
npc_s2_vehicle1.follow(npc1_s2_waypoints)

### scene2 npc 2 ###
npc2_s2_state = lgsvl.AgentState()
npc2_s2_state.transform.position = spawns[0].position + 180 * forward - 6 * right
npc2_s2_state.transform.rotation = lgsvl.Vector(0, 180, 0)

npc_s2_angle = npc1_s2_state.transform.rotation


npc2_s2_waypoints = []

#set npc_s2_vehicle2's waypoint1
npc2_s2_start = npc2_s2_state.transform.position - 2 * forward
npc2_s2_hit1 = sim.raycast(npc2_s2_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s2_wp1 = lgsvl.DriveWaypoint(position=npc2_s2_hit1.point, speed=npc_speed, angle=npc_s2_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s2_waypoints.append(npc2_s2_wp1)

#set npc_s2_vehicle2's waypoint2
npc2_s2_way1 = npc2_s2_start - 82 * forward
npc2_s2_hit2 = sim.raycast(npc2_s2_way1, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s2_wp2 = lgsvl.DriveWaypoint(position=npc2_s2_hit2.point, speed=npc_speed, angle=npc_s2_angle, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s2_waypoints.append(npc2_s2_wp2)

npc_s2_angle1 = lgsvl.Vector(0, 270, 0)

#set npc_s2_vehicle2's waypoint3
npc2_s2_way2 = npc2_s2_way1 - 2 * right
npc2_s2_hit3 = sim.raycast(npc2_s2_way2, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s2_wp3 = lgsvl.DriveWaypoint(position=npc2_s2_hit3.point, speed=npc_speed, angle=npc_s2_angle1, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s2_waypoints.append(npc2_s2_wp3)

#set npc_s2_vehicle2's waypoint4
npc2_s2_way3 = npc2_s2_way2 - 1000 * right
npc2_s2_hit4 = sim.raycast(npc2_s2_way3, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s2_wp4 = lgsvl.DriveWaypoint(position=npc2_s2_hit4.point, speed=npc_speed, angle=npc_s2_angle1, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s2_waypoints.append(npc2_s2_wp4)

#set npc_s2_vehicle2
npc_s2_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc2_s2_state)
npc_s2_vehicle2.follow(npc2_s2_waypoints)


#------- NPC car scene 3 -------#

### scene3 npc 1 ###
npc1_s3_state = lgsvl.AgentState()
npc1_s3_state.transform.position = spawns[0].position + 475 * forward - 100 * right
npc1_s3_state.transform.rotation = lgsvl.Vector(0, 90, 0)
npc_s3_angle = npc1_s3_state.transform.rotation

npc1_s3_waypoints = []

#set npc_s3_vehicle2's waypoint1
npc1_s3_start = npc1_s3_state.transform.position + 2 * right
npc1_s3_hit1 = sim.raycast(npc1_s3_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s3_wp1 = lgsvl.DriveWaypoint(position=npc1_s3_hit1.point, speed=npc_speed, angle=npc_s3_angle, idle=0,
                              trigger_distance=100, trigger=None)
npc1_s3_waypoints.append(npc1_s3_wp1)

#set npc_s3_vehicle2's waypoint2
npc1_s3_way1 = npc1_s3_start + 1000 * right
npc1_s3_wp2 = lgsvl.DriveWaypoint(position=lgsvl.Vector(npc1_s3_way1.x, npc1_s3_way1.y, npc1_s3_way1.z),
                                  speed=npc_speed, angle=npc_s3_angle, idle=0, trigger_distance=0, trigger=None)
npc1_s3_waypoints.append(npc1_s3_wp2)

#set npc_s3_vehicle1
npc_s3_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc1_s3_state)
npc_s3_vehicle1.follow(npc1_s3_waypoints)


### scene3 npc 2 ###
npc2_s3_state = lgsvl.AgentState()
npc2_s3_state.transform.position = spawns[0].position + 470 * forward - 100 * right
npc2_s3_state.transform.rotation = lgsvl.Vector(0, 90, 0)

npc2_s3_waypoints = []

#set npc_s3_vehicle2's waypoint1
npc2_s3_start = npc2_s3_state.transform.position + 2 * right
npc2_s3_hit1 = sim.raycast(npc2_s3_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s3_wp1 = lgsvl.DriveWaypoint(position=npc2_s3_hit1.point, speed=npc_speed, angle=npc_s3_angle, idle=0,
                              trigger_distance=100, trigger=None)
npc2_s3_waypoints.append(npc2_s3_wp1)

#set npc_s3_vehicle2's waypoint2
npc2_s3_way1 = npc2_s3_start + 88 * right
npc2_s3_hit2 = sim.raycast(npc2_s3_way1, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s3_wp2 = lgsvl.DriveWaypoint(position=npc2_s3_hit2.point, speed=npc_speed, angle=npc_s3_angle, idle=5,
                              trigger_distance=0, trigger=None)
npc2_s3_waypoints.append(npc2_s3_wp2)

npc_s3_angle1 = lgsvl.Vector(0, -180, 0)

#set npc_s3_vehicle2's waypoint3
npc2_s3_way2 = npc2_s3_way1 - 2 * forward
npc2_s3_hit3 = sim.raycast(npc2_s3_way2, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s3_wp3 = lgsvl.DriveWaypoint(position=npc2_s3_hit3.point, speed=npc_speed, angle=npc_s3_angle1, idle=0,
                              trigger_distance=0, trigger=None)
npc2_s3_waypoints.append(npc2_s3_wp3)

#set npc_s3_vehicle2's waypoint4
npc2_s3_way3 = npc2_s3_way2 - 1000 * forward
npc2_s3_wp4 = lgsvl.DriveWaypoint(position=lgsvl.Vector(npc2_s3_way3.x, npc2_s3_way3.y, npc2_s3_way3.z),
                                  speed=npc_speed, angle=npc_s3_angle1, idle=0, trigger_distance=0, trigger=None)
npc2_s3_waypoints.append(npc2_s3_wp4)

#set npc_s3_vehicle2
npc_s3_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc2_s3_state)
npc_s3_vehicle2.follow(npc2_s3_waypoints)


#------- NPC car scene 4 -------#

### scene4 npc 1 ###
npc1_s4_state = lgsvl.AgentState()
npc1_s4_state.transform.position = spawns[0].position + 475 * forward - 500 * right
npc1_s4_state.transform.rotation = lgsvl.Vector(0, 90, 0)
npc_s4_angle = npc1_s4_state.transform.rotation

npc1_s4_waypoints = []

#set npc_s3_vehicle2's waypoint1
npc1_s4_start = npc1_s4_state.transform.position + 2 * right
npc1_s4_hit1 = sim.raycast(npc1_s4_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc1_s4_wp1 = lgsvl.DriveWaypoint(position=npc1_s3_hit1.point, speed=npc_speed, angle=npc_s4_angle, idle=0,
                              trigger_distance=100, trigger=None)
npc1_s4_waypoints.append(npc1_s4_wp1)

#set npc_s3_vehicle2's waypoint2
npc1_s4_way1 = npc1_s4_start + 1000 * right
npc1_s4_wp2 = lgsvl.DriveWaypoint(position=lgsvl.Vector(npc1_s4_way1.x, npc1_s4_way1.y, npc1_s4_way1.z),
                                  speed=npc_speed, angle=npc_s3_angle, idle=0, trigger_distance=0, trigger=None)
npc1_s4_waypoints.append(npc1_s4_wp2)

#set npc_s3_vehicle1
npc_s4_vehicle1 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc1_s4_state)
npc_s4_vehicle1.follow(npc1_s4_waypoints)

### scene4 npc 2 ###
npc2_s4_state = lgsvl.AgentState()
npc2_s4_state.transform.position = spawns[0].position + 470 * forward - 550 * right
npc2_s4_state.transform.rotation = lgsvl.Vector(0, 90, 0)
npc_s4_angle = npc2_s4_state.transform.rotation

npc2_s4_waypoints = []

#set npc_s3_vehicle2's waypoint1
npc2_s4_start = npc2_s4_state.transform.position + 2 * right
npc2_s4_hit1 = sim.raycast(npc2_s4_start, lgsvl.Vector(0, -1, 0), layer_mask)
npc2_s4_wp1 = lgsvl.DriveWaypoint(position=npc2_s3_hit1.point, speed=npc_speed, angle=npc_s4_angle, idle=0,
                              trigger_distance=100, trigger=None)
npc2_s4_waypoints.append(npc2_s4_wp1)

#set npc_s3_vehicle2's waypoint2
npc2_s4_way1 = npc2_s4_start + 1000 * right
npc2_s4_wp2 = lgsvl.DriveWaypoint(position=lgsvl.Vector(npc2_s4_way1.x, npc2_s4_way1.y, npc2_s4_way1.z),
                                  speed=npc_speed, angle=npc_s4_angle, idle=0, trigger_distance=0, trigger=None)
npc2_s4_waypoints.append(npc2_s4_wp2)

#set npc_s3_vehicle1
npc_s4_vehicle2 = sim.add_agent("Sedan", lgsvl.AgentType.NPC, npc2_s4_state)
npc_s4_vehicle2.follow(npc2_s4_waypoints)


sim.run()