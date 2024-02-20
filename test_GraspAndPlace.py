"""
----- Grasp and place box by using only functions of manipulation.py -----
"""


import sys
import robotic as ry
sys.path.insert(0,'/home/guest/Downloads/rai-tutorials')
import manipulation as manip
import numpy as np
import time


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

# C.addFrame("box", "table") \
#     .setJoint(ry.JT.rigid) \
#     .setShape(ry.ST.ssBox, [.15,.06,.06,.005]) \
#     .setPosition([0.,.5,.7]) \
#     .setRelativePosition([-.0,.3-.055,.095]) \
#     .setColor([1, 0.5, 0]) \
#     .setContact(1) \
#     .setMass(.1)

# C.addFrame("obstacle", "table") \
#     .setJoint(ry.JT.rigid) \
#     .setShape(ry.ST.ssBox, [.06,.15,.06,.005]) \
#     .setColor([0, 0, 1]) \
#     .setRelativePosition([-.15,.3-.055,.095]) \
#     .setContact(1)

C.addFrame('obstacle', 'table') \
    .setShape(ry.ST.ssBox, [.2,.3,.05,.005]) \
    .setRelativePosition([-.15,.3-.055,.095]) \
    .setColor([.0,.0,1]) \
    .setContact(1)

C.addFrame("box", "table") \
    .setJoint(ry.JT.rigid) \
    .setShape(ry.ST.ssBox, [.1,.05,.05,0.]) \
    .setRelativePosition([-.0,.3-.055,.095]) \
    .setColor([1.,0.5,0.]) \
    .setContact(1) \
    .setMass(.1)
C.view()

time.sleep(1)
qHome = C.getJointState()   # get current joint states of the robot
gripper = "l_gripper";
palm = "l_palm";
box = "box";
table = "table";
boxSize = C.frame(box).getSize()


C.view_raise()
C.frame(box).setRelativePosition([-.0,.3-.055,.095])
C.frame(box).setRelativeQuaternion([1.,0,0,0])

# obstacle in [-.0,.3-.055,.095] with size [.2,.3,.05,.005]
place_pos = [[-.3, .2],[0.1, .2], [0,0]]
qStart = C.getJointState()  # get state of each joint (i.e 7 joints for panda)

graspDirection = 'xz'       # grasps box at x-side from the z-direction (top) 
placeDirection = 'z'
for i_pos in place_pos:
    place_position = i_pos
    place_orientation = [0,1,0.]

    M = manip.ManipulationModelling(C, helpers=[gripper])
    M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1, joint_limits=False)
    M.grasp_top_box(1., gripper, box, graspDirection)
    M.place_box(2., box, table, palm, placeDirection)
    M.target_relative_xy_position(2., box, table, place_position)
    M.target_x_orientation(2., box, place_orientation)
    M.solve()
    if not M.feasible:
            continue

    M2 = M.sub_motion(0)
    M2.retract([.0, .2], gripper)
    M2.approach([.8, 1.], gripper)
    M2.solve()
    if not M2.ret.feasible:
        continue

    M3 = M.sub_motion(1)
    M3.no_collision([], table, box)
    M3.no_collision([], box, 'obstacle')
    M3.bias(.5, qHome, 1e0)
    M3.solve()
    if not M3.ret.feasible:
        continue

    M2.play(C)
    C.attach(gripper, box)
    M3.play(C)
    C.attach(table, box)
