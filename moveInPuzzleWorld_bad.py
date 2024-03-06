"""
----- Grasp and place box by using functions of manipulation.py -----
"""


import sys
import robotic as ry
sys.path.insert(0,'/home/guest/Downloads/rai-tutorials')
#sys.path.insert(0,'../rai-tutorials')
import manipulation as manip
import numpy as np
import time

def approachObjFromTop(gripper, initial_pos, goal_pos, phases=[1,2]):

    """
    gripper: str
    initial_pos = param: initial position of the to be grabbed object
    goal_pos = param: goal position of the to be grabbed object
    """
    delta = goal_pos - initial_pos
    delta /= np.linalg.norm(delta)
    mat = np.eye(3) - np.outer(delta, delta)

    komo = ry.KOMO(C, 1., 32, 2, False)
    komo.addObjective(phases, ry.FS.positionDiff, [gripper, initial_pos], ry.OT.eq, mat)

    komo.addObjective([initial_pos], ry.FS.positionDiff, [gripper, initial_pos], ry.OT.eq, [1e1])
    komo.addObjective([goal_pos], ry.FS.positionDiff, [gripper, goal_pos], ry.OT.eq, [1e1])


def PickAndPlace(gripper, box, graspDirection, placeDirection):
    """
    returns 2 submotions: 
        M2: retract to avoid moving dragging box along 
        M3: actual motion??
    """
    M = manip.ManipulationModelling(C, helpers=[gripper])
    M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1, joint_limits=False)
    M.grasp_top_box(1., gripper, box, graspDirection)
    M.place_box(2., box, table, palm, placeDirection)
    M.target_relative_xy_position(2., box, table, place_position)
    M.target_x_orientation(2., box, place_orientation)
    M.solve()
    if not M.feasible:
        return None

    M2 = M.sub_motion(0)
    M2.retract([.0, .2], gripper)
    M2.approach([.8, 1.], gripper)
    M2.solve()
    if not M2.ret.feasible:
        return None

    M3 = M.sub_motion(1)
    M3.no_collision([], table, box)
    M3.no_collision([], box, 'obstacle')
    M3.bias(.5, qHome, 1e0)
    M3.solve()
    if not M3.ret.feasible:
        return None
    
    return M2, M3


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

#----------------------------
cube1_pos = [-1.5*0.04, 1.5*0.04, 0.0085]
cube2_pos = [-4.5*0.04, 2.5*0.04, 0.0085]
cube3_pos = [3.5*0.04, -4.5*0.04, 0.0085]

block1_pos = [-2.5*0.04, 4*0.04, 0.0085]
block2_pos = [1.5*0.04, 1*0.04, 0.0085]
block3_pos = [-4*0.04, -4.5*0.04, 0.0085]

longblock_pos = [2*0.04, -1.5*0.04, 0.0085]
corner_pos = [-2 * 0.04, -2.5 * 0.04, 0.0085] 

# Note: do not set contact
start = [-4*0.04, 0.0, 0.015]
q_goal = [-4*0.04, 4*0.04, 0.005]

C.addFrame(name='puzzle_world', parent='table') \
    .setShape(ry.ST.ssBox, [0.4, 0.4, 0.001, 0.0]) \
    .setRelativePosition([-0.2, 0.2, 0.05]) \
    .setColor([0, 1]) \
    .setContact(1)

C.addFrame(name='moving_object', parent='puzzle_world') \
    .setShape(ry.ST.ssBox, [0.03, 0.03, 0.03, 0.0]) \
    .setRelativePosition(start) \
    .setColor([1, .0, .0]) \
    .setContact(1)

def add_cube(name='cubic_obstacle', cube_pos=[0, 0, 0], lenXY_cube=0.04):
    C.addFrame(name=name, parent='puzzle_world') \
    .setShape(ry.ST.ssBox, [lenXY_cube, lenXY_cube, 0.017, 0.0]) \
    .setRelativePosition(cube_pos) \
    .setColor([.0,.0,1]) \
    .setContact(1)

def add_block(name='block_obstacle', block_pos=[0,0,0], orientation='horizontal'):
    if orientation == 'horizontal':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.08, 0.04, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0,.0,1]) \
            .setContact(1)
    elif orientation == 'vertical':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.08, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0,.0,1]) \
            .setContact(1)
    else:
        print("orientation must be either 'horizontal' or 'vertical'. ")


def add_longblock(name='longBlock_obstacle', block_pos=[0,0,0], orientation='horizontal'):
    if orientation == 'horizontal':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.16, 0.04, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0,.0,1]) \
            .setContact(1)
    elif orientation == 'vertical':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.16, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0,.0,1]) \
            .setContact(1)
    else:
        print("orientation must be either 'horizontal' or 'vertical'. ")


def add_cornerBlock(name='cornerBlock_obstacle', block_pos=[0, 0, 0], orientation='upper_right'):

    """
    :param name:        name of the corner block
    :param block_pos:   position of center of long horizontal 4x8 block
    :param orientation: upper/lower depending if long horizontal block is above or under the small block;
                        right/left depending if small block is on left or right end of long horizontal block
    """

    C.addFrame(name=name, parent='puzzle_world') \
        .setShape(ry.ST.ssBox, [0.08, 0.04, 0.017, 0.0]) \
        .setRelativePosition(block_pos) \
        .setColor([.0, .0, 1]) \
        .setContact(1)
    if orientation == 'upper_right':
        smallBlock_pos = [block_pos[0] + 0.02, block_pos[1] + 0.04, block_pos[2]]
        C.addFrame(name='{name}_1', parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'upper_left':
        smallBlock_pos = [block_pos[0] - 0.02, block_pos[1] + 0.04, block_pos[2]]
        C.addFrame(name='{name}_1', parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'lower_right':
        smallBlock_pos = [block_pos[0] + 0.02, block_pos[1] - 0.04, block_pos[2]]
        C.addFrame(name='{name}_1', parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'lower_left':
        smallBlock_pos = [block_pos[0] - 0.02, block_pos[1] - 0.04, block_pos[2]]
        C.addFrame(name='{name}_1', parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    else:
        print("orientation must be either 'upper_right', 'upper_left', 'lower_right' or 'lower_left'. ")


def add_goal(goal_pos, goal_size):
    C.addFrame(name='goal', parent='puzzle_world') \
    .setShape(ry.ST.ssBox, [goal_size, goal_size, 0.01, 0.0]) \
    .setRelativePosition(goal_pos) \
    .setColor([1., 1., 0]) \
    .setContact(0)
    
add_cube('cube1', cube1_pos)
add_cube('cube2', cube2_pos)
add_cube('cube3', cube3_pos)

add_block('block1', block1_pos, 'vertical')
add_block('block2', block2_pos, 'vertical')
add_block('block3', block3_pos, 'horizontal')

add_longblock('longBlock', longblock_pos, 'horizontal')
add_goal(q_goal, 0.08)
add_cornerBlock('cornerBlock', corner_pos, 'upper_left')

#----------------------------


bot = ry.BotOp(C, False)
bot.home(C)
C.view(True)

time.sleep(1)
qHome = C.getJointState()   # get current joint states of the robot
limits = C.getJointLimits()
gripper = "l_gripper";
palm = "l_palm";
box = "moving_object";
table = "puzzle_world";
boxSize = C.frame(box).getSize()


place_pos = [[-.3, .1], [0.1, .1]]
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
    

    bot.move(path=M2.path, times=[3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

    bot.gripperClose(ry._left, speed=.2)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    bot.move(path=M3.path, times=[3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

    bot.gripperMove(ry._left, width=.08, speed=.1)
    while not bot.gripperDone(ry._left):
        bot.sync(C)
    
bot.home(C)
