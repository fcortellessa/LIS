import robotic as ry
import numpy as np
import time


# position is defined by center of object
# position: distance between two holes on puzzle board is 4 cm

cube1_pos = [-1.5*0.04, 1.5*0.04, 0]
cube2_pos = [-4.5*0.04, 2.5*0.04, 0]
cube3_pos = [3.5*0.04, -4.5*0.04, 0]

block1_pos = [-2.5*0.04, 4*0.04, 0]
block2_pos = [1.5*0.04, 1*0.04, 0]
block3_pos = [-4*0.04, -4.5*0.04, 0]

longblock_pos = [2*0.04, -1.5*0.04, 0]
corner_pos = [-3.5*0.04, -3*0.04, 0]

# Note: do not set contact
start = [-4*0.04, 0.0, 0.0]
q_goal = [-4*0.04, 4*0.04, 0]


C = ry.Config()     # initialize empty config

# Note: think of height of table and add half of the height to all position z-coordinates
C.addFrame(name='puzzle_world') \
    .setShape(ry.ST.ssBox, [0.4, 0.4, 0.001, 0.0]) \
    .setPosition([0.0, 0.0, 0.0]) \
    .setColor([.3, .3, .3]) \
    .setContact(1)

# !! ToDo: make relative position [0, 0, 0] as top of puzzle_world table 
C.addFrame(name='moving_object', parent='puzzle_world') \
    .setShape(ry.ST.ssBox, [0.03, 0.03, 0.03, 0.0]) \
    .setRelativePosition(start) \
    .setColor([1, .0, .0]) \
    .setContact(1)


# build maze with available blocks: 3 cubes, 3 blocks, 1 long block and 1 corner block
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

def add_cornerBlock(name='cornerBlock_obstacle', block_pos=[0,0,0], orientation='upper_right'):
    C.addFrame(name=name, parent='puzzle_world') \
    .setRelativePosition(block_pos) \
    .setColor([.0,.0,1]) \
    .setContact(1)
    if orientation == 'upper_right':
        C.setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0])
    elif orientation == 'upper_left':
        C.setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, .0])
    elif orientation == 'lower_right':
        C.setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0])
    elif orientation == 'lower_left':
        C.setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0])
    else:
        print("orientation must be either 'upper_right', 'upper_left', 'lower_right' or 'lower_left'. ")

def add_goal(goal_pos, goal_size):
    C.addFrame(name='goal', parent='puzzle_world') \
    .setShape(ry.ST.ssBox, [goal_size, goal_size, 0.01, 0.0]) \
    .setRelativePosition(goal_pos) \
    .setColor([1., 1., 0]) \
    .setContact(0)
    

# ToDo: add goal area as non contact frame

add_cube('cube1', cube1_pos)
add_cube('cube2', cube2_pos)
add_cube('cube3', cube3_pos)

add_block('block1', block1_pos, 'vertical')
add_block('block2', block2_pos, 'vertical')
add_block('block3', block3_pos, 'horizontal')

add_longblock('longBlock', longblock_pos, 'horizontal')
add_goal(q_goal, 0.08)
# add_cornerBlock('cornerBlock', corner_pos, 'upper_right')

C.view(True)
