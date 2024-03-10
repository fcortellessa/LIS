import robotic as ry

# build maze with available blocks: 3 cubes, 3 blocks, 1 long block and 1 corner block
def build_puzzleWorld(C, parent='table'):

    # position is defined by center of object (distance between two holes on puzzle board is 4 cm)
    cube1_pos = [-1.5*0.04, 1.5*0.04, 0.0085]
    cube2_pos = [-4.5*0.04, 2.5*0.04, 0.0085]
    cube3_pos = [3.5*0.04, -4.5*0.04, 0.0085]
    
    block1_pos = [-2.5*0.04, 4*0.04, 0.0085]
    block2_pos = [1.5*0.04, 1*0.04, 0.0085]
    block3_pos = [-4*0.04, -4.5*0.04, 0.0085]

    longblock_pos = [2*0.04, -1.5*0.04, 0.0085]
    corner_pos = [-2 * 0.04, -2.5 * 0.04, 0.0085] # position of center of horizontal 4x8 block

    # Note: do not set contact
    start = [-4*0.04, 0.0, 0.015]
    q_goal = [-4*0.04, 4*0.04, 0.005]
 
    # Note: add half of the height of the table to all position z-coordinates
    C.addFrame(name='puzzle_world', parent=parent) \
        .setShape(ry.ST.ssBox, [0.4, 0.4, 0.001, 0.0]) \
        .setRelativePosition([-0.2, 0.2, 0.05]) \
        .setColor([0, 1]) \
        .setContact(1)
    C.addFrame(name='moving_object', parent='puzzle_world') \
        .setShape(ry.ST.ssBox, [0.03, 0.03, 0.03, 0.0]) \
        .setRelativePosition(start) \
        .setColor([1, .0, .0]) \
        .setContact(1)

    add_cube(C, 'cube1', cube1_pos)
    add_cube(C, 'cube2', cube2_pos)
    add_cube(C, 'cube3', cube3_pos)
    add_block(C, 'block1', block1_pos, 'vertical')
    add_block(C, 'block2', block2_pos, 'vertical')
    add_block(C, 'block3', block3_pos, 'horizontal')
    add_longblock(C, 'longBlock', longblock_pos, 'horizontal')
    add_cornerBlock(C, 'cornerBlock', corner_pos, 'upper_left')
    add_goal(C, q_goal, 0.08)
        
    return C


# ----------------- Helper functions -----------------
def add_cube(C, name='cubic_obstacle', cube_pos=[0, 0, 0], lenXY_cube=0.04):
    C.addFrame(name=name, parent='puzzle_world') \
        .setShape(ry.ST.ssBox, [lenXY_cube, lenXY_cube, 0.017, 0.0]) \
        .setRelativePosition(cube_pos) \
        .setColor([.0, .0, 1]) \
        .setContact(1)
    return C 

def add_block(C, name='block_obstacle', block_pos=[0, 0, 0], orientation='horizontal'):
    if orientation == 'horizontal':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.08, 0.04, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'vertical':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.08, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    else:
        print("orientation must be either 'horizontal' or 'vertical'. ")
    return C

def add_longblock(C, name='longBlock_obstacle', block_pos=[0, 0, 0], orientation='horizontal'):
    if orientation == 'horizontal':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.16, 0.04, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'vertical':
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.16, 0.017, 0.0]) \
            .setRelativePosition(block_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    else:
        print("orientation must be either 'horizontal' or 'vertical'. ")
    return C


def add_cornerBlock(C, name='cornerBlock_obstacle', block_pos=[0, 0, 0], orientation='upper_right'):

    """
    :param C:           Config object
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
        smallBlock_pos = [block_pos[0] + 0.02, block_pos[1] - 0.04, block_pos[2]]
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'upper_left':
        smallBlock_pos = [block_pos[0] - 0.02, block_pos[1] - 0.04, block_pos[2]]
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'lower_right':
        smallBlock_pos = [block_pos[0] + 0.02, block_pos[1] + 0.04, block_pos[2]]
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    elif orientation == 'lower_left':
        smallBlock_pos = [block_pos[0] - 0.02, block_pos[1] + 0.04, block_pos[2]]
        C.addFrame(name=name, parent='puzzle_world') \
            .setShape(ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0]) \
            .setRelativePosition(smallBlock_pos) \
            .setColor([.0, .0, 1]) \
            .setContact(1)
    else:
        print("orientation must be either 'upper_right', 'upper_left', 'lower_right' or 'lower_left'. ")
    return C
    

def add_goal(C, goal_pos, goal_size):
    C.addFrame(name='goal', parent='puzzle_world') \
        .setShape(ry.ST.ssBox, [goal_size, goal_size, 0.01, 0.0]) \
        .setRelativePosition(goal_pos) \
        .setColor([1., 1., 0]) \
        .setContact(0)
    return C
