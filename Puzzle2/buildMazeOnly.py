import robotic as ry

class PuzzleWorld:
    def __init__(self, C, parent = None, pos_puzzleWorld=[0.0, 0.3, 0.05], q_start = [-4*0.04, 0.0, 0.015], q_goal = [-4*0.04, 4*0.04, 0.005]):
        self.C = C
        self.parent = parent
        self.pos_puzzleWorld = pos_puzzleWorld
        self.q_goal = q_goal
        self.q_start = q_start

    def build(self):
        # position is defined by center of object (distance between two holes on puzzle board is 4 cm)
        cube1_pos = [-1.5*0.04, 1.5*0.04, 0.0085]
        cube2_pos = [-4.5*0.04, 2.5*0.04, 0.0085]
        cube3_pos = [3.5*0.04, -4.5*0.04, 0.0085]
        
        block1_pos = [-2.5*0.04, 4*0.04, 0.0085]
        block2_pos = [1.5*0.04, 1*0.04, 0.0085]
        block3_pos = [-4*0.04, -4.5*0.04, 0.0085]

        longblock_pos = [2*0.04, -1.5*0.04, 0.0085]
        corner_pos = [-2 * 0.04, -2.5 * 0.04, 0.0085] # position of center of horizontal 4x8 block
        
        border_height = 0.05
        border_thickness = 0.1
        # left border, right border, top border, bottom border
        border_pos = [([0.2+border_thickness/2, 0, border_height/2], 'vertical'),
                      ([-0.2-border_thickness/2, 0, border_height/2], 'vertical'),
                      ([0, 0.2+border_thickness/2, border_height/2], 'horizontal'), 
                      ([0, -0.2-border_thickness/2, border_height/2], 'horizontal')]
    
        self.C.addFrame("puzzle_world").setPosition(self.pos_puzzleWorld)
        
        self.C.addFrame("start", "puzzle_world") \
            .setShape(ry.ST.marker, size=[.02]) \
            .setRelativePosition(self.q_start) \
            .setColor([1, 0, 1])

        self.C.addFrame("goal", "puzzle_world") \
            .setShape(ry.ST.marker, size=[.02]) \
            .setRelativePosition(self.q_goal) \
            .setColor([1, 0, 1])

        # self._add_frame('puzzle_world', None, ry.ST.ssBox, [0.4, 0.4, 0.001, 0.0], self.pos_puzzleWorld, [0, 1], 1)
        self._add_border_boxes(border_pos, border_height, border_thickness)
        # self._add_movingObject('moving_object', 'puzzle_world', ry.ST.ssBox, [0.03, 0.03, 0.03, 0.0], self.q_start, [1, 0, 0])

        # self._add_cube('cube1', cube1_pos)
        self._add_cube('cube2', cube2_pos)
        self._add_cube('cube3', cube3_pos)
        self._add_block('block1', block1_pos, 'vertical')
        self._add_block('block2', block2_pos, 'vertical')
        self._add_block('block3', block3_pos, 'horizontal')
        self._add_long_block('longBlock', longblock_pos, 'horizontal')
        self._add_corner_block('cornerBlock', corner_pos, 'upper_left')
        # self._add_goal(self.q_goal, 0.08)
        
        return self.C

    def _add_frame(self, name, parent, shape_type, size, rel_pos, color, contact):
        if parent == None:
            return self.C.addFrame(name=name) \
                .setShape(shape_type, size) \
                .setPosition(rel_pos) \
                .setColor(color) \
                .setContact(contact)
        else:
            return self.C.addFrame(name=name, parent=parent) \
                .setShape(shape_type, size) \
                .setRelativePosition(rel_pos) \
                .setColor(color) \
                .setContact(contact)
    
    def _add_movingObject(self, name, parent, shape_type, size, rel_pos, color):
        return self.C.addFrame(name=name, parent=parent) \
            .setShape(shape_type, size) \
            .setRelativePosition(rel_pos) \
            .setColor(color) \
            .setJoint(ry.JT.rigid) \
            .setMass(1.0) \
            .setContact(1)


    def _add_border_boxes(self, border_pos, border_height, border_thickness):
        # Define the size and color of the border boxes
        box_color = [0.0, 1.0, 0.0, 0.2]  # Almost transparent color (RGBA)
        box_name = ['left', 'right', 'top', 'bottom']
        for i, pos in enumerate(border_pos):
            pos, orientation = pos[0], pos[1]
            if orientation == 'vertical':
                box_size = [border_thickness, 0.6, border_height, 0.0] 
            elif orientation == 'horizontal':
                box_size = [0.6, border_thickness, border_height, 0.0]
            border_box_name = f'border_box_{box_name[i]}'
            self.C.addFrame(name=border_box_name, parent='puzzle_world') \
                .setShape(ry.ST.ssBox, box_size) \
                .setRelativePosition(pos) \
                .setColor(box_color) \
                .setContact(1)
        
    def _add_cube(self, name, pos):
        return self._add_frame(name, 'puzzle_world', ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0], pos, [0, 0, 1], 1)

    def _add_block(self, name, pos, orientation):
        if orientation == 'horizontal':
            size = [0.08, 0.04, 0.017, 0.0]
        elif orientation == 'vertical':
            size = [0.04, 0.08, 0.017, 0.0]
        else:
            raise ValueError("orientation must be either 'horizontal' or 'vertical'.")
        return self._add_frame(name, 'puzzle_world', ry.ST.ssBox, size, pos, [0, 0, 1], 1)

    def _add_long_block(self, name, pos, orientation):
        if orientation == 'horizontal':
            size = [0.16, 0.04, 0.017, 0.0]
        elif orientation == 'vertical':
            size = [0.04, 0.16, 0.017, 0.0]
        else:
            raise ValueError("orientation must be either 'horizontal' or 'vertical'.")
        return self._add_frame(name, 'puzzle_world', ry.ST.ssBox, size, pos, [0, 0, 1], 1)

    def _add_corner_block(self, name, pos, orientation):
        size = [0.08, 0.04, 0.017, 0.0]
        frame = self._add_frame(name, 'puzzle_world', ry.ST.ssBox, size, pos, [0, 0, 1], 1)
        
        offset = 0.02
        if 'upper' in orientation:
            offset_y = -0.04
        else:
            offset_y = 0.04

        if 'left' in orientation:
            offset_x = -offset
        else:
            offset_x = offset

        self._add_frame('name{small}', 'puzzle_world', ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0], [pos[0] + offset_x, pos[1] + offset_y, pos[2]], [0, 0, 1], 1)

        return frame

    def _add_goal(self, pos, size):
        return self._add_frame('goal', 'puzzle_world', ry.ST.ssBox, [size, size, 0.01, 0.0], pos, [1., 1., 0], 0)



# -------- Usage --------

# C = ry.Config()
# puzzle_world = PuzzleWorld(C)
# puzzle_world.build()
# C.view(True)

# C = ry.Config()
# C.addFile(ry.raiPath("scenarios/pandaSingle.g"))
# puzzle_world = PuzzleWorld(C, 'None', [0.0, 0.3, 0.65], [-0.12, .0, .0], [0.12, .12, .0])
# puzzle_world.build()
# C.view(True)
