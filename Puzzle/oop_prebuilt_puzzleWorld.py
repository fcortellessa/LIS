import robotic as ry

class PuzzleWorld:
    def __init__(self, C, parent='table'):
        self.C = C
        self.parent = parent

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

        start = [-4*0.04, 0.0, 0.015]
        q_goal = [-4*0.04, 4*0.04, 0.005]

        self._add_frame('puzzle_world', 'table', ry.ST.ssBox, [0.4, 0.4, 0.001, 0.0], [-0.2, 0.2, 0.05], [0, 1], 1)
        self._add_frame('moving_object', 'puzzle_world', ry.ST.ssBox, [0.03, 0.03, 0.03, 0.0], start, [1, 0, 0], 1)

        self._add_cube('cube1', cube1_pos)
        self._add_cube('cube2', cube2_pos)
        self._add_cube('cube3', cube3_pos)
        self._add_block('block1', block1_pos, 'vertical')
        self._add_block('block2', block2_pos, 'vertical')
        self._add_block('block3', block3_pos, 'horizontal')
        self._add_long_block('longBlock', longblock_pos, 'horizontal')
        self._add_corner_block('cornerBlock', corner_pos, 'upper_left')
        self._add_goal(q_goal, 0.08)

        return self.C

    def _add_frame(self, name, parent, shape_type, size, rel_pos, color, contact):
        return self.C.addFrame(name=name, parent=parent) \
            .setShape(shape_type, size) \
            .setRelativePosition(rel_pos) \
            .setColor(color) \
            .setContact(contact)

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

        self._add_frame(name, 'puzzle_world', ry.ST.ssBox, [0.04, 0.04, 0.017, 0.0], [pos[0] + offset_x, pos[1] + offset_y, pos[2]], [0, 0, 1], 1)

        return frame

    def _add_goal(self, pos, size):
        return self._add_frame('goal', 'puzzle_world', ry.ST.ssBox, [size, size, 0.01, 0.0], pos, [1., 1., 0], 0)



# -------- Usage --------

# C = ry.Config()
# C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
# puzzle_world = PuzzleWorld(C, 'table')
# puzzle_world.build()
# C.view(True)
