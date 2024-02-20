import sys
import robotic as ry
sys.path.insert(0,'/home/guest/Downloads/rai-tutorials')
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
