# based on Eckhard's code: https://github.com/Tupryk/SHK_LIS/blob/main/maze_solver/main.py

import sys
import robotic as ry
import time
# import oop_prebuilt_puzzleWorld as pWorld
import buildMazeOnly as pWorld
import robot_execution as robex
import expanded_manipulation as manip
from expanded_manipulation import ManipulationModelling


def solve_maze_rrt(C: ry.Config, visual: bool=False) -> ry._robotic.SolverReturn:
    """
    Takes a ry.Config with just some obstacles and a start
    and goal marker frames.

    Returns the output of the RRT solver.

    Will generate an XYPhi joint and ssBox frame.
    """
    maze_pos = C.getFrame("puzzle_world").getPosition()
    start_pos = C.getFrame("start").getPosition()
    goal_pos = C.getFrame("goal").getPosition()

    
    C.getFrame("base").setPosition([0, 0, start_pos[2]])
    # C.addFrame("base").setPosition([0, 0, start_pos[2]])
    # C.addFrame("base").setPosition([0, 0, .7])
    # C.addFrame("ego", "base") \
    #    .setJoint(ry.JT.transXYPhi) \
    #    .setShape(ry.ST.ssBox, size=[.05, .1, .05, .002]) \
    #    .setColor([0, 1., 1.]) \
    #    .setContact(1)
    
    # [-1., 1., -1., 1., -3., 3.]

    q0 = [*start_pos[:2], 0]
    qT = [*goal_pos[:2], 0]

    C.setJointState(qT)
    rrt = ry.PathFinder()
    rrt.setProblem(C, [q0], [qT])
    ret = rrt.solve()
    C.view(True)
    if visual and ret.feasible:
        C.view(True)
        for i, q in enumerate(ret.x):
            C.addFrame(f"marker{i}") \
                .setShape(ry.ST.marker, size=[.04]) \
                .setPosition([q[0], q[1], 0]) \
                .setColor([1, 1, 1])
            C.setJointState(q)
            C.view()
            time.sleep(.1)
        C.view(True)

    return ret

ry.params_add({'rrt/stepsize':.05})


C = ry.Config()
C.addFile('world.g')

# q_pWorld = [0.0, 0.3, 0.65]

# q_pWorld = [0.0, 0.3, 0.85]
q_pWorld = [0.0, 0.3, 0.075]
q_start = [-0.12, .0, .0]
q_goal = [0.12, .12, .0]
puzzle_world = pWorld.PuzzleWorld(C, "table", q_pWorld, q_start, q_goal)
puzzle_world.build(False)

ret = solve_maze_rrt(C, True)

if not ret.feasible:
    print("The RRT solver was unable to find a feasible path.")
    exit()


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
# C.addFile(ry.raiPath('world.g'))

# q_pWorld = [0.0, 0.3, 0.85]
q_pWorld = [0.0, 0.3, 0.075]
q_start = [-0.12, .0, .0]
q_goal = [0.12, .12, .0]
puzzle_world = pWorld.PuzzleWorld(C, "table", q_pWorld, q_start, q_goal)
puzzle_world.build(True)

bpos = C.getFrame("start").getPosition()
C.getFrame("box").setColor([1, .5, .0])
C.getFrame("box").setRelativePosition([bpos[0], bpos[1], 0.075])
print(f'bpos: {bpos}')
# C.addFrame("box", "table") \
#     .setJoint(ry.JT.rigid) \
#     .setRelativePosition([bpos[0], bpos[1], 0.075]) \
#     .setShape(ry.ST.ssBox, size=[.03, .03, .03, .001]) \
#     .setColor([1, .0, .0]) \
#     .setContact(1)


time.sleep(1)
qHome = C.getJointState()   # get current joint states of the robot
limits = C.getJointLimits()



# Grab the box and follow the RRT path
man = ManipulationModelling(C)
man.setup_inverse_kinematics()
man.grasp_box(1., "l_gripper", "box", "l_palm", "y")
pose = man.solve()
print(pose)

if man.feasible:
    robot = robex.Robot(C, False)
    robot.execute_path_blocking(C, pose)
    robot.grasp(C)

    man = manip.ManipulationModelling(C)
    man.follow_path_on_plane(ret.x)
    path = man.solve()
    print(f'path: {path}')
    
    if man.feasible:
        try:
            robot.execute_path_blocking(C, path)
        except:
            print("Path is not feasible!")
    else: 
        print("Path is not feasible!")
C.view(True)
