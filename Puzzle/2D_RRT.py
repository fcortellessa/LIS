# using https://github.com/Tupryk/SHK_LIS/blob/main/maze_solver/main.py


import robotic as ry
import time
import oop_prebuilt_puzzleWorld as pWorld
import manipulation as manip
import robot_execution as robex
from komo_path import solve_path_motion
from maze_utils import solve_maze_rrt

ry.params_add({'rrt/stepsize':.05})

C = ry.Config()

q_pWorld = [-0.2, 0.2, 0.05]
q_start = [-4*0.04, 0.0, 0.015]
q_goal = [-4*0.04, 4*0.04, 0.005]
puzzle_world = pWorld.PuzzleWorld(C, 'table', q_pWorld, q_start, q_goal)
puzzle_world.build()

ret = solve_maze_rrt(C, True)

if not ret.feasible:
    print("The RRT solver was unable to find a feasible path.")
    exit()


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

q_pWorld = [-0.2, 0.2, 0.05]
q_start = [-4*0.04, 0.0, 0.015]
q_goal = [-4*0.04, 4*0.04, 0.005]
puzzle_world = pWorld.PuzzleWorld(C, 'table', q_pWorld, q_goal)
puzzle_world.build()

bpos = C.getFrame("start").getPosition()
C.addFrame("box") \
    .setPosition(bpos) \
    .setShape(ry.ST.ssBox, size=[.03, .03, .03, .001]) \
    .setColor([0., 0., 1.]) \
    .setContact(1) \
    .setMass(1.)

C.view()



# Grab the box and follow the RRT path
man = manip.ManipulationModelling(C)
man.setup_inverse_kinematics()
man.grasp_box(1., "l_gripper", "box", "l_palm", "y")
pose = man.solve()

if man.feasible:
    robot = robex.Robot(C)
    robot.execute_path_blocking(C, pose)
    robot.grasp(C)

    man = manip.ManipulationModelling(C)
    man.follow_path_on_plane(ret.x)
    path = man.solve()
    
    try:
        robot.execute_path_blocking(C, path)
    except:
        print("Path is not feasible!")

C.view(True)
