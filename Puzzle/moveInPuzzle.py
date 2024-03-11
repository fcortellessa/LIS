import sys
import robotic as ry
import build_puzzleWorld3 as pWorld 
sys.path.insert(0,'/home/daniel/PycharmProjects/flavia/rai-tutorials')
import manipulation as manip
import time


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

q_pWorld = [-0.2, 0.2, 0.05]
q_start = [-4*0.04, 0.0, 0.015]
q_goal = [-4*0.04, 4*0.04, 0.005]
puzzle_world = pWorld.PuzzleWorld(C, 'table', q_pWorld, q_start, q_goal)
puzzle_world.build()

# ----------------- Move in puzzle -----------------

bot = ry.BotOp(C, False)
bot.home(C)

qHome = C.getJointState()   # get current joint states of the robot
limits = C.getJointLimits()
gripper = "l_gripper";
palm = "l_palm";
box = "moving_object";
table = "table";

q_goal_abs = [q_goal[0]+q_pWorld[0], q_goal[1]+q_pWorld[1], q_goal[2]+q_pWorld[2]] 
qStart = C.getJointState()

graspDirection = 'yz'       # grasps box at y-side from the z-direction (top) 
placeDirection = 'z'
place_orientation = [0,1,0.]

M = manip.ManipulationModelling(C, helpers=[gripper])
M.setup_pick_and_place_waypoints(gripper, box, homing_scale=1e-1, joint_limits=False)
M.grasp_top_box(1., gripper, box, graspDirection)
M.place_box(2., box, table, palm, placeDirection)
M.target_relative_xy_position(2., box, table, q_goal_abs)
M.target_x_orientation(2., box, place_orientation)
M.solve()
if not M.feasible:
    print("M Not feasible")
    
    
M2 = M.sub_motion(0)
M2.retract([.0, .2], gripper)
M2.approach([.8, 1.], gripper)
M2.solve()
if not M.feasible:
    print("M2 Not feasible")

M3 = M.sub_motion(1)
M3.no_collision([], table, box)
M3.no_collision([], box, 'puzzle_world')
M3.bias(.5, qHome, 1e0)
M3.solve()
if not M.feasible:
    print("M3 Not feasible")

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
