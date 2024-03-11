import robotic as ry

# from prebuilt_puzzleWorld import build_puzzleWorld
import oop_prebuilt_puzzleWorld as pWorld 


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

# C = build_puzzleWorld(C)
q_pWorld = [-0.2, 0.2, 0.05]
q_start = [-4*0.04, 0.0, 0.015]
q_goal = [-4*0.04, 4*0.04, 0.005]
puzzle_world = pWorld.PuzzleWorld(C, 'table', q_pWorld, q_goal)
puzzle_world.build()
C.view(True)
