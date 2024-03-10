import robotic as ry
import numpy as np
import time
# from prebuild_puzzleWorld import build_puzzleWorld
import oop_prebuild_puzzleWorld as pWorld 


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))

# C = build_puzzleWorld(C)
puzzle_world = pWorld.PuzzleWorld(C)
puzzle_world.build()
C.view(True)
