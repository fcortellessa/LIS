import robotic as ry
import numpy as np
import time
from build_puzzleWorld2 import build_puzzleWorld


C = ry.Config()
C.addFile(ry.raiPath('scenarios/pandaSingle.g'))
C = build_puzzleWorld(C)
C.view(True)
