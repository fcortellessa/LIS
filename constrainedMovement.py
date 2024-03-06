import robotic as ry
import numpy as np

def move_constrainted(C, bot, distance = 0.25, orientation = 'vertical',prints = False):
    bot.sync(C,.5)
    gripper_pos = C.getFrame("l_gripper").getPosition()
    retract_to_pos = gripper_pos + [0.0, distance, 0.0]
    C.addFrame('retract_to_pos').setShape(ry.ST.marker, [.1, .1, .1]).setPosition(retract_to_pos).setColor([1, .5, 1])

    komo = ry.KOMO(C, 1, 1, 1, True)
    komo.addControlObjective([], 0, 0.1e1)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([1], ry.FS.positionDiff, ['l_gripper', 'retract_to_pos'], ry.OT.eq, [1e3])

    # orient block vertical
    if orientation == 'vertical':
        komo.addObjective([1], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [0, 0 , 1])    
        komo.addObjective([1], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, [1e1], [0, 1, 0])
        pass
    elif orientation == 'horizontal':
        komo.addObjective([1], ry.FS.vectorZ, ['l_gripper'], ry.OT.eq, [1e1], [1, 0, 0])
        komo.addObjective([1], ry.FS.vectorX, ['l_gripper'], ry.OT.eq, [1e1], [0, 1, 0]) 
             

    ret = ry.NLP_Solver(komo.nlp(), verbose=4) .solve()
    if prints == True:
        print(ret)
    
    q = komo.getPath()
    
    bot.moveTo(q[-1], 4)
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)
