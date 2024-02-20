"""
Moving sequentially to multiple way points with KOMO
"""

import robotic as ry
import sys
sys.path.insert(0,'/home/guest/Downloads/rai-tutorials')


C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
bot = ry.BotOp(C, False)

C.addFrame('box') \
    .setPosition([-.25, .1, .675]) \
    .setShape(ry.ST.ssBox, size=[.05,.05,.05,.005]) \
    .setColor([1,.5,0]) \
    .setContact(1)

C.addFrame('start') \
    .setShape(ry.ST.marker, size=[.3]) \
    .setPosition([-.2,.4,.7])

C.addFrame('goal') \
    .setShape(ry.ST.marker, size=[.3]) \
    .setPosition([.5,.4,.7])


C.view(False)
print(bot.get_qHome())
bot.home(C)


def moveGripper(goal_object):
    komo = ry.KOMO(config=C, phases=1., slicesPerPhase=2, kOrder=1, enableCollisions=False)
    komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", goal_object], ry.OT.eq, scale=[1e1])

    result = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
    if result.feasible:
        print(f'--- motion to {goal_object} FEASIBLE ---')
    else:
        print(f'---  !!! motion to {goal_object} NOT FEASIBLE !!! ---')

    q = komo.getPath()
    print(q)
    bot.move(path=q, times=[2.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)


moveGripper(goal_object="box")

komo = ry.KOMO(config=C, phases=1., slicesPerPhase=2, kOrder=1, enableCollisions=False)
komo.addObjective([1], ry.FS.positionDiff, ["l_gripper", "start"], ry.OT.eq, scale=[1e1])

result = ry.NLP_Solver(komo.nlp(), verbose=0) .solve()
if result.feasible:
    print('--- FEASIBLE ---')
else:
    print('---  !!! NOT FEASIBLE !!! ---')

q = komo.getPath()
print(q)
bot.move(path=q, times=[2.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

komo2 = ry.KOMO(config=C, phases=1., slicesPerPhase=2, kOrder=1, enableCollisions=False)
komo2.addObjective([1], ry.FS.positionDiff, ["l_gripper", "goal"], ry.OT.eq, scale=[1e0])

result = ry.NLP_Solver(komo2.nlp(), verbose=0) .solve()
if result.feasible:
    print('--- FEASIBLE ---')
else:
    print('---  !!! NOT FEASIBLE !!! ---')

q = komo2.getPath()
print(q)
bot.move(path=q, times=[2.])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)


del(C)
