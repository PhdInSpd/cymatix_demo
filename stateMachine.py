import time
from PoitionStateMachineSrc import PositionStateMachine
import myactuator_v3_library as ml

# testTon()
ml.motor_stop(1)
pss1 = PositionStateMachine(unitID=1,
                           totalCycles=0,
                           bigMoveVelocity=100,
                           smallMoveVelocity=50,
                           totalMove=18,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)

pss2 = PositionStateMachine(unitID=2,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=1000,
                           totalMove=360*5,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)

pss3 = PositionStateMachine(unitID=3,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=1000,
                           totalMove=360,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)
done: bool = False
while not done:
    axis1Running: bool = False
    axis2Running: bool = False
    axis3Running: bool = False

    try:
        axis1Running = pss1.runState() >=0
    except Exception as err:
        print(f"{pss1.unitID.__str__()}{err=}, {type(err)=}")
    
    try:
        axis2Running = pss2.runState() >=0
    except Exception as err:
        print(f"{pss2.unitID.__str__()} {err=}, {type(err)=}")
    

    try:
        axis3Running = pss3.runState() >=0
    except Exception as err:
        print(f"{pss3.unitID.__str__()} {err=}, {type(err)=}")

    done = not axis1Running and not axis2Running and not axis3Running

    time.sleep(0.002)


