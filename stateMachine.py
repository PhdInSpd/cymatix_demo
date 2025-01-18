import time
from PoitionStateMachineSrc import MotorStateMachine
import myactuator_v3_library as ml

# testTon()
ml.motor_stop(1)
profile = [[5,.50], # degress/sec , sec
           [6,.50],
           [7,.750],
           [8,.750],
           [9,.75],
           [10,.75],
           [20,.75],
           [30,.75],
           [40,.75],
           [50,.75],
           [0,1],
           [-5,.50],
           [-6,.50],
           [-7,.750],
           [-8,.75],
           [-9,.75],
           [-10,.75],
           [-20,.75],
           [-30,.75],
           [-40,.75],
           [-50,.75],
           [0,1]]
i: int =0
pss1 = MotorStateMachine(unitID=1,
                           totalCycles=0,
                           bigMoveVelocity=100,
                           smallMoveVelocity=50,
                           totalMove=18,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)

pss2 = MotorStateMachine(unitID=2,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=1000,
                           totalMove=360*2,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)

pss3 = MotorStateMachine(unitID=3,
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
        axis1Running = pss1.runProfile(profile) >=0
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


