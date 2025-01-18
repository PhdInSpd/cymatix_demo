import time
from PoitionStateMachineSrc import getMotorAngle, inRange
import myactuator_v3_library as ml
unitID: int = 1
totalMove: int = 360
bigMoveVelocity: int =-5*100
print(f"big move")
ml.motor_stop(unitID)
profile = [[5,.50],
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
while i<10:
    for point in profile:
        absStatus = ml.speed_closed_loop_control(unitID, point[0]*100)
        time.sleep(point[1])
        actualPos = getMotorAngle(unitID)
        print( actualPos, point[0] )
    print(f"loop {i} done")
    i += 1