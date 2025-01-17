import time
from PoitionStateMachineSrc import getMotorAngle, inRange
import myactuator_v3_library as ml
unitID: int = 1
totalMove: int = 360
bigMoveVelocity: int =30
print(f"big move")
ml.motor_stop(unitID)
startPos = getMotorAngle(unitID)
targetPos: int = (startPos+totalMove)*100
# runs away
absStatus = ml.absolute_position_closed_loop_control(unitID, bigMoveVelocity,targetPos)
#absStatus = ml.incremental_position_control(unitID, bigMoveVelocity,totalMove*100)
actualPos: float = absStatus["motor_angle"]

while not inRange(targetPos,actualPos*100):
    actualPos = getMotorAngle(unitID)
    print( actualPos )
    # time.sleep(0.002)
    time.sleep(.010)
print("move done")