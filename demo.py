import myactuator_v3_library as ml
import time
from typing import Final

deadBand: Final[int] = 5*100
def inRange(targetPos,accualPos, band=deadBand)->bool:
    max:float = targetPos + band
    min:float = targetPos - band
    return accualPos>=min and accualPos<=max

def getMotorAngle(unitID:int)->int:
    status2 = ml.read_motor_status_2(unitID)
    accualPos = status2["Motor Angle (degrees)"]
    return accualPos

# Speeding is not the moving bit
def isMoving()->bool:
    status = ml.read_motor_status_and_error()
    if status["Error Flags"] is None:
        return False
    return "Speeding" in status["Error Flags"]

bigMoveVelocity: Final[int] =3000
smallMoveVelocity: Final[int] =1000
totalMove : int = 360*1
noSmallMoves: int = 10
bigMovePause:float = .375
smallMovePause: float = .125

unitID: int = 1
# gains = ml.read_pid_parameter()
# print( gains )
ml.motor_stop(unitID)
#ml.motor_shutdown()
status2 = ml.read_motor_status_2(unitID)
startPos: float = status2["Motor Angle (degrees)"]

i:int =0
while i<10:
    print(f"big move{i}")
    targetPos: int = (startPos+totalMove)*100
    absStatus = ml.absolute_position_closed_loop_control(unitID, bigMoveVelocity,targetPos)
    accualPos: float = absStatus["motor_angle"]*100

    while not inRange(targetPos,accualPos):
        accualPos = getMotorAngle(unitID)*100
        time.sleep(0.002)
    time.sleep(bigMovePause)
    
    k: int = 1
    smallMove : float = totalMove/noSmallMoves
    while k<=noSmallMoves:
        print(f"small move{k}")
        smallTargetPos: int = int((startPos+totalMove-k*smallMove)*100)
        absStatus = ml.absolute_position_closed_loop_control(unitID, smallMoveVelocity,smallTargetPos)
        accualPos: float = absStatus["motor_angle"]*100

        while not inRange(smallTargetPos,accualPos):
            accualPos = getMotorAngle(unitID)*100
            time.sleep(0.002)
        k+=1
        time.sleep(smallMovePause)
    time.sleep(bigMovePause)
    i += 1

print('Done')
    
