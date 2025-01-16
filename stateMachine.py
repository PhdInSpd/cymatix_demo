import myactuator_v3_library as ml
import time
from typing import Final

from plc import Ton

deadBand: Final[int] = 5*100
def inRange(targetPos,accualPos, band=deadBand)->bool:
    max:float = targetPos + band
    min:float = targetPos - band
    return accualPos>=min and accualPos<=max

def getMotorAngle(unitID:int)->int:
    status2 = ml.read_motor_status_2(unitID)
    accualPos = status2["Motor Angle (degrees)"]
    return accualPos

class PositionStateMachine:
    state: int = 0
    startPos: int = 0
    bigCycle: int = 0 
    smallCycle: int = 0
    smallMove: float = 1
    startPos: int = 0
    smallTargetPos: int = 0
    tonPause = Ton(1)

    def __init__(self, unitID, totalCycles, bigMoveVelocity, smallMoveVelocity, totalMove, noSmallMoves, bigMovePause, smallMovePause):
        self.unitID = unitID
        self.totalCycles = totalCycles
        self.bigMoveVelocity = bigMoveVelocity
        self.smallMoveVelocity = smallMoveVelocity
        self.totalMove = totalMove
        self.noSmallMoves = noSmallMoves
        self.bigMovePause = bigMovePause
        self.smallMovePause = smallMovePause

    def runState(self)->int:
        match self.state:
            case 0: #initialize
                ml.motor_stop(self.unitID)
                self.startPos = getMotorAngle(self.unitID)
                self.cycles =0
                self.state=100

            case 100: #start bigmove
                print(f"big move{self.bigCycle}")
                self.targetPos = (self.startPos+self.totalMove)*100
                absStatus = ml.absolute_position_closed_loop_control(self.unitID, self.bigMoveVelocity,self.targetPos)
                self.state=200

            case 200: # wait big move done
                accualPos: float = getMotorAngle(self.unitID)*100
                if( inRange(self.targetPos,accualPos) ):
                    self.tonPause.SecTimeout = self.bigMovePause
                    self.tonPause.CLK(False)
                    self.state=300

            case 300: # bigmove pause
                if( self.tonPause.CLK(True) ):
                    self.smallCycle = 1
                    self.tonPause.CLK(False)
                    self.state=400

            case 400: # little move start
                self.smallMove  = self.totalMove/self.noSmallMoves
                print(f"small move{self.smallCycle}")
                self.smallTargetPos = int((self.startPos+self.totalMove-self.smallCycle*self.smallMove)*100)
                absStatus = ml.absolute_position_closed_loop_control(self.unitID, self.smallMoveVelocity,self.smallTargetPos)
                self.state = 500

            case 500: # little move done
                accualPos: float = getMotorAngle(self.unitID)*100
                if( inRange(self.smallTargetPos, accualPos) ):
                    self.tonPause.SecTimeout = self.smallMovePause
                    self.tonPause.CLK(False)
                    self.state=600

            case 600: # small move pause
                if( self.tonPause.CLK(True) ):
                    self.smallCycle += 1
                    self.tonPause.CLK(False)
                    if( self.smallCycle <= self.noSmallMoves ):
                        self.state=400
                    else:
                        self.tonPause.SecTimeout = self.bigMovePause
                        self.state = 700
            case 700:
                 if( self.tonPause.CLK(True) ):
                    self.bigCycle += 1
                    self.tonPause.CLK(False)
                    if( self.totalCycles <=0 or self.bigCycle<self.totalCycles ):
                        self.state=100
                    else:
                        self.state = -1
            # If an exact match is not confirmed, this last case will be used if provided
            case _:
                self.state=-1
        return self.state
    
pss1 = PositionStateMachine(unitID=1,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=200,
                           totalMove=360*1,
                           noSmallMoves=10,
                           bigMovePause=.375,
                           smallMovePause=.125)
while pss1.runState() >=0 :
    time.sleep(0.002)
print('Done')

