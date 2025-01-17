import myactuator_v3_library as ml
import time
from typing import Final

from plc import Ton

deadBand: Final[int] = 1.5*100
def inRange(targetPos,accualPos, band=deadBand)->bool:
    max:float = targetPos + band
    min:float = targetPos - band
    return accualPos>=min and accualPos<=max

def getMotorAngle(unitID:int)->int:
    status2 = ml.read_motor_status_2(unitID)
    accualPos = status2["Motor Angle (degrees)"]
    return accualPos

def testTon():
    ton = Ton(1)
    while( True ):
        if( ton.CLK(True) ):
            ton.CLK(False)
            print( ton.SecTimeout )
        time.sleep(0.002)

class PositionStateMachine:
    def __init__(self, unitID, totalCycles, bigMoveVelocity, smallMoveVelocity, totalMove, noSmallMoves, bigMovePause, smallMovePause):
        self.unitID = unitID
        self.totalCycles = totalCycles
        self.bigMoveVelocity = bigMoveVelocity
        self.smallMoveVelocity = smallMoveVelocity
        self.totalMove = totalMove
        self.noSmallMoves = noSmallMoves
        self.bigMovePause = bigMovePause
        self.smallMovePause = smallMovePause

        self.state: int = 0
        self.startPos: int = 0
        self.bigCycle: int = 0 
        self.smallCycle: int = 0
        self.smallMove: float = 1
        self.startPos: int = 0
        self.smallTargetPos: int = 0
        self.tonPause: Ton = Ton(1)
        
        #self.steady: SteadyCount = SteadyCount(steadySec=.375)
        self.tonInRange: Ton = Ton(.375)

    def runState(self)->int:
        match self.state:
            case 0: #initialize
                ml.motor_stop(self.unitID)
                self.startPos = getMotorAngle(self.unitID)
                self.cycles =0
                self.state=100

            case 100: #start bigmove
                print(f"big move{self.unitID}:{self.bigCycle}")
                self.tonInRange.SecTimeout = self.bigMovePause
                self.tonInRange.CLK(False)
                self.targetPos = (self.startPos+self.totalMove)*100
                absStatus = ml.absolute_position_closed_loop_control(self.unitID, self.bigMoveVelocity,self.targetPos)
                #self.steady.Reset()
                self.state=200

            case 200: # wait big move done
                accualPos: int = getMotorAngle(self.unitID)
                if( self.tonInRange.CLK( inRange(self.targetPos,accualPos*100) ) ):
                #if( self.steady.CLK(accualPos) ):
                    self.tonInRange.CLK( False )
                    self.tonPause.SecTimeout = self.bigMovePause
                    self.tonPause.CLK(False)
                    self.smallCycle = 1
                    self.state=400

            case 300: # bigmove pause
                if( self.tonPause.CLK(True) ):
                    self.tonPause.CLK(False)
                    self.state=400

            case 400: # little move start
                self.smallMove  = self.totalMove/self.noSmallMoves
                print(f"small move{self.unitID}:{self.smallCycle}")
                self.tonInRange.SecTimeout = self.smallMovePause
                self.tonInRange.CLK( False )
                self.smallTargetPos = int((self.startPos+self.totalMove-self.smallCycle*self.smallMove)*100)
                absStatus = ml.absolute_position_closed_loop_control(self.unitID, self.smallMoveVelocity,self.smallTargetPos)
                #self.steady.Reset()
                self.state = 500

            case 500: # little move done
                accualPos: int = getMotorAngle(self.unitID)
                if( self.tonInRange.CLK( inRange(self.smallTargetPos, accualPos*100) ) ):
                #if( self.steady.CLK(accualPos) ): 
                    self.tonInRange.CLK( False )
                    self.tonPause.SecTimeout = self.smallMovePause
                    self.tonPause.CLK(False)
                    #self.state=600
                    self.smallCycle += 1
                    self.tonPause.CLK(False)
                    if( self.smallCycle <= self.noSmallMoves ):
                        self.state=400
                    else:
                        self.tonPause.SecTimeout = self.bigMovePause
                        self.state = 700

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
                        print(f'Done {self.unitID}')
                        self.state = -1
            # If an exact match is not confirmed, this last case will be used if provided
            case _:
                self.state=-1
        return self.state
