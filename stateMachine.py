import time
import sys
import traceback
from datetime import datetime
from PoitionStateMachineSrc import MotorStateMachine
import myactuator_v3_library as ml
from plc import Ton

# Error logging function
def log_error(error_msg, exception=None):
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open('crash_log.txt', 'a') as f:
        f.write(f"\n{'='*60}\n")
        f.write(f"[{timestamp}] ERROR\n")
        f.write(f"{error_msg}\n")
        if exception:
            f.write(f"Exception: {exception}\n")
            traceback.print_exc(file=f)
        f.write(f"{'='*60}\n")
    print(f"ERROR: {error_msg}")
    if exception:
        print(f"Exception details written to crash_log.txt")

# testTon()
#ml.motor_stop(1)
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
# pss1 = MotorStateMachine(unitID=1,
#                            totalCycles=0,
#                            bigMoveVelocity=100,
#                            smallMoveVelocity=50,
#                            totalMove=int(360*500),
#                            noSmallMoves=7,
#                            bigMovePause=.375,
#                            smallMovePause=0.375)

pss2 = MotorStateMachine(unitID=2,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=1000,
                           totalMove=360*2,
                           noSmallMoves=19,
                           bigMovePause=.375,
                           smallMovePause=.125)

pss3 = MotorStateMachine(unitID=3,
                           totalCycles=0,
                           bigMoveVelocity=3000,
                           smallMoveVelocity=1000,
                           totalMove=360,
                           noSmallMoves=11,
                           bigMovePause=.375,
                           smallMovePause=.125)
# Main execution with comprehensive error handling
try:
    log_error("Starting main execution",
                exception=f"Tick")
    
    iteration_count = 0
    max_iterations = 100000  # Safety limit to prevent infinite loops
    tonLog: Ton = Ton(60*10)  # Log every 10 minutes
    done: bool = False
    while not done:
        iteration_count += 1
        #axis1Running: bool = False
        axis2Running: bool = False
        axis3Running: bool = False

        # try:
        #     #axis1Running = pss1.runProfile(profile) >=0
        #     axis1Running = pss1.runState() >=0
        # except Exception as err:
        #     print(f"{pss1.unitID.__str__()} {err=}, {type(err)=}")
        
        try:
            axis2Running = pss2.runState() >=0
        except Exception as err:
            error_msg = f"Motor {pss2.unitID} error: {err}, type={type(err)}"
            log_error(error_msg, err)
            print(error_msg)
        time.sleep(0.032)

        try:
            axis3Running = pss3.runState() >=0
        except Exception as err:
            error_msg = f"Motor {pss3.unitID} error: {err}, type={type(err)}"
            log_error(error_msg, err)
            print(error_msg)
        time.sleep(0.032)

        #done = not axis1Running and not axis2Running and not axis3Running
        done =  not axis2Running and not axis3Running
        if tonLog.CLK(True):
            tonLog.CLK(False)
            log_error("Alive check",
                      exception=f"Tick")
        #time.sleep(0.002)
        #time.sleep(0.02)
        #time.sleep(0.064)
    
    if iteration_count >= max_iterations:
        log_error("Reached maximum iteration limit - possible infinite loop")
        print("WARNING: Reached maximum iteration limit")
        
except KeyboardInterrupt:
    print("\nProgram interrupted by user")
    log_error("Program interrupted by user (Ctrl+C)")
except Exception as e:
    log_error("FATAL ERROR - Program crashed", e)
    print(f"\nFATAL ERROR: {e}")
    print("Full traceback written to crash_log.txt")
    traceback.print_exc()
    sys.exit(1)
finally:
    print("\nProgram terminating...")
    try:
        ml.motor_stop(2)
        ml.motor_stop(3)
        print("Motors stopped successfully")
    except Exception as e:
        log_error("Error stopping motors during cleanup", e)


