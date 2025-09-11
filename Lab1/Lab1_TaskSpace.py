"""
    MREN 410 QArm Lab1 Joint Control
    Developed from Quanser Research Example Code by Antonio Morales
    Last Updated: 09/08/2025

"""
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Imports
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import time
import numpy as np

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Timing Parameters and methods
startTime = 0
def elapsed_time():
    return time.time() - startTime
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Reset startTime before Main Loop
startTime = time.time()

# ------------ DECLARE LISTS AND VARIABLES FOR PLOTTING ----------------------




# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Main loop
with QArm(hardware=1, readMode=0) as myArm:
    # Arm setup
    myArmUtilities = QArmUtilities()
    ledCmd = np.array([0, 1, 0], dtype=np.float64)
    np.set_printoptions(precision=2, suppress=True)
    while myArm.status:
        start = elapsed_time()
        try:
            #---------------------SECTION 5 INVERSE KINEMATICS------------------
            # Take user input in task space
            result = myArmUtilities.take_user_input_task_space()
            
            # Split input into pos, wrist, grip commands
            positionCmd = result[0:3]
            gamma = result[3] 
            gripCmd = result[4]
            #----------------------- FILL IN BELOW -----------------------------
            # Compute phiCmd using QARM Inverse Kinematic function (FILL IN)

            # Command arm using phiCmd, gripCmd, ledCmd

            # -------------------------------------------------------------------
            # Wait 0.5 seconds before next command
            time.sleep(.5)
        except KeyboardInterrupt:
            myArm.terminate()
