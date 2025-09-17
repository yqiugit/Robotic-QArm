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
    ee_data = []
    ledCmd = np.array([0, 1, 0], dtype=np.float64)
    np.set_printoptions(precision=2, suppress=True)
    while myArm.status:
        start = elapsed_time()
        try:
            #---------------------SECTION 5 INVERSE KINEMATICS------------------
            # Take user input in task space
           # result = myArmUtilities.take_user_input_task_space()

            
            results = [(0.5, 0.025, 0.6, 0, 0), (0.5, -0.125, 0.6, 0, 0), (0.5, -0.2,  0.5, 0, 0), 
                        (0.5,-0.2,0.3,0,0),(0.5,-0.125,0.2,0,0),
                        (0.5, 0.025, 0.2, 0, 0), (0.5, 0.1, 0.3, 0, 0),
                        (0.5, 0.1, 0.5, 0, 0), (0.5, 0.025, 0.6, 0, 0)]


            # Split input into pos, wrist, grip commands
            for result in results:

                positionCmd = result[0:3]
                gamma = result[3] 
                gripCmd = result[4]
            #----------------------- FILL IN BELOW -----------------------------
            # Compute phiCmd using QARM Inverse Kinematic function (FILL IN)
                phiOptimal, phi = myArmUtilities.inverse_kinematics(positionCmd, gamma, myArm.measJointPosition[0:4])
            # Command arm using phiCmd, gripCmd, ledCmd
                myArm.read_write_std(phi,gripCmd, ledCmd)
                ee_loc,ee_rot=myArmUtilities.forward_kinematics(phi)
                ee_data.append(ee_loc[:3].flatten())
                print("added point")

            

            # -------------------------------------------------------------------
            # Wait 0.5 seconds before next command
                time.sleep(.5)
        except KeyboardInterrupt:
            myArm.terminate()
            break

    ee_data = np.array(ee_data)
    print(ee_data)
    # Plot end effector trajectory
    fig = plt.figure()
    ax = fig.add_subplot(projection = '3d')

    ax.plot(ee_data[:,0], ee_data[:,1], ee_data[:,2])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    plt.title("End Effector Trajectory")
    # Show all plotsc
    plt.show()