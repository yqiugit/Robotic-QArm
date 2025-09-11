"""
    MREN 410 QArm Lab1 Setup
    Developed from Quanser Research Example Code by Antonio Morales
    Last Updated: 09/8/2025

"""
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Imports
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import time

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
## Timing Parameters and methods
startTime = time.time()
def elapsed_time(startTime):
    return time.time() - startTime

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Joint Space Waypoint Matrix
waypoints = [np.array([0.0,0.0,0.0,0.0]),
             np.array([0.0,0.2,0.0,0.0]),
             np.array([0.0,-0.2,0.0,0.0]),
             np.array([0.0,0.0,0.0,0.0]),
             np.array([0.5,0.0,0.0,0.0]),
             np.array([-0.5,0.0,0.0,0.0]),
             np.array([0.0,0.0,0.0,0.0])]
# positional tollerance
tollerance = 0.1
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Main Loop
def main():
    # Arm setup
    myArm = QArm(hardware=1, readMode=0)
    myArmUtilities = QArmUtilities()
    ledCmd = np.array([0, 1, 0], dtype=np.float64)
    np.set_printoptions(precision=2, suppress=True)
    myArm.read_write_std(baseLED=ledCmd)

    # Data lists for plotting used in section 2 and 4
    position_data = []
    current_data = []
    temperature_data = []
    ee_data= []
    time_data = []

    while myArm.status:
        try:
            for i in range(len(waypoints)):
                phiCmd = waypoints[i]
                gripCmd = 0.0
                # Command Gripper
                location, rotation = myArmUtilities.forward_kinematics(phiCmd)
                error = 10

                while(error > tollerance):

                    #-------------------- SECTION 2 READ DATA -------------------------

                    # Read sensor values and write command. Tip: dont forget arguments, they are ready to use
                    

                    # Create local variables and set equal to class variables. Tip: use .copy()
                    
                    
                    # Add readings to lists. Use named lists already created for you

                    # Waypoint error -- copy from lab instructions --
                    

                    #------------- SECTION 4 FORWARDS KINEMATICS OF EE----------------
                    
                    # Compute ee_loc, ee_rot using QArm forward kinematics
                    
                    
                    
                    # Add result to list created for you. Only record xyz with ee_loc[:3].flatten()
                    

                    
                    #-------------------------------------------------------------------
                    # record time
                    time_data.append(elapsed_time(startTime))
                    
                    # Loop delay
                    time.sleep(0.1) 
            # Small loop delay between waypoints
                time.sleep(0.5)  
        except KeyboardInterrupt:
            print("user interupt detected")
            break
    # -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

    #Close arm properly
    myArm.terminate()

    # Convert lists to numpy arrays
    position_data = np.array(position_data)
    current_data = np.array(current_data)
    ee_data = np.array(ee_data)
    time_data = np.array(time_data)

    #------------------------------ SECTION 3 PLOT DATA -------------------------------
    #               PYPLOT REFERENCES CAN BE PROVIDED IN LAB BACKGROUND

    # Plot joint positions

    # Plot joint currents

    # Plot end effector trajectory

    # Show all plots
    
    #-------------------------------------------------------------------------

    print("Program Ended")

if __name__ == "__main__":
    main()