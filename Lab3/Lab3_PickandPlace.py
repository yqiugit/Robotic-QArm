"""
    MREN 410 Lab4 Pick and Place
    Developed from Quanser Code
    Last Updated: 08/18/2025
"""
# -- -- -- -- -- -- -- -- -- -- -- SETUP: READ BUT DONT CHANGE -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Imports
from pal.products.qarm import QArmRealSense
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
from hal.utilities.image_processing import ImageProcessing
import numpy as np
import cv2
import time

# -- -- -- -- -- -- -- -- -- -- -- SETUP: READ BUT DONT CHANGE -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Parameters
imageWidth = 640
imageHeight = 480

# ---------------- Waypoints  ----------------
PLACE_BASE = -0.8                  # DO NOT ADJUST
HOME_BASE = 0.00                   # DO NOT ADJUST
HOME_JOINTONE = 0.0                # DO NOT ADJUST
HOME_JOINTTWO = 0.9                # DO NOT ADJUST
base_pos = 0.00                    # DO NOT ADJUST
GRIP_OPEN, GRIP_CLOSE = 0.0, 0.78   # DO NOT ADJUST
WRIST = 0.0


#-----------------  SECTION 2: WAYPOINT POSITIONS -------------------- #
PICK_JOINTONE = 0.7             # Experimentally adjust
PICK_JOINTTWO = 0.3             # Experimentally adjust

PICK_GRIP_JOINTONE = 0.7      # Experimentally adjust
PICK_GRIP_JOINTTWO = 0.30       # Experimentally adjust

PLACE_JOINTONE = 0.5            # Experimentally adjust
PLACE_JOINTTWO = 0.4            # Experimentally adjust

PLACE_RELEASE_JOINTONE = 0.67   # Experimentally adjust
PLACE_RELEASE_JOINTTWO = 0.29  # Experimentally adjust

# --------------------------- READ BELOW TO UNDERSTAND SEQUENCE -------------------------------------------#
waypoints = [np.array([HOME_BASE,   HOME_JOINTONE,          HOME_JOINTTWO,           WRIST, GRIP_OPEN], dtype=float),    # Home points
             np.array([base_pos,    PICK_JOINTONE,          PICK_JOINTTWO,           WRIST, GRIP_OPEN], dtype=float),    # Pick Approach
             np.array([base_pos,    PICK_GRIP_JOINTONE,     PICK_GRIP_JOINTTWO,      WRIST, GRIP_OPEN], dtype=float),   # Pick Grip
             np.array([base_pos,    PICK_GRIP_JOINTONE,     PICK_GRIP_JOINTTWO,      WRIST, GRIP_CLOSE], dtype=float),   # Pick Grip
             np.array([PLACE_BASE,  PLACE_JOINTONE,         PLACE_JOINTTWO,          WRIST, GRIP_CLOSE], dtype=float),
             np.array([PLACE_BASE,  PLACE_RELEASE_JOINTONE, PLACE_RELEASE_JOINTTWO,  WRIST, GRIP_CLOSE], dtype=float),   # Place Approach
             np.array([PLACE_BASE,  PLACE_RELEASE_JOINTONE, PLACE_RELEASE_JOINTTWO,  WRIST, GRIP_OPEN], dtype=float),
             np.array([HOME_BASE,   HOME_JOINTONE,          HOME_JOINTTWO,           WRIST, GRIP_OPEN], dtype=float)]    # Place Release
#print(waypoints)
#
# positional tollerance
tollerance = 0.1

# debug bool 
verbose = False

# -- -- -- -- -- -- -- -- -- -- -- SETUP: READ BUT DONT CHANGE -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Main Loop
def main():
    global base_pos
    # Objects being set up
    print("am i here?")
    myArm = QArm(hardware=1, readMode=0)
    print("am i here?")
    myArmUtilities = QArmUtilities()
    camTool = ImageProcessing()
    np.set_printoptions(precision=2, suppress=True)
    # Initial variable states
    gripCmd = 0
    phiCmd = [0.0,0.0,0.8,0.0]
    ledCmd = np.array([0, 1, 0], dtype=np.float64)
    cx, cy = 0, 0
    base = 0.0
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- #
    with QArmRealSense(mode='RGB&DEPTH',
                       hardware=1,
                       deviceID=0,
                       frameWidthRGB=imageWidth,
                       frameHeightRGB=imageHeight,
                       frameWidthDepth=imageWidth,
                       frameHeightDepth=imageHeight,
                       readMode=1) as myCam1:
        try:
          while myArm.status:                
            for i in range(len(waypoints)):
                # ------------------ SECTION 1.0: DEPTH MASK
                # myCam1.read_RGB()
                # img = myCam1.imageBufferRGB
                # myCam1.read_depth(dataMode='M')
                # imgD = myCam1.imageBufferDepthM
                

                # img_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                # RGB_Binary = ImageProcessing.binary_thresholding(frame=img_HSV,lowerBounds=np.array([0,150,100]),upperBounds=np.array([10,255,255]))
                # #cv2.imshow("RGB Bin", RGB_Binary)
                # #img_Bone = cv2.applyColorMap(img_HSV, cv2.COLORMAP_BONE) 
                
                # # imgD = cv2.convertScaleAbs(imgD, alpha=51)
                # imgD = cv2.applyColorMap(img_HSV, cv2.COLORMAP_JET) 
                # imgD_HSV = cv2.cvtColor(imgD, cv2.COLOR_BGR2HSV)
                #cv2.imshow("RGB",img)  
                #cv2.imshow("RGBD",imgD)        
                #cv2.imshow("HSVD", imgD)
                #cv2.imshow("DHSV", imgD_HSV)

                
                #binary_image = ImageProcessing.binary_thresholding(frame=imgD_HSV,lowerBounds=np.array([18,150,100]),upperBounds=np.array([30,255,255]))
                
                
                #cv2.imshow("binary",binary_image)        
                #cv2.imshow("imgD", imgD)
                
                # Open and Close operation

                # ------------------- SECTION 3.0: OBJECT DETECTION -------------------
                if(i == 1):
                    tracking = True
                    while(tracking):
                        # IMPLEMENT COMPUTER VISION AND CENTERING HERE
                        myCam1.read_RGB()
                        first_image = myCam1.imageBufferRGB

                        # Convert RGB to HSV
                        hsv_image = cv2.cvtColor(first_image,cv2.COLOR_BGR2HSV)

                        # Binary Image Threshold
                        binary_image = ImageProcessing.binary_thresholding(frame=hsv_image,lowerBounds=np.array([0,150,100]),upperBounds=np.array([5,255,255]))
                        #cv2.imshow("binary",binary_image)
                        kernal = np.ones((5,5), np.uint8)
                        open_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernal)
                        mask_image = cv2.morphologyEx(open_image, cv2.MORPH_CLOSE, kernal)
                        mask_image = cv2.dilate(mask_image, kernal, iterations = 2)
                        
                        contours, hierarchy = cv2.findContours(mask_image,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                        #Filter and compute centroid from learn OpenCV Tutorial

                        for contour in contours:
                            M = cv2.moments(contour)
                            cx = int(M["m10"]/M["m00"])
                            cy = int(M["m01"]/M["m00"])
                            cv2.imshow("final", cv2.circle(mask_image, (cx,cy), 10, (127), -1))
                            #print(cx,cy)

                        # IMPLEMENT CONDITIONAL TO END TRACKING ONCE STABLE (tracking variable set to false)
                            # WHEN CONDITION IS MET, READ ARM BASE JOINT AND SET base variable EQUAL TO JOINT POS
                        

                        image_center_x, image_center_y = 320, 240
                        if(cx > 320):
                            cx -= 100
                        x_error = image_center_x - cx 
                        y_error = image_center_y - cy

                        # if(x_error > -25 and x_error < 25):
                        #     base = phiCmd[0]
                        #     tracking = False

                        if (x_error<-25 and phiCmd[0] > -1.2):
                            phiCmd[0] -=0.006

                        elif(x_error>25 and phiCmd[0] < 1.2):
                            phiCmd[0]+=0.006

                        else:
                            base = phiCmd[0]
                            tracking = False
                            
                        myArm.read_write_std(phiCMD=phiCmd, gprCMD=gripCmd, baseLED=ledCmd)     # THIS LINE SHOULD BE EXPLAINED IN INSTRUCTIONS

                        
                        # IF CONDITION IS NOT MET, COMMAND THE ARM TO MOVE OBJECT TO CENTER

                      
                      
                        pass # for empty loop, you can remove
                # ----------------- WAYPONT TRACKING: DO NOT ADJUST ------------------
                #----------------- WAYPOINT TASK IS AT TOP OF DOCUMENT ---------------
                # Update command based on waypoint
                print(base_pos)
                phiCmd = waypoints[i][0:4]
                gripCmd = waypoints[i][4]
                waypoints[1][0] =  base
                waypoints[2][0] =  base
                waypoints[3][0] =  base

                if i == 1: print(base, waypoints[i])
                # Command Gripper
                location, rotation = myArmUtilities.forward_kinematics(phiCmd)
                error = 10
                while(error > tollerance):
                    # Write command and read sensor values
                    myArm.read_write_std(phiCMD=phiCmd, gprCMD=gripCmd, baseLED=ledCmd)     # THIS LINE SHOULD BE EXPLAINED IN INSTRUCTIONS
                    #myArm.read_write_std(phiCMD=phiCmd, grpCMD=gripCmd, baseLED=ledCmd)   

                    # Read Class variables // joint pos is done for you 
                    jointPos = myArm.measJointPosition                                      # THESE LINES ARE ALSO IMPORTANT TO SHARE
                    ee_loc, ee_rot = myArmUtilities.forward_kinematics(jointPos)

                    #print(ee_loc) # DEBUGG
                    # CHECK IF FORWARDS KINEMATICS SAYS POSITION IS BAD
                    # END PROGRAM
                    if(ee_loc[2] < 0.0): raise Exception("Foward Kinematics Puts Arm in bad location.. try adjust waypoints to avoid potential collision")

                    # Caluclate position error from waypoint 
                    error = np.linalg.norm(jointPos[0:4] - waypoints[i][0:4])                    # this should be explained, maybe calculation done manually
                    # NOTE::: THIS DOES NOT REPRESENT SPACIAL DISTANCE, BUT RATHER THE TOTAL JOINT ERROR 
                    if verbose : print("Waypoint:", waypoints[i]," Current Joint Positions:", str(jointPos), " error:", error)
                    # Loop delay
                    time.sleep(0.1) 
                # Time between waypoints
                time.sleep(0.5) 
                cv2.waitKey(1)
        except KeyboardInterrupt:
            print("\nUser interrupt detected. Terminating QArm and OpenCV...")
        finally:
            myArm.terminate()
            cv2.destroyAllWindows()
            print("Program Ended")


if __name__ == "__main__":
    main()

