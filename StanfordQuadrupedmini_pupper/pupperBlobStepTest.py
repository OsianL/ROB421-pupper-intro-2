import os
import sys
import cv2
import curses
import numpy as np
import time
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display

def main():
    print("running main")

    """Main program
    """

    #Open the claw
    os.system("echo 2500000 > /sys/class/pwm/pwmchip0/pwm3/duty_cycle")
    time.sleep(2)

    #Setup P loop variables
    x_set_point = 320
    x_kp_value = 0.15/x_set_point
    x_pos = 320

    y_set_point = 240
    y_kp_value = 0.25/y_set_point
    y_pos = 240

    #*************************************************************
    # Movement Boilerplate
    #*************************************************************

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    # Create controller
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    #Setup State and command instances
    state = State()
    state.quat_orientation = np.array([1,0,0,0])
 
    command = Command()

    #************************************************************
    # OpenCV Setup
    #************************************************************
    
    # Initialize the webcam (0 is the default camera)
    cap = cv2.VideoCapture(0)

    # Set up SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # Adjust the parameters to detect blobs as needed
    params.filterByColor = True
    params.blobColor = 255  # Detect white blobs (0 for black blobs)

    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 20000

    params.filterByCircularity = True
    params.minCircularity = 0.1

    params.filterByConvexity = True
    params.minConvexity = 0.87

    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    #Flags for whether or not the ball has been found
    Detection = False
    firstDetectionFlag = False
    loopsSinceDetection = 0

    #********************************************************
    # Start of Main Loop
    #********************************************************

    #Handle the first loop iteration
    firstLoopFlag = True
    last_loop = time.time()
    now = time.time()

    #Loop timing
    vision_loop_time = 0.5
    last_loop = 0
    last_vision_loop = 0
    now = time.time()

    #In the main while loop, continually check the camera for the ball and move accordingly using two p loops.
    while True:
        #loop time
        last_loop = now
        now = time.time()
        deltaT = now-last_loop
        
        last_vision_loop += deltaT

        if(firstLoopFlag):
            firstLoopFlag = False
            state.behavior_state = BehaviorState.REST
            command.height = -0.05

        if (last_vision_loop > vision_loop_time):
        
            #Have the robot come to rest
            state.behavior_state = BehaviorState.REST
            time.sleep(0.25)

            # Capture frame-by-frame
            ret, frame = cap.read()
        
            # Convert frame to grayscale
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
            mask_blurred = cv2.GaussianBlur(mask, (9,9),0)
        
            # Detect blobs
            keypoints = detector.detect(mask_blurred)
        
            #Get the position of the first (if any) blobs detected
            #if Not detected, set x_pos = 0 and set some detection flags.
            if len(keypoints) > 0:
                x_pos = cv2.KeyPoint_convert(keypoints)[0][0]
                y_pos = cv2.KeyPoint_convert(keypoints)[0][1]
                print(cv2.KeyPoint_convert(keypoints))

                firstDetectionFlag = True
                Detection = True
                loopsSinceDetection = 0
            else: 
                x_pos = 0
                y_pos = 240
                Detection = False
                loopsSinceDetection += 1

            #Set the last vision looptime equal to zero
            last_vision_loop = 0

        else:

            #If needed, pause and wait for the ball to reappear
            if (firstDetectionFlag and not Detection) and (loopsSinceDetection < 3):
                continue

            #Have the robot enter trot mode
            state.behavior_state = BehaviorState.TROT

            #Calculate the yaw rate as a p controller
            x_error = x_set_point - x_pos
            yaw_rate = x_kp_value * x_error

            #Calculate the forward velocity as a p controller
            y_error = y_set_point - y_pos
            forward_velo = y_kp_value * y_error

            #Check if both the x and y errors are within tolerance
            if firstDetectionFlag and (x_error < 0.08) and (y_error < 0.08):
                break

            #Set the yaw and forward rates acording to the p loop
            print("passed Yaw rate: ", yaw_rate)
            command.yaw_rate = yaw_rate

            print("passed forward_velo: ", forward_velo)
            command.horizontal_velocity = np.array([forward_velo,0])

            #Output these values to the robot
            controller.run(state, command, disp)
            hardware_interface.set_actuator_postions(state.joint_angles)

    #Once Out of the main while loop, move forward and grab the ball:

    #Walk forward for 5 seconds
    state.behavior_state = BehaviorState.TROT
    command.horizontal_velocity = np.array([0.15,0])

    walk_forward_time = 5
    start_time = time.time()
    while True:
        if ((time.time() - start_time) > walk_forward_time):
            break
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)

    #Close the grabber on the ball
    os.system("echo 500000 > /sys/class/pwm/pwmchip0/pwm3/duty_cycle")
    time.sleep(3)



if __name__ == "__main__":
    main()
    print("Done")
