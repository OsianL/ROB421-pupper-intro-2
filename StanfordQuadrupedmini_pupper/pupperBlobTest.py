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
    x_set_point = 320
    x_kp_value = 1/x_set_point

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

    #Handle the first loop iteration
    firstLoopFlag = True
    last_loop = time.time()
    now = time.time()
    
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

    #Loop timing
    last_loop = 0
    now = time.time()

    
    while True:
        #loop time
        last_loop = now
        now = time.time()
        print("blob loop time: ", now - last_loop)

        # Capture frame-by-frame
        ret, frame = cap.read()
        
        # Convert frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
        mask_blurred = cv2.GaussianBlur(mask, (9,9),0)
        
        # Detect blobs
        keypoints = detector.detect(mask_blurred)
        
        if len(keypoints) > 0:
            x_pos = cv2.KeyPoint_convert(keypoints)[0][0]
            print(cv2.KeyPoint_convert(keypoints))
        else: 
            x_pos = 0

        x_error = x_set_point - x_pos

        # if now - last_loop < config.dt:
        #     continue

        if(firstLoopFlag):
            firstLoopFlag = False
            state.behavior_state = BehaviorState.REST
            command.height = -0.05
        else:
            state.behavior_state = BehaviorState.TROT
                    
        yaw_rate = x_kp_value * x_error
        
        print("passed Yaw rate: ", yaw_rate)

        command.yaw_rate = yaw_rate
        
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)    

if __name__ == "__main__":
    main()
    print("Done")
