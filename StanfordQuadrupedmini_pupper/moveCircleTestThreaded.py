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
    
    """Main program
    """
    x_set_point = 320
    x_kp_value = 1/x_set_point

    camera_dt = 0.100
    camera_last_frame = time.time()

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
    last_loop = 0 
    now = time.time() - config.dt
    
    # 0 is default, I changed to 1 to get the back camera of my surface
    cap = cv2.VideoCapture(0)  # Initialize webcam (0 is usually the default webcam)

    captured_last_loop = False

    

    while True:
        if now - last_loop < config.dt:
            print("Skipping Time")
            continue
        else: 
            print("Not skipping Time")
            last_loop = now
            now = time.time()
            #print("loop time: ", now-last_loop)

        if(firstLoopFlag):
            firstLoopFlag = False
            state.behavior_state = BehaviorState.REST
            command.height = -0.05
            print("First Loop Passed")
        else:
            state.behavior_state = BehaviorState.TROT

        if (now - camera_last_frame) > camera_dt:
            ret, frame = cap.read()  # Read a frame from the webcam

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
            mask_blurred = cv2.GaussianBlur(mask, (9,9),0)

            circles = cv2.HoughCircles(mask_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, 
                                        param1=200, param2=30, minRadius=5, maxRadius=150) #min 10, max 100 default
            print("Reading Camera")
            camera_last_frame = now
        # captured_last_loop = not captured_last_loop        

        # ret, frame = cap.read()  # Read a frame from the webcam

        # hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
        # mask_blurred = cv2.GaussianBlur(mask, (9,9),0)

        # circles = cv2.HoughCircles(mask_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, 
        #                             param1=200, param2=30, minRadius=5, maxRadius=150) #min 10, max 100 default
        
        if circles is not None:
            x_pos = circles[0][0][0]
        else: 
            x_pos = 0

        x_error = x_set_point - x_pos
                    
        yaw_rate = x_kp_value * x_error

        command.yaw_rate = yaw_rate

        # New: Trying to move forward when the ball is centred enough
        # if abs(x_error) < 5:
        #     command.horizontal_velocity = np.array([0.2,0])

        # print("x pos: ", x_pos)
        # print("error: ", x_error)
        # print("commanded yaw rate: ", yaw_rate)
        
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)    

if __name__ == "__main__":
    main()
    print("Done")
