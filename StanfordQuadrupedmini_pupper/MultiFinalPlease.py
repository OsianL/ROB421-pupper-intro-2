import concurrent.futures
import multiprocessing
import cv2
# import curses
import numpy as np
import time
from src.Controller import Controller
from src.Command import Command
from src.State import BehaviorState, State
from MangDang.mini_pupper.HardwareInterface import HardwareInterface
from MangDang.mini_pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
from MangDang.mini_pupper.display import Display

def image_process():
    print("processing image")
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

    x_set_point = 320
    x_kp_value = 1/x_set_point

    # Capture frame-by-frame
    ret, frame = cap.read()
    
    # Convert frame to grayscale
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

    yaw_rate = x_kp_value * x_error

    return yaw_rate

def move_robot(command, controller, state, disp, hardware_interface, yaw_rate, imaging_complete):
    print("in Move")
    while not imaging_complete.is_set():
        print("moving")
        command.yaw_rate = yaw_rate
        
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles) 
    return("done moving")

if __name__ == "__main__":
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

    #Loop timing
    last_loop = 0
    now = time.time()

    first_process = True

    # while True:
    with multiprocessing.Manager() as manager:
        imaging_complete = manager.Event()
        with concurrent.futures.ProcessPoolExecutor() as executor:
            while True:
                #Event stops running move when new image is done processing
                #Moves continuously based on previous yaw command until new one is done
                if first_process is True:
                    print("first move")
                    # print("imaging complete event: ", imaging_complete)
                    move = executor.submit(move_robot, command, controller, state, disp, hardware_interface, 0)
                    first_process = False
                else:
                    print("executing move")
                    # print("imaging complete event: ", imaging_complete)
                    move = executor.submit(move_robot, command, controller, state, disp, hardware_interface, yaw_rate.result())
                    print("move executed")
                #Captures image and calculates new yaw rate
                yaw_rate = executor.submit(image_process)
                yaw_result = yaw_rate.result()
                print("yaw rate: ", yaw_result)
                imaging_complete.set()

                # move_result = move.result()
                # print(move_result)

                imaging_complete.clear()
                