import concurrent.futures
import multiprocessing
import cv2
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
    print("Processing image")
    cap = cv2.VideoCapture(0)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 500
    params.maxArea = 20000
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = True
    params.minConvexity = 0.87
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(params)

    x_set_point = 320
    x_kp_value = 1/x_set_point

    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
    mask_blurred = cv2.GaussianBlur(mask, (9,9),0)
    keypoints = detector.detect(mask_blurred)

    if len(keypoints) > 0:
        x_pos = cv2.KeyPoint_convert(keypoints)[0][0]
        print("Keypoints:", cv2.KeyPoint_convert(keypoints))
    else: 
        x_pos = 0

    x_error = x_set_point - x_pos
    yaw_rate = x_kp_value * x_error

    cap.release()  # Release the camera resource
    return yaw_rate

def move_robot(command, controller, state, disp, hardware_interface, yaw_rate, imaging_complete):
    print("Moving robot")
    while not imaging_complete.is_set():
        command.yaw_rate = yaw_rate
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)
        time.sleep(0.1)  # Adding a small delay to avoid busy waiting
    print("Robot move function ended")

if __name__ == "__main__":
    config = Configuration()
    hardware_interface = HardwareInterface()
    disp = Display()
    disp.show_ip()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    state = State()
    state.quat_orientation = np.array([1,0,0,0])
    command = Command()

    imaging_complete = multiprocessing.Event()

    with concurrent.futures.ProcessPoolExecutor(max_workers=2) as executor:
        first_loop = True
        while True:
            print("Starting main loop iteration")

            if first_loop:
                print("Starting initial robot move process")
                move_future = executor.submit(move_robot, command, controller, state, disp, hardware_interface, 0, imaging_complete)
                first_loop = False
            else:
                print("Starting subsequent robot move process")
                move_future = executor.submit(move_robot, command, controller, state, disp, hardware_interface, yaw_rate, imaging_complete)

            print("Submitting image processing task")
            yaw_rate_future = executor.submit(image_process)
            yaw_rate = yaw_rate_future.result()  # Wait for the image processing to complete
            print("Yaw rate calculated:", yaw_rate)

            imaging_complete.set()  # Signal the move process to stop

            # Wait for the move process to acknowledge the stop signal
            move_future.result()

            imaging_complete.clear()  # Reset the event for the next iteration

            print("Main loop iteration completed")
