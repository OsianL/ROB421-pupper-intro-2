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

# From webcamCirclesTest.py 
# If a circle is detected outputs an array of [x coordinate, y coordinate, radius magnitude] in terms of camera frame pixels
def track_circles_in_webcam():
    # 0 is default, I changed to 1 to get the back camera of my surface
    cap = cv2.VideoCapture(0)  # Initialize webcam (0 is usually the default webcam)

    # while True:
    ret, frame = cap.read()  # Read a frame from the webcam

    if not ret:
        # break
        return None

    # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
    # gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian blur to reduce noise

    # cv2.imshow("grayscale", gray_blurred)

    # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
    #                            param1=200, param2=30, minRadius=10, maxRadius=100)

    # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
    #                            param1=200, param2=30, minRadius=10, maxRadius=100)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
    mask_blurred = cv2.GaussianBlur(mask, (9,9),0)

    # cv2.imshow("orange mask", mask_blurred)

    circles = cv2.HoughCircles(mask_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, 
                                param1=200, param2=30, minRadius=5, maxRadius=150) #min 10, max 100 default

    # if circles is not None:
    #     circles = np.round(circles[0, :]).astype("int")

    #     for (x, y, r) in circles:
    #         cv2.circle(frame, (x, y), r, (0, 255, 0), 4)  # Draw the circle
    #         cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)  # Draw the center
    #     print("Circles: ", circles)
    # else: 


    # cv2.imshow("Circle Tracker", frame)  # Display the frame

    # if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
    #     break

    cap.release()  # Release the webcam
    cv2.destroyAllWindows()  # Close all OpenCV windows
    return circles

def main():
    
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
    
    # 0 is default, I changed to 1 to get the back camera of my surface
    cap = cv2.VideoCapture(0)  # Initialize webcam (0 is usually the default webcam)

    # try:
    
    while True:
        

        # while True:
        ret, frame = cap.read()  # Read a frame from the webcam

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
        mask_blurred = cv2.GaussianBlur(mask, (9,9),0)

        circles = cv2.HoughCircles(mask_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, 
                                    param1=200, param2=30, minRadius=5, maxRadius=150) #min 10, max 100 default
        
        if circles is not None:
            x_pos = circles[0][0][0]
        else: 
            x_pos = 0

        x_error = x_set_point - x_pos

        now = time.time()
        if now - last_loop < config.dt:
            continue

        if(firstLoopFlag):
            firstLoopFlag = False
            state.behavior_state = BehaviorState.REST
        else:
            state.behavior_state = BehaviorState.TROT
            
        last_loop = time.time()
        
        print("x pos: ", x_pos)
        print("error: ", x_error)
            
        command.yaw_rate = x_kp_value * x_error

        # if circles is None:
        #     command.horizontal_velocity = np.array([0, 0])
        #     command.yaw_rate = 0.7
        # else:
        #     command.yaw_rate = 0
        #     command.horizontal_velocity = np.array([0.2, 0])
        
        controller.run(state, command, disp)
        hardware_interface.set_actuator_postions(state.joint_angles)        

if __name__ == "__main__":
    main()
    print("Done")
