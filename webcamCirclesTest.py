import cv2
import numpy as np

def track_circles_in_webcam():
    # 0 is default, I changed to 1 to get the back camera of my surface
    cap = cv2.VideoCapture(1)  # Initialize webcam (0 is usually the default webcam)

    while True:
        ret, frame = cap.read()  # Read a frame from the webcam

        if not ret:
            break

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convert frame to grayscale
        # gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)  # Apply Gaussian blur to reduce noise

        # cv2.imshow("grayscale", gray_blurred)

        # circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        #                            param1=200, param2=30, minRadius=10, maxRadius=100)

        # circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
        #                            param1=200, param2=30, minRadius=10, maxRadius=100)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (5, 150, 50), (25, 255, 255))
        mask_blurred = cv2.GaussianBlur(mask, (9,9),0)

        cv2.imshow("orange mask", mask_blurred)

        circles = cv2.HoughCircles(mask_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=50, 
                                   param1=200, param2=30, minRadius=10, maxRadius=100)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            for (x, y, r) in circles:
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)  # Draw the circle
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)  # Draw the center
            print("Circles: ", circles)

        cv2.imshow("Circle Tracker", frame)  # Display the frame

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to exit
            break

    cap.release()  # Release the webcam
    cv2.destroyAllWindows()  # Close all OpenCV windows

if __name__ == "__main__":
    track_circles_in_webcam()
