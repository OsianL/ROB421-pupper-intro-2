import cv2
import numpy as np
import time

# Initialize the webcam (0 is the default camera)
cap = cv2.VideoCapture(1)

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
    
    if not ret:
        break
    
    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 120, 50), (25, 255, 255))
    mask_blurred = cv2.GaussianBlur(mask, (9,9),0)
    
    # Detect blobs
    keypoints = detector.detect(mask_blurred)

    print(cv2.KeyPoint_convert(keypoints))
    
    # Draw detected blobs as red circles.
    # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of the blob
    frame_with_keypoints = cv2.drawKeypoints(frame, keypoints, np.array([]), (0, 0, 255),
                                             cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    cv2.imshow('blurred mask', mask_blurred)
    # Display the resulting frame with keypoints
    cv2.imshow('Webcam Feed with Blobs', frame_with_keypoints)
    
    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()