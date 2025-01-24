import ast
import cv2
import numpy as np
import re
import time
from math import pi
from codrone_edu.drone import *
import control

# Constants for the camera and drone settings
CAM_PORT = 1  # Camera port index
MAX_DISTANCE = 400  # Maximum distance for normalization


def init_cam():
    """
    Initializes the camera and sets up blob detection parameters.
    
    Returns:
        cam (cv2.VideoCapture): Camera object for capturing frames.
        result (bool): Boolean indicating if the first frame was captured successfully.
        first (numpy.ndarray): First frame captured from the camera.
        params (cv2.SimpleBlobDetector_Params): Parameters for blob detection.
    """
    print("Initializing camera...")
    cam = cv2.VideoCapture(CAM_PORT)

    # reading the input using the camera
    print("capture")

    # Background subtractor
    fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

    # Set up blob detection parameters
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 30000
    params.filterByColor = True
    params.blobColor = 255
    params.filterByConvexity = False
    params.filterByInertia = False

    # Warm up the camera to stabilize the feed
    print("Warming up camera...")
    for _ in range(20):
        cam.read()

    result, first = cam.read()
    if not result:
        print("Error: Unable to capture the initial frame.")

    return cam, result, first, params


def get_pos(cam, result, first, params):
    """
    Detects the position of the target object using blob detection.

    Args:
        cam (cv2.VideoCapture): Camera object for capturing frames.
        result (bool): Indicates if a frame was successfully captured.
        first (numpy.ndarray): The first frame for background subtraction.
        params (cv2.SimpleBlobDetector_Params): Parameters for blob detection.

    Returns:
        tuple: (x, y, s) position and size of the detected object.
        or
        bool: False if no object is detected.
    """
    x = None
    while x is None:
        result, image = cam.read()

        if not result:
            print("Error: Unable to capture frame.")
            return False

        # Background subtraction using the initial frame
        difference = cv2.absdiff(image, first)

        # convert original picture to grayscale
        gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # convert grayscale to black and white with thresholding
        thresh = cv2.threshold(gray, 17, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=15)

        # setting up blob detection
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(thresh)

        if keypoints == ():
            print("No object found.")
            return False

        # Extract position and size of the first detected blob
        for kp in keypoints:
            x = kp.pt[0]
            y = kp.pt[1]
            s = kp.size
            print("x", x, "y", y, "size", s, "height", calc_height(s))

    return x, y, s


def has_arrived(cam, result, first, params, x, y, z):
    """
    Checks if the drone has arrived at the target position.

    Args:
        cam (cv2.VideoCapture): Camera object for capturing frames.
        result (bool): Indicates if a frame was successfully captured.
        first (numpy.ndarray): The first frame for background subtraction.
        params (cv2.SimpleBlobDetector_Params): Parameters for blob detection.
        x (float): Target x-coordinate.
        y (float): Target y-coordinate.
        z (float): Target height.

    Returns:
        bool: True if the drone has arrived, False otherwise.
        or
        None: If the current position of the drone could not be determined.
    """
    pos = get_pos(cam, result, first, params)
    if pos is False:
        return None

    current_x, current_y, current_s = pos
    current_z = calc_height(current_s)
    return (abs(current_x) <= abs(x) + 25 and abs(current_x) >= abs(x) - 25
            and abs(current_y) <= abs(y) + 25 and abs(current_y) >= abs(y) - 25
            and abs(current_z) <= abs(z) + 10 and abs(current_z) >= abs(z) - 10)


def calc_height(diameter, blob=True):
    """
    Calculates the height of the object based on its diameter.

    Args:
        diameter (float): Diameter of the detected blob.
        blob (bool): Flag indicating whether to use blob-based calculation.

    Returns:
        float: Calculated height of the object.
    """
    if blob:
        return 366.5 - 0.0132 * pi * (0.5 * diameter)**2


def calc_distance(cam, result, first, params, x, y, z):
    """
    Calculates the distance between the drone's current position and the target position.

    Args:
        cam (cv2.VideoCapture): Camera object for capturing frames.
        result (bool): Indicates if a frame was successfully captured.
        first (numpy.ndarray): The first frame for background subtraction.
        params (cv2.SimpleBlobDetector_Params): Parameters for blob detection.
        x (float): Target x-coordinate.
        y (float): Target y-coordinate.
        z (float): Target height.

    Returns:
        tuple: (dx, dy, dz) distances in the x, y, and z directions.
        or
        None: If the current position could not be determined.
    """
    pos = get_pos(cam, result, first, params)
    if pos is False:
        return None

    current_x, current_y, current_s = pos

    d_x = x - current_x
    d_y = y - current_y
    d_z = z - calc_height(current_s)

    return d_x, d_y, d_z


def calc_angle(distance, throttle=False):
    """
    Calculates the angle adjustment based on the distance.

    Args:
        distance (float): Distance to the target in a specific direction.
        throttle (bool): Flag to indicate throttle adjustment.

    Returns:
        float: Calculated angle value.
    """
    if throttle:
        base_value = 32
    else:
        base_value = 15
    angle = distance * 100 / MAX_DISTANCE
    if angle < -100:
        angle = -100
    elif angle < -5 and angle > -15:
        angle = -base_value
    elif angle > 100:
        angle = 100
    elif angle > 5 and angle < 15:
        angle = base_value

    return angle


def move(cam, result, first, params, drone, x, y, z):
    """
    Moves the drone to the target position.

    Args:
        cam (cv2.VideoCapture): Camera object for capturing frames.
        result (bool): Indicates if a frame was successfully captured.
        first (numpy.ndarray): The first frame for background subtraction.
        params (cv2.SimpleBlobDetector_Params): Parameters for blob detection.
        drone (Drone): The drone object.
        x (float): Target x-coordinate.
        y (float): Target y-coordinate.
        z (float): Target height.
    """
    arrived = has_arrived(cam, result, first, params, x, y, z)
    correct_started = False
    correct_time = 0

    while not arrived and not correct_started and (correct_time < 0.5):
        distance = calc_distance(cam, result, first, params, x, y, z)

        # Stop moving if drone flies off view
        if distance is None:
            break

        d_x, d_y, d_z = distance

        drone.set_pitch(calc_angle(d_x))
        drone.set_roll(-calc_angle(d_y))
        drone.set_throttle(calc_angle(d_z, throttle=True))
        drone.move()

        arrived = has_arrived(cam, result, first, params, x, y, z)
        if arrived and not correct_started:
            correct_start = time.time()

        if correct_started:
            correct_time = time.time() - correct_start

    drone.reset_move_values()


def main():
    cam, result, first, params = init_cam()
    drone = Drone()
    drone.pair()

    while True:
        command = input("Enter command: [takeoff|land|move (x,y,z)|exit]\n")
        if command in ["exit", "quit", "q"]:
            drone.land()
            drone.close()
            cam.release()
            cv2.destroyAllWindows()
            return
        elif command == "takeoff":
            drone.takeoff()
        elif command == "land":
            drone.land()
        elif re.match(r"move \(-?[0-9]+,-?[0-9]+,-?[0-9]+\)", command):
            (x,y,z) = ast.literal_eval(command[5:])
            print(f"Flying to: ({x},{y},{z})")
            move(cam, result, first, params, drone, x, y, z)
        elif command == "state":
            print(drone.get_flight_state())
        elif command == "current loc":
            get_pos(cam, result, first, params)
        elif command == "up":
            drone.set_throttle(30)
            drone.move(1)
            drone.reset_move_values()
        elif command == "down":
            drone.set_throttle(-30)
            drone.move(1)
            drone.reset_move_values()
        elif re.match(r"manual \(-?[0-9]+,-?[0-9]+,-?[0-9]+\)", command):
            (x,y,z) = ast.literal_eval(command[7:])
            print(f"Flying to: ({x},{y},{z})")
            control.move(drone, x, y, z)

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
