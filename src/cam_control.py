import ast
import cv2
import numpy as np
import re
import time
from codrone_edu.drone import *
import control

CAM_PORT = 1
MAX_DISTANCE = 400


def init_cam():
    # initialize the camera
    # If you have multiple camera connected with current device,
    # assign a value in CAM_PORT variable according to that
    print("init cam")
    cam = cv2.VideoCapture(CAM_PORT)

    # reading the input using the camera
    print("capture")

    # Background subtractor
    fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 30000
    params.filterByColor = True
    params.blobColor = 255
    params.filterByConvexity = False
    params.filterByInertia = False

    n = 20
    print("loading...")
    for _ in range(n):
        cam.read()
        print("...")

    result, first = cam.read()
    # if not result:
    #     print("No image detected. Please! try again")

    return cam, result, first, params


def get_pos(cam, result, first, params):
    x = None
    while x is None:
        # showing result, it takes frame name and image output as arguments
        result, image = cam.read()

        if not result:
            print("No image detected. Please! try again")
            return False

        # original picture
        difference = cv2.absdiff(image, first)
        # cv2.imshow("diff", difference)

        # convert original picture to grayscale
        gray = cv2.cvtColor(difference, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        # cv2.imshow("grayscale", gray)

        # convert grayscale to black and white with thresholding
        thresh = cv2.threshold(gray, 17, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=15)
        # cv2.imshow("thresholded", thresh)

        # apply background subtraction

        # setting up blob detection
        detector = cv2.SimpleBlobDetector_create(params)
        keypoints = detector.detect(thresh)
        im_with_keypoints = cv2.drawKeypoints(
            thresh,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        # cv2.imshow("Keypoints", im_with_keypoints)
        print(keypoints)

        # get keypoints parameters
        # list of blobs keypoints
        for kp in keypoints:
            x = kp.pt[0]
            y = kp.pt[1]
            s = kp.size
            print(f"x", x, "y", y, "size", s)

    return x, y, s


def has_arrived(cam, result, first, params, drone, x, y):
    current_x, current_y, current_s = get_pos(cam, result, first, params)

    return(abs(current_x) <= abs(x) + 5 and abs(current_x) >= abs(x) - 5
           and abs(current_y) <= abs(y) + 5 and abs(current_y) >= abs(y) - 5)


def calc_distance(cam, result, first, params, drone, x, y):
    current_x, current_y, current_s = get_pos(cam, result, first, params)

    d_x = x - current_x
    d_y = y - current_y

    return d_x, d_y


def calc_angle(distance, throttle=False):
    if throttle:
        base_value = 8
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


def move(cam, result, first, params, drone, x, y):
    arrived = has_arrived(cam, result, first, params, drone, x, y)
    correct_started = False
    correct_time = 0

    # test whether more correcting time will improve end point
    while not arrived and not correct_started and (correct_time < 0.5):
        d_x, d_y = calc_distance(cam, result, first, params, drone, x, y)
        current_x, current_y, current_s = get_pos(cam, result, first, params)
        # print(f"angle: {calc_angle(d_x)}, current_x: {current_x}")
        # print(f"angle: {calc_angle(d_y)}, current_y: {current_y}")
        # print(f"angle: {calc_angle(d_z)}, current_z: {current_z}")

        drone.set_pitch(calc_angle(d_x))
        drone.set_roll(-calc_angle(d_y))
        drone.set_throttle(calc_angle(0, throttle=True))
        drone.move()

        arrived = has_arrived(cam, result, first, params, drone, x, y)
        if arrived and not correct_started:
            correct_start = time.time()

        if correct_started:
            correct_time = time.time() - correct_start

    print(f"(x,y,z) = ({drone.get_pos_x()}, {drone.get_pos_y()}, {drone.get_pos_z()})")
    print(f"bottom range = {drone.get_bottom_range()}")

    drone.reset_move_values()


def main():
    cam, result, first, params = init_cam()
    drone = Drone()
    drone.pair()

    # If image will detected without any error, show result
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
            move(cam, result, first, params, drone, x, y)
        elif command == "state":
            print(drone.get_flight_state())
        elif command == "current loc":
            # print(drone.get_position_data())
            # print(f"(x,y,z) = ({drone.get_pos_x()}, {drone.get_pos_y()}, {drone.get_pos_z()})")
            # print(f"bottom range = {drone.get_bottom_range()}")
            # print(f"front range = {drone.get_front_range()}")
            # print(f"flow x: {drone.get_flow_x()}, flow y: {drone.get_flow_y()}")
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
