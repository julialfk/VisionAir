import ast
import re
import time
from codrone_edu.drone import *


MAX_DISTANCE = 100


def get_pos(drone, use_bottom_range):
    current_x = drone.get_pos_x()
    current_y = drone.get_pos_y()
    if use_bottom_range:
        current_z = drone.get_bottom_range()
    else:
        current_z = drone.get_pos_z()

    if current_z >= 200 or current_z <= 0:
        current_z = drone.get_pos_z()

    return current_x, current_y, current_z


def has_arrived(drone, x, y, z, use_bottom_range):
    current_x, current_y, current_z = get_pos(drone, use_bottom_range)

    return (abs(current_x) <= abs(x) + 5 and abs(current_x) >= abs(x) - 5 and
            abs(current_y) <= abs(y) + 5 and abs(current_y) >= abs(y) - 5 and
            abs(current_z) <= abs(z) + 10 and abs(current_z) >= abs(z) - 10)


def calc_distance(drone, x, y, z, use_bottom_range):
    current_x, current_y, current_z = get_pos(drone, use_bottom_range)

    d_x = x - current_x
    d_y = y - current_y
    d_z = z - current_z

    return d_x, d_y, d_z


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


def move(drone, x, y, z, use_bottom_range=False):
    arrived = has_arrived(drone, x, y, z, use_bottom_range)
    # print(arrived)
    correct_started = False
    correct_time = 0

    # test whether more correcting time will improve end point
    while not arrived and not correct_started and (correct_time < 0.5):
        d_x, d_y, d_z = calc_distance(drone, x, y, z, use_bottom_range)
        current_x, current_y, current_z = get_pos(drone, use_bottom_range)
        # print(f"angle: {calc_angle(d_x)}, current_x: {current_x}")
        # print(f"angle: {calc_angle(d_y)}, current_y: {current_y}")
        # print(f"angle: {calc_angle(d_z)}, current_z: {current_z}")

        drone.set_pitch(calc_angle(d_x))
        drone.set_roll(-calc_angle(d_y))
        drone.set_throttle(calc_angle(d_z))
        drone.move()

        arrived = has_arrived(drone, x, y, z, use_bottom_range)
        if arrived and not correct_started:
            correct_start = time.time()

        if correct_started:
            correct_time = time.time() - correct_start

    print(f"(x,y,z) = ({drone.get_pos_x()}, {drone.get_pos_y()}, {drone.get_pos_z()})")
    print(f"bottom range = {drone.get_bottom_range()}")

    drone.reset_move_values()


def main():
    drone = Drone()
    drone.pair()
    drone.set_drone_LED(255,255,255,255)

    while(True):
        command = input("Enter command: [takeoff|land|move (x,y,z)|exit]\n")
        if command in ["exit", "quit", "q"]:
            drone.land()
            drone.close()
            return
        elif command == "takeoff":
            drone.takeoff()
        elif command == "land":
            drone.land()
        elif re.match(r"move \(-?[0-9]+,-?[0-9]+,-?[0-9]+\)", command):
            (x,y,z) = ast.literal_eval(command[5:])
            print(f"Flying to: ({x},{y},{z})")
            move(drone, x, y, z)
        elif command == "state":
            print(drone.get_flight_state())
        elif command == "current loc":
            print(drone.get_position_data())
            print(f"(x,y,z) = ({drone.get_pos_x()}, {drone.get_pos_y()}, {drone.get_pos_z()})")
            print(f"bottom range = {drone.get_bottom_range()}")
            print(f"front range = {drone.get_front_range()}")
            # print(f"flow x: {drone.get_flow_x()}, flow y: {drone.get_flow_y()}")
        elif command == "up":
            drone.set_throttle(30)
            drone.move(1)
            drone.reset_move_values()
        elif command == "down":
            drone.set_throttle(-30)
            drone.move(1)
            drone.reset_move_values()

    drone.close()

if __name__ == "__main__":
    main()