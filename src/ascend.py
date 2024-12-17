import ast
import json
# import matplotlib.pyplot as plt
import re
import time
from codrone_edu.drone import *
from control import move, get_pos
# from plot import plot


def log(drone, angle, throttle, move_time, file_name):
    flight_log = {"angle": angle, "throttle": throttle, "move_time": move_time}
    start = time.time()

    times, xs, ys, zs = [], [], [], []
    dt = 0
    while dt < move_time:
        dt = time.time() - start
        x, y, z = get_pos(drone, True)
        # f.write(f"{dt},{x},{y},{z}\n")
        xs.append(x)
        ys.append(y)
        zs.append(z)
        times.append(dt)

    flight_log["times"] = times
    flight_log["x"] = xs
    flight_log["y"] = ys
    flight_log["z"] = zs

    with open(f"logs/{file_name}.json", 'a') as f:
        f.write(json.dumps(flight_log))


def test(drone, angle, throttle, move_time, file_name=None):
    drone.set_pitch(angle)
    drone.set_throttle(throttle)
    # with open("log.txt", 'a') as f:
    #     f.write(f"angle, throttle, time = {angle}, {throttle}, {move_time}\n")
    #     f.write("time, x, y, z\n")

    if file_name == None:
        drone.move(move_time)
    else:
        drone.move()
        log(drone, angle, throttle, move_time, file_name)
    drone.reset_move_values()


def main():
    drone = Drone()
    drone.pair()

    angle = 10
    throttle = 50
    move_time = 5

    while True:
        command = input("Enter command: [takeoff|land|test|log [file_name]|set angle/throttle/time [value]|reset|exit]\n")
        if command in ["exit", "quit", "q"]:
            drone.land()
            drone.close()
            return
        elif command == "takeoff":
            drone.takeoff()
            move(drone, 0, 0, 30)
        elif command == "land":
            drone.land()
        elif command == "test":
            test(drone, angle, throttle, move_time)
        elif re.match(r"log .*", command):
            test(drone, angle, throttle, move_time, file_name=command[4:])
        elif re.match(r"set angle -?[0-9]+", command):
            angle = ast.literal_eval(command[10:])
            print(f"Set angle to: {angle}")
        elif re.match(r"set throttle -?[0-9]+", command):
            throttle = ast.literal_eval(command[13:])
            print(f"Set throttle to: {angle}")
        elif re.match(r"set time -?[0-9]+", command):
            move_time = ast.literal_eval(command[9:])
            print(f"Set movement time to: {move_time}")
        elif command == "reset":
            move(drone, 0, 0, 30)

    drone.close()


if __name__ == "__main__":
    main()