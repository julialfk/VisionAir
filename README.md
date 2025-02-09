# VisionAir

This project contains various scripts for controlling a drone using the CoDrone EDU library and OpenCV.

https://wiki.robotika.sk/robowiki/index.php?title=VisionAir:_Camera_Controlled_CoDrone_-_Anna_Karik%C3%B3_%26_Julia_Liem#Additional_tests

Below is an explanation of each file in the src directory.

1. `ascend.py`
This script was used for testing and logging drone movement. It allows the drone to move at a specified angle and throttle for a given time. The script includes a log function that records flight data and saves it to a JSON file.

2. `cam_control.py`
This is the main script used to control the drone with computer vision. It utilizes OpenCV for object detection and tracking. It includes two tracking methods:

Blob detection: Uses background subtraction and blob detection to identify the drone's position.
TLD tracker: Uses a Tracking-Learning-Detection (TLD) algorithm for object tracking (not completed).
The script allows the user to give movement commands based on the detected position of the drone.

3. `control.py`
This file contains basic movement control functions for the drone, including functions to move in different directions based on input commands. It currently is used as a helper module in cam_control.py and ascend.py for manual control, but was mainly utilized for experimenting with the drone movement.

4. `plot.py`
A script that visualizes drone movement data collected from `ascend.py`.

5. `camera 241213 blob detection coordinates.py`
Old file used for experimentation with blob detection.

6. `camera 241217 kcf object tracking.py`
Old file used for experimentation with other types of detection, including KCF and TLD.

7. `camera.py`
Old file similar to `camera 241213 blob detection coordinates.py`.

8. `kcf_detection.py`
Old file siilar to `camera 241217 kcf object tracking.py`.
