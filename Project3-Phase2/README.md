# A star Algorithm

## System requirements

1. python3

`sudo apt install python 3.11`

## Libraries used in Part1 and Part2:

numpy  ---> math operations

heapq  ---> priority heap

time   ---> to calculate time

cv2    ---> plotting

sys    ---> saving mp4

pygame ---> plotting map

math   ---> calculations

matplot---> plotting

rclpy  ---> ROS2

## Instructions to run the code:

## Part01

1. Run python3 proj3p2_Anbarasan_Manojkumar.py

`python3 proj3p2_Anbarasan_Manojkumar.py`

2. Enter start_node, end_node, radius of the bot and clearance in the following order:

`Enter Start x coordinates in mm:500`

`Enter Start y coordinates in mm:1000`

`Enter Start degrees in deg:0`

`Enter End x coordinates in mm:5750`

`Enter End y coordinates in mm:1000`

`Enter clearance in mm:10`

`Enter RPM1: 40`

`Enter RPM2: 20`

4. Once proj3p2_Anbarasan_Manojkumar.py completed running A_star_Anbarasan_ManojKumar.mp4 will be generated in the same folder location

5. Run A_star_Anbarasan_ManojKumar.mp4 to see the generated video

## Part02

1. Create workspace eg.`project3_ws`

2. Create a folder named "src" - `cd ~/project3_ws/src`

3. Copy the package folder into the "src" folder

4. Source your ROS  

`source /opt/ros/galactic/setup.bash`

5. Build the workspace from your workspace

`cd ~\project3_ws`

6. build --packages-select turtlebot3_project3

7. Source file - `source install/setup.bash`

8. Launch the world environment - `ros2 launch turtlebot3_project3 competition_world.launch.py`

9. Run the python code - `ros2 run turtlebot3_project3 temp.py`

Give the user inputs as shown for a test case

`Enter Start x coordinates in mm: 500`

`Enter Start y coordinates in mm: 1000`

`Enter Start degrees in deg: 0`

`Enter End x coordinates in mm: 5750`

`Enter End y coordinates in mm: 1000`

`Enter clearance in mm: 30`

`Enter left RPM: 40`

`Enter right RPM: 20`

A star algorithm will be executed and the action sets will be published to the cmd_vel topic

## Legend

BLACK             --->          Clearance

WHITE             --->          Screen

RED               --->          Obstacle

ORANGE            --->          Start

GREEN             --->          Goal

BLUE              --->          Explored

YELLOW            --->          Final Path

## Github link:

https://github.com/manojkumar1119/ENPM661/tree/main/Project3-Phase2

## Authors:

Manoj Kumar Selvaraj (120511257) 

Directory Id:manojs


Anbarasan Kandasamy  (120270697) 

Directory Id:anbuk
