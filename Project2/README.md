# Dijkstra's Algorithm


## System Requirements-Linux

1. python3

`sudo apt install python3.11`

## Libraries and Dependencies

1. cv2        --> plotting
2. time       --> display time taken
3. numpy      --> array processing
4. matplotlib --> plotting map
5. heapq      --> to implement heap queue

## Running the program

1. Run `dijkstra_ManojKumar_Selvaraj.py`

`python3 dijkstra_ManojKumar_Selvaraj.py`

2. Enter startx, starty and endx ,endy

3. If start or end points are in obstacle , it exits with a msg.

4. If not, dijkstra algorithm is computed.

5. Once done, time is displayed with backtracking nodes.

6. And the output is saved as a video.

## Example Image for node from start(10, 10) to goal(1191,491)

![Dijkstra](./images/output.jpg)

## Legend
`Black            --->    Walls and clearance area of obstacle`

`White            --->    Free space`

`Blue             --->    Explored Nodes`

`Red              --->    Obstacle`

`White in Blue    --->    Shortest Path` 


## GIT LINK:

https://github.com/manojkumar1119/ENPM661/tree/main/Project2


