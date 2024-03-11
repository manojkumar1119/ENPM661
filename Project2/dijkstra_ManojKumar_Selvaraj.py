#Libs used

import cv2
import numpy as np
import heapq
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


#Class for each node
class Node:

    #constructor
    def __init__(self, x ,y):
        self.x = x
        self.y = y

        self.neighbors = {}
        self.previousNode = None
        self.shortest_distance = float('inf')

    #less than method
    def __lt__(self, other):
        return self.shortest_distance < other.shortest_distance

    #hashing
    def __hash__(self):
        return hash((self.x, self.y))
    
# Function to draw space for rectangle
def obs_rectangle(obstacle_matrix, pt_x, pt_y , w, h,val):

    # Define obstacles 
    for i in range(pt_x, pt_x + h):
        for j in range(pt_y, pt_y + w):
            obstacle_matrix[i][j] = val
    return obstacle_matrix

#visualize the map 
def visualize_canvas(matrix, start_x, start_y, end_x, end_y):

    matrix_np = np.array(matrix)

    # Invert the array along the y-axis
    matrix_np = np.flipud(matrix_np)

    # Create a color map where -1 is black and 0 is white -2 is red
    cmap = plt.cm.colors.ListedColormap(['red', 'black', 'white'])

    # Create a matrix plot
    plt.matshow(matrix_np, cmap=cmap)

    plt.plot(start_x, start_y, 'go', label='Point') 
    plt.plot(end_x, end_y, 'bx', label='Point') 

    plt.gca().invert_yaxis()
    
    # Show the plot
    plt.show()

    plt.pause(5)
    plt.close()


#Enter starting coodinates
start_x = int(input("Enter Start_x coord:"))
start_y = int(input("Enter Start_y coord:"))

#Enter goal coordinates
end_x = int(input("Enter End_x coord:"))
end_y = int(input("Enter End_y coord:"))

#Setting the obstacle space as matrix
#-1 -->obstacle node
# 0 -->free nodes

# Defining the size of the canvas
Map_Width = 1200 #cols
Map_Height = 500 #rows

#Creating an empty matrix with all free space as 0
obstacle_matrix = np.zeros((Map_Height, Map_Width))

#Canvas (Map) clearance 5mm inwards
for i in range(0, Map_Height):
    for j in range(0, Map_Width):
        if((i<5) or (i>494) or (j<5) or (j>1194)):
            obstacle_matrix[i][j] = -1

#Define 1st rectagle without clearance
pt_x  = 0
pt_y  = 100
rec_w = 75
rec_h = 400

#CAlling with clearance
clearance = 5

#called twice once with clearance and once without clearance
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y-clearance , rec_w+2*clearance, rec_h+clearance,-1)
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y , rec_w, rec_h,-2)

#Define 2nd rectagle without clearance
pt_x  = 100
pt_y  = 275
rec_w = 75
rec_h = 400

obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x-clearance, pt_y-clearance , rec_w+2*clearance, rec_h,-1)
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y , rec_w, rec_h,-2)

#Defining inverted c shaped structure with 3 broken rectangles
#small rect 1
pt_x  = 50
pt_y  = 900
rec_w = 200
rec_h = 75

obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x-clearance, pt_y-clearance , rec_w+2*clearance, rec_h+2*clearance,-1)
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y , rec_w, rec_h,-2)

#small rect 2
pt_x  = 375
pt_y  = 900
rec_w = 200
rec_h = 75

obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x-clearance, pt_y-clearance , rec_w+2*clearance, rec_h+2*clearance,-1)
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y , rec_w, rec_h,-2)

#Larger rect 1
pt_x  = 50
pt_y  = 1020
rec_w = 80
rec_h = 400

obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x-clearance, pt_y-clearance , rec_w+2*clearance, rec_h+2*clearance,-1)
obstacle_matrix = obs_rectangle(obstacle_matrix, pt_x, pt_y , rec_w, rec_h,-2)

# Define the vertices of the hexagon

hexagon_pt1 = np.array([650, 100-clearance-2])
hexagon_pt2 = np.array([775+clearance, 175-clearance])
hexagon_pt3 = np.array([775+clearance, 325+clearance])
hexagon_pt4 = np.array([650, 400+clearance+2])
hexagon_pt5 = np.array([525-clearance, 325+clearance])
hexagon_pt6 = np.array([525-clearance, 175-clearance])

vertices = np.array([hexagon_pt1,hexagon_pt2,hexagon_pt3,hexagon_pt4,hexagon_pt5,hexagon_pt6])

# Fill the hexagon in the matrix
cv2.fillPoly(obstacle_matrix, [vertices], -1)

# Define the vertices of the hexagon without clearance

hexagon_pt1 = np.array([650, 100])
hexagon_pt2 = np.array([775, 175])
hexagon_pt3 = np.array([775, 325])
hexagon_pt4 = np.array([650, 400])
hexagon_pt5 = np.array([525, 325])
hexagon_pt6 = np.array([525, 175])
vertices = np.array([hexagon_pt1,hexagon_pt2,hexagon_pt3,hexagon_pt4,hexagon_pt5,hexagon_pt6])

# Fill the hexagon in the matrix
cv2.fillPoly(obstacle_matrix, [vertices], -2)

#Visualize the map created to verify 
#visualize_canvas(obstacle_matrix, start_x, start_y, end_x, end_y)

#Now as the matrix starts from 0,0 we need to invert since all coords are from origin
obstacle_matrix = obstacle_matrix[::-1]

#define action sets with cost
action_set = {
    'UP'        : ((0, 1), 1.0),
    'DOWN'      : ((0, -1), 1.0),
    'LEFT'      : ((-1, 0), 1.0),
    'RIGHT'     : ((1, 0), 1.0),

    'UP-LEFT'   : ((-1, 1), 1.4),
    'UP-RIGHT'  : ((1, 1), 1.4),
    'DOWN-LEFT' : ((-1, -1), 1.4),
    'DOWN-RIGHT': ((1, -1), 1.4)
}

#Function to check point is not in obstacle
def notinObstacle(x, y):
    if(obstacle_matrix[y][x]==0):
        return True
    else:
        return False

#Function to get all Nearby neighbors and has action sets based on x and y.
def getNearByNode(cur_Node):

        nearbynodes ={}

        #get node coords
        x, y = cur_Node.x, cur_Node.y
        
        #UP neighbor
        action, cost = action_set['UP']
        MOV_X, MOV_Y = action
        if ((y < Map_Height-1) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #DOWN neighbor
        action, cost = action_set['DOWN']
        MOV_X, MOV_Y = action
        if ((y > 0) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #LEFT neighbor
        action, cost = action_set['LEFT']
        MOV_X, MOV_Y = action
        if ((x > 0) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #RIGHT neighbor
        action, cost = action_set['RIGHT']
        MOV_X, MOV_Y = action
        if ((x < Map_Width-1) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #UP-LEFT neighbor
        action, cost = action_set['UP-LEFT']
        MOV_X, MOV_Y = action
        if (((y < Map_Height-1) and (x > 0)) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #UP-RIGHT neighbor
        action, cost = action_set['UP-RIGHT']
        MOV_X, MOV_Y = action
        if (((y < Map_Height-1) and (x < Map_Width-1)) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #DOWN-LEFT neighbor
        action, cost = action_set['DOWN-LEFT']
        MOV_X, MOV_Y = action
        if (((y > 0) and (x > 0)) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        #DOWN-RIGHT neighbor
        action, cost = action_set['DOWN-RIGHT']
        MOV_X, MOV_Y = action
        if (((y > 0) and (x < Map_Width-1)) and (notinObstacle(x + MOV_X, y + MOV_Y))):
            nearbynodes[Node(x + MOV_X, y + MOV_Y)] = cost

        return nearbynodes

#function for backtracking
def backTracking(node, shortest_path):
    print("\nBacktracking path is:\n")
    while node.previousNode != None:
        print(node.x, node.y, "\n")
        shortest_path.append((node.x, node.y))
        node = node.previousNode
    print(node.x, node.y, "\n")
    shortest_path.append((node.x, node.y))
    return shortest_path

#dijkstra algorithm using heapq
def dijkstraAlgo(start_x, start_y, end_x, end_y):
    global open_list
    global closed_list
    global shortest_path

    while len(open_list):
        #pop lowest dist from list
        cur_dist, cur_Node = heapq.heappop(open_list)
   
        #Checking current node is goal node
        if ((cur_Node.x == end_x) and (cur_Node.y == end_y)):
            print("Goal Node reached.....")
            shortest_path = backTracking(cur_Node, shortest_path)
            return True

        #check if the node is in closed list
        if (cur_Node.x, cur_Node.y) in closed_list:
            continue

        #if not append it to cosed list
        closed_list[(cur_Node.x, cur_Node.y)] = True

        #Get Neighbors of currentnode
        neighbors = getNearByNode(cur_Node)
        cur_Node.neighbors = neighbors

        for neighbor_Node, cost in neighbors.items():
            #set previous node
            neighbor_Node.previousNode = cur_Node
            #change distance
            neighbor_Node.shortest_distance = cur_dist + cost
            #add to open_list
            heapq.heappush(open_list, (neighbor_Node.shortest_distance, neighbor_Node))
    
    print("Goal not reached.....")
    return False

#Check if start  and End are in obstacle
crt_start = notinObstacle(start_x, start_y)
crt_end   = notinObstacle(end_x, end_y)

if(crt_start and crt_start):
    print("\nStart_x:",start_x," Start_y:",start_y)
    print("\nEnd_x:",end_x," End_y:",end_y)

if(crt_start==False):
    print("\n Start point is in obstacle")

if(crt_end ==False):
    print("\n End point is in obstacle")

if(crt_start==False or crt_end==False):
    exit(0)

#Defining two lists openlist and closed list
open_list = []
closed_list = {}
#define final path list
shortest_path = []

#Push start node to openlist
start_node = Node(start_x, start_y)
start_node.shortest_distance=0

heapq.heappush(open_list, (start_node.shortest_distance, start_node))


#Dijkstra algorithm .............
start = time.time()
status = dijkstraAlgo(start_x, start_y, end_x, end_y)
end = time.time()
print("Time taken:", (end-start))

