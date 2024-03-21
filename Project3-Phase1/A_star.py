#Libs used

import cv2
import numpy as np
import heapq
import time
import matplotlib.pyplot as plt

#Class Node
class Node:

    #constructor init
    def __init__(self, x ,y, theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.parentNode = None
        self.total_cost = float('inf')

    #lt
    def __lt__(self, other):
        return self.total_cost < other.total_cost

    #hash
    def __hash__(self):
        return hash((self.x, self.y))
    
#Calculating ax + by + c =0
def calculate_line_equation(pt1, pt2):

    x1, y1 = pt1
    x2, y2 = pt2

    a = y2 - y1
    b = x1 - x2

    c = -(a * x1 + b * y1)
    
    return a, b, c

#Calculating line eqns
def calculate_hex_line(vertices, clearance):

    side_equations = []
    side_equations_shift = []

    #iterating 6 lines
    for i in range(len(vertices)):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % len(vertices)]
        side_equations.append(calculate_line_equation(p1, p2))
        side_equations_shift.append(shift_line_equation(side_equations[i],clearance))

    return side_equations, side_equations_shift

#check if point is inside hexagon
def is_inside_hexagon(point, side_equations):
    x, y = point
    for a, b, c in side_equations:
        if a * x + b * y + c > 0:
            return False
    return True

#shfting polygon 
def shift_line_equation(eq, clearance):

    a, b, c = eq

    magnitude = np.sqrt(a**2 + b**2)

    c = c + clearance * magnitude

    return a, b, c

#function to normalize -ve angles
def normalize_angle(theta):
    #-ve angles
    while theta < 0:
        theta += 360

    #1 rotation angles
    while theta >= 360:
        theta -= 360

    return theta

#compare both angles
def check_angles(theta1, theta2):
    return normalize_angle(theta1) == normalize_angle(theta2)

#Function to check pt is in obs
def in_obstacle(x, y):
    global obs_matrix
    if(obs_matrix[y][x]!=0):
        return True
    else:
        return False

#Matrix values
# 0--> Free space
#-1--> Clearance 5 mm + robot clearance 5mm
#-2--> Obstacle

#Functon for creating a canvas
def createMap(obs_matrix, Canvas_Width, Canvas_Height, clearance_robot, clearance_map, side_equations, side_equations_shift):

    #Iterate image
    for i in range(0, Canvas_Height):
        for j in range(0, Canvas_Width):

            #increase map inwards
            if((i<(clearance_robot + clearance_map)) or (i>=(Canvas_Height - (clearance_robot + clearance_map))) or (j<(clearance_robot + clearance_map)) or (j>=(Canvas_Width - clearance_robot - clearance_map))):
                obs_matrix[i][j] = -1

            #defining rectangle 1 using half plane and semi-algebraic models
            #with clearance
            if( (i>=0) and (i< (400+ (clearance_robot + clearance_map))) and (j>= (100-(clearance_robot + clearance_map))) and (j < (175 + (clearance_robot + clearance_map)))):
                obs_matrix[i][j]= -1
            #without clearance
            if( (i>= 0) and (i<400) and (j>=100) and (j<175)):
                obs_matrix[i][j]= -2

            #defining rectangle 2 using half plane and semi-algebraic models
            #with clearance
            if( (i>=100 - (clearance_robot + clearance_map)) and (i< 500) and (j>= (275-(clearance_robot + clearance_map))) and (j < (350 + (clearance_robot + clearance_map)))):
                obs_matrix[i][j]= -1
            #without clearance
            if( (i>= 100) and (i<500) and (j>=275) and (j<350)):
                obs_matrix[i][j]= -2

           #defining c type rectangle
           #with clearance
            if( (i>=50 - (clearance_robot + clearance_map)) and (i< 125 + (clearance_robot + clearance_map)) and (j>= (900-(clearance_robot + clearance_map))) and (j < (1100 +(clearance_robot + clearance_map)))):
                obs_matrix[i][j]= -1
            #without clearance
            if( (i>= 50) and (i<125) and (j>=900) and (j<1100)):
                obs_matrix[i][j]= -2

            #with clearance
            if( (i>=375 - (clearance_robot + clearance_map)) and (i< 450 + (clearance_robot + clearance_map)) and (j>= (900-(clearance_robot + clearance_map))) and (j < (1100 +(clearance_robot + clearance_map)))):
                obs_matrix[i][j]= -1
            #without clearance
            if( (i>= 375) and (i<450) and (j>=900) and (j<1100)):
                obs_matrix[i][j]= -2

            #with clearance
            if( (i>=125) and (i< 375) and (j>= (1020-(clearance_robot + clearance_map))) and (j < (1100 + (clearance_robot + clearance_map)))):
                obs_matrix[i][j]= -1
            #without clearance
            if( (i>= 125) and (i<375) and (j>=1020) and (j<1100)):
                obs_matrix[i][j]= -2

            #defining hexagon
            #with clearance
            if is_inside_hexagon((j, i), side_equations):
                obs_matrix[i, j] = -1     
            #without clearance
            if is_inside_hexagon((j, i), side_equations_shift):
                obs_matrix[i, j] = -2

    return obs_matrix

#visualize the map 
def visualize_canvas(matrix):

    matrix_np = np.array(matrix)

    # Invert the array along the y-axis
    matrix_np = np.flipud(matrix_np)

    # Create a color map where -1 is black and 0 is white -2 is red
    cmap = plt.cm.colors.ListedColormap(['red', 'black', 'white'])

    # Create a matrix plot
    plt.matshow(matrix_np, cmap=cmap)

    plt.gca().invert_yaxis()
    
    # Show the plot
    plt.show()

#Enter start coords
#Strt_x = int(input("Enter Start x coordinates:"))
Strt_x = 15
#Strt_y = int(input("Enter Start y coordinates:"))
Strt_y = 15
#Strt_theta = int(input("Enter Start degrees:"))
Strt_theta = 90

#Enter goal coords
#End_x = int(input("Enter End x coordinates:"))
End_x = 15
#End_y = int(input("Enter End y coordinates:"))
End_y = 20
#End_theta = int(input("Enter End degrees:"))
End_theta = 90


# Defining the size of the map
Canvas_Width = 1200 
Canvas_Height = 500 

#Creating an empty matrix defiining canvas
obs_matrix = np.zeros((Canvas_Height, Canvas_Width))

#Enter radius of robot
#radius = int(input("Enter radius of robot:"))
radius = 5

#clearance of robot
clearance_robot = radius

#Enter clearance of robot
#clearance = int(input("Enter clearance of robot:"))
clearance =5 

#clearance map
clearance_map = clearance

#Creating map

# Define the vertices of the hexagon
hexagon_vertices = [(650, 100), (775, 175), (775, 325), (650, 400), (525, 325), (525, 175)]

# Calculate line equations for the sides of the hexagon
hex_eqn, hex_eqn_c = calculate_hex_line(hexagon_vertices, (clearance_robot+clearance_map))

#CAlling a function whic creates the map
obs_matrix = createMap(obs_matrix , Canvas_Width, Canvas_Height, clearance_robot, clearance_map, hex_eqn, hex_eqn_c)

#visualize_canvas(obs_matrix)

#Invert to make it same as cartesian coords
obs_matrix = obs_matrix[::-1]

#Check if strt and goal in obs
if(in_obstacle(Strt_x, Strt_y) and in_obstacle(End_x, End_y)):
    print("Both Start and End pts are in obstacle")
    exit(0)

elif(in_obstacle(Strt_x, Strt_y)):
    print("Start pt in obstacle")
    exit(0)

elif(in_obstacle(End_x, End_y)):
    print("End pt in obstacle")
    exit(0)

else:
    print("\nStart X:",Strt_x," Start Y:",Strt_y)
    print("\nEnd X:",End_x," End Y:",End_y)

#action set
def action_func(step_size):
    action_dict = {1:((step_size,0,0),step_size),
               2:((step_size*np.cos(np.pi/6),step_size*np.sin(np.pi/6),30), step_size),
               3:((step_size*np.cos(np.pi/3),step_size*np.sin(np.pi/3),60), step_size),
               4:((step_size*np.cos(-np.pi/6),step_size*np.sin(-np.pi/6),-30), step_size),
               5:((step_size*np.cos(-np.pi/3),step_size*np.sin(-np.pi/3),-60), step_size)}
    return action_dict


def child(current_state, current_action, list, goal):
    current_node = (current_state[0][0]+current_action[0][0], current_state[0][1]+current_action[0][1], current_state[0][2]+current_action[0][2])
    cost2_go = ((goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5
    # print(dist)
    cost = current_action[1]+cost2_go
    # print(cost)
    child_state = ((current_node),cost)
    bool_obstacle = in_obstacle(int(current_node[0]), int(current_node[1]))    #inobstacle call
    if bool_obstacle == False:
        list.append(child_state)
    return list

def action(current_state, goal):
    global action_set
    list = []
    current_action = action_set[1]
    list = child(current_state, current_action, list, goal)
    current_action = action_set[2]
    list = child(current_state, current_action, list, goal)
    current_action = action_set[3]
    list = child(current_state, current_action, list, goal)
    current_action = action_set[4]
    list = child(current_state, current_action, list, goal)
    current_action = action_set[5]
    list = child(current_state, current_action, list, goal)
    return list

def goal_thershold(current_state, goal):
    current_node = (current_state[0][0], current_state[0][1], current_state[0][2])
    goal_dist = ((goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5
    if goal_dist <= 1.5 and normalize_angle(current_node[2]) == normalize_angle(goal[2]):
        return True
    else:
        return False 

#action_dict = action_dict(step_size)
#l = action(c_s, g)
#print(l)

visitedMatrix = np.zeros((Canvas_Height*2, Canvas_Width*2, 12))

#Finding duplicate nodes
def find_duplicatenodes(x, y, theta):

    global visitedMatrix

    #Theshold 
    threshold = 0.5

    #Rounding off
    new_X = int(roundOff(x) / threshold)
    new_Y = int(roundOff(y) / threshold)

    new_theta = normalize_angle(theta) // 30

    #print(new_X, new_Y, new_theta)

    if(visitedMatrix[new_X][new_Y][new_theta] == 0):
        visitedMatrix[new_X][new_Y][new_theta] = 1
        return False
    else:
        return True

def roundOff(val):

    int_part = int(val)
    dec_part = val - int_part

    if dec_part < 0.25:
        return int_part
    
    elif dec_part < 0.75:
        return int_part + 0.5

    else:
        return int_part + 1

def A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y, End_theta):

    goal = (End_x, End_y, End_theta)

    #creating open list
    open_list = []
    open_list_dict = {}

    closed_list = {}

    #Push strt node to open _ lst
    start_node = Node(Strt_x, Strt_y, Strt_theta)

    start_node.total_cost= ((goal[0]-Strt_x)**2 + (goal[1]-Strt_y)**2 )**0.5

    heapq.heappush(open_list, (start_node.total_cost, start_node))

    open_list_dict[(start_node.x, start_node.y , start_node.theta)] = start_node.total_cost
 
    #define step size as 10 (L)
    step_size = 10.0

    global action_set

    action_set = action_func(step_size)

    while(len(open_list)):

        #Pop from open list
        cur_cost, cur_Node = heapq.heappop(open_list)

        current_Node = ((cur_Node.x, cur_Node.y, cur_Node.theta), cur_cost)

        #Checking for goal node
        if(goal_thershold(current_Node, goal) == True):
            print("Goal Node reached.....")
            #shortest_path = backTracking(cur_Node, shortest_path)
            return True

        #check if the node is in closed list
        if (cur_Node.x, cur_Node.y , cur_Node.theta) in closed_list:
            continue

        #Append to closed list
        closed_list[(cur_Node.x, cur_Node.y , cur_Node.theta)] = True

        #creating child
        child = action(current_Node, goal)

        #Iterating all child
        for tmp_node, child_cost in child:
            print("In child\n")
            child_node = (float("{:.1f}".format(tmp_node[0])), float("{:.1f}".format(tmp_node[1])), tmp_node[2])
            #check in visited matrix
            if find_duplicatenodes(child_node[0], child_node[1], child_node[2]):
                print("In visited matrix")
                #check if child is present in open_list
                if (child_node[0], child_node[1], child_node[2]) in open_list_dict:
                    print("In open list")

                    #check for cost
                    if child_cost < open_list_dict[(child_node[0], child_node[1], child_node[2])] :

                        open_list_dict[(child_node[0], child_node[1], child_node[2])]  = child_cost

                else:
                    print("Addd to open")

                    open_list_dict[(child_node[0], child_node[1] , child_node[2])] = child_cost
                
                
            Child_Node = Node(child_node[0], child_node[1] ,child_node[2])
            Child_Node.total_cost = child_cost
            Child_Node.parentNode = cur_Node

            heapq.heappush(open_list, (Child_Node.total_cost, Child_Node))

action_set = {}
A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y, End_theta)
            







        








