#!/usr/bin/env python3

####################################################################
######Authors : Anbarasan Kandasamy and Manoj Kumar Selvaraj########
####################################################################

#Libs used
import cv2
import numpy as np
import heapq
import time
import pygame
import sys
import math
import matplotlib.pyplot as plt

#Class Node
class Node:
    #constructor init
    def __init__(self, x ,y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.parentNode = None
        self.total_cost = 0
        self.h_cost = 0
        self.W_rpm = (0,0)
    #lt
    def __lt__(self, other):
        return self.total_cost < other.total_cost
    #hash
    def __hash__(self):
        return hash((self.x, self.y, self.theta))

#function to normalize -ve angles
def normalize_angle(theta):

    #-ve angles
    while theta < 0:
        theta += 360

    #1 rotation angles
    while theta >= 360:
        theta -= 360

    return theta

#Function to check pt is in obs
def in_obstacle(x, y):
    #checking inside map
    if( (x <= (clearance + BOT_RADIUS)) or (x >= (Canvas_Width - clearance - BOT_RADIUS)) or (y <= (clearance + BOT_RADIUS)) or (y >= (Canvas_Height - clearance - BOT_RADIUS)) ):
        return True
    #checking 1st rectangle
    if( (x >= (150 - (clearance + BOT_RADIUS))) and (x <= (175 + (clearance + BOT_RADIUS))) and (y >= (100 - (clearance + BOT_RADIUS)))):
        return True
    #checking 2nd rectangle
    if( (x >= (250 - (clearance + BOT_RADIUS))) and (x <= (275 + (clearance + BOT_RADIUS))) and (y <= (100 + (clearance + BOT_RADIUS)))):
        return True
    #checking circle
    dist_sqrd = (x - 420) ** 2 + (y - 120) ** 2
    return dist_sqrd <= (60+clearance+BOT_RADIUS) ** 2

    #else return False
    return False

def visualize_canvas(matrix, start_x, start_y, end_x, end_y):

    matrix_np = np.array(matrix)

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

#Matrix values
# 0--> Free space
#-1--> Clearance 5 mm + robot clearance 5mm
#-2--> Obstacle

#Functon for creating a canvas
def createMap(obs_matrix, Canvas_Width, Canvas_Height, clearance, BOT_RADIUS):

    #Iterate image
    for y in range(0, Canvas_Height):
        for x in range(0, Canvas_Width):
            #checking inside map
            if( (x <= (clearance + BOT_RADIUS)) or (x >= (Canvas_Width - clearance - BOT_RADIUS)) or (y <= (clearance + BOT_RADIUS)) or (y >= (Canvas_Height - clearance - BOT_RADIUS)) ):
                obs_matrix[y][x] = -1
            #checking 1st rectangle
            if( (x >= (150 - (clearance + BOT_RADIUS))) and (x <= (175 + (clearance + BOT_RADIUS))) and (y >= (100 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -1
            if( (x >= (150)) and (x <= (175)) and (y >= (100 ))):
                obs_matrix[y][x] = -2
            #checking 2nd rectangle
            if( (x >= (250 - (clearance + BOT_RADIUS))) and (x <= (275 + (clearance + BOT_RADIUS))) and (y <= (100 + (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -1
            if( (x >= (250)) and (x <= (275)) and (y <= (100))):
                obs_matrix[y][x] = -2
            #checking circle
            dist_sqrd = (x - 420) ** 2 + (y - 120) ** 2
            if(dist_sqrd <= (60+clearance+BOT_RADIUS) ** 2):
                obs_matrix[y][x] = -1
            if(dist_sqrd <= (60) ** 2):
                obs_matrix[y][x] = -2
    return obs_matrix

# Defining the size of the map
Canvas_Width = 600 
Canvas_Height = 200

#defining turtle bot parameters in cm
BOT_RADIUS = 22
BOT_WHEEL_RADIUS = 3.3
BOT_WHEEL_DISTANCE = 28.7

while(1):
    #Enter start coords
    Strt_x =     int(input("Enter Start x coordinates in mm:"))
    Strt_y =     int(input("Enter Start y coordinates in mm:"))
    Strt_theta = int(input("Enter Start degrees in deg:"))
  
    #Enter goal coords
    End_x =     int(input("Enter End x coordinates in mm:"))
    End_y =     int(input("Enter End y coordinates in mm:"))
  
    #Enter clearance of robot
    clearance = int(input("Enter clearance in mm:"))
    clearance /= 10
      
    #check start
    value = in_obstacle(Strt_x/10, Strt_y/10)
    if(value == True):
        print("Start in Obstacle....")
        continue

    #check goal
    value = in_obstacle(End_x/10, End_y/10)
    if(value == True):
        print("Goal in Obstacle....")
        continue
    break

#Enter RPM1,2
rpm_1 = int(input("Enter RPM1: "))
rpm_2 = int(input("Enter RPM2: "))

#Creating an empty matrix for map
obs_matrix = np.zeros((Canvas_Height, Canvas_Width))

#convert mm coords to cm
Strt_x /= 10 
Strt_y /= 10 
End_x /= 10 
End_y /= 10 
rpm_1 /= 60
rpm_2 /= 60

#Creating map for pygame
obs_matrix = createMap(obs_matrix , Canvas_Width, Canvas_Height, clearance, BOT_RADIUS)
#visualize_canvas(obs_matrix, Strt_x, Strt_y, End_x, End_y)

#Parameters
r = BOT_WHEEL_RADIUS   #cm
L = BOT_WHEEL_DISTANCE #cm
d_t = 0.1
time_step = 1

#Define action sets
rpm_action = [(0, rpm_1), (rpm_1, 0), (rpm_1, rpm_1),(0, rpm_2), (rpm_2, 0),
               (rpm_2, rpm_2), (rpm_1, rpm_2), (rpm_2, rpm_1)]

#Converts action sets to x,y
def action_func(current_state):
    action_dict = {}

    for i, rpm in enumerate(rpm_action):
        d_theta = current_state[0][2]
        t = 0
        cost = 0
        x = current_state[0][0]
        y = current_state[0][1]
        d_thetaR = (np.pi * d_theta) / 180

        for j in  range(int(time_step/d_t)):
            t += d_t
            Xs = x
            Ys = y
            dx = (r/2)*(2*np.pi*rpm[0]+2*np.pi*rpm[1])*math.cos((d_thetaR))*d_t
            dy = (r/2)*(2*np.pi*rpm[0]+2*np.pi*rpm[1])*math.sin((d_thetaR))*d_t
            x+=dx
            y+=dy
            d_thetaR += (r/L)*(2*np.pi*rpm[1]-2*np.pi*rpm[0])*d_t
            cost += math.sqrt(math.pow(dx, 2)+math.pow(dy, 2))
        d_theta = (180 * (d_thetaR)) / np.pi
        action_dict[i+1] = ((x, y, normalize_angle(d_theta)), cost)
    
    return action_dict

#create child for action sets
def child(current_state, current_action, lst, goal, rpm):
    current_node = (current_action[0][0], current_action[0][1], current_action[0][2])
    cost2_go = ((goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5
    
    cost = (current_action[1] + current_state[1], cost2_go)
    
    child_state = (current_node,cost,rpm)

    bool_obstacle = in_obstacle((current_node[0]), (current_node[1]))    #inobstacle call

    if bool_obstacle == False:
        lst.append(child_state)

    return lst

def action(current_state, goal):
    action_dict = action_func(current_state)
    lst = []
    for key, value in action_dict.items():
        lst = child(current_state, value, lst, goal, rpm_action[key-1])
    return lst

#check goal is reached or not
def goal_thershold(current_state, goal):
    current_node = (current_state[0][0], current_state[0][1], current_state[0][2])
    goal_dist = ((goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5

    goal_threshold = 1
    if (goal_dist <= goal_threshold):
        return True
    else:
        return False 

#visited matrix
visitedMatrix = np.zeros((Canvas_Width*2, Canvas_Height*2))

#Finding duplicate nodes
def find_duplicatenodes(x, y, theta):

    global visitedMatrix

    #Theshold 
    threshold = 0.5

    #Rounding off x and y
    new_X = int(roundOff(x) / threshold)
    new_Y = int(roundOff(y) / threshold)

    if(visitedMatrix[new_X][new_Y] == 0):
        visitedMatrix[new_X][new_Y] = 1
        return True
    else:
        return False

#roundoff for value
def roundOff(val):

    int_part = int(val)
    dec_part = val - int_part

    if dec_part < 0.25:
        return int_part
    
    elif dec_part < 0.75:
        return int_part + 0.5

    else:
        return int_part + 1

#func to perform back tracking of final Path
def back_tracking(Node):

    path =[]
    while Node.parentNode != None:
        path.append((Node.x, Node.y,Node.theta,Node.W_rpm))
        Node = Node.parentNode
    path.append((Node.x, Node.y, Node.theta,Node.W_rpm))

    return path

#A* Algorithm code
def A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y,rpm_1,rpm_2):

    global final_path

    #Goal 
    goal = (End_x, End_y)

    #creating open list
    open_list = []

    #Heuristic cost
    no_h_cost = {(Strt_x, Strt_y, Strt_theta):0}   #  shoud be zero
    closed_list = {}

    all_nodes = []

    #Start Node
    start_node = Node(Strt_x, Strt_y, Strt_theta)

    start_node.total_cost =  0  
    start_node.h_cost =  ( (goal[0]-Strt_x)**2 + (goal[1]-Strt_y)**2 )**0.5
    start_node.W_rpm = (0, 0)
    #Push to open_list defined
    heapq.heappush(open_list, (start_node.total_cost+start_node.h_cost, start_node.h_cost, start_node))    #can add heuristic as the second element

    global step_size

    global action_set

    #Iterating till OPen list is empty
    while len(open_list):

            #Pop from open list
            cur_cost, heur, cur_Node = heapq.heappop(open_list)
            current_Node = ((cur_Node.x, cur_Node.y, cur_Node.theta), no_h_cost[roundOff(cur_Node.x), roundOff(cur_Node.y), cur_Node.theta])
            #Checking for goal node
            if(goal_thershold(current_Node, goal) == True):
                print(".............GOAL NODE Reached............")
                
                final_path = back_tracking(cur_Node)
                return True, all_nodes

            closed_list[(roundOff(current_Node[0][0]), roundOff(current_Node[0][1]))] = True
            #creating child
            child = action(current_Node, goal)

            par_child = [(current_Node[0][0], current_Node[0][1])]

            for tmp_node, child_cost, rpm in child:

                child_node = (tmp_node[0], tmp_node[1], tmp_node[2])
                
                if (roundOff(child_node[0]), roundOff(child_node[1])) in closed_list:
                    continue

                #check in visited matrix 3d
                if find_duplicatenodes(child_node[0], child_node[1], child_node[2]):
                    Child_Node = Node(roundOff(child_node[0]), roundOff(child_node[1]),child_node[2])
                    Child_Node.W_rpm = (rpm[0], rpm[1])
                    Child_Node.total_cost = child_cost[0]            # gcost
                    Child_Node.h_cost = child_cost[1]                # h cost
                    Child_Node.parentNode = cur_Node
                    heapq.heappush(open_list, (Child_Node.total_cost+Child_Node.h_cost, Child_Node.h_cost, Child_Node))      # fcost = g+h  (f_cost, child)   can be(f,h,child)
                    no_h_cost[(roundOff(child_node[0]), roundOff(child_node[1]), child_node[2])] = child_cost[0]     # gcost
                    par_child.append((roundOff(child_node[0]), roundOff(child_node[1])))

                else:
                    #Compare costs
                    if(child_cost[0] < no_h_cost[(roundOff(current_Node[0][0]), roundOff(current_Node[0][1]), current_Node[0][2])]):   # comparing with gcost (but should comapare f cost)
                        Child_Node = Node(roundOff(child_node[0]), roundOff(child_node[1]),child_node[2])
                        Child_Node.W_rpm = (rpm[0], rpm[1])
                        Child_Node.total_cost = child_cost[0]
                        Child_Node.h_cost = child_cost[1]
                        Child_Node.parentNode = cur_Node
                        heapq.heappush(open_list, (Child_Node.total_cost+Child_Node.h_cost, Child_Node.h_cost, Child_Node))
                        no_h_cost[(roundOff(child_node[0]), roundOff(child_node[1]), child_node[2])] = child_cost[0]
            all_nodes.append(par_child)
    return False, all_nodes

#Action set
action_set = {}

#Final path list
final_path = []

start_time = time.time()

path, all_nodes = A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y,rpm_1,rpm_2)   #A* algo

end_time = time.time()
print("Time Taken for A* is :", (end_time-start_time), " seconds")

plot_path = []

path_list = []

# Function to convert coords for pygame
def conv_coords(x, y):
    convd_x = x
    convd_y = Canvas_Height - y 
    return int(convd_x), int(convd_y)

#RPM_LIST
rpm_list = []

#Check path is found or not
if(path==False):
    print("........Path not found.......")

else:
    final_path = final_path[::-1]
    print("############# Final Path is: #############")
    for x,y,theta,W_rpm in final_path:
        plot_path.append(conv_coords((x),(y)))
        print(x*10,y*10,theta)
        rpm_list.append((W_rpm[0] , W_rpm[1]))
#print(rpm_list)

def ros_path(path_rpm):  # path_rpm----> list of sets of rpm and theta ie.(rpml,rpmr,theta). This theta is parent node theta
    for i in path_rpm:
        #x_vel = (r/2)*(2*np.pi*i[0]+2*np.pi*i[1])
        #y_vel = (r/2)*(2*np.pi*i[0]+2*np.pi*i[1])
        lin_vel = (r/2)*(2*np.pi*i[0]+2*np.pi*i[1])
        ang_vel = (r/L)*(2*np.pi*i[1]-2*np.pi*i[0])
        vel = (lin_vel/100, ang_vel)
        path_list.append(vel)
    path_list.append((0.0,0.0))
    return path_list

path_list = ros_path(rpm_list)
#print(path_list)


# # Init Pygame
pygame.init()

screen = pygame.display.set_mode((Canvas_Width, Canvas_Height))
pygame.display.set_caption("A* Algorithm")

BLACK  = (0, 0, 0)            #---> Clearance
WHITE  = (255, 255, 255)      #--->Screen
RED    = (255, 0, 0)          #--->Obstacle
ORANGE = (255, 165, 0)        #--->Start
GREEN  = (0, 255, 0)          #--->Goal
BLUE   = (0, 0, 255)          #--->Explored
YELLOW = (255, 255, 0)        #--->Final Path

obs_matrix = obs_matrix[::-1]

running = True

#Saving video
video_name = 'A_star_Anbarasan_ManojKumar.mp4'
out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (Canvas_Width, Canvas_Height))

#Frames to store for video
flag = 8000

explore_nodes = []
print("Saving video...")
# Iterate over each list of all_nodes
for all_node in all_nodes:
    tmp = []
    for i in range(0,len(all_node)):
        tmp.append((conv_coords(all_node[i][0],all_node[i][1])))
    explore_nodes.append(tmp)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.fill(WHITE)

    #PLot map
    for x in range(Canvas_Height):
        for y in range(Canvas_Width):
            value = obs_matrix[x][y]
            color = None
            if value == 0:
                color = WHITE
            elif value == -1:
                color = BLACK
            elif value == -2:
                color = RED

            if color:
                pygame.draw.rect(screen, color, (y,x,1,1))

    #Display start and goal node
    pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
    pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

    # Iterate over each explored nodes
    index = 0
    for explore_node in explore_nodes:
        index+=1
        for i in range(1,len(explore_node)):
            pygame.draw.line(screen, BLUE,(explore_node[0][0],explore_node[0][1]) , (explore_node[i][0],explore_node[i][1]) , 2)
            pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
            pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

        if(index<=50):
            pygame_img = pygame.surfarray.array3d(screen)
            final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
            out.write(final_Img)
            pygame.display.flip()

        if(index%flag == 0):
            pygame_img = pygame.surfarray.array3d(screen)
            final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
            out.write(final_Img)
            pygame.display.flip()

   #Iterating final path
    for i in range(len(plot_path) - 1):
        pygame.draw.line(screen, YELLOW, plot_path[i], plot_path[i + 1], 2)

        pygame_img = pygame.surfarray.array3d(screen)
        time.sleep(0.005)
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()

    #Display last frame
    pygame_img = pygame.surfarray.array3d(screen)
    final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
    out.write(final_Img)
    pygame.display.flip()
    for i in range (0,40):
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()
        time.sleep(0.5)
    out.release()
    pygame.quit()
    sys.exit()
