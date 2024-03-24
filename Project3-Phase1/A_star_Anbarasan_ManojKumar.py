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

########################################define step size as 10 (L)#########################################
step_size = 10.0
###########################################################################################################

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
    #lt
    def __lt__(self, other):
        return self.total_cost < other.total_cost
    #hash
    def __hash__(self):
        return hash((self.x, self.y, self.theta))
    
#Calculating ax + by + c =0
def calc_line_eq(pt1, pt2):

    #Get points 1 and 2
    x1, y1 = pt1
    x2, y2 = pt2
 
    #Calculating coefficients
    a = y2 - y1
    b = x1 - x2
    c = -(a * x1 + b * y1)
    
    return a, b, c

#Calculating line eqns
def calc_hex_line(vertices, clearance):

    side_equations = []
    side_equations_shift = []

    #iterating 6 lines
    for i in range(len(vertices)):
        p1 = vertices[i]
        p2 = vertices[(i + 1) % len(vertices)]
        side_equations.append(calc_line_eq(p1, p2))
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
    mag = np.sqrt(a**2 + b**2)
    c = c + clearance * mag

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

    #Checking boundary conditions
    if(x >= Canvas_Width or y>= Canvas_Height):
        return True

    global obs_matrix

    #Check for obstacle
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

#Enter start coords
Strt_x =     int(input("Enter Start x coordinates:"))
Strt_y =     int(input("Enter Start y coordinates:"))
Strt_theta = int(input("Enter Start degrees:"))

#Enter goal coords
End_x =     int(input("Enter End x coordinates:"))
End_y =     int(input("Enter End y coordinates:"))
End_theta = int(input("Enter End degrees:"))

# Defining the size of the map
Canvas_Width = 1200 
Canvas_Height = 500 

#Creating an empty matrix for map
obs_matrix = np.zeros((Canvas_Height, Canvas_Width))

#Enter radius of robot
radius = int(input("Enter radius of robot:"))

#clearance of robot
clearance_robot = radius

#Enter clearance of robot
clearance = int(input("Enter clearance of robot:"))

#clearance map
clearance_map = clearance

#Creating map

# Define hexagon points
hex_vertices = [(650, 100), (775, 175), (775, 325), (650, 400), (525, 325), (525, 175)]

# Calculate line equations for the sides of the hexagon
hex_eqn, hex_eqn_c = calc_hex_line(hex_vertices, (clearance_robot+clearance_map))

#CAlling a function which creates the canvas
obs_matrix = createMap(obs_matrix , Canvas_Width, Canvas_Height, clearance_robot, clearance_map, hex_eqn, hex_eqn_c)

#Invert to make it same as --- cartesian coords
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
    print("End X  :",End_x," End Y  :",End_y)

#Action sets are defined here
def action_func(step_size, current_state):
    action_dict = {1:((step_size*np.cos(np.radians(current_state[0][2])),step_size*np.sin(np.radians(current_state[0][2])),0), step_size),
                   2:((step_size*np.cos(np.radians(current_state[0][2])+np.pi/6),step_size*np.sin(np.radians(current_state[0][2])+np.pi/6),30), step_size),
                   3:((step_size*np.cos(np.radians(current_state[0][2])+np.pi/3),step_size*np.sin(np.radians(current_state[0][2])+np.pi/3),60), step_size),
                   4:((step_size*np.cos(np.radians(current_state[0][2])-np.pi/6),step_size*np.sin(np.radians(current_state[0][2])-np.pi/6),-30), step_size),
                   5:((step_size*np.cos(np.radians(current_state[0][2])-np.pi/3),step_size*np.sin(np.radians(current_state[0][2])-np.pi/3),-60), step_size)}
    return action_dict

#create child for 5 action sets
def child(current_state, current_action, lst, goal):
    current_node = (current_state[0][0]+current_action[0][0], current_state[0][1]+current_action[0][1], normalize_angle(current_state[0][2]+current_action[0][2]))
    cost2_go = ( (goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5
    
    cost = (current_action[1] + current_state[1], cost2_go)
    
    child_state = (current_node,cost)

    bool_obstacle = in_obstacle(int(current_node[0]), int(current_node[1]))    #inobstacle call

    if bool_obstacle == False:
        lst.append(child_state)

    return lst

#Appending actions
def action(current_state, goal, step_size):
    action_set = action_func(step_size, current_state)
    lst = []
    current_action = action_set[1]
    lst = child(current_state, current_action, lst, goal)
    current_action = action_set[2]
    lst = child(current_state, current_action, lst, goal)
    current_action = action_set[3]
    lst = child(current_state, current_action, lst, goal)
    current_action = action_set[4]
    lst = child(current_state, current_action, lst, goal)
    current_action = action_set[5]
    lst = child(current_state, current_action, lst, goal)
    return lst

#check goal is reached
def goal_thershold(current_state, goal):
    current_node = (current_state[0][0], current_state[0][1], current_state[0][2])
    goal_dist = ((goal[0]-current_node[0])**2 + (goal[1]-current_node[1])**2 )**0.5

    if (goal_dist <= 1.5) and (normalize_angle(current_node[2]) == normalize_angle(goal[2])):
        return True
    else:
        return False 

#visited matrix 3d
visitedMatrix = np.zeros((Canvas_Width*2, Canvas_Height*2, 12))

#Finding duplicate nodes
def find_duplicatenodes(x, y, theta):

    global visitedMatrix

    #Theshold 
    threshold = 0.5

    #Rounding off x and y
    new_X = int(roundOff(x) / threshold)
    new_Y = int(roundOff(y) / threshold)

    #Normalize the theta
    new_theta = normalize_angle(theta) // 30

    if(visitedMatrix[new_X][new_Y][new_theta] == 0):
        visitedMatrix[new_X][new_Y][new_theta] = 1
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
        path.append((Node.x, Node.y,Node.theta))
        Node = Node.parentNode
    path.append((Node.x, Node.y, Node.theta))

    return path

#A* Algorithm code
def A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y, End_theta):

    global final_path

    #Goal 
    goal = (End_x, End_y, End_theta)

    #creating open list
    open_list = []

    #Heuristic cost
    no_h_cost = {(Strt_x, Strt_y, Strt_theta):( (goal[0]-Strt_x)**2 + (goal[1]-Strt_y)**2 )**0.5}
    closed_list = {}

    all_nodes = []

    #Start Node
    start_node = Node(Strt_x, Strt_y, Strt_theta)

    start_node.total_cost =  0  
    start_node.h_cost =  ( (goal[0]-Strt_x)**2 + (goal[1]-Strt_y)**2 )**0.5

    #Push to open_list defined
    heapq.heappush(open_list, (start_node.total_cost, start_node))

    global step_size

    global action_set

    #Iterating till OPen list is empty
    while len(open_list):

            #Pop from open list
            cur_cost, cur_Node = heapq.heappop(open_list)
            current_Node = ((cur_Node.x, cur_Node.y, cur_Node.theta), no_h_cost[roundOff(cur_Node.x), roundOff(cur_Node.y), cur_Node.theta])
            
            #Checking for goal node
            if(goal_thershold(current_Node, goal) == True):
                print(".............GOAL NODE Reached............")
                
                final_path = back_tracking(cur_Node)
                return True, all_nodes

            closed_list[(roundOff(current_Node[0][0]), roundOff(current_Node[0][1]))] = True

            #creating child
            child = action(current_Node, goal, step_size)

            par_child = [(current_Node[0][0], current_Node[0][1])]

            #iterating child
            for tmp_node, child_cost in child:

                child_node = (float("{:.1f}".format(tmp_node[0])), float("{:.1f}".format(tmp_node[1])), tmp_node[2])

                #check if the node is in closed list
                if (roundOff(child_node[0]), roundOff(child_node[1])) in closed_list:
                    continue

                #check in visited matrix 3d
                if find_duplicatenodes(child_node[0], child_node[1], child_node[2]):
                    Child_Node = Node(child_node[0], child_node[1] ,child_node[2])
                    Child_Node.total_cost = child_cost[0]
                    Child_Node.h_cost = child_cost[1]
                    Child_Node.parentNode = cur_Node
                    heapq.heappush(open_list, (Child_Node.total_cost+Child_Node.h_cost, Child_Node))
                    no_h_cost[(roundOff(child_node[0]), roundOff(child_node[1]), child_node[2])] = child_cost[0]
                    par_child.append((roundOff(child_node[0]), roundOff(child_node[1])))

                else:
                    #Compare costs
                    if(child_cost[0] < no_h_cost[(roundOff(current_Node[0][0]), roundOff(current_Node[0][1]), current_Node[0][2])]):
                        Child_Node = Node(child_node[0], child_node[1] ,child_node[2])
                        Child_Node.total_cost = child_cost[0]
                        Child_Node.h_cost = child_cost[1]
                        Child_Node.parentNode = cur_Node
                        heapq.heappush(open_list, (Child_Node.total_cost+Child_Node.h_cost, Child_Node))
                        no_h_cost[(roundOff(child_node[0]), roundOff(child_node[1]), child_node[2])] = child_cost[0]
            all_nodes.append(par_child)
    return False, all_nodes

#Action set
action_set = {}

#Final path list
final_path = []

start_time = time.time()

path, all_nodes = A_starAlgo(Strt_x, Strt_y, Strt_theta, End_x, End_y, End_theta)   #A* algo

end_time = time.time()

plot_path = []

# Function to convert coords for pygame
def conv_coords(x, y):
    convd_x = x
    convd_y = Canvas_Height - y 
    return convd_x, convd_y

#Check path is found or not
if(path==False):
    print("........Path not found.......")

else:
    final_path = final_path[::-1]
    print("#######Final Path is:#######")
    for x,y,theta in final_path:
        plot_path.append(conv_coords(roundOff(x),roundOff(y)))
        print(x,y,theta)

print("Time Taken for A* is :", (end_time-start_time), " seconds")

# Init Pygame
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


    out.release()
    pygame.quit()
    sys.exit()
