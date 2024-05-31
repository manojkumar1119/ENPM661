#import the libraries
import cv2
import numpy as np
import heapq
import time
import pygame
import sys
import math
import matplotlib.pyplot as plt
import time

# Define the canvas size
Canvas_Width = 500 
Canvas_Height = 500

# Function to convert coords for pygame
def conv_coords(x, y):
    convd_x = x
    convd_y = Canvas_Height - y 
    return int(convd_x), int(convd_y)

#class NOde
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# Function to check if a point is in an obstacle
def in_obstacle(x, y,clearance=0, BOT_RADIUS=0):
    #checking inside map
    if( (x <= (clearance + BOT_RADIUS)) or (x >= (Canvas_Width - clearance - BOT_RADIUS)) or (y <= (clearance + BOT_RADIUS)) or (y >= (Canvas_Height - clearance - BOT_RADIUS)) ):
        return True
    if( (x <= (125 - (clearance + BOT_RADIUS))) and (y >= (20 - (clearance + BOT_RADIUS))) and (y <= (80 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (70 - (clearance + BOT_RADIUS))) and (x <= (240 + (clearance + BOT_RADIUS))) and (y >= (110 - (clearance + BOT_RADIUS)))and (y <= (165 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (165 + (clearance + BOT_RADIUS))) and (y >= (165 - (clearance + BOT_RADIUS)))and (y <= (220 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (250 + (clearance + BOT_RADIUS))) and (y >= (220 - (clearance + BOT_RADIUS)))and (y <= (290 - (clearance + BOT_RADIUS)))):
        return True
    
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (375 + (clearance + BOT_RADIUS))) and (y >= (80 - (clearance + BOT_RADIUS)))and (y <= (125 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (350 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (250 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (320 - (clearance + BOT_RADIUS))) and (x <= (440 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (185 - (clearance + BOT_RADIUS)))):
        return True

    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (410 + (clearance + BOT_RADIUS))) and (y >= (250 - (clearance + BOT_RADIUS)))and (y <= (325 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (340 + (clearance + BOT_RADIUS))) and (y >= (325 - (clearance + BOT_RADIUS)))and (y <= (400 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (420 + (clearance + BOT_RADIUS))) and (y >= (400 - (clearance + BOT_RADIUS)))and (y <= (450 - (clearance + BOT_RADIUS)))):
        return True
    
    if( (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
        return True

    return False

# Function to visualize the canvas with obstacles
def visualize_canvas(matrix, start_x, start_y, end_x, end_y):
    matrix_np = np.array(matrix)
    cmap = plt.cm.colors.ListedColormap(['red', 'black', 'white'])
    plt.matshow(matrix_np, cmap=cmap)
    plt.plot(start_x, start_y, 'go', label='Start') 
    plt.plot(end_x, end_y, 'bx', label='Goal') 
    plt.gca().invert_yaxis()
    plt.legend()
    plt.show()

# Function to create the obstacle matrix
def createMap(Canvas_Width, Canvas_Height, clearance=0, BOT_RADIUS=0):
    obs_matrix = np.zeros((Canvas_Height, Canvas_Width))
    for y in range(Canvas_Height):
        for x in range(Canvas_Width):
            if (x <= 0) or (x >= Canvas_Width) or (y <= 0) or (y >= Canvas_Height):
                obs_matrix[y][x] = -1
            if( (x <= (125 - (clearance + BOT_RADIUS))) and (y >= (20 - (clearance + BOT_RADIUS))) and (y <= (80 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (70 - (clearance + BOT_RADIUS))) and (x <= (240 + (clearance + BOT_RADIUS))) and (y >= (110 - (clearance + BOT_RADIUS)))and (y <= (165 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (165 + (clearance + BOT_RADIUS))) and (y >= (165 - (clearance + BOT_RADIUS)))and (y <= (220 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (250 + (clearance + BOT_RADIUS))) and (y >= (220 - (clearance + BOT_RADIUS)))and (y <= (290 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2

            if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (375 + (clearance + BOT_RADIUS))) and (y >= (80 - (clearance + BOT_RADIUS)))and (y <= (125 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (350 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (250 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (320 - (clearance + BOT_RADIUS))) and (x <= (440 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (185 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2

            if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (410 + (clearance + BOT_RADIUS))) and (y >= (250 - (clearance + BOT_RADIUS)))and (y <= (325 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (340 + (clearance + BOT_RADIUS))) and (y >= (325 - (clearance + BOT_RADIUS)))and (y <= (400 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
            if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (420 + (clearance + BOT_RADIUS))) and (y >= (400 - (clearance + BOT_RADIUS)))and (y <= (450 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2

            if( (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
    return obs_matrix

def back_tracking(Node):
    path =[]
    while Node.parent != None:
        path.append((Node.x, Node.y))
        Node = Node.parent
    path.append((Node.x, Node.y))

    return path

#combine 2 trees alternatively
def alternate_combine(list1, list2):
    result = []
    len1 = len(list1)
    len2 = len(list2)
    
    min_len = min(len1, len2)
    for i in range(min_len):
        result.append(list1[i])
        result.append(list2[i])
    if len1 > min_len:
        result.extend(list1[min_len:])
    elif len2 > min_len:
        result.extend(list2[min_len:])
    return result

#rrt-connect
def rrt_connect(start, goal, obs_matrix, step_size, iterations):
    # Initialize trees for both directions
    tree_start = [start]
    tree_goal = [goal]

    for _ in range(iterations):
        random_pt = Node(np.random.randint(Canvas_Width), np.random.randint(Canvas_Height))
        nearest_start = min(tree_start, key=lambda node: math.sqrt((random_pt.x - node.x) ** 2 + (random_pt.y - node.y) ** 2))
        distance_start = math.sqrt((random_pt.x - nearest_start.x) ** 2 + (random_pt.y - nearest_start.y) ** 2)
        theta_start = math.atan2(random_pt.y - nearest_start.y, random_pt.x - nearest_start.x)

        if distance_start > step_size:
            new_x_start = nearest_start.x + step_size * math.cos(theta_start)
            new_y_start = nearest_start.y + step_size * math.sin(theta_start)
        else:
            new_x_start = random_pt.x
            new_y_start = random_pt.y

        new_point_start = Node(int(new_x_start), int(new_y_start))

        if in_obstacle(new_point_start.x, new_point_start.y):
            continue

        new_point_start.parent = nearest_start
        tree_start.append(new_point_start)

        random_pt = Node(np.random.randint(Canvas_Width), np.random.randint(Canvas_Height))
        nearest_goal = min(tree_goal, key=lambda node: math.sqrt((random_pt.x - node.x) ** 2 + (random_pt.y - node.y) ** 2))
        distance_goal = math.sqrt((random_pt.x - nearest_goal.x) ** 2 + (random_pt.y - nearest_goal.y) ** 2)
        theta_goal = math.atan2(random_pt.y - nearest_goal.y, random_pt.x - nearest_goal.x)

        if distance_goal > step_size:
            new_x_goal = nearest_goal.x + step_size * math.cos(theta_goal)
            new_y_goal = nearest_goal.y + step_size * math.sin(theta_goal)
        else:
            new_x_goal = random_pt.x
            new_y_goal = random_pt.y

        new_point_goal = Node(int(new_x_goal), int(new_y_goal))

        if in_obstacle(new_point_goal.x, new_point_goal.y):
            continue

        new_point_goal.parent = nearest_goal
        tree_goal.append(new_point_goal)

        # Check if the two trees are connected
        for node_start in tree_start:
            for node_goal in tree_goal:
                if math.sqrt((node_start.x - node_goal.x) ** 2 + (node_start.y - node_goal.y) ** 2) <= 7.0:
                    # Path found, connect the trees
                    path_start = []
                    path_goal = []

                    current = node_start
                    while current is not None:
                        path_start.append(current)
                        current = current.parent
                    path_start.reverse()

                    current = node_goal
                    while current is not None:
                        path_goal.append(current)
                        current = current.parent

                    #path_goal.reverse()
                    all_nodes = alternate_combine(tree_start, tree_goal)
                    return (path_start + path_goal),(all_nodes)

    return None,None

# Function to check if a point is in an obstacle
def is_point_in_obstacle(point,clearance=0, BOT_RADIUS=0):
    x,y= point
    #checking inside map
    if( (x <= (clearance + BOT_RADIUS)) or (x >= (Canvas_Width - clearance - BOT_RADIUS)) or (y <= (clearance + BOT_RADIUS)) or (y >= (Canvas_Height - clearance - BOT_RADIUS)) ):
        return True
    #checking 1st rectangle
    
    if( (x <= (125 - (clearance + BOT_RADIUS))) and (y >= (20 - (clearance + BOT_RADIUS))) and (y <= (80 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (70 - (clearance + BOT_RADIUS))) and (x <= (240 + (clearance + BOT_RADIUS))) and (y >= (110 - (clearance + BOT_RADIUS)))and (y <= (165 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (165 + (clearance + BOT_RADIUS))) and (y >= (165 - (clearance + BOT_RADIUS)))and (y <= (220 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (125 - (clearance + BOT_RADIUS))) and (x <= (250 + (clearance + BOT_RADIUS))) and (y >= (220 - (clearance + BOT_RADIUS)))and (y <= (290 - (clearance + BOT_RADIUS)))):
        return True
    
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (375 + (clearance + BOT_RADIUS))) and (y >= (80 - (clearance + BOT_RADIUS)))and (y <= (125 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (350 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (250 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (320 - (clearance + BOT_RADIUS))) and (x <= (440 + (clearance + BOT_RADIUS))) and (y >= (125 - (clearance + BOT_RADIUS)))and (y <= (185 - (clearance + BOT_RADIUS)))):
        return True

    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (410 + (clearance + BOT_RADIUS))) and (y >= (250 - (clearance + BOT_RADIUS)))and (y <= (325 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (340 + (clearance + BOT_RADIUS))) and (y >= (325 - (clearance + BOT_RADIUS)))and (y <= (400 - (clearance + BOT_RADIUS)))):
        return True
    if( (x >= (300 - (clearance + BOT_RADIUS))) and (x <= (420 + (clearance + BOT_RADIUS))) and (y >= (400 - (clearance + BOT_RADIUS)))and (y <= (450 - (clearance + BOT_RADIUS)))):
        return True
    
    if( (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
        return True

    #else return False
    return False

def is_collision(point):
   # Simple collision checking with obstacles
   if is_point_in_obstacle(point):
       return True
   return False

def check_connection(point1, point2):
   point1 = (point1.x, point1.y)
   point2 = (point2.x, point2.y)
   steps = int(np.linalg.norm(np.array(point2) - np.array(point1)) / STEP_SIZE)
   for i in range(steps):
       t = i / steps
       interpolated_point = np.array(point1) + t * (np.array(point2) - np.array(point1))
       if is_collision(interpolated_point):
           return False
   return True

# Main code
if __name__ == "__main__":
    #Enter start Node
    Strt_x = 15
    Strt_y = 15

    #Enter goal Node
    End_x = 485
    End_y = 485
    # Check start and goal points in obstacle space
    if in_obstacle(Strt_x, Strt_y):
        print("Start point is in obstacle.")
        sys.exit()
    if in_obstacle(End_x, End_y):
        print("Goal point is in obstacle.")
        sys.exit()

    # Create obstacle matrix
    obs_matrix = createMap(Canvas_Width, Canvas_Height)
    #visualize_canvas(obs_matrix, Strt_x, Strt_y, End_x, End_y)

    # RRT algorithm
    start_node = Node(Strt_x, Strt_y)
    goal_node =  Node(End_x, End_y)

    STEP_SIZE = 5
    ITERATIONS = 20000

    #calling RRT algo
    total_time = 0 
    total_distance = 0
    for i in range(0,1):
        start_time = time.time()
        final_path,path = rrt_connect(start_node, goal_node, obs_matrix, STEP_SIZE, ITERATIONS)
        if final_path:
            #perform path smoothing
            # Use shortcutting to smooth the path
            smoothed_path = [final_path[0]]
            i = 0
            while i < len(final_path) - 1:
                j = i + 2
                while j < len(final_path):
                    if check_connection(smoothed_path[-1], final_path[j]):
                        j += 1
                    else:
                        break
                smoothed_path.append(final_path[j - 1])
                i = j - 1
            
            final_path = smoothed_path
        else:
            print("Path not found!")
            exit(0)
            break

        distance=0
        for i in range(len(final_path) - 1):
            point1 = final_path[i]
            point2 = final_path[i + 1]
            distance +=  math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
        total_distance += distance
        end_time = time.time() - start_time
        total_time += end_time
        print(f"RRT connect time: {end_time:.6f} seconds")
        print(f"RRT Connect distance:: {distance:.6f} mm")
    print("Total_time:",total_time)
    print("Total_distance:",total_distance)

# # Init Pygame
pygame.init()

screen = pygame.display.set_mode((Canvas_Width, Canvas_Height))
pygame.display.set_caption("RRT Connect Algorithm")

BLACK  = (0, 0, 0)            #---> Clearance
WHITE  = (255, 255, 255)      #--->Screen
RED    = (255, 0, 0)          #--->Obstacle
ORANGE = (255, 165, 0)        #--->Start
GREEN  = (0, 255, 0)          #--->Goal
BLUE   = (0, 0, 255)          #--->Explored
YELLOW = (255, 255, 0)        #--->Final Path
PURPLE = (128,0,128)
NEON_GREEN = (252, 118, 106)
GREEN = (0,255,0)

obs_matrix = obs_matrix[::-1]

running = True

#Saving video
video_name = 'RRT_Connect.mp4'
out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc(*'mp4v'), 10, (Canvas_Width, Canvas_Height))

#Frames to store for video
flag = 8000

explore_nodes = []
print("Saving video...")

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
                color = BLACK
            if color:
                pygame.draw.rect(screen, color, (y,x,1,1))

    #Display start and goal node
    pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
    pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

    # Iterate over each explored nodes
    index = 0
    for explore_node in path:
        if(explore_node.parent == None):
            continue
        pygame.draw.line(screen, NEON_GREEN,(conv_coords(explore_node.x,explore_node.y)) , (conv_coords(explore_node.parent.x,explore_node.parent.y)) , 2)
        pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
        pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

        pygame_img = pygame.surfarray.array3d(screen)
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()
        index+=1

    #Iterating final path
    for i in range(len(final_path) - 2):
        pygame.draw.line(screen, GREEN, (conv_coords(final_path[i].x,final_path[i].y)) , (conv_coords(final_path[i+1].x,final_path[i+1].y)), 2)

        pygame_img = pygame.surfarray.array3d(screen)
        time.sleep(5)
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()

    out.release()
    pygame.quit()
    sys.exit()
