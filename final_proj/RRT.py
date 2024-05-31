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
# def in_obstacle(x, y,clearance=0, BOT_RADIUS=0):
#     #checking inside map
#     if( (x <= (clearance + BOT_RADIUS)) or (x >= (Canvas_Width - clearance - BOT_RADIUS)) or (y <= (clearance + BOT_RADIUS)) or (y >= (Canvas_Height - clearance - BOT_RADIUS)) ):
#         return True
#     #checking 1st rectangle
#     if( (x >= (150 - (clearance + BOT_RADIUS))) and (x <= (175 + (clearance + BOT_RADIUS))) and (y >= (100 - (clearance + BOT_RADIUS)))):
#         return True
#     #checking 2nd rectangle
#     if( (x >= (250 - (clearance + BOT_RADIUS))) and (x <= (275 + (clearance + BOT_RADIUS))) and (y <= (100 + (clearance + BOT_RADIUS)))):
#         return True
#     #checking circle
#     dist_sqrd = (x - 420) ** 2 + (y - 120) ** 2
#     return dist_sqrd <= (60+clearance+BOT_RADIUS) ** 2

def in_obstacle(x, y,clearance=0, BOT_RADIUS=0):
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
    
    if( (x <= (240 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
        return True

    #else return False
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

            if( (x <= (240 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
    return obs_matrix

def back_tracking(Node):

    path =[]
    while Node.parent != None:
        path.append((Node.x, Node.y))
        Node = Node.parent
    path.append((Node.x, Node.y))

    return path

def rrt(start, goal, obs_matrix, step_size, iterations):
    #nodes tree
    tree = [start]
    #run till iterations
    for _ in range(iterations):
        #getting random pts
        random_pt = Node(np.random.randint(Canvas_Width), np.random.randint(Canvas_Height))
        #Getting nearest node based on distance
        nearest_node = min(tree, key=lambda node: math.sqrt((random_pt.x - node.x) ** 2 + (random_pt.y - node.y) ** 2))
        distance = math.sqrt((random_pt.x - nearest_node.x) ** 2 + (random_pt.y - nearest_node.y) ** 2)

        theta = math.atan2(random_pt.y - nearest_node.y, random_pt.x - nearest_node.x)

        if distance > step_size:
            new_x = nearest_node.x + step_size * math.cos(theta)
            new_y = nearest_node.y + step_size * math.sin(theta)
        else:
            new_x = random_pt.x
            new_y = random_pt.y
        
        # Create the new point
        new_point = Node(int(new_x), int(new_y))
        
        # Check if the new point is in obstacle space or goes out of bounds
        if in_obstacle(new_point.x, new_point.y):
            continue
        
        # Check for collision along the line between nearest node and new point
        new_point.parent = nearest_node
        tree.append(new_point)
        
        # Check if the new point is close enough to the goal point
        if math.sqrt((new_point.x - goal.x) ** 2 + (new_point.y - goal.y) ** 2) < 5:
            goal.parent = new_point
            tree.append(goal)
            final_path = back_tracking(new_point)
            final_path = final_path[::-1]
            return final_path,tree

    return None,None

# Main code
if __name__ == "__main__":
    #Enter start Node
    #Strt_x = int(input("Enter Start x coordinates: "))
    Strt_x = 15
    #Strt_y = int(input("Enter Start y coordinates: "))
    Strt_y = 15

    #Enter goal Node
    #End_x = int(input("Enter End x coordinates: "))
    End_x = 485
    #End_y = int(input("Enter End y coordinates: "))
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
    ITERATIONS = 10000

    #calling RRT algo
    start_time = time.time()
    final_path,path = rrt(start_node, goal_node, obs_matrix, STEP_SIZE, ITERATIONS)
    end_time = time.time() - start_time
    print(f"RRT : {end_time:.6f} seconds")

    # Visualize the path
    if path:
        path_x = [node.x for node in path]
        path_y = [node.y for node in path]
        #for i in range(0, len(path_x)):
            #print(path_x[i], path_y[i])
        #visualize_canvas(obs_matrix, Strt_x, Strt_y, End_x, End_y)
        #plt.plot(path_x, path_y, '-r')
        plt.show()
    else:
        print("Path not found!")
        exit(0)

# # Init Pygame
pygame.init()

screen = pygame.display.set_mode((Canvas_Width, Canvas_Height))
pygame.display.set_caption("RRT Algorithm")

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
video_name = 'RRT.mp4'
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
                color = RED

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
        pygame.draw.line(screen, BLUE,(conv_coords(explore_node.x,explore_node.y)) , (conv_coords(explore_node.parent.x,explore_node.parent.y)) , 2)
        pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
        pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

        pygame_img = pygame.surfarray.array3d(screen)
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()
        index+=1

    #Iterating final path
    for i in range(len(final_path) - 1):
        pygame.draw.line(screen, YELLOW, (conv_coords(final_path[i][0],final_path[i][1])) , (conv_coords(final_path[i+1][0],final_path[i+1][1])), 2)

        pygame_img = pygame.surfarray.array3d(screen)
        time.sleep(0.005)
        final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
        out.write(final_Img)
        pygame.display.flip()

    out.release()
    pygame.quit()
    sys.exit()
