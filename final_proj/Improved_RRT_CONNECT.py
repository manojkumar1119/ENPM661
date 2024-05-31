import cv2
import numpy as np
import heapq
import time
import pygame
import sys
import math
import matplotlib.pyplot as plt
import random
import time

# Define the canvas size
Canvas_Width = 500 
Canvas_Height = 500

# Function to convert coords for pygame
def conv_coords(x, y):
    convd_x = x
    convd_y = Canvas_Height - y 
    return int(convd_x), int(convd_y)

class Node:
    def __init__(self, position):
        self.position = position
        self.parent = None
class RRTConnectGoalBiased:
    def __init__(self, start, goal, bounds, pbias=0.4, step_size=5.0, max_iterations=20000):
        self.start = Node(start)
        self.goal = Node(goal)
        self.bounds = bounds
        self.pbias = pbias
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.tree_start = [self.start]
        self.tree_goal = [self.goal]
        self.path = None
        self.explored_nodes_start = []
        self.explored_nodes_goal = []
    
    def distance(self, node1, node2):
        return np.linalg.norm(np.array(node1.position) - np.array(node2.position))
    
    def sample(self):
        # Probability of sampling the goal point
        if random.random() < self.pbias:
            return self.goal.position
        else:
            return [random.uniform(self.bounds[0], self.bounds[1]), random.uniform(self.bounds[2], self.bounds[3])]
    
    def nearest_node(self, tree, sample_point):
        min_dist = float('inf')
        nearest_node = None
        for node in tree:
            dist = self.distance(node, Node(sample_point))
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node
    
    def new_node(self, nearest, sample_point):
         # Calculate the direction vector and its length
        P1 = np.array(sample_point) - np.array(nearest.position)

        # Calculate the norms (lengths) of P1 and P2
        norm_P1 = np.linalg.norm(P1)

        if norm_P1 == 0:
            cos_theta = 0
        else:
        # Calculate the cosine of the angle between P1 and P2
            cos_theta = P1 / norm_P1

        # Calculate the new node position using the cosine of the angle
        new_position = np.array(nearest.position) + cos_theta * self.step_size
                
        # Create the new node
        new_node = Node(new_position)
        new_node.parent = nearest
        
        return new_node
    
    def extend(self, tree, sample_point, explored_nodes):
        nearest_node = self.nearest_node(tree, sample_point)
        new_node = self.new_node(nearest_node, sample_point)
        
        # Check if new node is within bounds and doesn't collide with obstacles
        if new_node and (self.bounds[0] <= new_node.position[0] <= self.bounds[1] and
                         self.bounds[2] <= new_node.position[1] <= self.bounds[3]):
            # Check for collisions
            if not self.is_collision(new_node.position):
                tree.append(new_node)
                explored_nodes.append(new_node)
                return new_node
        return None
    
    def is_collision(self, point):
        # Simple collision checking with obstacles
        if self.is_point_in_obstacle(point):
            return True
        return False

    # Function to check if a point is in an obstacle
    def is_point_in_obstacle(self,point,clearance=0, BOT_RADIUS=0):
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
        
        if((x >= (50 - (clearance + BOT_RADIUS))) and (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
            return True

        #else return False
        return False
    
    def build_path(self, node):
        path = []
        while node:
            path.append(node.position)
            node = node.parent
        return path[::-1]
    
    def connect(self):
        for iteration in range(self.max_iterations):
            sample_point = self.sample()
            
            # Extend start tree
            new_node_start = self.extend(self.tree_start, sample_point, self.explored_nodes_start)
            
            # Extend goal tree
            new_node_goal = self.extend(self.tree_goal, sample_point, self.explored_nodes_goal)
            
            if new_node_start and new_node_goal:
                # Calculate distance between new nodes
                distance_between_nodes = self.distance(new_node_start, new_node_goal)
                if distance_between_nodes <= (self.step_size + 2):
                    self.link_trees(new_node_start, new_node_goal)
                    return True
        return False
    
    def link_trees(self, new_node_start, new_node_goal):
        # Link the start and goal nodes and build the path
        # Adjust path to account for the link between the trees
        # Path from start to the connection point
        path_start = self.build_path(new_node_start)
        
        # Path from goal to the connection point
        path_goal = self.build_path(new_node_goal)
        
        # Reverse the goal path to join with the start path
        path_goal.reverse()
        
        # Combine both paths and set the final path
        self.path = path_start + path_goal[1:]
    
    def plan(self):
        # Plan the path by connecting the trees
        success = self.connect()
        if success:
            # Smooth the path
            self.smooth_path()
        return self.path
    
    def smooth_path(self):
        if not self.path:
            return
        
        # Use shortcutting to smooth the path
        smoothed_path = [self.path[0]]
        i = 0
        while i < len(self.path) - 1:
            j = i + 2
            while j < len(self.path):
                # Check if the direct connection is valid (no collision)
                if self.check_connection(smoothed_path[-1], self.path[j]):
                    j += 1
                else:
                    break
            # Add the last valid point
            smoothed_path.append(self.path[j - 1])
            i = j - 1
        
        # Set the smoothed path as the final path
        self.path = smoothed_path
    
    def check_connection(self, point1, point2):
        # Check if a direct line segment between two points collides with obstacles
        steps = int(np.linalg.norm(np.array(point2) - np.array(point1)) / self.step_size)
        for i in range(steps):
            t = i / steps
            interpolated_point = np.array(point1) + t * (np.array(point2) - np.array(point1))
            if self.is_collision(interpolated_point):
                return False
        return True
    
    def plot_explored_nodes_and_path(self):
        plt.figure(figsize=(10, 10))
        
        # Plot explored nodes from start tree
        explored_nodes_start = np.array(self.explored_nodes_start)
        plt.scatter(explored_nodes_start[:, 0], explored_nodes_start[:, 1], color='blue', s=10, label='Explored Nodes Start Tree')
        
        # Plot explored nodes from goal tree
        explored_nodes_goal = np.array(self.explored_nodes_goal)
        plt.scatter(explored_nodes_goal[:, 0], explored_nodes_goal[:, 1], color='red', s=10, label='Explored Nodes Goal Tree')
        
        # Plot the final path
        if self.path:
            path_array = np.array(self.path)
            plt.plot(path_array[:, 0], path_array[:, 1], color='green', label='Final Path')
        
        # Plot start and goal points
        #plt.scatter(self.start.position[0], self.start.position[1], color='yellow', marker='*', s=100, label='Start')
        #plt.scatter(self.goal.position[0], self.goal.position[1], color='magenta', marker='*', s=100, label='Goal')
        
        
        plt.title('RRT-Connect Goal-Biased Algorithm')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.show()
        #plt.savefig("out.png")

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
def createMap(Canvas_Width, Canvas_Height):
    clearance = 0
    BOT_RADIUS = 0
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

            if((x >= (50 - (clearance + BOT_RADIUS))) and (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
                obs_matrix[y][x] = -2
    return obs_matrix

def alternate_combine(list1, list2):
    # Create an empty list to store the result
    result = []
    
    # Determine the lengths of both lists
    len1 = len(list1)
    len2 = len(list2)
    
    # Find the length of the shorter list
    min_len = min(len1, len2)
    
    # Take alternate elements from both lists up to the length of the shorter list
    for i in range(min_len):
        result.append(list1[i])
        result.append(list2[i])
    
    # Add remaining elements from the longer list, if any
    if len1 > min_len:
        result.extend(list1[min_len:])
    elif len2 > min_len:
        result.extend(list2[min_len:])
    
    return result

# Function to check if a point is in an obstacle
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
    
    if((x >= (50 - (clearance + BOT_RADIUS))) and (x <= (300 - (clearance + BOT_RADIUS))) and (y >= (330 - (clearance + BOT_RADIUS))) and (y <= (420 - (clearance + BOT_RADIUS)))):
        return True

    #else return False
    return False

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
    bounds = [0, Canvas_Width, 0, Canvas_Height]

    
    
    total_time = 0 
    total_distance = 0
    for i in range(0,20):
        start_time = time.time()
        rrt = RRTConnectGoalBiased((Strt_x, Strt_y), (End_x, End_y), bounds)
        path = rrt.plan()
        distance=0
        for i in range(len(path) - 1):
            point1 = path[i]
            point2 = path[i + 1]
            distance +=  math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
        total_distance += distance
        end_time = time.time() - start_time
        total_time += end_time
        
        print(f"IMPROVED RRT_CONNECT time: {end_time:.6f} seconds")
        print(f"Improved RRT Connect distance:: {distance:.6f} mm")
    print("Total_time:",total_time/20.0)
    print("Total_distance:",total_distance/20.0)


    #rrt.plot_explored_nodes_and_path()
    print(path)

    # Visualize the path
    if path:
        path_x = [node[0] for node in path]
        path_y = [node[1] for node in path]
        for i in range(0, len(path_x)):
            print(path_x[i], path_y[i])
        #visualize_canvas(obs_matrix, Strt_x, Strt_y, End_x, End_y)
        #plt.plot(path_x, path_y, '-r')
        #plt.show()
    else:
        print("Path not found!")
        exit(0)

    # # Init Pygame
    pygame.init()
    
    screen = pygame.display.set_mode((Canvas_Width, Canvas_Height))
    pygame.display.set_caption("Improved RRT Connect Algorithm")
    
    BLACK  = (0, 0, 0)            #---> Clearance
    WHITE  = (255, 255, 255)      #--->Screen
    RED    = (255, 0, 0)          #--->Obstacle
    ORANGE = (255, 165, 0)        #--->Start
    GREEN  = (0, 255, 0)          #--->Goal
    BLUE   = (0, 0, 255)          #--->Explored
    YELLOW = (255, 255, 0)        #--->Final Path
    NEON_GREEN = (252, 118, 106)
    GREEN = (0,255,0)
    
    obs_matrix = obs_matrix[::-1]
    
    running = True
    
    #Saving video
    video_name = 'Improved_RRT-CONNECT.mp4'
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
   
        all_nodes = alternate_combine(rrt.explored_nodes_start, rrt.explored_nodes_goal)
        # Iterate over each explored nodes
        index = 0
        for explore_node in all_nodes:
            if(explore_node.parent == None):
                continue
            pygame.draw.line(screen, NEON_GREEN,(conv_coords(explore_node.position[0],explore_node.position[1])) , (conv_coords(explore_node.parent.position[0],explore_node.parent.position[1])) , 2)
            pygame.draw.circle(screen, ORANGE, (conv_coords(Strt_x, Strt_y)), 5)
            pygame.draw.circle(screen, GREEN, (conv_coords(End_x, End_y)), 5)

            pygame_img = pygame.surfarray.array3d(screen)
            final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
            out.write(final_Img)
            pygame.display.flip()
            index+=1

        #Iterating final path
        elem = path[0]
        path = path[1:]
        for val in path:
            pygame.draw.line(screen, GREEN, (conv_coords(elem[0],elem[1])) , (conv_coords(val[0],val[1])), 2)
    
            pygame_img = pygame.surfarray.array3d(screen)
            time.sleep(1)
            final_Img = cv2.cvtColor(pygame_img.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
            out.write(final_Img)
            pygame.display.flip()
            elem = val
        out.release()
        pygame.quit()
        sys.exit()

