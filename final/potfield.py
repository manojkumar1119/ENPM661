#importing the libraeires
import numpy as np
import matplotlib.pyplot as plt
import time

#define map
Canvas_Width = 600
Canvas_Height = 300

#attraction
ROBOT_CHARGE = 5.0
#repulsion
OBSTACLE_CHARGE= 100.0

# Define start and goal points
start = (1, 150)
goal = (599, 150)

#defineing the obstacle cirecles with radius
obstacles = [(112, 242.5, 40), (263, 90, 70), (445, 220, 37.5)]

# Define obs to check inobstacle space
def in_obs(x, y):
    if (x < 0 or x > Canvas_Width - 1 or y < 0 or y > Canvas_Height - 1):
        return True
    # checking 1st circle
    dist_sqrd = (x - 112) ** 2 + (y - 242.5) ** 2
    if dist_sqrd <= (40) ** 2:
        return True
    # checking 2nd circle
    dist_sqrd = (x - 263) ** 2 + (y - 90) ** 2
    if dist_sqrd <= (70) ** 2:
        return True
    # checking 3rd circle
    dist_sqrd = (x - 445) ** 2 + (y - 220) ** 2
    if dist_sqrd <= (37.5) ** 2:
        return True
    return False

#toatl potential field calculaton
def calc_potfield(goal):
    #calcuate potentila field map
    pot_map = np.zeros((Canvas_Width, Canvas_Height))
    #finding potential field for all points in the map
    for x in range(Canvas_Width):
        for y in range(Canvas_Height):
            #attractive force from goal
            attr_force = calc_goalforce(x, y, goal[0], goal[1])
            #repulsive from obstacle
            repul_force = calc_obsforce(x, y)
            pot_map[x][y] = attr_force + repul_force
    return pot_map

#function to get goal attractive force
def calc_goalforce(x, y, goal_x, goal_y):
    #F=0.5*k*d
    return 0.5 * ROBOT_CHARGE * np.hypot(x - goal_x, y - goal_y)

#function to get the repulsive force
def calc_obsforce(x, y):
    #check in obstacle space
    if in_obs(x, y):
        return 0.5 * OBSTACLE_CHARGE
    #else charge 0
    else:
        return 0.0

#maiin pot field calc function
def potential_field(start, goal):
    # calc potential field
    pot_map = calc_potfield(goal)
    #setting inital path
    final_path = [start]
    x, y = start
    #iterationg till goal is reached
    while (x != goal[0]) or (y != goal[1]):
        #set max value to sort
        min_pot = float("inf")
        #store new position
        new_pos = [-1,-1]
        #4 action sets
        for new_x in range(-1, 2):
            for new_y in range(-1, 2):
                #add new action sets
                dx = x + new_x
                dy = y + new_y
                #check if it within map size
                if dx >= 0 and dx < Canvas_Width and dy >= 0 and dy < Canvas_Height:
                    #new pot val
                    new_potval = pot_map[dx][dy]
                    #sort
                    if min_pot > new_potval:
                        min_pot = new_potval
                        new_pos[0] = dx
                        new_pos[1] = dy
        x = new_pos[0]
        y = new_pos[1]
        final_path.append((new_pos[0], new_pos[1]))
    print("Goal Reached....")
    #return final_path
    return final_path

#draw map
def draw_canvas():
    fig, ax = plt.subplots()
    ax.set_xlim(0, Canvas_Width)
    ax.set_ylim(0, Canvas_Height)
    # Draw circles obstacles
    for i, j, r in obstacles:
        circle = plt.Circle((i, j), r, color='black')
        ax.add_artist(circle)
    #plot start and goal
    plt.plot(start[0], start[1], "go")
    plt.plot(goal[0], goal[1], "go")

#plot final path
def plot_path(path):
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    plt.plot(path_x, path_y, "-r")
    plt.title("Potential Field")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.show()

#plot the map
draw_canvas()
#potential field call
start_time = time.time()
final_path = potential_field(start, goal)
end_time = time.time()
print("Time taken for Potential Field Algorithm:", end_time - start_time, "seconds")
print("Final path length:",len(final_path))
#plot the final path
plot_path(final_path)
