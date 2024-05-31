#importing the libraries
import numpy as np
import matplotlib.pyplot as plt
import heapq
import time

# Define constants or map
Canvas_Width = 600
Canvas_Height = 300

# Heuristic function 
def heur(node0, node1):
    return ((node0[0] - node1[0]) ** 2 + (node0[1] - node1[1]) ** 2) ** 0.5

#find neighbor
def find_neigh(node):
    #get x and y
    x, y = node
    #finding the neigghbor up, down left and right
    neigh = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
    return [(x, y) for x, y in neigh if not in_obs(x, y)]

#bi astar
def bi_astar(start, goal, thresh, iterations):
    #define forw list and back ist
    forw_open_lst = [(0, start)]
    back_open_lst = [(0, goal)]
    #cost
    forw_cost = {start: 0}
    back_cost = {goal: 0}
    forw_parent = {}
    back_parent = {}
    #for plotting
    forw_closed_lst = set()
    back_closed_lst = set()

    index = 0
    #runiing biastar
    while forw_open_lst and back_open_lst and (index < iterations):
        #poping current one
        forw_curr = heapq.heappop(forw_open_lst)[1]
        back_curr = heapq.heappop(back_open_lst)[1]

        #explored for plotting
        forw_closed_lst.add(forw_curr)
        back_closed_lst.add(back_curr)

        #chck if both trees meet
        if (heur(forw_curr, back_curr) <= thresh):
            return (combine_path(forw_curr, back_curr, forw_parent, back_parent),forw_closed_lst, back_closed_lst)

        #get the neighbors
        forw_neigh = find_neigh(forw_curr)
        back_neigh = find_neigh(back_curr)

        #iterate the neighbors forward tree
        for neighbor in forw_neigh:
            total_cost = forw_cost[forw_curr] + 1
            if neighbor in forw_cost and total_cost >= forw_cost[neighbor]:
                continue
            #updatingthe cost
            if not in_obs(neighbor[0], neighbor[1]):
                forw_cost[neighbor] = total_cost
                cost = total_cost + heur(neighbor, goal)
                heapq.heappush(forw_open_lst, (cost, neighbor))
                forw_parent[neighbor] = forw_curr
        #iterate the neighbors forward tree
        for neighbor in back_neigh:
            total_cost = back_cost[back_curr] + 1
            if neighbor in back_cost and total_cost >= back_cost[neighbor]:
                continue
            #update the cost values
            if not in_obs(neighbor[0], neighbor[1]):
                back_cost[neighbor] = total_cost
                cost = total_cost + heur(neighbor, start)
                heapq.heappush(back_open_lst, (cost, neighbor))
                back_parent[neighbor] = back_curr
        #increment iterations
        index += 1

    return None, None, None  # No path found 

# find path func
def find_path(curr, parent):
    #ist elem
    path = [curr]
    #iterating for the parent
    while curr in parent:
        curr = parent[curr]
        path.append(curr)
    return path

#func to combine both paths
def combine_path(forw_curr, back_curr, forw_parent, back_parent):
    path_forw = find_path(forw_curr, forw_parent)
    path_forw = path_forw[::-1]
    path_back = find_path(back_curr, back_parent)
    #merging the both paths
    return path_forw + path_back

# for checking with obs space 
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

#define start and end node
start = (1, 150)
goal =  (599, 150)
threshold = 5
iterations = 10000

#running bi-a star
start_time = time.time()
final_path, forw_exp, back_exp = bi_astar(start, goal, threshold, iterations)
end_time = time.time()
print("Time taken for BI-Astar:", end_time - start_time, "seconds")
print("Final path length:",len(final_path))

#if path found then
if final_path:
    print("Path found")
    grid = np.zeros((Canvas_Height, Canvas_Width))
    for y in range(Canvas_Height):
        for x in range(Canvas_Width):
            if in_obs(x, y):
                grid[y, x] = 1
    plt.imshow(grid, cmap='Greys', origin='upper')
    #explored nodes plot
    for elem in forw_exp:
        plt.plot(elem[0], elem[1], 'bx', markersize=4)  
    for elem in back_exp:
        plt.plot(elem[0], elem[1], 'yx', markersize=4)  
    # final path
    path_x1 = [x for x, y in final_path]
    path_y1 = [y for x, y in final_path]
    #plotth he final path
    plt.plot(path_x1, path_y1, color='red', linewidth=2)
    plt.plot(start[0], start[1], 'go')
    plt.plot(goal[0], goal[1], 'go')  
    plt.title('BI-Astar')
    plt.xlabel('X')
    plt.ylabel('Y')
    #show
    plt.gca().invert_yaxis()
    plt.show()
else:
   print("path Not found.")
