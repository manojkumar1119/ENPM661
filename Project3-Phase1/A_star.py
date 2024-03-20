#Libs used

import cv2
import numpy as np
import heapq
import time
import matplotlib.pyplot as plt

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
start_x = int(input("Enter Start x coordinates:"))
start_y = int(input("Enter Start y coordinates:"))

#Enter goal coords
end_x = int(input("Enter End x coordinates:"))
end_y = int(input("Enter End y coordinates:"))

# Defining the size of the map
Canvas_Width = 1200 
Canvas_Height = 500 

#Creating an empty matrix defiining canvas
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

# Define the vertices of the hexagon
hexagon_vertices = [(650, 100), (775, 175), (775, 325), (650, 400), (525, 325), (525, 175)]

# Calculate line equations for the sides of the hexagon
hex_eqn, hex_eqn_c = calculate_hex_line(hexagon_vertices, (clearance_robot+clearance_map))

#CAlling a function whic creates the map
obs_matrix = createMap(obs_matrix , Canvas_Width, Canvas_Height, clearance_robot, clearance_map, hex_eqn, hex_eqn_c)

visualize_canvas(obs_matrix)

#Invert to make it same as cartesian coords
obs_matrix = obs_matrix[::-1]
