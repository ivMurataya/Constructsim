#! /usr/bin/env python

"""
ROS Artificial Potential Fields gradient descent path planning exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: May 2021
Usage: roslaunch unit5 unit5_exercise.launch run_gradient_descent:=true
"""

import rospy

def get_neighbours_and_gradients(index, width, height, potential_field):
    """
    Identifies neighbor nodes and their respective discrete gradient values
    Inspects the 8 adjacent neighbors
    Checks if neighbor is inside the map boundaries
    Returns a list containing [discrete_gradient, index of neighbor node] pairs
    """
    neighbours_and_gradients = []

    # Get the index value of the 8 adjacent neighbors
    upper = index - width
    left = index - 1
    upper_left = index - width - 1
    upper_right = index - width + 1
    right = index + 1
    lower_left = index + width - 1
    lower = index + width
    lower_right = index + width + 1

    # upper neighbor
    if upper > 0:
      discrete_gradient = potential_field[upper] - potential_field[lower]
      neighbours_and_gradients.append([discrete_gradient, upper])

    # left neighbor
    if left % width > 0:
      discrete_gradient = potential_field[left] - potential_field[right]
      neighbours_and_gradients.append([discrete_gradient, left])

    # upper left neighbor
    if upper_left > 0 and upper_left % width > 0:
      discrete_gradient = potential_field[upper_left] - potential_field[lower_right]
      neighbours_and_gradients.append([discrete_gradient, upper_left])

    # upper right neighbor
    if upper_right > 0 and (upper_right) % width != (width - 1):
      discrete_gradient = potential_field[upper_right] - potential_field[lower_left]
      neighbours_and_gradients.append([discrete_gradient, upper_right])

    # right neighbor
    if right % width != (width + 1):
      discrete_gradient = potential_field[right] - potential_field[left]
      neighbours_and_gradients.append([discrete_gradient, right])

    # lower left neighbor
    if lower_left < height * width and lower_left % width != 0:
      discrete_gradient = potential_field[lower_left] - potential_field[upper_right]
      neighbours_and_gradients.append([discrete_gradient, lower_left])

    # lower neighbor
    if lower <= height * width:
      discrete_gradient = potential_field[lower] - potential_field[upper]
      neighbours_and_gradients.append([discrete_gradient, lower])

    # lower right neighbor
    if (lower_right) <= height * width and lower_right % width != (width - 1):
      discrete_gradient = potential_field[lower_right] - potential_field[upper_left]
      neighbours_and_gradients.append([discrete_gradient, lower_right])

    return neighbours_and_gradients

def euclidean_distance(index_a, index_b, map_width):
    """
    Euclidean distance between grid cells provided as linear indexes on a flat map
    """
    a = indexToGridCell(index_a, map_width)
    b = indexToGridCell(index_b, map_width)

    return ((a[0]-b[0])**2 + (a[1]-b[1])**2)**0.5

def indexToGridCell(flat_map_index, map_width):
    """
    Converts a linear index of a flat map to grid cell coordinate values
    flat_map_index: a linear index value, specifying a cell/pixel in an 1-D array
    map_width: the map's width 
    returns: list with [x,y] grid cell coordinates
    """
    grid_cell_map_x = flat_map_index % map_width
    grid_cell_map_y = flat_map_index // map_width
    return [grid_cell_map_x, grid_cell_map_y]

def gradient_descent(start_index, goal_index, width, height, potential_field_data, descent_viz):
    ''' 
    Performs gradient descent on an artificial potential field 
    with a given start, goal node and a total potential field
    '''
    max_iterations = 400
    current_iteration = 0
    tolerance_goal = 4
    reached_goal = False
    curret = start_index
    path = []
    rospy.loginfo("Gradient descent: Done with initialization")

    while current_iteration < max_iterations:
        descent_viz.draw(curret,potential_field_data[curret])
        distance = euclidean_distance(current_iteration,goal_index,width)
        if distance < tolerance_goal:
            reached_goal = True
            rospy.loginfo("Gradient descent: Goal reached")
            descent_viz.draw(goal_index,0)
            break
        neighbors = get_neighbours_and_gradients(curret,width,height, potential_field_data)
        min_neighborIndex =  min(neighbors, key=lambda x: x[0])[1]
        path.append(min_neighborIndex)
        curret = min_neighborIndex
        current_iteration += 1
    
    if not reached_goal:
        rospy.logwarn("GRadient Descente Probably stuck at critical Point!!")
        pass
    else:
        return path


