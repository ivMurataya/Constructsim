#! /usr/bin/env python

"""
Greedy Best-First Search path planning algorithm exercise
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit3 unit3_gbfs_exercise.launch
"""

import rospy
import math

def find_neighbors(index, width, height, costmap, orthogonal_step_cost):
  """
  Identifies neighbor nodes inspecting the 8 adjacent neighbors
  Checks if neighbor is inside the map boundaries and if is not an obstacle according to a threshold
  Returns a list with valid neighbour nodes as [index, step_cost] pairs
  """
  neighbors = []
  # length of diagonal = length of one side by the square root of 2 (1.41421)
  diagonal_step_cost = orthogonal_step_cost * 1.41421
  # threshold value used to reject neighbor nodes as they are considered as obstacles [1-254]
  lethal_cost = 1

  upper = index - width
  if upper > 0:
    if costmap[upper] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[upper]/255
      neighbors.append([upper, step_cost])

  left = index - 1
  if left % width > 0:
    if costmap[left] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[left]/255
      neighbors.append([left, step_cost])

  upper_left = index - width - 1
  if upper_left > 0 and upper_left % width > 0:
    if costmap[upper_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_left]/255
      neighbors.append([index - width - 1, step_cost])

  upper_right = index - width + 1
  if upper_right > 0 and (upper_right) % width != (width - 1):
    if costmap[upper_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[upper_right]/255
      neighbors.append([upper_right, step_cost])

  right = index + 1
  if right % width != (width + 1):
    if costmap[right] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[right]/255
      neighbors.append([right, step_cost])

  lower_left = index + width - 1
  if lower_left < height * width and lower_left % width != 0:
    if costmap[lower_left] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_left]/255
      neighbors.append([lower_left, step_cost])

  lower = index + width
  if lower <= height * width:
    if costmap[lower] < lethal_cost:
      step_cost = orthogonal_step_cost + costmap[lower]/255
      neighbors.append([lower, step_cost])

  lower_right = index + width + 1
  if (lower_right) <= height * width and lower_right % width != (width - 1):
    if costmap[lower_right] < lethal_cost:
      step_cost = diagonal_step_cost + costmap[lower_right]/255
      neighbors.append([lower_right, step_cost])

  return neighbors

def indexToWorld(flatmap_index, map_width, map_resolution, map_origin = [0,0]):
    """
    Converts a flatmap index value to world coordinates (meters)
    flatmap_index: a linear index value, specifying a cell/pixel in an 1-D array
    map_width: number of columns in the occupancy grid
    map_resolution: side lenght of each grid map cell in meters
    map_origin: the x,y position in grid cell coordinates of the world's coordinate origin
    Returns a list containing x,y coordinates in the world frame of reference
    """
    # convert to x,y grid cell/pixel coordinates
    grid_cell_map_x = flatmap_index % map_width
    grid_cell_map_y = flatmap_index // map_width
    # convert to world coordinates
    x = map_resolution * grid_cell_map_x + map_origin[0]
    y = map_resolution * grid_cell_map_y + map_origin[1]

    return [x,y]

def greedy_bfs(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
  ''' 
  Performs Greedy Best-First Search on a costmap with a given start and goal node
  '''

  # create an open_list
  open_list = []

  # set to hold already processed nodes
  closed_list = set()

  # dict for mapping children to parent
  parents = dict()

  # dict for mapping g costs (travel costs) to nodes
  h_costs = dict()

  # determine the h cost (heuristic cost) for the start node
  from_xy = indexToWorld(start_index, width, resolution, origin)
  to_xy = indexToWorld(goal_index, width, resolution, origin)
  h_cost = euclidean_distance(from_xy, to_xy)
  #start the starts node h_cost
  h_costs[start_index] = h_cost
  # add start node to open list
  open_list.append([start_index, h_cost])

  shortest_path = []

  path_found = False
  rospy.loginfo('Greedy BFS: Done with initialization IVAN')

  # Main loop, executes as long as there are still nodes inside open_list
  while open_list:

    # sort open_list according to the lowest 'h_cost' value (second element of each sublist)
    #open_list.sort(key = lambda x: x[1]) 
    #open_list = sorted(open_list, key=lambda item: manhattan_distance(item[1][0],item[1][1]))
    open_list = sorted(open_list, key=lambda item: manhattan_distance(indexToWorld(item[0], width, resolution, origin), to_xy))

    
    # extract the first element (the one with the lowest 'g_cost' value)
    current_node = open_list.pop(0)[0]

    # Close current_node to prevent from visting it again
    closed_list.add(current_node)

    # Optional: visualize closed nodes
    grid_viz.set_color(current_node,"pale yellow")

    # If current_node is the goal, exit the main loop
    if current_node == goal_index:
      path_found = True
      break

    # Get neighbors of current_node
    neighbors = find_neighbors(current_node, width, height, costmap, resolution)

    # Loop neighbors
    for neighbor_index, step_cost in neighbors:

      # Check if the neighbor has already been visited
      if neighbor_index in closed_list:
        continue

      # calculate g_cost of neighbour considering it is reached through current_node
      #h_cost = h_costs[current_node] + step_cost

      # Check if the neighbor is in open_list
      in_open_list = False
      for idx, element in enumerate(open_list):
        if element[0] == neighbor_index:
          in_open_list = True
          break

      # CASE 1: neighbor already in open_list
      if in_open_list:
        if h_cost < h_costs[neighbor_index]:
          # Update the node's g_cost inside g_costs
          h_costs[neighbor_index] = h_cost
          parents[neighbor_index] = current_node
          # Update the node's g_cost inside open_list
          open_list[idx] = [neighbor_index, h_cost]

      # CASE 2: neighbor not in open_list
      else:
        # Set the node's g_cost inside g_costs
        h_costs[neighbor_index] = h_cost
        parents[neighbor_index] = current_node
        # Add neighbor to open_list
        open_list.append([neighbor_index, h_cost])

        # Optional: visualize frontier
        grid_viz.set_color(neighbor_index,'orange')

  rospy.loginfo('Greedy BFS: Done traversing nodes in open_list')

  if not path_found:
    rospy.logwarn('Greedy BFS: No path found!')
    return shortest_path

  # Reconstruct path by working backwards from target
  if path_found:
      node = goal_index
      shortest_path.append(goal_index)
      while node != start_index:
          shortest_path.append(node)
          # get next node
          node = parents[node]
  # reverse list
  shortest_path = shortest_path[::-1]
  rospy.loginfo('Greedy BFS: Done reconstructing path')

  return shortest_path

# ---------------------------- IVAN -----------------------------
# two x,y points for testing your code
# point_a = [5.5,10.2]
# point_b = [-8.0,4.6]

def euclidean_distance(a,b):
    x = (a[0] - b[0])**2
    y = (a[1] - b[1])**2 
    d = math.sqrt(x + y)
    return d
# h = euclidean_distance(point_a,point_b)
# print(point_a)
# print(point_b)
# print(h)

def manhattan_distance (a,b):   
    x = abs (a[0] - b[0])
    y = abs (a[1] - b[1])
    d = x + y
    return d

# manhattan = manhattan_distance(point_a,point_b)
# print("manhattan", manhattan)

# -------------------------------------------------------------------
