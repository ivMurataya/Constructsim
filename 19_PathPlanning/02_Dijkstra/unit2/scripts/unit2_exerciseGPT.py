#! /usr/bin/env python


import rospy

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


def dijkstra(start_index, goal_index, width, height, costmap, resolution, origin, grid_viz):
    ''' 
    Performs Dijkstra's shortes path algorithm search on a costmap with a given start and goal node
    '''
    # Open list: stores nodes to be explored, each as [index, g_cost]
    open_list = [[start_index, 0]]
    
    # Closed list: tracks visited nodes
    closed_list = set()
    
    # Parents dictionary: maps child node to its parent
    parents = {}
    
    # g_costs dictionary: maps node to its g_cost
    g_costs = {start_index: 0}
    
    # Result path (to be filled later)
    shortest_path = []
    
    # Flag to indicate whether a path to goal was found
    path_found = False
    
    # Optional ROS logging message
    rospy.loginfo('Dijkstra: Initialization done!')
    
    # Temporary return for testing purposes

    # Main loop
    while open_list:
        # Sort open_list by g_cost (ascending)
        open_list.sort(key=lambda x: x[1])

        # Pop the node with the lowest g_cost
        current_node = open_list.pop(0)[0]

        # Add current node to closed_list
        closed_list.add(current_node)

        # Optional: visualize the closed node
        grid_viz.set_color(current_node, "pale yellow")

        # Check if goal is reached
        if current_node == goal_index:
            path_found = True
            break

        # Get neighboring nodes
        neighbours = find_neighbors(current_node, width, height, costmap, resolution)

         # Loop over each neighbor
        for neighbor in neighbours:
            neighbor_index, step_cost = neighbor

            # Skip if neighbor is already visited
            if neighbor_index in closed_list:
                continue

            # Calculate tentative g_cost to neighbor
            g_cost = g_costs[current_node] + step_cost

            # Check if neighbor is in open_list
            in_open_list = False
            for i, (idx, _) in enumerate(open_list):
                if idx == neighbor_index:
                    in_open_list = True
                    open_list_index = i
                    break

            if in_open_list:
                # Case 1: Update if new g_cost is better
                if g_cost < g_costs[neighbor_index]:
                    g_costs[neighbor_index] = g_cost
                    parents[neighbor_index] = current_node
                    open_list[open_list_index][1] = g_cost
            else:
                # Case 2: Add new neighbor to open_list
                g_costs[neighbor_index] = g_cost
                parents[neighbor_index] = current_node
                open_list.append([neighbor_index, g_cost])
                grid_viz.set_color(neighbor_index, 'orange')

    # Optional ROS log
    rospy.loginfo('Dijkstra: Done traversing nodes in open_list')
    # Path reconstruction
    if not path_found:
        # Optional warning
        rospy.logwarn('Dijkstra: No path found!')
        return []

    # Reconstruct path from goal to start
    node = goal_index
    while node != start_index:
        shortest_path.append(node)
        node = parents[node]
    shortest_path.append(start_index)  # Don't forget to add the start node
    shortest_path.reverse()

    # Optional info log
    # rospy.loginfo('Dijkstra: Done reconstructing path')

    return shortest_path




