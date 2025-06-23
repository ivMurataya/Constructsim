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
    path_found = False
    # create an open_list
    open_list = []
    # set to hold already processed nodes
    closed_list = set()
    # dict for mapping children to parent
    parents = dict()
    # dict for mapping g costs (travel costs) to nodes
    g_costs = dict()
    # set the start's node g_cost
    g_costs[start_index] = 0
    shortest_path = []
    # add start node to open list
    open_list.append([start_index, 0])
    rospy.loginfo('Dijkstra: Done with initialization')


    while open_list:
        open_list.sort(key=lambda x: x[1])
        # print("openlist" ,open_list)
        current_node = open_list.pop(0)[0]
        # print("current node", current_node)
        closed_list.add(current_node)
        # print("open list " , open_list)
        # print("closed list", closed_list)
        grid_viz.set_color(current_node,"pale yellow")
        if current_node == goal_index:
            path_found = True
            break
        # It takes in the index of the current node, the map width and height, the costmap, and the map's resolution as arguments 
        neighbours = find_neighbors(current_node, width, height,costmap,resolution )
        
        #print (neighbours)
        for neighbor_index, step_cost in neighbours:
            if neighbor_index in closed_list:
                continue
            g_cost = g_costs[current_node] + step_cost

            found = False
            for i, (node_index, c) in enumerate(open_list):
                if node_index == neighbor_index:
                    position = i
                    found = True
                    break
                    
            if found:
                if g_cost < g_costs[neighbor_index]:
                    # Update the node's g_cost inside g_costs
                    g_costs[neighbor_index] = g_cost
                    parents[neighbor_index] = current_node
                    # Update the node's g_cost inside open_list
                    open_list[position] = [neighbor_index, g_cost]
              
            else:   
                g_costs[neighbor_index] = g_cost
                parents[neighbor_index] = current_node
                open_list.append([neighbor_index,g_cost])
                grid_viz.set_color(neighbor_index,'orange')


        rospy.loginfo('Dijkstra: Done traversing nodes in open_list')

        if path_found == False:
            rospy.logwarn('Dijkstra: No path found!')
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
    rospy.loginfo('Dijkstra: Done reconstructing path')
    return shortest_path

#dijkstra("a",2,2,2,2,2,2,3)



