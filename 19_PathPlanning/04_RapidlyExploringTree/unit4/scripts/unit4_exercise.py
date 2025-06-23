#! /usr/bin/env python



import rospy

from bresenham import bresenham
from math import atan2, cos, sin
from random import randrange as rand


# Node class
class Node:
    def __init__(self, coordinates, parent=None):
        # coordinates: list with [x,y] values of grid cell coordinates
        self.coordinates = coordinates
        # parent: Node object
        self.parent = parent

def calculate_distance(p1, p2):
    """
    Calculates distance between two [x,y] coordinates.
    p1: the point (as list containing x and y values) from which to measure 
    p2: the point (as list containing x and y values) to which to measure
    returns: the distance
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    distance = (dx ** 2 + dy ** 2)**0.5
    return distance

def calculate_angle(p1, p2):
    """
    Calculates the angle of a straight line between two [x,y] coordinates.
    p1: the point (as list containing x and y values) from which to measure 
    p2: the point (as list containing x and y values) to which to measure
    returns: the angle in radians
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    theta = atan2(dy, dx)
    return theta

def collision_detected(p1, p2, map, map_width):
    """
    Test if two nodes are separated by an obstacle by tracing a line between them
    p1: the point (as list containing x and y values) from which the check starts
    p2: the point (as list containing x and y values) at which the check ends
    map: the map containing free space and obstacles
    map_width: the map's width
    returns: True if a collision is detected, False if no collision is found
    """
    # Compute cells covered by the line p1-p2 using the Bresenham ray tracing algorithm
    covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))
    # Check if any of the cells is an obstacle cell
    for cell in covered_cells:
        # Access an element in a 1D-array (map) providing index = x + map_width*y
        if map[cell[0]+map_width*cell[1]]:
            # Detects a collision if map has a 1
            return True
    # No collision
    return False

def find_closest_node(random_pt, node_list):
    """
    Finds the closest node in the tree
    random_pt: a [x,y] point (as a list)
    node_list: list that keeps all nodes in the tree
    returns: Node instance that is the closest node in the tree
    """
    if not node_list:
        return None
    shortest = float('inf')
    closest_node = None

    ## Add your code ##
    for node in node_list:
        distance = calculate_distance(random_pt, node.coordinates)
        if distance < shortest:
            shortest = distance
            closest_node = node
    return closest_node

def create_new_branch_point(p1, p2, max_distance):
    """
    Creates a new point at the max_distance towards a second point
    p1: the point to go from (as a list containing x and y values)
    p2: the point to go to (as a list containing x and y values)
    max_distance: the expand distance (in grid cells)
    returns: new point as a list containing x and y values
    """
    ## Add your code ##
    distance = calculate_distance(p1,p2)
    angle = calculate_angle(p1,p2)
    if max_distance > distance:
        max_distance = distance

    new_x = p1[0] + (max_distance * cos(angle))  
    new_y = p1[1] + (max_distance * sin(angle))  


    return [int(round(new_x)), int(round(new_y))]

def test_goal(p1, p_goal, tolerance):
    """
    Test if goal has been reached considering a tolerance distance
    p1: a [x,y] point (as a list) from where to test
    p_goal: a [x,y] point (as a list) corresponding to the goal
    tolerance: distance margin (in grid cells) allowed around goal
    returns: True goal is within tolerance, False if goal is not within tolerance
    """
    ## Add your code ##
    
    distance = calculate_distance(p1, p_goal)
    if distance < tolerance:
        rospy.loginfo('RRT: Goal reached')
        return True

    return False

def rrt(initial_position, target_position, width, height, map, map_resolution, map_origin, tree_viz):
    ''' 
    Performs Rapidly-Exploring Random Trees (RRT) algorithm on a costmap with a given start and goal node
    '''
    ## Add your code ##
    root_node = Node (initial_position)
    nodes = []
    nodes.append(root_node)
    iterations = 0
    max_iterations = 10000
    max_branch_lenght = 20
    goal_tolerance = 20
    path = []

    while True:
        iterations +=1
        if iterations > max_iterations: 
            rospy.loginfo("RRT: Max iteration exceeded")
            return path   
        rospy.sleep(0.01)
        random_point = [rand(width),rand(height)]
        closest_node = find_closest_node(random_point,nodes)
        candidate_point = create_new_branch_point(closest_node.coordinates, random_point, max_branch_lenght)

        
        if not collision_detected(closest_node.coordinates, candidate_point, map, width):
            latestNode = Node(candidate_point, closest_node)
            nodes.append(latestNode)
            tree_viz.append(latestNode)
            
            if test_goal(latestNode.coordinates,target_position,goal_tolerance):
                path.append(latestNode.coordinates)
                node = latestNode
                while node.parent:
                    path.append(node.coordinates)
                    node = node.parent
                path.append(initial_position)
                path.reverse()
                rospy.loginfo("IVAN")
                break
        

        
    return path
        


    


# root_node = Node([1,1])
# node1 = Node([5,5],root_node)

# print(root_node.coordinates)
# print(root_node.parent)
# print(node1.coordinates)
# print(node1.parent.coordinates)
