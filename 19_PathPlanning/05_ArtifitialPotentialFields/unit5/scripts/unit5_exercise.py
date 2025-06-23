#!/usr/bin/env python

"""
ROS node for Artificial Potential Fields
Author: Roberto Zegers R.
Copyright: Copyright (c) 2021, Roberto Zegers R.
License: BSD-3-Clause
Date: Mai 2021
"""

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
import copy
import random

def conical_attractive_force(current_cell, goal_cell, K = 0.3):
    """
    Calculates the linear attractive force for one grid cell with respect to the target
    current_cell: a list containing x and y values of one map grid cell
    goal_cell: a list containing x and y values of the target grid cell
    K: potential attractive constant
    returns: linear attractive force scaled by the potential attractive constant
    """
    dx = goal_cell[0] - current_cell[0]
    dy = goal_cell[1] - current_cell[1]
    distance = (dx ** 2 + dy ** 2)**0.5
    
    
    return distance * K

def quadratic_attractive_force(current_cell, goal_cell, K = 0.015):
    """
    Calculates the quadratic attractive force for one grid cell with respect to the target
    current_cell: a list containing x and y values of one map grid cell
    goal_cell: a list containing x and y values of the target grid cell
    K: potential attractive constant
    returns: quadratic attractive force scaled by the potential attractive constant
    """
    dx = goal_cell[0] - current_cell[0]
    dy = goal_cell[1] - current_cell[1]
    distance = (dx ** 2 + dy ** 2)**0.5
    
    
    return (distance**2) * K


def random_force(K = 1.0):
    """
    Toy function that calculates a random force for one grid cell
    returns: a random force value scaled by the constant K
    """
    return K * random.uniform(0, 100)

def populate_attractive_field(height, width, goal_xy):
    """ 
    Creates an attractive field
    returns: 1D flat grid map
    """
    field = [0] * height * width
    for row in range(height):
      for col in range(width):
        #force_value = random_force()
        #force_value = conical_attractive_force([row,col],goal_xy)
        force_value = quadratic_attractive_force([row,col],goal_xy)
        # Assign to potential field
        field[row + width * col] = force_value
    return field

def costmap_callback(costmap_msg):
    global attractive_field, repulsive_field, total_field
    total_field = copy.deepcopy(costmap_msg)
    #sum of attractive and repulsive maps
    total_field.data = [0] * total_field.info.height * total_field.info.width

    show_text_in_rviz('Ready to accept commands!')
    
    attractive_field = copy.deepcopy(costmap_msg)
    attractive_field.data = [0] * attractive_field.info.height * attractive_field.info.width
    
    ### add your code starting here ###
    
    # Create a Repulsive Potential Field from a Costmap
    repulsive_field = copy.deepcopy(costmap_msg)
    # repulsive_field.data = [1] * repulsive_field.info.height * repulsive_field.info.width
    repulsive_field.data = list(repulsive_field.data)
    #print("IVAN" , repulsive_field.data)
    for i in range(len(repulsive_field.data)):
        if repulsive_field.data[i] == -1:
            repulsive_field.data[i] = 100

    # Change grid map values from unknown to occupied
    print("Repulstive", len(repulsive_field.data))
    print("Repulsive" , repulsive_field.data)


    
    # Publish repulsive field
    repulsive_field_publisher.publish(repulsive_field)
    

def new_goal_callback(data):
    global attractive_field, repulsive_field, total_field
    # Convert goal position from Rviz into grid cell x,y value
    goal_grid_map = world_to_grid_map(data.pose)
    # Populate attractive field data
    unbounded_attractive_field = populate_attractive_field(attractive_field.info.height, attractive_field.info.width, goal_grid_map)
    # Limit the max value of a grid cell to 100 (max value allowed by nav_msgs/OccupancyGrid msg)
    attractive_field.data = [100 if x > 100 else int(x) for x in unbounded_attractive_field]
    print("Attractive", len(attractive_field.data))
    print("Attractive", attractive_field.data)
    # Publish attractive field
    attractive_field_publisher.publish(attractive_field)
    
    ### add your code starting here ###
    # Calculate and publish a Total Potential Field
    for i in range(len(total_field.data)):
        total_field.data[i] = attractive_field.data[i] + repulsive_field.data[i]
    print("Total", len(total_field.data))
    print("Total", total_field.data)
    
    total_field.data = rescale_to_max_value(100 , total_field.data)
    total_field_publisher.publish(total_field)

    print("Reescaled", len(total_field.data))
    print("Rescaled", total_field.data)


    
    

def rescale_to_max_value(max_value, input_list):
    """ Rescale each value inside input_list into a target range of 0 to max_value """
    scale_factor = max_value / float(max(input_list))
    # Multiply each item by the scale_factor
    input_list_rescaled = [int(x * scale_factor) for x in input_list]
    return input_list_rescaled

def show_text_in_rviz(text):
    text_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(2.0),
                pose=Pose(Point(0.0, 0.0, 2.00), Quaternion(0, 0, 0, 1)),
                scale=Vector3(1.5, 1.5, 1.5),
                header=Header(frame_id='map'),
                color=ColorRGBA(0.0, 0.0, 0.0, 1.0),
                text=text)
    text_publisher.publish(text_marker)

def world_to_grid_map(world_pose):
    """ Convert Pose() message in world frame of reference to grid map cell/pixel coordinates """
    grid_cell_x = int((world_pose.position.x - attractive_field.info.origin.position.x) / attractive_field.info.resolution)
    grid_cell_y = int((world_pose.position.y - attractive_field.info.origin.position.y) / attractive_field.info.resolution)
    return [grid_cell_x, grid_cell_y]


if __name__ == '__main__':
    try:
        rospy.init_node('artificial_potential_field_exercise')

        repulsive_field_publisher = rospy.Publisher("repulsive_field", OccupancyGrid, queue_size=10)
        attractive_field_publisher = rospy.Publisher("attractive_field", OccupancyGrid, queue_size=10)
        total_field_publisher = rospy.Publisher("total_field", OccupancyGrid, queue_size=10)
        text_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        costmap_subscriber = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, costmap_callback)
        goal_subscriber = rospy.Subscriber("move_base_simple/goal", PoseStamped, new_goal_callback)

        attractive_field = OccupancyGrid()
        repulsive_field = OccupancyGrid()
        total_field = OccupancyGrid()

        rospy.loginfo("Artificial Potential Field Exercise Running")
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
