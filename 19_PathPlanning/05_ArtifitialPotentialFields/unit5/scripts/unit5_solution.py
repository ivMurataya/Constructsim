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


def random_force(K = 1.0):
    """
    Toy function that calculates a random force for one grid cell
    returns: a random force value scaled by the constant K
    """
    return K * random.uniform(0, 100)

def conical_attractive_force(current_cell, goal_cell, K = 1.0):
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
    return K * distance

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
    quadratic_attractive_force = distance**2
    return K * quadratic_attractive_force

def populate_attractive_field(height, width, goal_xy):
    """ 
    Creates an attractive field
    returns: 1D flat grid map
    """
    field = [0] * height * width
    for row in range(height):
      for col in range(width):
        #force_value = random_force()
        force_value = conical_attractive_force([row, col], goal_xy)
        #force_value = quadratic_attractive_force([row, col], goal_xy)
        # Assign to potential field
        field[row + width * col] = force_value
    return field

def costmap_callback(costmap_msg):
    global attractive_field, repulsive_field, total_field
    total_field = copy.deepcopy(costmap_msg)
    show_text_in_rviz('Ready to accept commands Sol!')
    attractive_field = copy.deepcopy(costmap_msg)
    attractive_field.data = [0] * attractive_field.info.height * attractive_field.info.width

    # Create a Repulsive Potential Field from a Costmap
    repulsive_field = copy.deepcopy(costmap_msg)
    repulsive_field.data = list(repulsive_field.data)

    # Change grid map values from unknown to occupied
    for index, value in enumerate(repulsive_field.data):
      if value == -1:
        repulsive_field.data[index] = 100
    
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
    # Publish attractive field
    attractive_field_publisher.publish(attractive_field)
    
    # Calculate and publish a Total Potential Field
    total_field_unscaled_values = [a + b for a, b in zip(attractive_field.data, repulsive_field.data)]
    total_field.data = rescale_to_max_value(100, total_field_unscaled_values)
    total_field_publisher.publish(total_field)  

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