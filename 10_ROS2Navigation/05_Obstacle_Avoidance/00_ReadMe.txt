SAME PROJECT AS PATH PLANING BUT ADDING GLOBAL AND LOCAL COSTMAPS
ALSO HAVING THE COMPLETE NAVIGATION SERVER IN A LAUNCH FILE

In this unit, you will learn how Nav2 avoids static and dynamic obstacles.

Topics covered in this unit are:

    An explanation of Costmap 2D
    Global Costmaps
    Local Costmaps
    How Nav2 uses Costmaps to avoid obstacles
    FINALLY: integrate all the navigation parts in a single launch file


Add the list of settings above to the planner_server.yaml you created in the previous unit (at the bottom of the file).
In the previous exercise, you did not have the obstacle_layer adding obstacles to the global_costmap
Include the "obstacle_layer" in the plugins list before the "inflation_layer". Otherwise, the newly detected obstacles will not be enlarged by the "inflation_layer" plugin.

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.15
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
        

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      always_send_full_costmap: True
      
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0


source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 run gazebo_ros spawn_entity.py -entity gas_bottle3 -database gas_bottle -x -0.6 -y -0.3


To add a local costmap, you must add a series of params to the controller configuration file (at the bottom of the file).
https://get-help.theconstruct.ai/t/cannot-properly-visualize-local-costmap-in-rviz2-exercise-5-3-unit-5/23295


local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 2
      height: 2
      resolution: 0.05
      # robot_radius: 0.15
      footprint: "[ [0.089, 0.069], [0.089, -0.069], [-0.089, -0.069], [-0.089, 0.069] ]"
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.35
      static_layer:
        map_subscribe_transient_local: True
      always_send_full_costmap: True

















