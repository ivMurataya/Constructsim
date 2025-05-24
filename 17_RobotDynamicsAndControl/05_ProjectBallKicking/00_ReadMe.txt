Project:   Ball Kicking
Before starting with the Project, you will need to spawn a soccer ball and goal into the simulation. You can do this by executing the following commands:
rosrun gazebo_ros spawn_model -file /home/user/.gazebo/models/robocup_3Dsim_goal/model.sdf -sdf -model goal -x 3


rosrun gazebo_ros spawn_model -file /home/user/.gazebo/models/robocup_3Dsim_ball/model.sdf -sdf -model ball -y 0.2 -x 0.1 -z 0.0

Also, make sure you have installed the sympy and pydy-code-gen libraries:
pip install sympy
pip install pydy-code-gen


In this project you are going to program a simple dynamic controller for the RRBot arm that will move the robot for kicking a ball on the floor.
The idea is to move both links back for some angle, and then leave the arm fall freely. The accumulated potential energy will be converted into kinetic energy and transferred to the ball, as seen in the figure aside.
For moving the robot, you will compute the joint torques based on a pre-defined trajectory of the joints.
This trajectory will be defined with a simple cubic interpolator.
