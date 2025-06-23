Welcome to Unit 5! In this unit, you'll dive into Artificial Potential Fields (APF)—a classic, elegant, and surprisingly intuitive approach to robotic path planning.
Why APF?
In robotics, the speed and responsiveness of a path planning algorithm are often just as important as its accuracy. 
Algorithms like Dijkstra and A* offer precision but can be resource-intensive, especially in large environments. 
On the other hand, sampling-based methods like RRT are faster but tend to produce rough, zig-zag paths.
That’s where Artificial Potential Fields come into play. Originally developed to provide fast and reactive navigation, 
APF models the environment as a mathematical potential field, where the robot is pulled toward the goal and pushed away from obstacles.
What Will You Learn?
By the end of this unit, you will:
    Understand the fundamentals of 2D grid-based path planning using APF.
    Learn how to mathematically model attractive forces (towards the goal) and repulsive forces (away from obstacles).
    Combine these forces into a total potential function.
    Use gradient descent to follow the path of steepest descent—like a ball rolling downhill.
    Tune your parameters to balance speed, safety, and precision.
    Test your algorithm using ROS and Gazebo, bringing theory to life in a simulated robot.
Visualization Tip
Think of APF like a landscape of hills and valleys: obstacles are tall mountains, and the goal is a deep valley.
Your robot “rolls downhill” from its start point, naturally steering clear of obstacles and heading straight for the goal.
