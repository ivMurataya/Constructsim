

Part I: Declare a virtual function in the base class

Go to the code editor and open up the base_robot_manager.h and base_robot_manager.cpp files inside the robot_manager_inheritance package from Exercise 4.2.2.

Tasks

    Add a std::string protected member variable robot_name to keep the name of the robot.
    Add a second std::string protected member variable robot_location to store where the robot is located.
    Initialize the above variables to a hard-coded name and location of your choice.
    Add a new virtual method void displayRobotDetails() and use it to display the robot name and location to the screen each time the ConfigOutputCallback() method gets triggered. Use std::cout or ROS_INFO(), whichever suits you best

rosrun robot_manager_inheritance robot_manager_base_class_test_node


Test the /robot_manager_output service and confirm the robot details are printed to screen.

Additionally in WebShell #1 you should see the robot name and location printed out to the console.
Notice this is just an example, your otuput is probably slightly different.

[ INFO] [1649337647.229293471]: Enable Output Service created
[ INFO] [1649330069.471218739]: Robot Name: Waste Allocation Load Lifter Earth-Class
[ INFO] [1649330069.471286873]: Robot Location: Emeryville, California
[ INFO] [1649330069.471318803]: Robot Manager console output enabled.

Part II: Override the virtual function behaviour in the derived class



Open up the files mobile_robot_robot_manager.h and mobile_robot_robot_manager.cpp from Exercise 4.2.2 in your code editor.

Tasks

    Modify the derived class: Add two private instance variables battery_charge_level (of type float) and type_of_battery (of type std::string) with default values of 0.55 and Lead acid batteries respectively.
    Override the displayRobotDetails() method in the derived class to display the battery_charge_level and type_of_battery in addition to the information that was displayed by the method in the base class.

Compile your code and run the executable that creates the derived class object:
rosrun robot_manager_inheritance mobile_robot_robot_manager_node
rosservice call /robot_manager_output "data: true"
rosservice call /robot_manager_output "data: false"

Additionally in WebShell #1 you should see the robot name, location, battery charge level and type of battery printed out to the console.
Notice this is just an example, your otuput is probably slightly different.

[ INFO] [1649337647.229293471]: Enable Output Service created
[ INFO] [1649337647.239991102]: Odometry subscriber created
[ INFO] [1649337653.683163023]: Robot Name: Waste Allocation Load Lifter Earth-Class
[ INFO] [1649337653.683397388]: Robot Location: Emeryville, California
[ INFO] [1649337653.683586535]: Battery Charge Level: 0.550000
[ INFO] [1649337653.683646119]: Type of Battery: Lead acid batteries
[ INFO] [1649337653.683697607]: Robot Manager console output enabled.