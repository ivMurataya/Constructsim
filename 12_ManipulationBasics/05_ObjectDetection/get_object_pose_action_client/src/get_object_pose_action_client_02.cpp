#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

//Import the ROS2 C++ client libraries to work with Actions (rclcpp_action) and (rclcpp)
//#include "t3_action_msg/action/move.hpp"
//#include "actions_quiz_msg/action/distance.hpp"
/*

        02 Script that iterates over the Primitive Object to get the shape of both objects

*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"

class MyActionClient : public rclcpp::Node
{
public:
  //using Distance = actions_quiz_msg::action::Distance;
  //using GoalHandleMove = rclcpp_action::ClientGoalHandle<Distance>;

  using Grasping = grasping_msgs::action::FindGraspableObjects;
  using GoalHndleGrasp = rclcpp_action::ClientGoalHandle<Grasping>;




//In the constructor of the class, we initialize a ROS2 node named my_action_client. Also, note that we initialize the goal_done_ variable to false.
  explicit MyActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("my_action_client", node_options), goal_done_(false)
  {
  //Then inside the constructor body we create an ActionClient object that connects to the /move_robot_as Action Server
    this->client_ptr_ = rclcpp_action::create_client<Grasping>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "find_objects");
//Finally, we create a timer object, with a callback method named send_goal
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MyActionClient::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;
//The line below cancels the timer so that it only gets executed one time (in this example, we don't want to keep sending goals to the Action Server):
    this->timer_->cancel();

    this->goal_done_ = false;
//Here, check if the Action Server is up and running. If not, we print a message to the node's log:
    if (!this->client_ptr_) {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }

//Here, we wait for the Action Server to start for 10 seconds. If it is not ready after the 10 seconds pass.
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      this->goal_done_ = true;
      return;
    }

//Next, a Goal() object of the Move action type is created. Then, we access the secs variable of the Action goal and assign it an arbitrary numerical value in seconds. In this example its 5 seconds
    auto goal_msg = Grasping::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
//Here, define the different callbacks for the Action Client:
//The first line above is to initialize the object that is used to set callback methods for goal acceptance, feedback and completion.
    auto send_goal_options = rclcpp_action::Client<Grasping>::SendGoalOptions();
//As you can see, they are defined immediately after                
    send_goal_options.goal_response_callback =
      std::bind(&MyActionClient::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&MyActionClient::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&MyActionClient::result_callback, this, _1);

      //Finally, we send the goal to the Action Server using the async_send_goal method
/*We provide two arguments for this method:
    A goal message, in this case, goal_msg
    The callback methods of the client
This async_send_goal() method returns a future to a goal handle. 
This future goal handle will be completed when the Server has processed the goal, 
whether that's being accepted or rejected by the Server. So, you must assign a 
callback method to be triggered when the future is completed 
(the goal has been accepted or rejected). In this case, this method is 
goal_response_callback()
        */
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Grasping>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

//So, this method is triggered when the goal has been either accepted or rejected by the server. We can know this by checking the goal_handle value
  void goal_response_callback(const GoalHndleGrasp::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

//Here, print the feedback string to the node's log.
  void feedback_callback(
    GoalHndleGrasp::SharedPtr,
    const std::shared_ptr<const Grasping::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(), "Feedback received:");
  }

  void result_callback(const GoalHndleGrasp::WrappedResult & result)
  //First, check the result.code variable to see what happened with your goal. Next, print the status variable from our result
  {
    this->goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Received graspable objects");
        for (const auto & object : result.result->objects) {
            RCLCPP_INFO(this->get_logger(), "Object has %zu primitive poses:", object.object.primitive_poses.size());
            if (!object.object.primitive_poses.empty()) {
                    for (size_t i = 0; i < object.object.primitive_poses.size(); ++i) {
                    auto position = object.object.primitive_poses[i].position;
                    RCLCPP_INFO(this->get_logger(), "Object position: x=%.3f, y=%.3f, z=%.3f",position.x, position.y, position.z);
                    auto shapepp = object.object.primitives[i].type;
                    auto dim = object.object.primitives[i].dimensions;
                    RCLCPP_INFO(this->get_logger(), "Object type: x=%i", shapepp);
                    RCLCPP_INFO(this->get_logger(), "Object dimention: x=%.3f, y=%.3f, z=%.3f",dim[0], dim[1], dim[2]);
                    }
                    
                } else {
                    RCLCPP_WARN(this->get_logger(), "Object has no primitive pose data");
                }
            }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "Result received:  total distance ");
    

  }
};  // class MyActionClient


//the main function that uses executors
//Here you are creating an instance of the MyActionClient(). Then we initialize one MultiThreadedExecutor object.
//While the goal has not been completed, while (!action_client->is_goal_done()), you will spin the executor executor.spin_some();
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<MyActionClient>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done()) {
    executor.spin_some();
  }

  rclcpp::shutdown();
  return 0;
}


/*

ros2 action send_goal /find_objects grasping_msgs/action/FindGraspableObjects "{plan_grasps: false}"


# Set to false to disable grasp planning, returning only the objects found
bool plan_grasps
---
# Graspable objects found
GraspableObject[] objects

# Additional, non-graspable objects which may be support surfaces
Object[] support_surfaces
---
# Publish objects as they are detected and grasp planned
GraspableObject object

------------------------------------------------------------------------------------------------------------------------------------

ros2 interface show grasping_msgs/msg/GraspableObject

 
###########################################################
# This message describes an object + grasp data
Object object
moveit_msgs/Grasp[] grasps

------------------------------------------------------------------------------------------------------------------------------------


ros2 interface show grasping_msgs/msg/Object

###########################################################
# This message describes an object.

# Many of the geometric items below lack a stamp/frame_id,
# header stamp/frame_id should be used there
std_msgs/Header header

# An object might have a name
string name

# An object might have a known (named) support surface
string support_surface

# Objects might have properties, such as type/class, color, etc.
ObjectProperty[] properties

###########################################################
# Objects have many possible descriptions
#  The following are the possible description formats

# Perception modules often represent an object as a cluster of points
#  Is considered valid if the number of points > 0
sensor_msgs/PointCloud2 point_cluster

# MoveIt prefers solid primitives or meshes as a description of objects
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses

shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses

# An object representing a support surface might be described by a plane
# Is considered valid if coefficients are not all 0s.
shape_msgs/Plane surface

------------------------------------------------------------------------------------------------------------------------------------


geometry_msgs/Pose[] primitive_poses

ros2 interface show geometry_msgs/msg/Pose

# A representation of pose in free space, composed of position and orientation.

Point position
Quaternion orientation






ros2 interface show shape_msgs/msg/SolidPrimitive
# Defines box, sphere, cylinder, and cone.
# All shapes are defined to have their bounding boxes centered around 0,0,0.

uint8 BOX=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 CONE=4

# The type of the shape
uint8 type

# The dimensions of the shape
float64[<=3] dimensions  # At no point will dimensions have a length > 3.

# The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array.

# For type BOX, the X, Y, and Z dimensions are the length of the corresponding sides of the box.
uint8 BOX_X=0
uint8 BOX_Y=1
uint8 BOX_Z=2

# For the SPHERE type, only one component is used, giving the sphere's radius.
uint8 SPHERE_RADIUS=0

# The center line is oriented along the Z-axis for the CYLINDER and CONE types, 
# Therefore, the CYLINDER_HEIGHT (CONE_HEIGHT) component of dimensions gives the
# height of the cylinder (cone).
# The CYLINDER_RADIUS (CONE_RADIUS) component of dimensions provides the radius of
# the base of the cylinder (cone).
# Cone and cylinder primitives are defined to be circular. Therefore, the tip of the cone
# is pointing up along the +Z axis.

uint8 CYLINDER_HEIGHT=0
uint8 CYLINDER_RADIUS=1

uint8 CONE_HEIGHT=0
uint8 CONE_RADIUS=1

*/