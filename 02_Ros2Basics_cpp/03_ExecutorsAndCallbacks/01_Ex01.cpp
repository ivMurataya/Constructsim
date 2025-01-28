#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);

  // Instantiate a node
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("executor_example_1_node");
    
  RCLCPP_INFO(node->get_logger(), "Bacon pancakes, making bacon pancakes");


/*The line above creates an Executor object responsible for the execution of Callbacks for one or more Nodes. Since you are creating a SingleThreadedExecutor object, all Callbacks in the Node are run in the same thread.*/
  rclcpp::executors::SingleThreadedExecutor executor;
  /*Now, use the add_node() method to add Nodes to the Executor.*/
  executor.add_node(node);
  executor.spin(); //Finally, run the Single-threaded Executor by calling its spin() method.

  rclcpp::shutdown();
  return 0;
}
