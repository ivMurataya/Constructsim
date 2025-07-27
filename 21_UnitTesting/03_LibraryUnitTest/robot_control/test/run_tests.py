import rosunit
import rotate_robot_test_cases

# rosunit

#  unncoment for Run rotate_robot_test_cases with a class
# rosunit.unitrun('robot_control', 'rotate_robot_test_cases', 'rotate_robot_test_cases.MyTestSuite')

# Run rotate_robot_test_cases2 with a test name
rosunit.unitrun('robot_control', 'rotate_robot_test_cases2', 'rotate_robot_test_cases2.MyTestSuite.test_a')
rosunit.unitrun('robot_control', 'rotate_robot_test_cases2', 'rotate_robot_test_cases2.MyTestSuite.test_b')
