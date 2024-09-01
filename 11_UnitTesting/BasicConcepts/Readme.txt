
Testing tools in ROS

Let's start talking about tests! Basically, we can differentiate between 3 main tools that we are going to use when creating tests for our ROS projects. These are:

    gtest: This tool is used for testing C++ code at the library level (at the C++ API level). It is Google's C++ test framework. You can check the repository here: https://github.com/google/googletest

    unittest: This tool is used for testing Python code at the library level (at the Python API level). You can check the official site here: http://pythontesting.net/framework/unittest/unittest-introduction/

    rostest: This tool is used for testing at the ROS node level. Here you can check the official ROS Wiki page: http://wiki.ros.org/rostest

Usually for ROS projects, you will have to use rostest in combination with one of the above (gtest or unittest), depending on if your project uses Python or C++.

Within this course, as you may have already noticed, we are going to be working with Python. This means that we are going to focus on the rostest and unittest frameworks. In the future, we will probably create a C++ equivalent course for this one, where we will explain more about the gtest framework.


Levels of testing

Basically, we can divide the tests into 3 different groups. They are:

    Library unit tests: These are the tests without ROS. This means that they are not meant to test any ROS-related issue. These tests are performed using the unittest or gtest frameworks by themselves.

    ROS unit tests: These will test your ROS code related to a single node. They will start your node and test its external API, like published or subscribed topics, etc. These tests are performed using a combination of a rostest alongside either a unittest or gtest.

    ROS integration tests: These will start up multiple nodes and test that they all work together as expected. For instance, that your robot does what it's supposed to do. These tests are also done using a combination of a rostest with a unittest/gtest.

