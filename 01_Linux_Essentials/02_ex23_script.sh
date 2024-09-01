#!/bin/bash

echo 'Going to my_scripts folder...'
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts/
echo 'Listing contents...'
ls -la
echo 'Changing permissions...'
chmod 777 move_bb8_square.py
echo 'Listing contents...'
ls -la
