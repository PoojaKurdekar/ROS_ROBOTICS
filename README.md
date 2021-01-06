# ROS_ROBOTICS
Differential Robot model and controller



# ros_robotics
ROS Robotics

(/home/user/catkin_ws_1/src/ros_robotics)

1. ROS Robotics By Example -- Carol Fairchild & Dr.Thomas L.Harman
Complete chapter 1,2 & 3 from above book.

2. Creating own Two wheeled Differential Robot
Reference : Learning Robotics Using Python by Lentin Joseph(chapter 3)

3. Launching and moving robot

 roslaunch ros_robotics diff_wheeled_gazebo_final.launch
 
 cd ~/catkin_ws_1/src/matlab/propotional_matlab
 
 python propotinal_head_.py
 
 python circularpath.py
 
 python move_robot.py
 
 python Propotional.py



4. ROBOT moving along given waypoints using proportional controller

 roslaunch ros_robotics diff_wheeled_gazebo_final.launch
 
 cd catkin_ws_1/src/matlab/propotional_matlab/
 
 python waypoints.py
 
 

5. Two ROBOTS moving along given waypoints using proportional controller

 roslaunch ros_robotics main.launch
 
 cd catkin_ws_1/src/matlab/propotional_matlab/
 
 python waypoints_test.py
 
 
 

6. One ROBOT following another ROBOT and controlling the master robot using proportional controllers.

  roslaunch ros_robotics main.launch
  
  roslaunch ros_robotics irobot_follow_turtle.launch
  
  cd ~/catkin_ws_1/src/matlab/propotional_matlab
  
  python propotinal_head_.py
  
  python tele_keyborad.py
  
  

     Reference : https://www.theconstructsim.com/make-robot-follow-another-robot/
