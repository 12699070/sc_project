# sc_project
Group Project 6 of the subject 41014 Sensors and Control for Mechatronic Systems at UTS

Setup procedures:
1) Install ROS from the wiki page
2) Goto https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
3) Select your ROS version (just under top tabs of the webpage) and follow the procedure
4) Make sure to have all dependent packages installed (PC Setup link in 6.1.1)
5) Restart PC
6) Select turtlebot model, from command lines: export TURTLEBOT3_MODEL=waffle (this model has a camera)
6) From command lines: source devel/setup.bash
7) From command lines: roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
8) From command lines: rosrun rviz rviz
9) From rviz: add topics (+ symbol) -> image -> depth


-----------------------

ROS notes:
rzviz: https://www.youtube.com/watch?v=jQ0DPzjmdz4&t=598s (9:27)

check msg type: rosmsg show topic_name

TOPIC: http://wiki.ros.org/rostopic
Display messages published to a topic: rostopic echo /topic_name


-----------------------
OpenCV
cv_bridge setup: http://wiki.ros.org/cv_bridge
cv_bridge tutorial: http://wiki.ros.org/cv_bridge/Tutorials

Line follower http://edu.gaitech.hk/turtlebot/line-follower.html


-----------------------
Tasks

IMAGE PROCESSING:
_Convert depth info to depth map
_Depth map to openCV for corner detection
_Calculate orientation and distance

CONTROL:




