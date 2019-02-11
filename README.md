# CMP3103M-ROS-ROBOTICS
CMP3103M Group Work Code For ROS Robotics (TurtleBot)


Connect to real turtle-bot


1) sudo openvpn rpi.ovpn
2) wget https://raw.githubusercontent.com/marc-hanheide/network-scripts/master/ros-network.sh -P ~/
3) source ~/ros-network.sh 192.168.2.1
4) roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find turtlebot_gazebo)/worlds/empty.world
5) roslaunch kobuki_keyop keyop.launch
6) roslaunch turtlebot_rviz_launchers view_robot.launch

Create ros workspace

1) mkdir -p ~/catkin_ws/src; cd ~/catkin_ws/src; catkin_init_workspace .;
2) catkin_create_pkg commanding_velocity rospy std_msgs geometry_msgs
3) mkdir ~/catkin_ws/src/commanding_velocity/scripts
4) cd ~/catkin_ws
   catkin_make
5) source ~/catkin_ws/devel/setup.bash
6) roscore
7) source ~/catkin_ws/devel/setup.bash
   spyder

Then load the scripts from the dir: /home/student/catkin_ws/src/command_velocity/scripts/
