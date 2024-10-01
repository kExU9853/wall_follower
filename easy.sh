
#!/bin/bash

gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py; exec bash"

gnome-terminal -- bash -c "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True; exec bash"
gnome-terminal -- bash -c "ros2 launch wall_follower wall_follower.launch.py; exec bash"

