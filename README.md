# AssistanT
The Turtlebot3 assistant project.

# Docker Installation
Install Desktop OS Ubuntu 16.04 LTS

Install Docker-CE using the following [instructions](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/)

# Docker run

The first time You start the docker, You have to execute the following command

```
# TO HAVE WIFI network working
docker run -it --name assistant_turtle_dev --net=host  -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw lyubomyrd/assistant:latest

docker run -it --name assistant_turtle_dev -p 8080:8080 -p 8090:8090 -p 9090:9090 -e DISPLAY -e LOCAL_USER_ID=$(id -u) -v /tmp/.X11-unix:/tmp/.X11-unix:rw lyubomyrd/assistant:latest
```

# Docker run

Next times, please execute
```
sudo docker start assistant_turtle_dev
```

# Designation:

The following designation will be used through the rest of the repository

Remote PC: R

Turtlebot(Raspberry): Tx, (x=A,B,C,...)

# Turtlebot start

Turtlebot is controller using ROS which is installed both on PC and Raspberry.



To execute the bringup sequence You have to perform the following steps:



## Steps that has to be performed inside the docker container's terminator window


1R) roscore

Connect to RPI (note that IP address might be different - this depends on network settings)

2R) ssh ubuntu@10.42.0.1

(Password: ubuntu)

Now You have a separeate window inside the terminator that controlles Raspberry, therefore Tx notation corresponds to input into this window.

3TA)  roslaunch turtlebot3_bringup turtlebot3_robot.launch

4R) roslaunch turtlebot3_bringup turtlebot3_remote.launch

# Teleoperation

1R) roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Mapping

Create a map:

1R) roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping

Save a map (do NOT close the previous window vefore the next step):

2R) rosrun map_server map_saver -f ~/map

After this step You can close 1R and 2R from mapping.

# Navigation

1R) roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml

# Development
Type pycharm to run PyCharm-community


# Troubleshooting
## time mismatch
convinient way to overcome issues with timer mismatch (as turtlebot wifi is isolated from external webserver for date-time verification)
when sshed to turtlebot run:
date --set="$(ssh user@server date)"
where <user@server> is your PC user and IP in Turtle's local network with appropriate time running

See wiki for Troubleshooting

# Start working with assistant

1. ``` roscore ```  
``` ssh -Y ubuntu@10.42.0.1```  
``` roslaunch turtlebot3_bringup turtlebot3_robot.launch ```  
``` roslaunch turtlebot3_bringup turtlebot3_remote.launch ```  

2. ```roslaunch assistant_launch navigation.launch map_file:=/*name_of_map_file*/```  
launch navigation on a map saved in */home/user/workspace/turtlebot3/base/src/assistant/assistant_gazebo/maps/* 

3. * to simply move platform around on the map:
   ```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch```
                      
   * to make robot navigating from current position to a certain point: 
   ``` rosrun assistant_action main_script.py ```


# Working in Simulation
1. ```roscore```  
2. ```roslaunch assistant_launch simulation.launch```  
Launch gazebo simulation for a turtlebot in the testing world *(for now - ucu_test.world)*.

3. ```roslaunch assistant_launch navigation.launch map_file:= ```  
Launch rviz with robot being able to navigate on a given map *(map_file)*
map should be saved in */home/user/workspace/turtlebot3/base/src/assistant/assistant_gazebo/maps*/ directory

4. ```rosrun assistant_action main_script.py```  
Command for assistant to navigate on a map from his current location to the point stated in the main_script.py  
*(for now - 'x': -2.16564114295, 'y': -5.48124138424)*  
