# Description 

This project consists of two Python scripts - a REST server and a REST client - designed to monitor the status of a robot controlled by the Robot Operating System (ROS). The REST server exposes an endpoint to provide the current status of the robot, while the REST client continuously queries this endpoint to display the status in real-time.
We are using flask API for a lightweight web framework 

## dependecies 

[turtlebot](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
this link has the needed dependencies to be downloaded 

# Steps

## Running the simulator 
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
![image](https://github.com/Saketh-Gurram/robot-rest-api/assets/95581205/aef51349-07ee-4084-8dbd-c2c77a468a20)

## Running the teleop(to control the bot)
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Check if the bot is moving using the 'waxd s ' keys

## Running rviz
We need to run rviz to map the world generated in gazeboo 
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
```
* after the world is mapped,save the map 
* It will be saved in two files with extensions .yaml and .pgm

# running Navigation

run the same code for starting the gazeboo world 

run the following code to open rviz with the map made.
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
* Click on 2D Pose estimation and click and drag to match the positon and direction of the bot located in the gazeboo simulation 
* Click on the 2D Nav Goal nd drag the final destination point to where the bot must reach 

The bot will move to the final point without hitting any obstacles as mapped 

# Running the Scripts 

## Running the Server
* Go to your catkin_workspace, eg catkin_ws/src
```bash
git clone https://github.com/Saketh-Gurram/robot-rest-api.git
cd ..
catkin_make
roslaunch robot-rest-api server.launch
```

* the luanch file contains the code to run the server side 

## Running of clients 
```bash
cd robot-rest-api
cd src
python client.py
```
