# assignment1_rt

## Notes 

Tested on WSL2 with Ubuntu 20.04 and ROS Noetic

The package 'assignment1_rt' contains 2 nodes written in C++

### ui_node

This node :
- Spawns a new turtle in the environment named turtle2
- Makes the user able to select the turtle they want to control (turtle1 or turtle2), and the velocity of the robot along x,y and around z.
- Then, the command is send for 1 second, and then the robot stop, and the user can insert the command again. 


### distance_node

This node : 
- Checks the euclidian distance between turtle1 and turtle2 and publish it on the topic /turtle_distance
- Stops the moving turtle if the two turtles are too close (<1.5)
- Stops the moving turtle if the position is too close to the boundaries (<1.5)


## Setup and run

Clone this repository in your ROS Workspace and compile with : 

```bash
catkin_make
```

Then source your Workspace with

```bash
source ./devel/setup.bash
```

Call in a terminal:

```bash
roscore
``` 

Then, you can run the first node to display the screen with the first turtle with

```bash
rosrun turtlesim turtlesim_node 
```

To run the first node of the assignment you have to open a new window and execute : 

```bash
rosrun assignment1_rt ui_node.py
```
To run the second node of the assignment you have to open a new window and execute : 

```bash
rosrun assignment1_rt distance_node.py
```
