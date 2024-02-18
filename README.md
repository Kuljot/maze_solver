# maze_solver
Maze Solver Bot Simulation

## Requirements
* ROS2 Humble/Iron
* OpenCV
* Gazebo

## Installation
```
git clone https://github.com/Kuljot/maze_solver.git 
cd maze_solver
colcon build
```

## Run the simulation
After sourcing the workspace
```
source install/setup.bash
```
Run  
```
ros2 launch maze_solver launch_sim.launch.py
```
You will seethe simulation of a differential drive bot in gazebo , a maze is surrounding the bot. 
![Screenshot from 2024-02-18 11-07-53](https://github.com/Kuljot/maze_solver/assets/33811204/ddfeec40-737b-43e0-946e-20c4df40000b)

And a camera overhead.

![Screenshot from 2024-02-18 11-08-11](https://github.com/Kuljot/maze_solver/assets/33811204/47a0f172-abcb-4df6-a012-2c3f4b7e0a9e)



## Drive the bot 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```
![Screenshot from 2024-02-18 11-08-29](https://github.com/Kuljot/maze_solver/assets/33811204/eb81a560-7d11-4735-8b68-368416ae7115)


## Run the SLAM Toolbox for map creation
```
ros2 launch maze_solver slam.launch.py
```
![image](https://github.com/Kuljot/maze_solver/assets/33811204/57587fc5-47f3-43ad-bedf-53d41284569a)

After that you can drive the bot around to create the map.


## Run the localiszation node 
```
ros2 run maze_solver bot_localizer_node
``` 

![Screenshot from 2024-02-18 11-10-19](https://github.com/Kuljot/maze_solver/assets/33811204/913bc931-3de9-4dd7-b18e-be783062ceca)

It will use the camera's input image to localise the bot using the smallest contour size detection also apply thinning method to convert the free path into graph. 
You can see the intermediate steps of image processing for bot localisation. 

It will also show a node graph in which end-nodes are shown in red with indices, tri junctions in blue and turning points in green.

## Run the path creation node 
```
ros2 run maze_solver ros2 run maze_solver path_planner_node 5
``` 
![Screenshot from 2024-02-18 11-12-10](https://github.com/Kuljot/maze_solver/assets/33811204/0e57a16c-1121-46ba-80bf-395905b6a077)

If the localization node is still running then you can run the above path planning node with an intezer argument that is the end-node index. 
It will find the nearest node to the bot and use an A* algorithm to trace the path to go to the input node. Blue path represents the A* search traversal and green path is the shortest path.



