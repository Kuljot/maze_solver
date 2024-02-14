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
You will seethe simulation of a differential drive bot in gazebo , a maze is surrounding the bot and a camera overhead.
![image](https://github.com/Kuljot/maze_solver/assets/33811204/9b05b29c-e18e-4919-afd4-ce787252498a)






## Drive the bot 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
```

## Run the SLAM Toolbox for map creation
```
ros2 launch maze_solver slam.launch.py
```
![image](https://github.com/Kuljot/maze_solver/assets/33811204/57587fc5-47f3-43ad-bedf-53d41284569a)

After that you can drive the bot around to create the map.

## Run the path creation node 
```
ros2 run maze_solver bot_localizer_node
``` 

It will use the camera's input image to localise the bot using the contour size detection also apply thinning method to convert the free path into graph

![image](https://github.com/Kuljot/maze_solver/assets/33811204/3edd5a73-0c91-4324-9e64-baef33521b8f)




