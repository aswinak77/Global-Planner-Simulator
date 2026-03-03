GLOBAL PLANNER SIMULATOR
_________________________

This project implements a Global Planner Simulator using ROS1 which generated a 2D Grid world with a grid size of 100 and resolution of 0.05. It populates random obstacle and random start and goal pose in each launch. It finds the shortest path from the start to end node using A* Algorithm.

This planner uses two node :

- Grid map node: publishes /map , /start and /goal
- Global planner node: Computes the shortest path form start to goal pose using A*  Algorithm. It subscribes to the three topic that are published by the Grid map node and publishes a /global_path which constructs the path from start to goal.


How to run
___________

1. Build the Package
2. Launch the file simulator.launch 

roslaunch global_planner_sim simulator.launch

3. Add the topic /map, /start and /goal, you can see the randomly generated obstacles, start and goal positions

4. Now add the topic /global_path to visualize the path from start to the goal point.
