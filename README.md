# pytwb_demo
pytwb_demo is

- pytwb:
<p style="text-indent:1em;">
pytwb is a tool for Behavior Tree development and execution using Python. It has a function to execute Behavior Tree description by XML.
</p>

- vector_map:
<p style="text-indent:1em;">
It has a function to output a vector format map by converting SLAM map described in occupied grid format into a representation consisting of geometric straight lines and curves. In addition, it has a function to represent a group of areas in the map, such as rooms and corridors, as Python objects, add properties to it, and save them.
</p>

The main purpose of the pytwb_demo is to provide a sample of a ROS application implemented using the functions of , and this is an example of turtlebot3 searching for cola cans using a depth camera by realsense using a Behavior Tree by Python.

The realization of the execution environment is fully dependent on the GitHub sea-bass' “turtlebot3_behavior_demos”. The simulation environment (demo-world) of the turtlebot3_behavior_demos by Gazebo and Rviz is used as it is. pytwb_demo also uses part of the Python implementation of its Behavior.

So installing pytwb_demo starts with installing turtlebot3_behavior_demos, because pytwb_demo contains only the execution environment of Behavior Tree, and does not contain the part that executes it using turtlebot3. 
The demo as a whole corresponds to the pre-stage movement work to start the picking work. Inside a building with a complex shape, it automatically enumerates positions with good visibility, searches for cola cans using a camera while the robot moves to those points, and if it finds a cola can, it stops and looks again. A series of operations are implemented to confirm the position, measure the coordinates, calculate the position coordinates of the robot suitable for picking cola cans, and move to that point.
Here, by using pytwb and vector_map,
- Developing Incremental Behavior Tree Applications
- Analyze the map and choose a good vantage point
- Locating objects on the move and calculating their coordinates
- Calculate the coordinates of the work place relative to the object without interfering with the surroundings

etc. is implemented.