# pytwb_demo Code Explanation

## Overview
The code in pytwb_demo is organized around Python and XML under behavior, trees, lib directories under src/pytwb_demo/pytwb_demo.
First, let us try to grasp the overall structure by viewing trees/sim.xml. The skeletal structure of trees/sim.xml is as follows.
In this xml code, the following behaviors are executed in order within the Sequence.

```
<root>  
     <BehaviorTree ID="MainTree">  
     <Sequence name="main_loop" memory="[True]">  
         <Commander name="commander"/>  
         <SetLocatio name="set_locations" />  
         <Retry name="find_loop" num_failures="[10]">  
--- action body  
         </Retry>  
     </Sequence>  
     </BehaviorTree>  
</root>  
```

1. Commander: Create a [vector map](https://github.com/RobotSpatialCognition/vector_map) from the given SLAM map and save it in the black board's “geometric_map” tag. Also, create a Messenger object and store it in the “commander” tag as well. The Messenger object has the function of receiving reports from other behaviors and grasping the current state of the robot.
2. SetLocation: From the vector map created above, calculate the coordinates of the candidate points to be traversed to find the coke cans. The candidate points are constructed by the center coordinates of large areas of line-of-sight separated by narrow points within the building. Save the list of candidate points in the “set_list” tag of the black board.
3. Body code surrounded by Retry: Executes multiple behaviors required to move while looking for coke cans with camera.

## Detail of core part
The core part enclosed by Retry executes the following behaviors in order in the Sequence.

1. Parallel(line 8): Surveillance by depth camera and robot locomotion are executed in parallel. Surrounding monitoring is by LookForCoke behavior. When LookForCoke finds a coke can, it saves its world coordinates to the black board's target_pose tag. Robot movement is based on GetLocation and GoToPose behavior. GetLocation dequeues destinations one by one from the list of destination candidates created by SetLocation. GoToPose actually issues a request to Nav2's /navigate_to_pose Action. This Parallel ends when the robot finds a coke can or reaches destination.
2. SetWatchLocation(line 16): The above Parallel ends when the robot finds a coke can, even if it is in the middle of moving. In that case, the robot will move again to the point where it saw the cola can and calculate the posture for performing the confirmation work.
3. Part enclosed by Selector(line 17): Execute Sequence and Retry in order. A Selector is used to prevent Retry from being executed if the Sequence part succeeds. The Sequence part moves to the point calculated by SetWatchLocation, uses Watch behavior, and confirms that the coke can can be seen stably.
4. ScheduleDestination: Based on the estimated position of the cola can and the information on the vector map, calculate the appropriate approach angle and position for picking the cola can and save it in the target_pose tag.
5. GoToPose: Move to the point calculated above.
6. Viewer: You should be able to see the Coke can, so display it.

## More on LookForCoke, Watch, Viewer (behavior/setwatchlocations.py)
LookForCoke and Watch both have the ability to explore cola cans using the depth camera, but they work slightly differently. Both are implemented using a common object TargetSeeker. TargetSeeker receives the normal image and depth image generated by depth camera, and detects the red target from the normal image using OpenCV. When detected, the coordinates are calculated in relative coordinates with the robot as the origin from the depth image and the relative position on the screen, and then converted to world coordinates using tf2. Also, the position of the cola can is marked on the received normal image and displayed again.

Watch also accumulates the locations of the detected coke cans in a PointBag object (lib/pointlib.py), verifies that multiple close measurements are obtained, and then calculates the average of the coordinate measurements. and On the other hand, with LookForCoke, if a cola can is seen even once, it is taken as a measured value.

The Viewer is a simple camera image display behavior that receives and displays images regardless of the presence of a cola can.

## Details of ScheduleDestination(behavior/setwatchlocations.py)
ScheduleDestination calculates the final posture that the robot should reach based on the coke can coordinates detected by the Watch behavior and the vector map. The body of the computation is done by get_approach_pose(lib/geolib.py). First, it calculates the distance for walls within 0.5m. The farthest direction from these walls is calculated, and from there the desired pose of the robot is calculated so as to bring the robot closer to the object for picking action. These calculations can be easily done with the function of the vector map.