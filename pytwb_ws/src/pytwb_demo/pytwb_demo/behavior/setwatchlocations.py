import math
from sympy import Point, symbols, solve

import py_trees
from pytwb.common import behavior

from lib.geolib import get_pose, get_approach_pose

#
## schedule robot pose when a target object(coke) is found by using vector map
#
# location list for error recovery
@behavior
class SetWatchLocations(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name, debug=False):
        super(SetWatchLocations, self).__init__(name)
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb
        self.debug = debug

        world = bb.get('geometric_map')
        r = world.get_regions()[0]
        self.region = r
        pose_list = []
        for sr in r.get_subregions():
            c = sr.get_weight_center()
            pose_list.append([c.x, c.y, 0])
        bb.set("watch_list", pose_list)
    
    def initialise(self) -> None:
        # clear glanced_point
        self.target = None
        if not self.bb.exists("glanced_point"): return
        self.target = self.bb.get("glanced_point")
        if not self.target: return
        self.bb.set("glanced_point", None)
        self.logger.info("SetWatchLocations")

        # undo target pose list to prepare in case of watch failure
        target_pose = self.bb.get("target_pose")
        pose_list = self.bb.get("pose_list")
        pose_list.insert(0, target_pose)

    def update(self):
        # if nothing found, return FILURE
        if not self.target: return py_trees.common.Status.FAILURE

        # start watching phase
        target_point = Point(self.target.x, self.target.y)
        view_point = self.target.transform.translation
        observation_point = Point(view_point.x, view_point.y)
        target_pose = list(get_pose(observation_point, target_point))
        if self.debug:
            print(f'target_pose x:{target_pose[0]}, y:{target_pose[1]}, theta:{math.degrees(target_pose[2])}')
            print(f'target_point x:{float(target_point.x)}, y:{float(target_point.y)}')
        watch_list = []
        self.bb.set("target_pose", target_pose.copy())
        self.bb.set("watch_origin", target_pose.copy())
        for _ in range(3):
            target_pose[2] += math.pi / 3 * 2
            watch_list.append(target_pose.copy())
        self.bb.set("watch_list", watch_list)
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

## decide location to visit for recovery when watching fails
## derived from "turtlebot3_behavior_demo"
@behavior
class GetWatchLocation(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name):
        super(GetWatchLocation, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        """ Checks for the status of the navigation action """
        pose_list = self.bb.get("watch_list")
        if len(pose_list) == 0:
            self.logger.info("No objects found by watching")
            return py_trees.common.Status.FAILURE
        else:
            target_pose = pose_list.pop(0)
            self.logger.info(f"Selected location x:{target_pose[0]},x:{target_pose[1]}")
            self.bb.set("target_pose", target_pose)
            return py_trees.common.Status.SUCCESS

@behavior
class GetGlancedLocation(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name):
        super(GetGlancedLocation, self).__init__(name)
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb

    def initialise(self) -> None:
        # clear glanced_point
        self.target = None
        if not self.bb.exists("glanced_point"): return
        object_point_bag = self.bb.get("glanced_point")
        if not object_point_bag: return
        self.object_point = Point(object_point_bag.x, object_point_bag.y)
        self.bb.set("glanced_point", None)
        self.origin = self.bb.get("watch_origin")
        self.observation_point = Point(self.origin[0], self.origin[1])

        # undo target pose list to prepare in case of watch failure
        target_pose = self.bb.get("target_pose")
        pose_list = self.bb.get("pose_list")
        pose_list.insert(0, target_pose)

    def update(self):
        # if nothing found, return FILURE
        if not self.target: return py_trees.common.Status.FAILURE

        # calculate pose
        target_pose = get_pose(self.observation_point, self.object_point)
        self.bb.set("target_pose", target_pose)
        return py_trees.common.Status.SUCCESS

@behavior
class ScheduleDestination(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ScheduleDestination, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()
    
    def update(self):
        dest = self.bb.get('found_point')
        if not dest:
            return py_trees.common.Status.FAILURE
        world = self.bb.get("geometric_map")
        region = world.get_regions()[0]
        pose = get_approach_pose(region, dest, dest.location)
#        dp = Point(dest.x, dest.y)
#        pp = Point(pose[0], pose[1])
#        print(f'final target _x:{dest.last_point._x},_y:{dest.last_point._y},dist:{float(dp.distance(pp))}')
        self.bb.set('target_pose', pose)
        self.logger.info(f'selected target [x: {dest.x}, y: {dest.y}]')
        self.logger.info(f'selected pose [x: {pose[0]}, y: {pose[1]}, theta: {math.degrees(pose[2])}]')
        if self.bb.exists('commander'):
            self.bb.get('commander').report(self.name, dest, pose)
        return py_trees.common.Status.SUCCESS
