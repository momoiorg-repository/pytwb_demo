import py_trees

from pytwb.common import behavior

@behavior
class SetLocations(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name, location_file):
        super(SetLocations, self).__init__(name)
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb

        world = bb.get('geometric_map')
        r = world.get_root_region()
        self.region = r
        pose_list = []
        for p in r.subregion_centers:
            pose_list.append([p[0], p[1], 0])
        bb.set("pose_list", pose_list)

    def update(self):
        return py_trees.common.Status.SUCCESS


@behavior
class GetLocation(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name, location_file):
        super(GetLocation, self).__init__(name)
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        """ Checks for the status of the navigation action """
        pose_list = self.bb.get("pose_list")
        if len(pose_list) == 0:
            self.logger.info("No locations available")
            return py_trees.common.Status.FAILURE
        else:
            target_pose = pose_list.pop()
            self.logger.info(f"Selected location x:{target_pose[0]},y:{target_pose[1]}")
            self.bb.set("target_pose", target_pose)
            if self.bb.exists('commander'):
                self.bb.get('commander').report(self.name, target_pose)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

