import py_trees

from pytwb.common import behavior

#
## decide visit points based on information from vector map
#
@behavior
class SetLocations(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name):
        super(SetLocations, self).__init__(name)
        bb = py_trees.blackboard.Blackboard()
        self.bb = bb

        world = bb.get('geometric_map')
        r = world.get_root_region()
        self.region = r
        pose_list = []
        for sr in r.get_subregions():
            c = sr.get_weight_center()
            pose_list.append([float(c.x), float(c.y), 0])
        bb.set("pose_list", pose_list)

    def update(self):
        return py_trees.common.Status.SUCCESS
