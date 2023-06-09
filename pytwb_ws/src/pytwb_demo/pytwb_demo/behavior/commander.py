import numpy as np

import py_trees
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from pytwb.common import behavior
from vector_map import get_map, get_map_ROS, SimulationSpace

# keep track of robot status
# by creating vector map and receiving reports from other behaviors
class Messenger:
    def __init__(self, map_file, node) -> None:
        qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        self.image_sub = node.create_subscription(
            OccupancyGrid,
            "/map",self.map_callback,
            qos_profile)

        self.bb = py_trees.blackboard.Blackboard()
        self.bb.set("commander", self)
        self.world = None
        self.ss = None
#        world = get_map_ROS(map_file) # read map and create vector map
#        self.world = world
#        r = world.get_root_region()
#        ss = SimulationSpace(r) # show map display
#        ss.show_outer_boundary()
#        r.ss = ss
#        self.ss = ss
#        self.bb.set("geometric_map", world)
        self.dispatcher = {
            "get_loc": self.handle_get_loc,
            "schedule_final_target": self.handle_sched
        }
        self.state = 'preparing'
    
    def map_callback(self, data):
        width = data.info.width #map width, from nav_msgs/msg/MapMetaData
        height = data.info.height #map height, from nav_msgs/msg/MapMetaData
        img_data = np.array(data.data).reshape(height, width)
        resolution = data.info.resolution
        o = data.info.origin.position
        origin = (o.x, o.y)
        self.world = get_map(img_data, resolution, origin)

    # receive status reports from other behaviors    
    def report(self, name, *arg):
        driver = self.dispatcher.get(name)
        if driver: driver(name, arg)
        else: print(f'unknown report: {name}')
    
    def handle_get_loc(self, name, arg):
        loc = arg[0]
        x = loc[0]
        y = loc[1]
        self.ss.create_mark(x, y, mark='x')
        self.state = 'searching'
    
    def handle_sched(self, name, arg):
        pose = arg[1]
        dest = arg[0]
        self.ss.create_mark(dest.x, dest.y, mark='@', color='red')
        self.ss.create_mark(pose[0], pose[1], mark='*')
        self.state = 'approaching'
        
@behavior
class Commander(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name, map_file, node):
        super(Commander, self).__init__(name)
        self.messenger = Messenger(map_file, node)
        
    def update(self):
        if not self.messenger.world:
            return py_trees.common.Status.RUNNING
        if not self.messenger.ss:
            r = self.world.get_root_region()
            ss = SimulationSpace(r) # show map display
            ss.show_outer_boundary()
            r.ss = ss
            self.ss = ss
            self.bb.set("geometric_map", self.world)
        return py_trees.common.Status.SUCCESS

