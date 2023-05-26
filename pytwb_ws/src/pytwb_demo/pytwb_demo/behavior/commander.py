import py_trees
from pytwb.common import behavior
from vector_map import get_map_ROS, SimulationSpace

class Messenger:
    def __init__(self) -> None:
        self.bb = py_trees.blackboard.Blackboard()
        self.bb.set("commander", self)
        world = get_map_ROS('./pytwb_demo/pytwb_demo/resource/map')
        self.world = world
        r = world.get_root_region()
        ss = SimulationSpace(r)
        r.ss = ss
        self.ss = ss
        self.bb.set("geometric_map", world)
        self.dispatcher = {
            "get_loc": self.handle_get_loc,
            "schedule_final_target": self.handle_sched
        }
        self.state = 'preparing'
        
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
    def __init__(self, name):
        super(Commander, self).__init__(name)
        Messenger()
        
    def update(self):
        return py_trees.common.Status.SUCCESS

