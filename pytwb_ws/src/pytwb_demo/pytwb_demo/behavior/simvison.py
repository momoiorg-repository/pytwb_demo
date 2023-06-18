import math
import cv2

import py_trees
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pytwb.common import behavior

from lib.pointlib import TransformHelper, PointEx, PointBag
from lib.simlib import point_coordinate, find_coke

#
## camera driver
#
class TargetSeeker:
    instance = None
    instance_is_free = False
    node = None

    @classmethod
    def get_instance(cls, node):
        if not cls.node or cls.node != node:
            cls.instance = TargetSeeker(node)
            cls.node = node
        elif not cls.instance:
            cls.instance = TargetSeeker(node)
        else:
            cls.instance.initialize()
        cls.isinstance_is_free = False
        return cls.instance
    
    def free(self):
        self.instance_is_free = True
    
    def __init__(self, node):
        self.bridge = CvBridge()
        self.node = node
        self.image_sub = node.create_subscription(Image,"/intel_realsense_r200_depth/image_raw",self.pic_callback,10)
        self.depth_sub = node.create_subscription(Image,"/intel_realsense_r200_depth/depth/image_raw",self.depth_callback,10)
        self.bag_points = []
        self.cand_points = []
        self.helper = TransformHelper(node)
        cv2.startWindowThread()
        self.coke_center = None
        self.found = False


    def pic_callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return        
        coke_loc = find_coke(self.cv_image)
        if not coke_loc:
            return
        else:
            self.coke_center = coke_loc
            self.found = True

    def depth_callback(self, data):
        if not self.found:
            if hasattr(self, 'cv_image'):
#                cv2.destroyWindow('camera')
                cv2.imshow('camera', self.cv_image)
                cv2.waitKey(1)
            return
        self.found = False
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)
            return
        center = self.coke_center
        x = center[0] # by pic cell
        y = center[1] # by pic cell
        distance = depth_image[y][x]
        if math.isinf(distance): return
        color  = (0, 255, 0)
        radius = 20
        cv2.circle(self.cv_image, center, radius, color)
#        cv2.destroyWindow('camera')
        cv2.imshow('camera', self.cv_image)
        cv2.waitKey(1)
        x = depth_image.shape[1] / 2 - x # horizontal axis
        y -= depth_image.shape[0] / 2 # virtical axis
        loc = point_coordinate(x, distance) # convert pic cell unit into local coordinate (base_link)
        point = PointEx(loc)
        point.v_x = x
        point.distance = distance
        point.type = "coke"
        self.bag_points.append(point)
    
    def initialize(self):
        self.bag_points = []
        self.cand_points = []
        
    def get_candidate(self):
        if len(self.bag_points) < 1: return None
        target = self.bag_points[-1]
        if not self.helper.transform(target):
            if not self.helper.transform(target):
                return None
        self.bag_points = []
        return target
    
    def update(self):
        if len(self.bag_points) < 1: return
        self.helper.prepare()
        for point in self.bag_points:
            self.helper.conv(point)
            if point.valid:
                self.register_flist(point)
        self.bag_points = []
    
    def register_flist(self, point):
        cand = None
        min_d = 0.01
        for found in self.cand_points:
            d = (found.x-point.x)**2 + (found.y-point.y)**2 + (found.z-point.z)**2
            if d >= min_d: continue
            cand = found
            min_d = d
        if not cand:
            cand = PointBag(point)
            cand.type = point.type
            self.cand_points.append(cand)
        else:
            cand.append(point)


@behavior
class Viewer(py_trees.behaviour.Behaviour):
    desc = 'just camera snap shot'

    def __init__(self, name, node, mode="permanent"):
        super(Viewer, self).__init__(name)
        self.mode = mode
        self.displayed = []
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
    
    def initialise(self):
        self.seeker = TargetSeeker.get_instance(self.node)

    def update(self):
        if self.mode == 'one_shot':
            return py_trees.common.Status.SUCCESS
        elif self.mode == 'permanent':
            return py_trees.common.Status.RUNNING
        if len(self.seeker.bag_points) < 1: return py_trees.common.Status.RUNNING
        self.seeker.update()
        for target in self.seeker.cand_points:
            point = target.last_point
            print(f'_x:{point._x}, _y:{point._y}, x:{target.x}, y:{target.y}')
        if len(self.seeker.cand_points) < 1:
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
#        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.seeker.free()

@behavior
class LookForCoke(py_trees.behaviour.Behaviour):
    desc = 'decide target location by a single glance'

    def __init__(self, name, node, debug=False):
        super(LookForCoke, self).__init__(name)
        self.target = None
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
        self.debug = debug

    def initialise(self):
        self.seeker = TargetSeeker.get_instance(self.node)
        self.logger.info("start watching for coke")

    def update(self):
        target = self.seeker.get_candidate()
        if not target: return py_trees.common.Status.RUNNING
        self.bb.set('glanced_point', target)
        self.target = target
        if self.debug:
            self.logger.info(f"local addr _x:{target._x}, _y:{target._y}, off:{target.v_x}, distance:{target.distance}")
            self.logger.info(f"found at x:{target.x} y:{target.y}")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        self.seeker.free()
        if self.target:
            self.logger.info(f"found at x:{self.target.x} y:{self.target.y} new_status:{new_status}")
        else:
            self.logger.info(f'no coke found, new status:{new_status}')

@behavior
class Watch(py_trees.behaviour.Behaviour):
    desc = 'watch target repeatedly to get reliable location data'

    def __init__(self, name, node, debug=False):
        super(Watch, self).__init__(name)
        self.target = None
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
        self.debug = debug

    def initialise(self):
        self.seeker = TargetSeeker.get_instance(self.node)
        self.logger.info("watching target")
        self.loop = 0

    def update(self):
        self.loop += 1
        if len(self.seeker.bag_points) < 1:
            if self.loop > 10:
               return py_trees.common.Status.FAILURE
            else:
               return py_trees.common.Status.RUNNING
        self.seeker.update()
        max_count = 0
        target = None
        for c in self.seeker.cand_points:
            if c.count > max_count:
                target = c
                max_count = c.count
        if max_count < 10:
            if self.loop > 5: return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        self.bb.set('found_point', target)
        self.target = target
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.seeker.free()
        if self.target:
            self.logger.info(f"found at x:{self.target.x} y:{self.target.y}")
            if self.debug:
                p = self.target.last_point
                self.logger.info(f"local addr _x:{p._x}, _y:{p._y}, off:{p.v_x}, distance:{p.distance}")
        else:
            self.logger.info('no coke found')
