import py_trees
import transforms3d

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

from pytwb.common import behavior

#
## behaviors in this file is originated by "turtlebot3_behavior_demo"
#
@behavior
class GetLocation(py_trees.behaviour.Behaviour):
    desc = 'Gets a location pose from the pose list'

    def __init__(self, name):
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
            if not target_pose:
                return py_trees.common.Status.FAILURE
            self.logger.info(f"Selected location x:{target_pose[0]},x:{target_pose[1]}")
            self.bb.set("target_pose", target_pose)
            if self.bb.exists('commander'):
                self.bb.get('commander').report(self.name, target_pose)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")

#
## worker for navigation
## the independent object is allocated to cope with repeated invocation and interruption of GoToPose
#
class MoveWorker:
    instance = None
    busy = False
    node = None

    @classmethod
    def get(cls, node):
        if not cls.instance or node != cls.node:
            cls.instance = MoveWorker(node)
            cls.node = node
        cls.instance.busy = True
        return cls.instance
    
    def free(self):
        self.busy = False

    def __init__(self, node):
        self.node = node
        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.client.wait_for_server()

    def initialise(self, pose):
        """ Sends the initial navigation action goal """
        self.pose = pose
        self.goal_status = None
        x, y, theta = pose
        self.goal = self.create_move_base_goal(x, y, theta)
        self.send_goal_future = self.client.send_goal_async(self.goal)
        self.send_goal_future.add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        res = future.result()
        if res is None or not res.accepted:
            return
        future = res.get_result_async()
        future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # If there is a result, we consider navigation completed and save the
        # result code to be checked in the `update()` method.
        self.goal_status = future.result().status

    def update(self):
        return self.goal_status

    def create_move_base_goal(self, x, y, theta):
        """ Creates a MoveBaseGoal message from a 2D navigation pose """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal.pose.pose.orientation.w = quat[0]
        goal.pose.pose.orientation.x = quat[1]
        goal.pose.pose.orientation.y = quat[2]
        goal.pose.pose.orientation.z = quat[3]
        return goal

@behavior
class GoToPose(py_trees.behaviour.Behaviour):
    desc = 'actual move action'
    
    def __init__(self, name, node):
        super(GoToPose, self).__init__(name)
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()
    
    def initialise(self):
        self.worker = MoveWorker.get(self.node)
        target_pose = None
        # Check if there is a pose available in the blackboard
        if self.bb.exists("target_pose"):
            target_pose = self.bb.get("target_pose")
        self.pose = target_pose
        x, y, theta = self.pose
        self.logger.info(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
        if not target_pose:
            self.worker.free()
            return
        self.worker.initialise(target_pose)
    
    def update(self):
        if not self.pose: return py_trees.common.Status.INVALID
        # If there is a result, we can check the status of the action directly.
        # Otherwise, the action is still running.
        goal_status = self.worker.update()
        if goal_status is not None:
            if goal_status == GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.RUNNING
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        if not self.pose: return
        self.pose = None
        self.worker.free()
