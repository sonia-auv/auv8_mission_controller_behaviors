import rospy

from flexbe_core import EventState, Logger

from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry
import math


class MoveZByStep(EventState):

    def __init__(self, depth, step):
        super(MoveZByStep, self).__init__(outcomes=['continue', 'failed'])

        self.position = None
        self.orientation = None
        self.target_reached = False
        self.current_position = None

        self.move_queue = None

        self.set_global_target = None
        self.target_reach_sub = None

        self.depth = depth
        self.step = step

    def target_reach_cb(self, data):

        if data.target_is_reached > 0 and len(self.move_queue) > 0:
            self.move_z(self.move_queue.pop(0))
        else:
            self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position
        self.orientation = position.pose.pose.orientation

    def wait_until_position_is_get(self):
        while not self.position:
            pass
        return

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_global_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.move_queue = []

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.wait_until_position_is_get()

        delta_z = self.depth - self.position.z

        if delta_z > 0:
            nb_step = int(math.floor(delta_z/self.step))
            mod = delta_z % self.step
            sign = 1
        else:
            nb_step = int(math.ceil(delta_z/self.step))
            mod = delta_z % self.step - self.step
            sign = -1

        abs_nb_step = abs(nb_step)

        for i in range(1, abs_nb_step + 1):
            self.move_queue.append(self.position.z + self.step * i * sign)

        if mod != 0:
            if abs_nb_step > 0:
                self.move_queue.append(self.move_queue[-1] + mod)
            else:
                self.move_queue.append(self.position.z + mod)

        print(self.move_queue)

        self.target_reached = False

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.move_z(self.move_queue.pop(0))

    def move_z(self, dist):
        try:
            self.set_global_target(self.position.x,
                                   self.position.y,
                                   dist,
                                   0,
                                   0,
                                   self.orientation.z)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'continue'

    def on_exit(self, userdata):
        self.target_reach_sub.unregister()
        self.current_position.unregister()
