import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from proc_control.msg import TargetReached, set_global_target_action
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry
import math


class MoveZByStepActionLib(EventState):

    def __init__(self, depth, step):
        super(MoveZByStepActionLib, self).__init__(outcomes=['continue', 'failed'])

        self.position = None
        self.orientation = None
        self.target_reached = False
        self.current_position = None

        self.move_queue = None

        self.target_reach_sub = None

        self.depth = depth
        self.step = step

        self.set_global_target = '/proc_control/set_global_target'
        self._client = ProxyActionClient({self._set_global_target: set_global_target_action})  # pass required clients as dict (topic: type)

        # It may happen that the action client fails to send the action goal.
        self._error = False

    def execute(self, userdata):
        if self._error:
            return 'command_error'
            # Check if the action has been finished
        if self._client.has_result(self.set_global_target):
            result = self._client.get_result(self.set_global_target)
            self.target_reached = result.target_reached
        if self.target_reached > 0:
            return 'continue'

    def on_enter(self, userdata):

        self.move_queue = []

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.wait_until_position_is_get()

        delta_z = self.param_z - self.position.z

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

    def move_z(self, depth, step):
        try:
            self.set_global_target(self.position.x,
                                   self.position.y,
                                   depth,
                                   0,
                                   0,
                                   self.orientation.z)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        print('Set position z = {0} with steps of {1}' .format(depth, step))

    def on_exit(self, userdata):
        if not self._client.has_result(self.set_global_target):
            self._client.cancel(self.set_global_target)
            Logger.loginfo('Cancelled active action goal.')

        self.target_reach_sub.unregister()
        self.current_position.unregister()
