import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget
from nav_msgs.msg import Odometry


class moveGlobalYaw(EventState):

    def __init__(self, yaw):
        super(moveGlobalYaw, self).__init__(outcomes=['continue', 'failed'])

        self.position = None
        self.orientation = None
        self.target_reached = False
        self.param_heading = yaw

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def current_position_cb(self, position):
        self.position = position.pose.pose.position
        self.orientation = position.pose.pose.orientation

    def wait_until_position_is_get(self):
        while not (self.position and self.orientation):
            pass
        return

    def on_enter(self, userdata):

        rospy.wait_for_service('/proc_control/set_global_target')
        set_global_target = rospy.ServiceProxy('/proc_control/set_global_target', SetPositionTarget)

        self.current_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.current_position_cb)

        self.position = None
        self.orientation = None
        self.wait_until_position_is_get()

        print self.position.z

        try:
            response = set_global_target(self.position.x,
                                         self.position.y,
                                         self.position.z,
                                         self.orientation.x,
                                         self.orientation.y,
                                         self.param_heading)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'failed'

        rospy.loginfo('Set position Yaw = %f' % self.param_heading)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'continue'

    def on_exit(self, userdata):
        self.target_reach_sub.unregister()
        self.current_position.unregister()
