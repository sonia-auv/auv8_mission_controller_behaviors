import rospy

from flexbe_core import EventState, Logger

from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget


class RotateYawRelative(EventState):

    def __init__(self, relativeYaw):
        super(RotateYawRelative, self).__init__(outcomes=['continue', 'failed'])
        self.set_local_target = None
        self.target_reach_sub = None

        self.actual_position_x = 0.0
        self.actual_position_y = 0.0
        self.just_one_time = 0
        self.target_reached = False

        self.relativeYaw = relativeYaw

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        try:
            self.set_local_target(0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  self.relativeYaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position yaw = %f' % self.relativeYaw)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'continue'

    def on_exit(self, userdata):
        self.target_reach_sub.unregister()
