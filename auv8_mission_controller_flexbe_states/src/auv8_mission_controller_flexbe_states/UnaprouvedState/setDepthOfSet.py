import rospy

from flexbe_core import EventState, Logger
from proc_navigation.srv import SetDepthOffset


class SetDepthOfSet(EventState):

    def __init__(self):
        super(RotateYawRelative, self)
        self.set_initial_depth = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_navigation/set_depth_offset', timeout=2)
        self.set_initial_depth = rospy.ServiceProxy('/proc_navigation/set_depth_offset', SetDepthOffset)
        try:
            self.set_initial_depth()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def execute(self, userdata):
        return 'succeeded'

    def end(self):
        pass
