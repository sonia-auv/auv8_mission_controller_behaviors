import rospy

from flexbe_core import EventState, Logger
from proc_navigation.srv import SetDepthOffset


class SetDepthOfSet(EventState):

    def __init__(self):
        super(SetDepthOfSet, self).__init__(outcomes=['continue', 'failed'])
        self.set_initial_depth = None

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_navigation/set_depth_offset', timeout=2)
        self.set_initial_depth = rospy.ServiceProxy('/proc_navigation/set_depth_offset', SetDepthOffset)
        try:
            self.set_initial_depth()
        except rospy.ServiceException as err:
            rospy.logerr(err)
	    return 'failed'

    def execute(self, userdata):
        return 'continue'

    def on_exit(self, userdata):
        pass
