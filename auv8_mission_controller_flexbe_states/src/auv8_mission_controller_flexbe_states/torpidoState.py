import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ActuatorDoActionSrv


class Torpedo(EventState):

    def __init__(self, no_id):
        super(Torpedo, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.do_action = None

        self.action = ActuatorDoActionSrv()
        self.param_id = no_id

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', ActuatorDoActionSrv)
        try:
            self.do_action(self.action.ELEMENT_TORPEDO, int(self.param_id), self.action.ACTION_TORPEDO_LAUNCH)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Torpedo : %i is launch' % int(self.param_id))
        return 'continue'

    def on_exit(self, userdata):
        pass
