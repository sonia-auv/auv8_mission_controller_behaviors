import rospy

from flexbe_core import EventState, Logger
from sonia_common.srv import ActuatorDoActionSrv


class Droppers(EventState):

    def __init__(self, param):
        super(Droppers, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.do_action = None

        self.action = ActuatorDoActionSrv()
        self.param_id = param

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', ActuatorDoActionSrv)
        try:

            self.do_action(self.action.ELEMENT_DROPPER,int(self.param_id), self.action.ACTION_DROPPER_LAUNCH)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Dropper : %i is launch' % int(self.param_id))
        return 'continue'

    def end(self, userdata):
        pass
