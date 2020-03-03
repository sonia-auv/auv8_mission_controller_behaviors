import rospy

from flexbe_core import EventState, Logger
from proc_actuators.srv import cmActionSrv, cmActionSrvRequest


class Torpido(EventState):
    """
            -- no_id uint8      Enter 1:Side Port 2:Side StarBoard

            <= continue			Indicates that the camera started
            <= failed			Indicates that the camera didn't stated

    """
    def __init__(self, no_id):
        super(Torpido, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.do_action = None

        self.action = cmActionSrvRequest()
        self.action_dic = {'1': self.action.SIDE_PORT, '2': self.action.SIDE_STARBOARD}

        self.param_id = no_id

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', cmActionSrv)
        try:

            self.do_action(self.action.ELEMENT_TORPEDO, self.action_dic[str(int(self.param_id))], self.action.ACTION_TORPEDO_ARMED)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Torpido : %i is launch' % int(self.param_id))
        return 'continue'

    def on_exit(self, userdata):
        pass