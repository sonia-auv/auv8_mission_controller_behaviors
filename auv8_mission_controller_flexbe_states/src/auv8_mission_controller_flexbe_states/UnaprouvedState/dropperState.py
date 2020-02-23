import rospy

from flexbe_core import EventState, Logger
from proc_actuators.srv import cmActionSrv, cmActionSrvRequest


class Droppers(EventState):

    def __init__(self):
        super(RotateYawRelative, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.do_action = None

        self.action = cmActionSrvRequest()
        self.action_dic = {'1': self.action.SIDE_PORT, '2': self.action.SIDE_STARBOARD}

    def define_parameters(self):
        self.parameters.append(Parameter('param_id', 1, 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', cmActionSrv)
        try:

            self.do_action(self.action.ELEMENT_DROPPER, self.action_dic[str(int(self.param_id))], self.action.ACTION_DROPPER_LAUNCH)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Dropper : %i is launch' % int(self.param_id))
        return 'succeeded'

    def end(self):
        pass
