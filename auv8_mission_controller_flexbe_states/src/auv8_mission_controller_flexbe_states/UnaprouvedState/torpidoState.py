import rospy

from flexbe_core import EventState, Logger
from proc_actuators.srv import cmActionSrv, cmActionSrvRequest


class Torpido(EventState):

    def __init__(self):
        super(Torpido, self)
        self.start_time = None
        self.do_action = None

        self.action = cmActionSrvRequest()
        self.action_dic = {'1': self.action.SIDE_PORT, '2': self.action.SIDE_STARBOARD}
        self.arme_dic = {'1': self.action.ACTION_TORPEDO_ARMED, '2': self.action.ACTION_DROPPER_LAUNCH}

    def define_parameters(self):
        self.parameters.append(Parameter('param_id', 1, 'Torpido id'))
        self.parameters.append(Parameter('param_launch', 1, 'Launch'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_actuators/cm_action_srv')
        self.do_action = rospy.ServiceProxy('/proc_actuators/cm_action_srv', cmActionSrv)
        try:

            self.do_action(self.action.ELEMENT_TORPEDO, self.action_dic[str(int(self.param_id))], self.arme_dic[str(int(self.param_launch))])

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Torpido : %i is launch' % int(self.param_id))
        return 'succeeded'

    def end(self):
        pass