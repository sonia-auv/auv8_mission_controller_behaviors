import rospy

from flexbe_core import EventState, Logger
from proc_control.srv import SetControlMode, SetControlModeRequest, SetDecoupledTarget


class Switch(EventState):

    def __init__(self, mode):
        super(Switch, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.set_mode = None

        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}

        self.param_mode = mode

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)
        try:

            self.set_mode(self.mode_dic[str(int(self.param_mode))])
            if self.param_mode == 0:
                rospy.wait_for_service('/proc_control/set_local_decoupled_target')
                set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)
                set_local_target(0.0,
                                 0.0, 0.0, 0.0, 0.0, 0.0,
                                 False, False, False, True, True, False)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

    def execute(self, userdata):
        rospy.loginfo('Switch : %i is executed' % int(self.param_mode))
        return 'continue'

    def on_exit(self, userdata):
        pass
