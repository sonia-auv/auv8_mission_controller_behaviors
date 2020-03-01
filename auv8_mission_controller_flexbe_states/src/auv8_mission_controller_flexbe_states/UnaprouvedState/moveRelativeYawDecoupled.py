import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget


class MoveRelativeYawTDecoupled(EventState):

    def __init__(self):
        super(MoveRelativeYawTDecoupled, self).__init__(outcomes=['continue', 'failed'])
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_angle_yaw', 1.0, 'Angle change for Yaw'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        try:
            set_local_target(0.0, 0.0, 0.0,
                             0.0, 0.0,self.param_angle_yaw,
                             True, True, True, True, True, False)
            self.target_reached = False
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative angle yaw decoupled = %f' % self.param_angle_yaw)
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'succeeded'

    def on_exit(self, userdata):
        self.target_reach_sub.unregister()