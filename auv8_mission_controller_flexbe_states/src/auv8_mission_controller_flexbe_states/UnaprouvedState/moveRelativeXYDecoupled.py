import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget


class MoveRelativeXYDecoupled(EventState):

    def __init__(self):
        super(MoveRelativeXYDecoupled, self).__init__(outcomes=['continue', 'failed'])
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_x', 1.0, 'Distance to travel'))
        self.parameters.append(Parameter('param_distance_y', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)
        self.target_reached = False
        try:
            set_local_target(self.param_distance_x,
                             self.param_distance_y, 0.0, 0.0, 0.0, 0.0,
                                  False, False, True, True, True, True)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative decoupled position x = %f' % self.param_distance_x)
        rospy.loginfo('Set relative decoupled position y = %f' % self.param_distance_y)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()