import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetDecoupledTarget


class MoveRelativeZDecoupled(EventState):

    def __init__(self):
        super(RotateYawRelative, self)
        self.target_reached = False

    def define_parameters(self):
        self.parameters.append(Parameter('param_distance_z', 1.0, 'Distance to travel'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def on_enter(self, userdata):
        self.target_reached = 0
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        try:
            set_local_target(0.0, 0.0,self.param_distance_z,
                             0.0, 0.0, 0.0,
                             True, True, False, True, True, True)

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)
        rospy.loginfo('Set relative decoupled position z = %f' % self.param_distance_z)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'succeeded'

    def end(self):
        self.target_reach_sub.unregister()