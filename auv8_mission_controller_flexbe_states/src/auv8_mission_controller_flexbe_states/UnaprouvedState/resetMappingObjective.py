import rospy

from flexbe_core import EventState, Logger
from proc_mapping.srv import ObjectiveReset


class ResetMappingObjective(EventState):

    def __init__(self):
        super(ResetMappingObjective, self)

    def define_parameters(self):
        self.parameters.append(Parameter('param_objective_type', 2, 'Type'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_mapping/objective_reset')
        self.objective_reset = rospy.ServiceProxy('/proc_mapping/objective_reset', ObjectiveReset)

        try:
            self.objective_reset(self.param_objective_type)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        typeName = "All" if self.param_objective_type == -1 else "Buoy" if self.param_objective_type == 0 else "Fence" if self.param_objective_type == 1 else "Pinger" if self.param_objective_type == 2 else "UNKNOWN"
        
        rospy.loginfo('Reset objective %d = %s', self.param_objective_type, typeName)

    def execute(self, userdata):
        return 'succeeded'

    def end(self):
        pass
