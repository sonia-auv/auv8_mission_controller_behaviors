import rospy

from flexbe_core import EventState, Logger
from controller_mission.srv import CurrentMission, CurrentMissionResponse
class Log(EventState):

    def __init__(self):
        super(Log, self).__init__(outcomes=['continue', 'failed'])
        self.do_action = None

    def define_parameters(self):
        self.parameters.append(Parameter('param_start', 1, 'Started(1) or finished(2)'))
        self.parameters.append(Parameter('param_message', 'The message', 'The message to log'))


    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        rospy.wait_for_service('/mission_executor/current_mission')
        self.do_action = rospy.ServiceProxy('/mission_executor/current_mission', CurrentMission)
        pass

    def execute(self, userdata):

        try:

            mission_name = self.do_action().mission
            rospy.loginfo('Mission: \'%s\' Message: \'%s\'  State: \'%s\'' % (mission_name, self.param_message, 'started' if self.param_start == 1 else 'finished'))

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        return 'succeeded'

    def on_exit(self, userdata):
        pass