import rospy

from flexbe_core import EventState, Logger
from proc_control.srv import SetPositionTarget


class MoveSpeed(EventState):

    def __init__(self, speed,yaw):
        super(MoveSpeed, self).__init__(outcomes=['continue', 'failed'])
        self.set_local_target = None

        self.target_reached = False

        self.param_speed_x = speed
        self.param_orientation_yaw = yaw

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  self.param_orientation_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set sub speed = %f' % self.param_speed_x)
        rospy.loginfo('Set global orientation yaw = %f' % self.param_orientation_yaw)

    def execute(self, userdata):
        if self.target_reached:
            return 'continue'

    def on_exit(self, userdata):
        pass
