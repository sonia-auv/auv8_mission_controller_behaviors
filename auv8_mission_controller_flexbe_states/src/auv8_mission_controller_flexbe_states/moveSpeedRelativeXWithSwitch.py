import rospy

from flexbe_core import EventState, Logger
from proc_control.srv import SetPositionTarget, SetControlMode, SetControlModeRequest
from nav_msgs.msg import Odometry
import math


class MoveRelativeSpeedXWithSwitch(EventState):

    def __init__(self, distance, speed, yaw):
        super(MoveRelativeSpeedXWithSwitch, self).__init__(outcomes=['continue', 'failed'])
        self.set_local_target = None
        self.odom = None
        self.first_position = None
        self.position = None

        self.mode = SetControlModeRequest()
        self.mode_dic = {'0': self.mode.PositionModePID, '1': self.mode.PositionModePPI, '2': self.mode.VelocityModeB}

        self.param_distance_x = distance
        self.param_speed_x = speed
        self.param_orientation_yaw = yaw

    def on_enter(self, userdata):

        rospy.wait_for_service('/proc_control/set_control_mode')
        self.set_mode = rospy.ServiceProxy('/proc_control/set_control_mode', SetControlMode)
        try:

            self.set_mode(self.mode_dic[str(int(2))])

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.wait_for_service('/proc_control/set_local_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_target', SetPositionTarget)

        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

        self.wait_until_position_is_get()

        try:
            self.set_local_target(self.param_speed_x,
                                  0.0,
                                  self.position.z,
                                  0.0,
                                  0.0,
                                  self.param_orientation_yaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set distance to travel x = %f' % self.param_distance_x)
        rospy.loginfo('Set relative speed x = %f' % self.param_speed_x)
        rospy.loginfo('Set global orientation yaw = %f' % self.param_orientation_yaw)

    def wait_until_position_is_get(self):
        while not self.first_position:
            pass
        return

    def odom_cb(self, odom_data):
        if self.first_position is None:
            self.first_position = odom_data.pose.pose.position

        self.position = odom_data.pose.pose.position

    def execute(self, userdata):
        # Check if both position are set. If not, check pass
        if self.first_position is None or self.position is None:
            return

        x = self.first_position.x - self.position.x
        y = self.first_position.y - self.position.y

        distance = math.sqrt(x * x + y * y)

        if distance >= self.param_distance_x:
            try:
                self.set_local_target(0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      self.param_orientation_yaw)
            except rospy.ServiceException as exc:
                rospy.loginfo('Service did not process request: ' + str(exc))
            return 'continue'

    def on_exit(self, userdata):
        self.odom.unregister()
