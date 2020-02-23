import rospy

from ..mission_state import EventState
from nav_msgs.msg import Odometry


class SavePosition(EventState):

    def __init__(self):
        super(SavePosition, self)

        self.odom =None
        self.position = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def odom_cb(self, odom_data):
        self.position = odom_data.pose.pose

    def on_enter(self, userdata):
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

    def execute(self, userdata):
        position = self.position
        if position is not None:
            ud.generic_data_field_1 = position
            return 'succeeded'

    def end(self):
        self.odom.unregister()
