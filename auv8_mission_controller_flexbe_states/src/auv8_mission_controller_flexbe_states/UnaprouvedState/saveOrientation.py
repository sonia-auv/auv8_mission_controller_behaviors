import rospy

from ..mission_state import EventState
from nav_msgs.msg import Odometry
import time

class SaveOrientation(EventState):

    def __init__(self):
        super(SaveOrientation, self)

        self.odom = None
        self.orientation = None

    def define_parameters(self):
        pass

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def odom_cb(self, odom_data):
        self.orientation = odom_data.pose.pose.orientation.z

    def on_enter(self, userdata):
        self.odom = rospy.Subscriber('/proc_navigation/odom', Odometry, self.odom_cb)

    def execute(self, userdata):
        time.sleep(3)
        orientation = self.orientation
        rospy.set_param('/save_orientation',orientation)
        rospy.loginfo("Orientation saved : {}".format(orientation))
        return 'succeeded'

    def on_exit(self, userdata):
        self.odom.unregister()
