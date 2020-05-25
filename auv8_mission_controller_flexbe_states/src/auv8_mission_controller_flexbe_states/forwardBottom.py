import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget
from nav_msgs.msg import Odometry


class ForwardBottom(EventState):

    def __init__(self, topic_to_listen, distance_z=0.5, bonding_box=200, depth_max=1, pixel_to_victory=200):
        super(ForwardBottom, self).__init__(outcomes=['continue', 'failed', 'aborted'])
        self.vision_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target = None
        self.buoy_position = None
        self.target_reach_sub = None
        self.sub_position = None

        self.position_in_z = None

        self.param_distance_z = distance_z
        self.param_bounding_box = bonding_box
        self.param_distance_max_z = depth_max
        self.param_nb_pixel_to_victory = pixel_to_victory
        self.param_topic_to_listen = topic_to_listen

    def sub_position_cb(self, position_data):
        self.position_in_z = position_data.pose.pose.position.z

    def vision_cb(self, vision_data):
        if vision_data.width >= self.param_nb_pixel_to_victory:
            self.victory = True

        if abs(vision_data.x) >= self.param_bounding_box or abs(vision_data.y) >= self.param_bounding_box:
            self.vision_is_unreached = True

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, position_z):
        try:
            self.set_local_target(0.0, 0.0, position_z, 0.0, 0.0, 0.0,
                                  True, True, False, True, True, True)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'failed'

        rospy.loginfo('Set relative position z = %f' % position_z)

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.sub_position = rospy.Subscriber('/proc_navigation/odom', Odometry, self.sub_position_cb)

        self.victory = False
        self.vision_is_unreached = False
        self.target_reached = False
        self.position_in_z = None

        self.set_target(self.param_distance_z)

    def execute(self, userdata):

        if self.position_in_z >= self.param_distance_max_z or self.victory:
            self.set_target(0.0)
            return 'continue'

        if self.position_in_z is not None and self.target_reached:
            if self.param_distance_z + self.position_in_z >= self.param_distance_max_z:
                self.set_target(self.param_distance_max_z - self.position_in_z)
            else:
                self.set_target(self.param_distance_z)

        if self.vision_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'aborted'

    def on_exit(self, userdata):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()
