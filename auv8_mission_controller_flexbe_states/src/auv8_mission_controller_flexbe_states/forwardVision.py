import rospy

from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget


class ForwardVision(EventState):

    def __init__(self, distance_forward=1, initial_bondingBox=600, final_bondingBox=100, threshold_width=100, target_width=0.3, pixel_victory=400, topic_to_listen):
        super(ForwardVision, self).__init__(outcomes=['continue', 'failed','lost'])
        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_local_target = None
        self.buoy_position = None
        self.target_reach_sub = None

        self.adjust_bounding_box = 0.0
        self.initial_bounding_box = 0.0
	self.param_distance_x = distance_forward
	self.param_initial_bounding_box = initial_bondingBox
	self.param_final_bounding_box = final_bondingBox
	self.param_threshold_width = threshold_width
	self.param_vision_target_width_in_meter = target_width
	self.param_nb_pixel_to_victory = pixel_victory
	self.param_topic_to_listen = topic_to_listen

    def vision_cb(self, vision_data):

        bounding_box = self.adjust_bounding_box * vision_data.width + self.initial_bounding_box

        if abs(vision_data.x) >= bounding_box or abs(vision_data.y) >= bounding_box:
            self.buoy_is_unreached = True

        if vision_data.width >= self.param_nb_pixel_to_victory:
            self.victory = False

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def set_target(self, position_x):
        try:
            self.set_local_target(position_x, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  False, True, True, True, True, True)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
	    return 'failed'

        rospy.loginfo('Set relative position x = %f' % position_x)
        rospy.loginfo('Set relative position yaw = %f' % 0.0)

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget)

        self.buoy_position = rospy.Subscriber(str(self.param_topic_to_listen), VisionTarget, self.vision_cb)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.adjust_bounding_box = (self.param_final_bounding_box - self.param_initial_bounding_box) / (self.victory -
                                                                                            self.param_threshold_width)

        self.initial_bounding_box = self.param_initial_bounding_box - self.param_threshold_width * self.adjust_bounding_box

        self.buoy_is_unreached = False
        self.target_reached = False
        self.victory = False

        self.set_target(self.param_distance_x)

    def execute(self, userdata):

        if self.victory:
            self.set_target(0.0)
            return 'continue'

        if self.buoy_is_unreached or self.target_reached:
            self.set_target(0.0)
            return 'lost'

    def on_exit(self, userdata):
        self.buoy_position.unregister()
        self.target_reach_sub.unregister()
