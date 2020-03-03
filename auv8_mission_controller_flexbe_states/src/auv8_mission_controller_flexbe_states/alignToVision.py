import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget


class AlignToVision(EventState):
    def __init__(self, bonding_box_in_pixel=200, target_width_in_meter, topic_to_listen, pixel_to_victory, nb_max_align=10, max_queue_size=10, control_bonding_box_y):
        super(AlignToVision, self).__init__(outcomes=['continue', 'failed', 'forward'])
        self.set_local_target = None
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target_topic = None

        self.vision_position_y = 0
        self.vision_position_z = 0

        self.heading = 0

        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0

        self.target_reached = False
        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        self.is_align_with_heading_active = False
        self.victory = False

        self.vision_x_pixel = None
        self.vision_y_pixel = None

        self.param_bonding_bax = bonding_box_in_pixel
        self.param_vision_target_width_in_meter = target_width_in_meter
        self.param_topic_to_listen = topic_to_listen
        self.param_nb_pixel_to_victory = pixel_to_victory
        self.param_maximum_nb_alignment = nb_max_align
        self.param_max_queue_size = max_queue_size
        self.param_control_bounding_box_in_y = control_bonding_box_y

        self.count = 0

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def find_y_pos_to_matches_to_control_bounding_box(self, pos_y):
        if pos_y <= self.param_control_bounding_box_in_y:
            return self.param_control_bounding_box_in_y + 0.1
        else:
            return pos_y

    def vision_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)

        if len(self.vision_x_pixel) == self.param_max_queue_size and len(self.vision_y_pixel) == self.param_max_queue_size:
            self.parse_vision_data(vision_data.width)

    def parse_vision_data(self, width):

            for i in self.vision_x_pixel:
                self.averaging_vision_x_pixel += i

            for i in self.vision_y_pixel:
                self.averaging_vision_y_pixel += i

            self.averaging_vision_x_pixel /= len(self.vision_x_pixel)
            self.averaging_vision_y_pixel /= len(self.vision_y_pixel)

            if width >= self.param_nb_pixel_to_victory:
                self.victory = True

            if abs(self.averaging_vision_x_pixel) <= self.param_bounding_box:
                self.vision_is_reach_y = True
            else:
                self.vision_is_reach_y = False

            if abs(self.averaging_vision_y_pixel) <= self.param_bounding_box:
                self.vision_is_reach_z = True
            else:
                self.vision_is_reach_z = False

            pixel_to_meter = width / self.param_vision_target_width_in_meter

            if self.target_reached:
                self.vision_position_y = self.averaging_vision_x_pixel / pixel_to_meter
                self.vision_position_z = self.averaging_vision_y_pixel / pixel_to_meter * -1
                self.align_submarine()

    def align_submarine(self):
        vision_position_y = self.find_y_pos_to_matches_to_control_bounding_box(self.vision_position_y)
        vision_position_z = self.vision_position_z

        vision_position_y = vision_position_y * (self.vision_position_y / abs(self.vision_position_y))

        target_z = 0.1 * (vision_position_z / abs(vision_position_z))

        if not self.vision_is_reach:
            rospy.logdebug('set_target : 2 - align_submarine')
            self.set_target(vision_position_y, target_z, 0.0, False, False, True)

    def set_target(self, position_y, position_z, position_yaw, keepY, keepZ, keepYaw):
        try:
            self.set_local_target(0.0, position_y, position_z, 0.0, 0.0, position_yaw,
                                  True, keepY, keepZ, True, True, keepYaw)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))

        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_local_decoupled_target')
        self.set_local_target = rospy.ServiceProxy('/proc_control/set_local_decoupled_target', SetDecoupledTarget, persistent=True)

        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)

        self.vision_x_pixel = deque([], maxlen=self.param_max_queue_size)
        self.vision_y_pixel = deque([], maxlen=self.param_max_queue_size)

        self.vision_x_pixel.clear()
        self.vision_y_pixel.clear()

        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        self.count += 1

    def execute(self, userdata):

        self.vision_is_reach = self.vision_is_reach_y and self.vision_is_reach_z

        if self.victory and self.vision_is_reach:
            rospy.loginfo('Vision is Reach : %s', str(self.vision_is_reach))
            rospy.loginfo('Pixel Vision in x : %f', self.averaging_vision_x_pixel)
            rospy.loginfo('Pixel Vision in y : %f', self.averaging_vision_y_pixel)
            rospy.logdebug('set_target : 2 - run')
            self.set_target(0.0, 0.0, 0.0, False, False, False)
            return 'continue'

        if self.vision_is_reach:
            rospy.logdebug('set_target : 2 - run')
            self.set_target(0.0, 0.0, 0.0, False, False, False)
            return 'forward'

        if self.count >= self.param_maximum_nb_alignment:
            return 'failed'

    def on_exit(self, userdata):
        self.vision_subscriber.unregister()
        self.target_reach_sub.unregister()
