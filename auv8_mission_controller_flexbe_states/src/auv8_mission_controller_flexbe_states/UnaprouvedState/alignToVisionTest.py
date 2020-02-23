import rospy

from Queue import deque
from flexbe_core import EventState, Logger
from proc_control.msg import TargetReached
from proc_control.srv import SetPositionTarget, SetDecoupledTarget
from proc_image_processing.msg import VisionTarget


class AlignToVisionTest(EventState):

    def __init__(self):
        super(RotateYawRelative, self)
        self.set_local_target = None
        self.vision_subscriber = None
        self.target_reach_sub = None
        self.set_local_target_topic = None

        self.vision_position_y = 0
        self.vision_position_z = 0

        self.heading = 0

        self.averaging_vision_x_pixel = 0.0
        self.averaging_vision_y_pixel = 0.0

        self.target_distance = 0.0
        self.target_width = 0.0
        self.target_height = 0.0
        self.vision_distance = 0.5

        self.distance_to_cover_y = 0.0
        self.distance_to_cover_z = 0.0
        self.distance_to_cover_x = 0.0

        self.target_reached = False
        self.vision_is_reach_y = False
        self.vision_is_reach_z = False
        self.vision_is_reach = False

        self.is_align_with_heading_active = False
        self.victory = False
        self.ready_to_advance = False

        self.vision_x_pixel = None
        self.vision_y_pixel = None

        #Parameters to determine target distance.
        self.focal_size = 12.5 #mm
        self.sensor_height = 5.3 #mm
        self.sensor_width = 7.1 #mm

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_bounding_box', 200, 'bounding box in pixel'))
        self.parameters.append(Parameter('param_heading', 10, 'Yaw rotation to align vision')) 
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red', 'Topic to listen'))
        self.parameters.append(Parameter('param_distance_to_victory', 2, 'Minimal distance to ram (m)'))
        self.parameters.append(Parameter('param_maximum_nb_alignment', 4, 'Maximum number of alignment'))
        self.parameters.append(Parameter('param_max_queue_size', 10, 'Maximum size of queue'))
        self.parameters.append(Parameter('param_object_real_height', 100, 'Object height (mm)'))
        self.parameters.append(Parameter('param_object_real_width', 100, 'Object width (mm)'))
        self.parameters.append(Parameter('param_image_height', 1544, 'Image height (px)'))
        self.parameters.append(Parameter('param_image_width', 2064, 'Image width (px)'))
        self.parameters.append(Parameter('param_offset_y', 0, 'Lateral offset'))
        self.parameters.append(Parameter('param_offset_z', 0, 'Vertical offset'))

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

        self.count = 0

    def execute(self, userdata):

        self.vision_is_reach = self.vision_is_reach_y and self.vision_is_reach_z

        if self.victory and self.vision_is_reach:
            rospy.loginfo('Vision is Reach : %s', str(self.vision_is_reach))
            rospy.loginfo('Pixel Vision in x : %f', self.averaging_vision_x_pixel)
            rospy.loginfo('Pixel Vision in y : %f', self.averaging_vision_y_pixel)
            self.set_target(0.0, 0.0, 0.0, 0.0, False, False, False, False)
            return 'succeeded'

        if self.count >= self.param_maximum_nb_alignment:
            return 'aborted'

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def vision_cb(self, vision_data):
        self.vision_x_pixel.append(vision_data.x)
        self.vision_y_pixel.append(vision_data.y)

        if len(self.vision_x_pixel) == self.param_max_queue_size and len(self.vision_y_pixel) == self.param_max_queue_size:
            self.parse_vision_data(vision_data.width, vision_data.height)

    def parse_vision_data(self, width, height):
            for i in self.vision_x_pixel:
                self.averaging_vision_x_pixel += i

            for i in self.vision_y_pixel:
                self.averaging_vision_y_pixel += i

            self.averaging_vision_x_pixel /= len(self.vision_x_pixel)
            self.averaging_vision_y_pixel /= len(self.vision_y_pixel)

            rospy.loginfo('position x : %f' % self.averaging_vision_x_pixel)
            rospy.loginfo('position y : %f' % self.averaging_vision_y_pixel)

            if abs(self.averaging_vision_x_pixel) <= self.param_bounding_box:
                self.vision_is_reach_y = True
                self.distance_to_cover_y = 0.0
            else:
                self.vision_is_reach_y = False

            if abs(self.averaging_vision_y_pixel) <= self.param_bounding_box:
                self.vision_is_reach_z = True
                self.distance_to_cover_z = 0.0
            else:
                self.vision_is_reach_z = False
            rospy.loginfo("Object height into image (px): %f" % height)
            rospy.loginfo("Object width into image (px): %f" % width)
            #self.target_distance = 0.0136827 * ((self.param_object_real_width * self.param_image_height) / height)
            self.target_distance = (self.focal_size * self.param_object_real_height * self.param_image_height) / (height * self.sensor_height)

            rospy.loginfo('Target distance : %f' % self.target_distance)

            if self.vision_is_reach_y and self.vision_is_reach_z:
                if self.target_width >= self.param_object_real_width and self.target_height >= self.param_object_real_height:
                    if self.target_distance <= self.param_distance_to_victory:
                        self.victory = True
                        self.ready_to_advance = False
                    else:
                        self.ready_to_advance = True
                else:
                    self.ready_to_advance = True
            else:
                self.ready_to_advance = False

            if self.target_reached and not self.victory:
                pixel_to_meter_width = width / self.param_object_real_width
                pixel_to_meter_height = height / self.param_object_real_height

                if not self.vision_is_reach_z:
                    self.distance_to_cover_z = ((self.averaging_vision_y_pixel - (
                                self.param_image_height / 2)) / pixel_to_meter_height) / 100
                    rospy.loginfo('distance to cover z : %f' % self.distance_to_cover_z)

                if not self.vision_is_reach_y:
                    self.distance_to_cover_y = ((self.averaging_vision_x_pixel - (
                                self.param_image_width / 2)) / pixel_to_meter_width) / 100
                    rospy.loginfo('distance to cover y : %f' % self.distance_to_cover_y)

                    if self.vision_is_reach_z:
                        self.is_align_with_heading_active = True
                    else:
                        self.is_align_with_heading_active = False
                self.align_submarine()

    def align_submarine(self):

        if self.ready_to_advance:
            self.ready_to_advance = False
            if self.target_distance < 0.5:
                distance_to_cover_x = self.target_distance
            else:
                distance_to_cover_x = 0.5
        else:
            distance_to_cover_x = 0.0

        if self.is_align_with_heading_active and not self.vision_is_reach_y:
            if self.distance_to_cover_y != 0:
                self.heading = self.param_heading * (self.distance_to_cover_y / abs(self.distance_to_cover_y))
            rospy.loginfo('set_target_mode : align_heading')
            self.set_target(0.0, self.distance_to_cover_y, 0.0, self.heading, True, True, True, False)

        elif not self.vision_is_reach:
            rospy.loginfo('set_target_mode : align_submarine')
            self.set_target(distance_to_cover_x, self.distance_to_cover_y, self.distance_to_cover_z, 0.0, False, False, False, True)

        elif not self.victory:
            rospy.loginfo('set_target_mode : move_forward')
            self.set_target(distance_to_cover_x,0.0, 0.0, 0.0, False, True, True, True)

        rospy.loginfo('Align number %i' % self.count)

        self.count += 1

    def set_target(self, position_x, position_y, position_z, position_yaw, keepX, keepY, keepZ, keepYaw):

        rospy.loginfo('Set relative position x = %f' % position_x)
        rospy.loginfo('Set relative position y = %f' % position_y)
        rospy.loginfo('Set relative position z = %f' % position_z)
        rospy.loginfo('Set relative position yaw = %f' % position_yaw)

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def end(self):
        self.vision_subscriber.unregister()
        self.target_reach_sub.unregister()