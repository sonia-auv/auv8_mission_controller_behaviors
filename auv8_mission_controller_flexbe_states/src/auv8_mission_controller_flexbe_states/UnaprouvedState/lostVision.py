import rospy
import time

from flexbe_core import EventState, Logger
from proc_image_processing.msg import VisionTarget


class LostVision(EventState):

    def __init__(self):
        super(LostVision, self).__init__(outcomes=['continue', 'failed'])
        self.vision_subscriber = None

        self.last_target_received_time = None

        self.count = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_topic_to_listen', '/proc_image_processing/buoy_red_result', 'Name of topic to listen'))
        self.parameters.append(Parameter('param_max_time_lost_in_sec', 10, 'Maximum time without target'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def vision_cb(self, vision_data):
        self.last_target_received_time = time.time()

    def on_enter(self, userdata):
        self.vision_subscriber = rospy.Subscriber(self.param_topic_to_listen, VisionTarget, self.vision_cb)
        self.count += 1
        self.last_target_received_time = time.time()

    def execute(self, userdata):
        if time.time() - self.last_target_received_time > self.param_max_time_lost_in_sec:
            return 'succeeded'

    def on_exit(self, userdata):
        self.vision_subscriber.unregister()
