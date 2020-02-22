import rospy

from flexbe_core import EventState, Logger

from proc_image_processing.srv import execute_cmd
from provider_vision.srv import start_stop_media



class LaunchVision(EventState):
    """
        I need to do this

        -- param_node_name      string Detection task
        -- camera_no uint8      Enter 1:Front 2:Bottom
        -- param_cmd uint8      Enter 1:Open  2:Close

        <= continue			Indicates that the camera started
        <= failed			Indicates that the camera didn't stated

    """
    def __init__(self, param_node_name, camera_no, param_cmd):
        super(LaunchVision, self).__init__(outcomes=['continue', 'failed'])
        self.execute_vision_cmd = None
        self.start_stop_vision = None
        self.camera_no = camera_no
        self.param_node_name = param_node_name
        self.param_cmd = param_cmd

    def on_enter(self, userdata):
        try:
            rospy.wait_for_service('/proc_image_processing/execute_cmd')
            self.execute_vision_cmd = rospy.ServiceProxy('/proc_image_processing/execute_cmd', execute_cmd)
            rospy.wait_for_service('/provider_vision/start_stop_camera')
            self.start_stop_vision = rospy.ServiceProxy('/provider_vision/start_stop_camera', start_stop_media)
        except rospy.ServiceException as exc:
            rospy.loginfo('Cannot access service' + str(exc))
            return 'failed'
        try:
            if self.camera_no == 1:
                self.start_stop_vision('Front_GigE', 1)
                self.start_stop_vision('Bottom_GigE', 2)
                self.param_media_name = '/provider_vision/Front_GigE'
            else:
                self.start_stop_vision('Front_GigE', 2)
                self.start_stop_vision('Bottom_GigE', 1)
                self.param_media_name = '/provider_vision/Bottom_GigE'

            self.execute_vision_cmd(self.param_node_name,
                                    self.param_node_name,
                                    self.param_media_name,
                                    self.param_cmd)
        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
            return 'failed'

    def execute(self, userdata):
        return 'continue'

    def end(self, userdata):
        pass
