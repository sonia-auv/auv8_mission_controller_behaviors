import rospy
import math

from flexbe_core import EventState, Logger
from sonia_common.srv import PingerLocationService
from sonia_common.srv import SetDecoupledTarget
from sonia_common.msg import TargetReached


class Hydro(EventState):

    def __init__(self, frequency):
        super(Hydro, self).__init__(outcomes=['continue', 'failed'])
        self.target_reached = False
        self.param_frequency = frequency

    def target_reach_cb(self, data):
        self.target_reached = data.target_is_reached

    def on_enter(self, userdata):
        rospy.wait_for_service('/proc_control/set_global_decoupled_target')
        self.set_global_target = rospy.ServiceProxy('/proc_control/set_global_decoupled_target', SetDecoupledTarget)

        rospy.loginfo('try hydro srv')
        rospy.wait_for_service('/proc_mapping/pinger_location_service')
        self.pinger_location_service = rospy.ServiceProxy('/proc_mapping/pinger_location_service', PingerLocationService)
        rospy.loginfo('succeed hydro srv')
        self.target_reached = 0

        try:
            response = self.pinger_location_service(self.param_frequency)

            pose = response.pingerLocation.pose

            self.set_global_target(pose.position.x,
                                   pose.position.y,
                                   0.0,
                                   0.0,
                                   0.0,
                                   pose.orientation.z * 180 / math.pi,
                                   False, False, True, True, True, False)
            
            rospy.loginfo('Set global position x = %f y = %f' % (pose.position.x, pose.position.y))

        except rospy.ServiceException as exc:
            rospy.loginfo('Service did not process request: ' + str(exc))
        
        self.target_reach_sub = rospy.Subscriber('/proc_control/target_reached', TargetReached, self.target_reach_cb)

    def execute(self, userdata):
        if self.target_reached > 0:
            return 'continue'
        pass

    def end(self, userdata):
        self.target_reach_sub.unregister()
        pass