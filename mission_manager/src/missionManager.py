#! /usr/bin/env python2

import rospy

from mission_manager.srv import CurrentMission, CurrentMissionResponse
from provider_kill_mission.msg import MissionSwitchMsg
from flexbe_msgs.msg import BehaviorExecutionActionGoal

class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager')
        rospy.loginfo("Starting mission manager")
        self.current_mission = None

        self.mission_switch_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                    self.handle_mission_switch_activated)

        self.flexbe_behavior_pub = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=10)

        rospy.Service('mission_executor/current_mission', CurrentMission, self.handle_current_mission)

        rospy.spin()

    def handle_mission_switch_activated(self, msg):
        rospy.loginfo("Mission switch state change: {}".format(msg.state))
        rospy.loginfo("Mission: {}".format(self.current_mission))
        if self.current_mission:
            if msg.state:
                self.handle_start_mission(None)
            else:
                self.handle_stop_mission(None)
    
    def handle_current_mission(self, req):
        rospy.loginfo("Mission selected")
        return CurrentMissionResponse(self.current_mission)

    def handle_start_mission(self, req):
        rospy.loginfo("Start mission")
        self.flexbe_behavior_pub.publish('{goal: {behavior_name: "Example Behavior"}}')

    def handle_stop_mission(self, req):
        rospy.loginfo("Stop mission")
        pass

if __name__ == '__main__':
    MissionManager()