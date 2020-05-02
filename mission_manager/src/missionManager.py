#! /usr/bin/env python2

import rospy

from mission_manager.msg import MissionNameMsg
from provider_kill_mission.msg import MissionSwitchMsg
from flexbe_msgs.msg import BehaviorExecutionActionGoal
from std_msgs.msg import Empty

class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager')
        self.current_mission = None

        self.mission_switch_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                    self.handle_mission_switch_activated)
        self.mission_name_sub = rospy.Subscriber('/mission_manager/mission_name_msg', MissionNameMsg, 
                                                    self.handle_current_mission)

        self.flexbe_behavior_pub = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=5)
        self.flexbe_command_pub = rospy.Publisher('/flexbe/commands/preempt', Empty, queue_size=5)

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
        rospy.loginfo("Mission selected : {}".format(req.name))
        self.current_mission = req.name

    def handle_start_mission(self, req):
        rospy.loginfo("Start mission")
        msg = BehaviorExecutionActionGoal()
        msg.goal.behavior_name = 'Example Behavior'
        self.flexbe_behavior_pub.publish(msg)

    def handle_stop_mission(self, req):
        rospy.loginfo("Stop mission")
        msg = Empty()
        self.flexbe_command_pub.publish(msg)

if __name__ == '__main__':
    MissionManager()