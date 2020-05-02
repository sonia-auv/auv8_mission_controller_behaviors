#! /usr/bin/env python2

import os
import rospy
import rospkg

from mission_manager.srv import ListMissions, ListMissionsResponse

from mission_manager.msg import MissionNameMsg
from provider_kill_mission.msg import MissionSwitchMsg
from flexbe_msgs.msg import BehaviorExecutionActionGoal
from std_msgs.msg import Empty

class MissionManager:

    def __init__(self):
        rospy.init_node('mission_manager')

        self.current_mission = None
        self.missions = None

        # Get manifest folder
        rp = rospkg.RosPack()
        self.missions_directory = os.path.join(rp.get_path('auv8_mission_controller_flexbe_behaviors'), 'manifest')

        # Subscribers
        self.mission_switch_sub = rospy.Subscriber('/provider_kill_mission/mission_switch_msg', MissionSwitchMsg,
                                                    self.handle_mission_switch_activated)
        self.mission_name_sub = rospy.Subscriber('/mission_manager/mission_name_msg', MissionNameMsg, 
                                                    self.handle_current_mission)
        
        # Publishers
        self.flexbe_behavior_pub = rospy.Publisher('/flexbe/execute_behavior/goal', BehaviorExecutionActionGoal, queue_size=5)
        self.flexbe_command_pub = rospy.Publisher('/flexbe/commands/preempt', Empty, queue_size=5)

        # Services
        rospy.Service('mission_executor/list_missions', ListMissions, self.handle_list_missions)

        rospy.spin()

    # Handler for mission switch ON/OFF
    def handle_mission_switch_activated(self, msg):
        rospy.loginfo("Mission switch state change: {}".format(msg.state))
        rospy.loginfo("Mission: {}".format(self.current_mission))
        if self.current_mission:
            if msg.state:
                self.handle_start_mission(None)
            else:
                self.handle_stop_mission(None)
    
    # Handler to get selected mission name
    def handle_current_mission(self, req):
        rospy.loginfo("Mission selected : {}".format(req.name))
        self.current_mission = req.name

    # Handler to start the mission
    def handle_start_mission(self, req):
        rospy.loginfo("Start mission")
        msg = BehaviorExecutionActionGoal()
        msg.goal.behavior_name = 'Example Behavior'
        self.flexbe_behavior_pub.publish(msg)

    # Handler to stop the mission
    def handle_stop_mission(self, req):
        rospy.loginfo("Stop mission")
        msg = Empty()
        self.flexbe_command_pub.publish(msg)

    # Handler to list every available missions
    def handle_list_missions(self, req):
        self.missions = []
        self.load_missions_file(self.missions_directory)
        self.missions.sort(key=str.lower)
        missions_list = None
        for mission in self.missions:
            if not missions_list:
                missions_list = mission
            else:
                missions_list = missions_list + ',' + mission
        rospy.loginfo(missions_list)
        return ListMissionsResponse(missions_list)

    # Function to load all mission files
    def load_missions_file(self, directory):
        for file in os.listdir(directory):
            file_path = os.path.join(directory, file)
            if not os.path.isfile(file_path):
                self.load_missions_file(file_path)
                continue
            self.missions.append(file_path.replace(self.missions_directory + '/', ''))

if __name__ == '__main__':
    MissionManager()