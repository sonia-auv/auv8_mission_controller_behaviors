#!/usr/bin/env python
import rospy

import json
import time
from Tkinter import *
import tkFileDialog as filedialog

from utils.bags import startControlBag
from flexbe_core import EventState, Logger
from provider_thruster.msg import ThrusterEffort
from proc_control.srv import EnableThrusters

class ThrusterControlManual(EventState):
    '''
    State to control the thrusters efforts by passing parameters.
    '''

    def __init__(self, bag_name, bag_time, T1 = 0, T2 = 0, T3 = 0, T4 = 0, T5 = 0, T6 = 0, T7 = 0, T8 = 0):
        super(ThrusterControlManual, self).__init__(outcomes = ['continue', 'failed'])
        self.T1 = T1
        self.T2 = T2
        self.T3 = T3
        self.T4 = T4
        self.T5 = T5
        self.T6 = T6
        self.T7 = T7
        self.T8 = T8
        self.bag_name = bag_name
        self.bag_time = bag_time

        self.thruster_count = 8
        self.thruster_min_effort = -30
        self.thruster_max_effort = 30
        self.thruster_stop_effort = 0
        self.started = False

        self.ids = [ThrusterEffort.UNIQUE_ID_T1, 
                    ThrusterEffort.UNIQUE_ID_T2,
                    ThrusterEffort.UNIQUE_ID_T3,
                    ThrusterEffort.UNIQUE_ID_T4,
                    ThrusterEffort.UNIQUE_ID_T5,
                    ThrusterEffort.UNIQUE_ID_T6,
                    ThrusterEffort.UNIQUE_ID_T7,
                    ThrusterEffort.UNIQUE_ID_T8]
    
    def execute(self, userdata):
        if not self.started:
            self.started = True
            thrusters_efforts = []
            try:
                Logger.loginfo('Bag name: {0}, Duration: {1}'.format(self.bag_name, self.bag_time))
                startControlBag(self.bag_name, self.bag_time)
                thrusters_efforts.append(int(self.T1))
                thrusters_efforts.append(int(self.T2))
                thrusters_efforts.append(int(self.T3))
                thrusters_efforts.append(int(self.T4))
                thrusters_efforts.append(int(self.T5))
                thrusters_efforts.append(int(self.T6))
                thrusters_efforts.append(int(self.T7))
                thrusters_efforts.append(int(self.T8))
                Logger.loginfo('Wait {} seconds...'.format(self.bag_time))
                time.sleep(float(self.bag_time)) 
            except:
                Logger.loginfo('Error with your mission.')
                return 'failed' 
            return 'continue'

    def on_enter(self, userdata):
        try:
            rospy.wait_for_service('/proc_control/enable_thrusters',timeout=2)
            self.enable_thrusters_service = rospy.ServiceProxy('/proc_control/enable_thrusters', EnableThrusters)
        except rospy.ROSException:
            Logger.loginfo('Error contacting thrusters service.')
            return 'failed'

        try:
            self.publisher = rospy.Publisher("/provider_thruster/thruster_effort", ThrusterEffort, queue_size=10)
        except:
            Logger.loginfo('Error connection to thruster effort publisher.')
            return 'failed'

        try:
            self.enable_thrusters_service(isEnable=False)
        except rospy.ServiceException as err:
            Logger.loginfo('Error enabling thrusters control.')
            return 'failed'

    def on_exit(self, userdata):
        self.set_zeros()

    def on_stop(self):
        self.set_zeros()

    def set_efforts(self, thrusters_efforts):
        Logger.loginfo("Efforts: \n")
        for i in range(self.thruster_count):
            Logger.loginfo("\tThruster -> {0} | Effort -> {1}".format(i+1, thrusters_efforts[i]))
            self.publisher.publish(ID=self.ids[i], effort=thrusters_efforts[i])

    def set_zeros(self):
        for i in range(self.thruster_count):
            self.publisher.publish(ID=self.ids[i], effort=self.thruster_stop_effort)
