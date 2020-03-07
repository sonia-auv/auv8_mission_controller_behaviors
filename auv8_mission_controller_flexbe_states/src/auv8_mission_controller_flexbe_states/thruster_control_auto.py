#!/usr/bin/env python

'''
 * file	ThrusterControlByValue.py
 * \author	 Alexandre Desgagne <alexandre1998@live.ca>
 * \coauthor Camille Sauvain <camille.sauvain.1@etsmtl.net>
 * \date	 19/02/20
 *
 * \copyright Copyright (c) 2020 S.O.N.I.A. AUV All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. AUV software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. AUV software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. AUV software. If not, see <http://www.gnu.org/licenses/>.
'''

import rospy

import json
import time
import subprocess
import shlex
from Tkinter import *
import tkFileDialog as filedialog

from utils.phase import Phase
from utils.bags import startControlBag
from flexbe_core import EventState, Logger
from provider_thruster.msg import ThrusterEffort
from proc_control.srv import EnableThrusters

class ThrusterControlAuto(EventState):
    '''
    State to control the thrusters efforts by passing a json file.
    '''

    def __init__(self):
        super(ThrusterControlAuto, self).__init__(outcomes = ['continue', 'failed'])
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
            phases = []
            bag_time = 0
            try:
                root = Tk()
                filename = filedialog.askopenfilename(initialdir = "~/Workspaces/ros_sonia_ws/src/proc_control/config/OpenLoopMotors",
                                                      title = "Select file",
                                                      filetypes = (("text files","*.json"),("all files",".*")))
                f = open(filename, "r")
                phases_file = json.load(f)
                root.destroy()
            except:
                Logger.loginfo('Error loading file {}'.format(self.json_file))
                return 'failed'
            try:
                bag_name = phases_file["bag_name"]
                for phase in phases_file['phases']:
                    phases.append(Phase(phase['phase_name'], phase['time'], phase['thrusters'])) 
                    bag_time += int(phase["time"])
                Logger.loginfo('Bag name: {0}, Duration: {1}'.format(bag_name, bag_time))
                startControlBag(bag_name, bag_time)
                for phase in phases:
                    Logger.loginfo('Phase name: {}'.format(str(phase.name)))
                    Logger.loginfo('Phase time: {}'.format(str(phase.time)))
                    self.set_efforts(phase.thruster_control)
                    Logger.loginfo('Wait {} seconds...'.format(phase.time))
                    time.sleep(float(phase.time)) 
            except:
                Logger.loginfo('Error with your mission. Ensure your json file is well written.')
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
