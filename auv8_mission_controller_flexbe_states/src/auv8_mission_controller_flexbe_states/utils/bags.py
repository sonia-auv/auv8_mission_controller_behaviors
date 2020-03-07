import os
import shlex
import subprocess

def startControlBag(name, time):
    try:
        os.chdir(os.path.expanduser('~/Workspaces/ros_sonia_ws/src/proc_control/script'))
        command = "./Record_open_loop.sh " + name + " " + str(time)
        command = shlex.split(command)
        subprocess.Popen(command)
    except:
        print "Bag error"
    git