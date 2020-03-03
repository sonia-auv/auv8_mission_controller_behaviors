import rospy

from flexbe_core import EventState, Logger


class TimesOut(EventState):

    def __init__(self, duration=10):
        super(TimesOut, self).__init__(outcomes=['continue', 'failed'])
        self.start_time = None
        self.current_second = 0
        self.duration = duration

    def on_enter(self, userdata):
        self.start_time = rospy.get_time()
        rospy.loginfo("Asked time : {} sec".format(self.duration))

    def execute(self, userdata):
        time = rospy.get_time() - self.start_time

        int_time = int(time)

        if int_time > self.current_second:
            self.current_second = int_time

            rospy.loginfo("Time left : {} sec".format(float(self.duration) - int_time))

        self.current_second = int(time)

        if time >= self.duration:
            return 'continue'

    def on_exit(self, userdata):
        pass
