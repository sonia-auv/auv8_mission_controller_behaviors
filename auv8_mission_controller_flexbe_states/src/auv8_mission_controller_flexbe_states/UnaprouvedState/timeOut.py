import rospy

from flexbe_core import EventState, Logger


class TimesOut(EventState):

    def __init__(self):
        super(RotateYawRelative, self)
        self.start_time = None
        self.current_second = 0

    def define_parameters(self):
        self.parameters.append(Parameter('param_time', 1, 'Times Out'))
        self.parameters.append(Parameter('param_to_return', 'succeeded', 'Times Out'))

    def get_outcomes(self):
        return ['succeeded', 'aborted', 'preempted']

    def on_enter(self, userdata):
        self.start_time = rospy.get_time()
        rospy.loginfo("Asked time : {} sec".format(self.param_time))

    def execute(self, userdata):
        time = rospy.get_time() - self.start_time

        int_time = int(time)

        if int_time > self.current_second:
            self.current_second = int_time

            rospy.loginfo("Time left : {} sec".format(float(self.param_time) - int_time))

        self.current_second = int(time)

        if time >= self.param_time:
            return str(self.param_to_return)

    def end(self):
        pass
