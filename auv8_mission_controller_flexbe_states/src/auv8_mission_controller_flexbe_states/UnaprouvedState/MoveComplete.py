import rospy

from flexbe_core import EventState, Logger
from geometry_msgs.msg import Point, Vector3
from sonia_common.msg import AddPose
from std_msgs.msg import Bool
from time import time, sleep

class MoveComplete(EventState):
    '''
    State to make a complete move command.

    -- MoveX        float64     
    -- MoveY        float64     
    -- MoveZ        float64     
    -- RotX         float64     
    -- RotY         float64     
    -- RotZ         float64     
    -- Frame        uint8   0 : Absolute position and absolute angle   
                            1 : Relative position and relative angle
                            2 : Relative position and absolute angle
                            3 : Absolute position and relative angle
    -- Time         uint8   
    -- Fine         float64 
    -- Rotation     bool    

    '''
    def __init__(self, MoveX, MoveY, MoveZ, RotX, RotY, RotZ, Frame, Fine, Time = 10 , Rotation=True):
        
        super(MoveComplete, self).__init__(outcomes=['continue', 'failed'])
        
        self.target_reached = False

        self.param_Move_X = MoveX
        self.param_Move_Y = MoveY
        self.param_Move_Z = MoveZ
        self.param_Rotation_X = RotX
        self.param_Rotation_Y = RotY
        self.param_Rotation_Z = RotZ
        self.param_Frame = Frame
        self.param_Time = Time
        self.param_Fine = Fine
        self.param_Rotation = Rotation
        self.runTime = 0
        self.StartTime = 0

    def target_reach_cb(self, data):
        print(data)
        self.target_reached = data.data   

    def on_enter(self, userdata):
        self.add_pose_pub = rospy.Publisher('/proc_control/add_pose', AddPose, queue_size=2)

        pointXYZ = Point(self.param_Move_X, self.param_Move_Y, self.param_Move_Z)
        vectorXYZ = Vector3(self.param_Rotation_X, self.param_Rotation_Y, self.param_Rotation_Z)
        self.add_pose_pub.publish(pointXYZ, vectorXYZ, self.param_Frame, self.param_Time, self.param_Fine, self.param_Rotation)
        
        self.runTime = 0
        self.StartTime = time()
        self.target_reached_sub = rospy.Subscriber('/proc_control/target_reached', Bool, self.target_reach_cb)

    def execute(self, userdata):
        self.runTime = time() - self.StartTime
        sleep(5)
        if self.target_reached > 0:
            return 'continue'
        #else:
        #    return 'failed'
        elif self.runTime > self.param_Time:
            return 'failed'   

    def on_exit(self, userdata):
        self.target_reached_sub.unregister()
        self.add_pose_pub.unregister()   
        