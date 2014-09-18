#!/usr/bin/env python
import rospy
import roslib
import actionlib

roslib.load_manifest("camera_cb_detector")
from camera_calibrator.calibration_plate import *
from camera_cb_detector.msg import *
from calibration_msgs.msg import CalibrationPattern
from sensor_msgs.msg import Image, CameraInfo

class CalibrationDetectorServer:
    def __init__(self, name):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(self._action_name, ConfigAction, None, False)
        self._server.register_goal_callback(self.goal_callback)
        self._server.register_preempt_callback(self.preempt_callback)
        self._imageSub = rospy.Subscriber("image_raw", Image, self.img_callback)
        self._featurePub = rospy.Publisher("features", CalibrationPattern)
        self._board = None
        self._server.start()

    def goal_callback(self):
        goal = self._server.accept_new_goal()
        rospy.loginfo("[%s] Starting calibration plate detector" % rospy.get_name())
        # create the calibrtion plate
        if goal.pattern == ConfigGoal.CHECKERBOARD:
            self._board = Chessboard(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y)
        elif goal.pattern == ConfigGoal.CIRCLES:
            self._board = Circles(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y)
        elif goal.pattern == ConfigGoal.ACIRCLES:
            self._board = ACircles(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y)
        else:
            self._server.set_aborted()
            self._board = None
            
        if self._server.is_preempt_requested():
            self._board = None
            self._server.set_preempted()
        
    def preempt_callback(self):
        self._board = None
        self._server.set_preempted()

    def img_callback(self, img):
        rospy.logdebug("[%s] Got new image message" % rospy.get_name())
        if(self._featurePub and self._server.is_active()):
            ok, corners, rgb, (x_scale, y_scale) = self._board.detect(img)
            msg = self._board.mk_pattern_msg(ok, corners)
            msg.header = img.header
            self._featurePub.publish(msg)

if __name__=='__main__':
    rospy.init_node("image_calibration_detector")
    rospy.loginfo("[%s] Starting..." % rospy.get_name())

    
    autostart = rospy.get_param("~autostart", False)
    num_cols = rospy.get_param("~num_cols", 8)
    num_rows = rospy.get_param("~num_rows", 6)
    dim = rospy.get_param("~dim", 0.02943)
    pattern = rospy.get_param("~pattern", ConfigGoal.CHECKERBOARD)
    
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-a", "--autostart", dest="autostart", type="string", default=autostart,help="Autostart image_calibration_detector")
    parser.add_option("-c", "--cols", dest= "num_cols", type="int", default=num_cols, help="Number of cols in calibration pattern")
    parser.add_option("-r", "--rows", dest= "num_rows", type="int", default=num_rows, help="Number of rows in calibration pattern")
    parser.add_option("-d", "--dim", dest = "dim", type="float", default=dim, help="The length in meters between calibration points")
    parser.add_option("-p", "--pattern", dest="pattern", type="int", default=pattern, help="The type of calibration pattern (ENUM: 0: CHECKERBOARD, 1: CIRCLES, 2: ACIRCLES")
    
    options, args = parser.parse_args()
    
    autostart = options.autostart.lower()=='true'
    num_cols = options.num_cols
    num_rows = options.num_rows
    dim = options.dim
    pattern = options.pattern
    
    rospy.loginfo("[%s] Settings: " % rospy.get_name() +
              "\n autostart = " + str(autostart) +
              "\n num_cols  = " + str(num_cols) +
              "\n num_rows  = " + str(num_rows) +
              "\n dim       = " + str(dim) +
              "\n pattern   = " + str(pattern) )
            
    CalibrationDetectorServer(rospy.get_name())
    
    if(autostart):
        rospy.loginfo("[%s] Auto-starting: %dx%d %f" % (rospy.get_name(), num_cols, num_rows , dim))
        pub = rospy.Publisher("~goal", ConfigActionGoal, latch=True)
        msg = ConfigActionGoal()
        msg.goal.num_x = num_cols
        msg.goal.num_y = num_rows
        msg.goal.spacing_x = dim
        msg.goal.spacing_y = dim
        msg.goal.pattern = pattern
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "auto-start"
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = "auto-start"
        pub.publish(msg)
    else:
        print "Autostart is false"
    rospy.spin()
