#!/usr/bin/env python
import rospy
import roslib
import actionlib

roslib.load_manifest("image_cb_detector")
from camera_calibrator.calibration_plate import *
from image_cb_detector.msg import *
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
        self._calImagePub = rospy.Publisher("calibration_image", Image)
        self._board = None
        self._server.start()

    def goal_callback(self):
        goal = self._server.accept_new_goal()
        rospy.loginfo("[%s] Starting calibration plate detector" % rospy.get_name())
        # create the calibrtion plate
        if goal.pattern == ConfigGoal.CHECKERBOARD:
            self._board = Chessboard(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y, goal.flags)
        elif goal.pattern == ConfigGoal.CIRCLES:
            self._board = Circles(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y, goal.flags)
        elif goal.pattern == ConfigGoal.ACIRCLES:
            self._board = ACircles(self._action_name, goal.num_x, goal.num_y, goal.spacing_x, goal.spacing_y, goal.flags)
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
        if(self._featurePub and self._board and self._server.is_active()):
            ok, corners, mono, (x_scale, y_scale) = self._board.detect(img)
            msg = self._board.mk_pattern_msg(ok, corners)
            msg.header = img.header
            self._featurePub.publish(msg)
            mono_msg = self._board.mk_image_msg(mono, corners, ok)
            mono_msg.header = img.header
            self._calImagePub.publish(mono_msg)

if __name__=='__main__':
    rospy.init_node("image_calibration_detector")
    
    autostart = rospy.get_param("~autostart", False)
    num_cols = rospy.get_param("~num_cols", 8)
    num_rows = rospy.get_param("~num_rows", 6)
    dim = rospy.get_param("~dim", 1)
    pattern = rospy.get_param("~pattern", ConfigGoal.CHECKERBOARD)
    flags = rospy.get_param("~flags", 0)
    
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-a", "--autostart", dest="autostart", type="string", default=autostart,help="Autostart image_calibration_detector")
    parser.add_option("-c", "--cols", dest= "num_cols", type="int", default=num_cols, help="Number of cols in calibration pattern")
    parser.add_option("-r", "--rows", dest= "num_rows", type="int", default=num_rows, help="Number of rows in calibration pattern")
    parser.add_option("-d", "--dim", dest = "dim", type="float", default=dim, help="The length in meters between calibration points")
    parser.add_option("-p", "--pattern", dest="pattern", type="int", default=pattern, help="The type of calibration pattern (ENUM: 0: CHECKERBOARD, 1: CIRCLES, 2: ACIRCLES")
    parser.add_option("-f", "--flags", dest="flags", type="int", default=flags, help="Flags to send to the calibrator")
    
    options, args = parser.parse_args()
    
    autostart = options.autostart.lower()=='true'
    num_cols = options.num_cols
    num_rows = options.num_rows
    dim = options.dim
    pattern = options.pattern
    flags = options.flags
    
    rospy.loginfo("[%s][autostart] %s" % (rospy.get_name(), str(autostart)))
    rospy.loginfo("[%s][num_cols] %s" %(rospy.get_name(), str(num_cols)))
    rospy.loginfo("[%s][num_rows] %s" %(rospy.get_name(), str(num_rows)))
    rospy.loginfo("[%s][dim] %s" % (rospy.get_name(), str(dim)))
    rospy.loginfo("[%s][pattern] %s" % (rospy.get_name(), str(pattern)))
    rospy.loginfo("[%s][flags] %s" % (rospy.get_name(), str(flags)))
            
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
        msg.goal.flags = flags
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "auto-start"
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = "auto-start"
        pub.publish(msg)
    else:
        print "Autostart is false"
    rospy.spin()
