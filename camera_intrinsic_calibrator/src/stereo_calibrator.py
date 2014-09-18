#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest("camera_intrinsic_calibrator")
import actionlib
import message_filters
import functools
from calibrator.approxsync import ApproximateTimeSynchronizer
from calibrator.calibrator import *
from calibrator.parser import *
# Messages
from camera_intrinsic_calibrator.msg import *
from calibration_msgs.msg import CalibrationPattern
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo

class CalibratorParams:
    features = "features"
    camera_info = "camera_info"
    service_name = "driver/set_camera_info"
    camera_name = "camera"
    file_name = "camera.yaml"
    
def mk_coverage_msg(progress, good):
    
    feedback_msg = CalibrationCoverage()
    if(progress):
        for p in progress:
            if p[0] == "X":
                feedback_msg.x.min = p[1]
                feedback_msg.x.max = p[2]
                feedback_msg.x.percent = p[3]
            elif p[0] == "Y":
                feedback_msg.y.min = p[1]
                feedback_msg.y.max = p[2]
                feedback_msg.y.percent = p[3]
            elif p[0] == "Size":
                feedback_msg.size.min = p[1]
                feedback_msg.size.max = p[2]
                feedback_msg.size.percent = p[3]
            elif p[0] == "Skew":
                feedback_msg.skew.min = p[1]
                feedback_msg.skew.max = p[2]
                feedback_msg.skew.percent = p[3]
        feedback_msg.goodenough = good
    return feedback_msg

class StereoCalibratorServer:
    def __init__(self, name, synchronizer, use_service, left_params, right_params):
        self._action_name = name
        self._use_service = use_service
        self._left_params = left_params
        self._right_params = right_params
        self._server = None
        
        self._lsub = message_filters.Subscriber(left_params.features,  CalibrationPattern)
        self._rsub = message_filters.Subscriber(right_params.features,  CalibrationPattern)
        ts = synchronizer([self._lsub, self._rsub], 4)
        ts.registerCallback(self.feature_callback)

        self._linfo = message_filters.Subscriber(left_params.camera_info,  CameraInfo)
        self._rinfo = message_filters.Subscriber(right_params.camera_info, CameraInfo)
        info = synchronizer([self._linfo, self._rinfo], 4)
        info.registerCallback(self.camerainfo_callback)
        
        if(use_service):
            rospy.loginfo("Wating for service " + rospy.resolve_name(left_params.service_name) + "...")
            try:
                rospy.wait_for_service(left_params.service_name, 5)
                rospy.loginfo("OK")
            except rospy.ROSException:
                rospy.logerr("Service not found")
                rospy.signal_shutdown('Quit')
                
            self.set_left_info_service = rospy.ServiceProxy(left_params.service_name, sensor_msgs.srv.SetCameraInfo)
            
            rospy.loginfo("Wating for service " + rospy.resolve_name(right_params.service_name) + "...")
            try:
                rospy.wait_for_service(right_params.service_name, 5)
                rospy.loginfo("OK")
            except rospy.ROSException:
                rospy.logerr("Service not found")
                rospy.signal_shutdown('Quit')
                
            self.set_right_info_service = rospy.ServiceProxy(right_params.service_name, sensor_msgs.srv.SetCameraInfo)
        
        self._calibrator = None
        self._camerainfo = None
        
    def camerainfo_callback(self, lmsg, rmsg):
        rospy.logdebug("in camerainfo_callback")
        
        if(lmsg.binning_x > 1 or lmsg.binning_y > 1 or
           rmsg.binning_x > 1 or rmsg.binning_y > 1):
            rospy.logerror("Attempting to calibrate a binned image!")
            self._camera_info = None
            return
        # Check that sizes match
        if not self._server:
            if(lmsg.width == rmsg.width and
               lmsg.height == rmsg.height and
               lmsg.binning_x == rmsg.binning_x and
               lmsg.binning_y == rmsg.binning_y and
               lmsg.roi == rmsg.roi):
                #set camera info and start action server
                rospy.logdebug("cameras match")
                self._camerainfo = lmsg

                self._server = actionlib.SimpleActionServer(self._action_name, StereoCalibratorAction, None, False)
                self._server.register_goal_callback(self.goal_callback)
                self._server.register_preempt_callback(self.preempt_callback)
                self._server.start()
            else:
                rospy.logerr("Left and Right Camera Infos do not match!")
                self._camera_info = None
        
    def goal_callback(self):
        goal = self._server.accept_new_goal()
        rospy.loginfo("Starting calibration")
        if(not self._camerainfo):
            self._server.set_aborted(None, "Haven't received valid camera info message yet!")
            return
        self._calibrator = StereoCalibrator(self._camerainfo)
            
        if self._server.is_preempt_requested():
            self._calibrator = None
            self._server.set_preempted()
            
    def preempt_callback(self):
        self._calibrator = None
        self._server.set_preempted()
        
    def feature_callback(self, lmsg, rmsg):
        rospy.logdebug("Got new feature messages")
        if(self._server and self._calibrator and self._server.is_active()):
            [(progress, good), (lprogress, lgood), (rprogress, rgood)] = self._calibrator.handle_msg(lmsg, rmsg)
            feedback = mk_coverage_msg(progress, good)
            left_feedback = mk_coverage_msg(lprogress, lgood)
            right_feedback = mk_coverage_msg(rprogress, rgood)
            
            stereo_feedback_msg = StereoCalibratorFeedback()
            stereo_feedback_msg.stereo_feedback = feedback
            stereo_feedback_msg.left_feedback = left_feedback
            stereo_feedback_msg.right_feedback = right_feedback
            self._server.publish_feedback(stereo_feedback_msg)
            if good:
                self.calibrate()
    
    def calibrate(self):
        if(self._server and self._calibrator and self._server.is_active()):
            rospy.loginfo("Doing calibration...")
            left_D, left_I, left_R, left_P, right_D, right_I, right_R, right_P = self._calibrator.do_calibration()
            result_msg = StereoCalibratorResult()
            result_msg.left_result = self._calibrator.l.mk_camera_info_msg(left_D, left_I, left_R, left_P)
            result_msg.right_result = self._calibrator.r.mk_camera_info_msg(right_D, right_I, right_R, right_P)
            print result_msg.left_result
            print result_msg.right_result
            self._server.set_succeeded(result_msg)
            
            if(self._use_service):
                rospy.loginfo("Saving calibration to camera")
                self.set_left_info_service(result_msg.left_result)
                self.set_right_info_service(result_msg.right_result)
            else:
                rospy.loginfo("Saving calibration file")
                writeCalibration(self._left_params.file_name, self._left_params.camera_name, result_msg.left_result)
                writeCalibration(self._right_params.file_name, self._right_params.camera_name, result_msg.right_result)
            
            rospy.loginfo("Finished Calibrating")
            self._calibrator = None
            #rospy.signal_shutdown('Finished')
                    
    
if __name__=='__main__':
    rospy.init_node("stereo_calibrator")
    rospy.loginfo("Starting %s" % rospy.get_name())
    
    # Get parameters
    left_params = CalibratorParams()
    right_params = CalibratorParams()
    autostart = rospy.get_param("~autostart", False)
    use_service = rospy.get_param("~use_service", True)
    approximate = rospy.get_param("~approximate", 0.0)  

    left_params.features     = "%s/features" % rospy.remap_name("left")
    left_params.camera_info  = "%s/camera_info" % rospy.remap_name("left")
    left_params.service_name = "%s/set_camera_info" % rospy.remap_name("left_camera")
    left_params.camera_name  = rospy.get_param("~left_camera_name", "left")
    left_params.file_name    = rospy.get_param("~left_file_name", "left.yaml")
    
    right_params.features     = "%s/features" % rospy.remap_name("right")
    right_params.camera_info  = "%s/camera_info" % rospy.remap_name("right")
    right_params.service_name = "%s/set_camera_info" % rospy.remap_name("right_camera")
    right_params.camera_name  = rospy.get_param("~right_camera_name", "right")
    right_params.file_name    = rospy.get_param("~right_file_name", "right.yaml")
    
    # If command line options exist, override parameters
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-a", "--autostart", dest="autostart", type="string", default=autostart,help="Autostart image_calibration_detector")
    parser.add_option("-s", "--use_service", dest="use_service", type="string", default=str(use_service), help="Use service to set parameters")
    parser.add_option("-x", "--approximate", dest="approximate", type="float", default=approximate, help="Approximate sync time for cameras (0 for synced)")
    parser.add_option("-l", "--left_camera_name", dest= "left_camera_name", type="string", default=left_params.camera_name, help="Name of the left camera to use in the calibration file if not using service")
    parser.add_option("-L", "--left_file_name", dest= "left_file_name", type="string", default=left_params.file_name, help="Name and location to save the left calibration file if not using service")
    parser.add_option("-r", "--right_camera_name", dest= "right_camera_name", type="string", default=right_params.camera_name, help="Name of the right camera to use in the calibration file if not using service")
    parser.add_option("-R", "--right_file_name", dest= "right_file_name", type="string", default=right_params.file_name, help="Name and location to save the right calibration file if not using service")
    
    options, args = parser.parse_args()
    
    autostart = options.autostart.lower()=='true'
    use_service = options.use_service.lower()=='true'
    approximate = options.approximate
    left_params.camera_name = options.left_camera_name
    left_params.file_name = options.left_file_name
    right_params.camera_name = options.right_camera_name
    right_params.file_name = options.right_file_name
    
    # Show user the parameters used
    rospy.loginfo("Settings for " + rospy.get_name() + ": " +
                  "\n autostart           = " + str(autostart) +
                  "\n use_service         = " + str(use_service) +
                  "\n approximate         = " + str(approximate) +
                  "\n **LEFT CAMERA** " +
                  "\n features topic      = " + rospy.resolve_name(left_params.features) +
                  "\n camera_info topic   = " + rospy.resolve_name(left_params.camera_info) +
                  "\n set_camera_info srv = " + rospy.resolve_name(left_params.service_name) +
                  "\n save camera name    = " + left_params.camera_name +
                  "\n save file name      = " + left_params.file_name +
                  "\n **RIGHT CAMERA** " +
                  "\n features topic      = " + rospy.resolve_name(right_params.features) +
                  "\n camera_info topic   = " + rospy.resolve_name(right_params.camera_info) +
                  "\n set_camera_info srv = " + rospy.resolve_name(right_params.service_name) +
                  "\n save camera name    = " + right_params.camera_name +
                  "\n save file name      = " + right_params.file_name )
                
    if (approximate == 0.0):
        sync = message_filters.TimeSynchronizer
    else:
        sync = functools.partial(ApproximateTimeSynchronizer, slop=approximate)
    
    StereoCalibratorServer(rospy.get_name(), sync, use_service, left_params, right_params)
    
    # Autostart if requested
    if (autostart):
        rospy.loginfo("Auto-starting " + rospy.get_name())
        pub = rospy.Publisher("~goal", StereoCalibratorActionGoal, latch=True)
        msg = StereoCalibratorActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "auto-start"
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = "auto-start"
        pub.publish(msg)
    
    rospy.spin()
