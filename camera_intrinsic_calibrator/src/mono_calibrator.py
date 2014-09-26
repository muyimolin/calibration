#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Oceaneering Space Systems / NASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Allison Thackston
import roslib; roslib.load_manifest("camera_intrinsic_calibrator")
import rospy
import actionlib
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

class MonoCalibratorServer:
    def __init__(self, name, use_service, params):
        self._action_name = name
        self._use_service = use_service
        self._camera_name = params.camera_name
        self._file_name = params.file_name
        self._server = None
        
        self._feature_sub = rospy.Subscriber(params.features, CalibrationPattern, self.feature_callback)
        self._camerainfo_sub = rospy.Subscriber(params.camera_info, CameraInfo, self.camerainfo_callback)
        
        if(use_service):
            rospy.loginfo("[%s] Wating for service " % rospy.get_name() + rospy.resolve_name(params.service_name) + "...")
            try:
                rospy.wait_for_service(params.service_name, 5)
                rospy.loginfo("OK")
            except rospy.ROSException:
                rospy.logerr("[%s] Service not found" % rospy.get_name())
                rospy.signal_shutdown('Required camera service not found')
                
            self.set_camera_info_service = rospy.ServiceProxy(params.service_name, sensor_msgs.srv.SetCameraInfo)
        
        self._calibrator = None
        self._camerainfo = None
        
    def camerainfo_callback(self, msg):
        self._camerainfo = msg
        if(msg.binning_x > 1 or msg.binning_y > 1):
            rospy.logerror("Attempting to calibrate a binned image!")
            self._camerainfo = None
            return
        if not self._server:
            self._server = actionlib.SimpleActionServer(self._action_name, MonoCalibratorAction, None, False)
            self._server.register_goal_callback(self.goal_callback)
            self._server.register_preempt_callback(self.preempt_callback)
            self._server.start()    
        
    def goal_callback(self):
        goal = self._server.accept_new_goal()
        rospy.loginfo("[%s] Starting calibration" % rospy.get_name())
        if(not self._camerainfo):
            self._server.set_aborted(None, "Haven't received camera info message yet")
            return
        calib_flags = 0
        if goal.fix_principal_point:
            calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        if goal.fix_aspect_ratio:
            calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
        if goal.zero_tangent_dist:
            calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
        if (goal.num_coeffs > 3):
            calib_flags |= cv2.CALIB_RATIONAL_MODEL
        if (goal.num_coeffs < 6):
            calib_flags |= cv2.CALIB_FIX_K6
        if (goal.num_coeffs < 5):
            calib_flags |= cv2.CALIB_FIX_K5
        if (goal.num_coeffs < 4):
            calib_flags |= cv2.CALIB_FIX_K4
        if (goal.num_coeffs < 3):
            calib_flags |= cv2.CALIB_FIX_K3
        if (goal.num_coeffs < 2):
            calib_flags |= cv2.CALIB_FIX_K2
        if (goal.num_coeffs < 1):
            calib_flags |= cv2.CALIB_FIX_K1
        
        calib_ranges = None
        if (len(goal.param_ranges)==4):
            calib_ranges = goal.param_ranges
        
        self._calibrator = MonoCalibrator(self._camerainfo, flags=calib_flags, param_ranges=calib_ranges)
            
        if self._server.is_preempt_requested():
            self._calibrator = None
            self._server.set_preempted()
            
    def preempt_callback(self):
        self._calibrator = None
        self._server.set_preempted()
        
    def feature_callback(self, msg):
        rospy.logdebug("[%s] Got new feature message" % rospy.get_name())
        if (self._server and self._calibrator and self._server.is_active()):
            progress, goodenough = self._calibrator.handle_msg(msg)
            feedback = mk_coverage_msg(progress, goodenough)

            mono_feedback_msg = MonoCalibratorFeedback()
            mono_feedback_msg.feedback = feedback
            self._server.publish_feedback(mono_feedback_msg)
            if goodenough:
                self.calibrate()
                
    def calibrate(self):
        if(self._server.is_active() and self._calibrator):
            rospy.loginfo("[%s] Doing calibration..." % rospy.get_name())
            D,I,R,P = self._calibrator.do_calibration()
            result_msg = MonoCalibratorResult()
            result_msg.result = self._calibrator.mk_camera_info_msg(D, I, R, P)
            print result_msg
            self._server.set_succeeded(result_msg)
            
            if(self._use_service):
                rospy.loginfo("[%s] Saving calibration to camera" % rospy.get_name())
                self.set_camera_info_service(result_msg.result)
            else:
                rospy.loginfo("[%s] Saving calibration file to %s" % (rospy.get_name(), self._file_name))
                writeCalibration(self._file_name, self._camera_name, result_msg.result)
            
            rospy.loginfo("[%s] Finished Calibrating" % rospy.get_name())
            self._calibrator = None
            
        
    
if __name__=='__main__':
    rospy.init_node("mono_calibrator")
    rospy.loginfo("[%s] Starting.." % rospy.get_name())
    params = CalibratorParams()
    use_service = rospy.get_param("~use_service", True)
    autostart = rospy.get_param("~autostart", False)

    params.features     = rospy.remap_name("features")
    params.camera_info  = rospy.remap_name("camera_info")
    params.service_name = "%s/set_camera_info" % rospy.remap_name("camera")
    params.camera_name  = rospy.get_param("~camera_name", "camera")
    params.file_name    = rospy.get_param("~file_name", "camera.yaml")
    
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-a", "--autostart", dest="autostart", type="string", default=autostart,help="Autostart image_calibration_detector")
    parser.add_option("-s", "--use_service", dest="use_service", type="string", default=str(use_service), help="Use service to stet parameters")
    parser.add_option("-c", "--camera_name", dest= "camera_name", type="string", default=params.camera_name, help="Name of the camera to use in the calibration file if not using service")
    parser.add_option("-f", "--file_name", dest= "file_name", type="string", default=params.file_name, help="Name and location to save the calibration file if not using service")
    
    options, args = parser.parse_args()
    
    autostart = options.autostart.lower()=='true'
    use_service = options.use_service.lower()=='true'
    params.camera_name = options.camera_name
    params.file_name = options.file_name
    
    rospy.loginfo("[%s] Settings: " % rospy.get_name() +
                  "\n autostart           = " + str(autostart) +
                  "\n use_service         = " + str(use_service) +
                  "\n features topic      = " + rospy.resolve_name(params.features) +
                  "\n camera_info topic   = " + rospy.resolve_name(params.camera_info) +
                  "\n set_camera_info srv = " + rospy.resolve_name(params.service_name) +
                  "\n save camera name    = " + params.camera_name +
                  "\n save file name      = " + params.file_name )
    
    MonoCalibratorServer(rospy.get_name(), use_service, params)
    
    if (autostart):
        rospy.loginfo("[%s] Auto-starting..." % rospy.get_name())
        pub = rospy.Publisher("~goal", MonoCalibratorActionGoal, latch=True)
        msg = MonoCalibratorActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "auto-start"
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = "auto-start"
        msg.goal.num_coeffs = 3
        pub.publish(msg)
        
    rospy.spin()
