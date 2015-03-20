#!/usr/bin/env python
#
# Copyright (c) 2014 United States Government as represented by the
# National Aeronotics and Space Administration.  All Rights Reserved
#
# Author: Allison Thackston
# Created: 24 Oct 2014
#
# Developed jointly by NASA/JSC and Oceaneering Space Systems
#
# Licensed under the NASA Open Source Agreement v1.3 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://opensource.org/licenses/NASA-1.3
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###########################################################################

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
            rospy.loginfo("[%s] fix principle point" % self._action_name)
        if goal.fix_aspect_ratio:
            calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
            rospy.loginfo("[%s] fix aspect ratio" % self._action_name)
        if goal.zero_tangent_dist:
            calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
            rospy.loginfo("[%s] zero tangent dist" % self._action_name)
        if (goal.num_coeffs > 3):
            calib_flags |= cv2.CALIB_RATIONAL_MODEL
            rospy.loginfo("[%s] rational model" % self._action_name)
        if (goal.num_coeffs < 6):
            calib_flags |= cv2.CALIB_FIX_K6
            rospy.loginfo("[%s] fix K6" % self._action_name)
        if (goal.num_coeffs < 5):
            calib_flags |= cv2.CALIB_FIX_K5
            rospy.loginfo("[%s] fix K5" % self._action_name)
        if (goal.num_coeffs < 4):
            calib_flags |= cv2.CALIB_FIX_K4
            rospy.loginfo("[%s] fix K4" % self._action_name)
        if (goal.num_coeffs < 3):
            calib_flags |= cv2.CALIB_FIX_K3
            rospy.loginfo("[%s] fix K3" % self._action_name)
        if (goal.num_coeffs < 2):
            calib_flags |= cv2.CALIB_FIX_K2
            rospy.loginfo("[%s] fix K2" % self._action_name)
        if (goal.num_coeffs < 1):
            calib_flags |= cv2.CALIB_FIX_K1
            rospy.loginfo("[%s] fix K1" % self._action_name)
        
        calib_ranges = None
        if (len(goal.param_ranges)==4):
            calib_ranges = goal.param_ranges
            rospy.loginfo("[%s] params: [%f, %f, %f %f]" % (self._action_name, param_ranges[0], param_ranges[1], param_ranges[2], param_ranges[3]))
        
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
    
    # Show user the parameters used
    rospy.loginfo("[" + rospy.get_name() + "] ~autostart:=" + str(autostart))
    rospy.loginfo("[" + rospy.get_name() + "] ~use_service:=" + str(use_service))
    rospy.loginfo("[" + rospy.get_name() + "] features:=" + rospy.resolve_name(params.features))
    rospy.loginfo("[" + rospy.get_name() + "] camera_info:=" + rospy.resolve_name(params.camera_info))
    rospy.loginfo("[" + rospy.get_name() + "] set_camera_info:=" + rospy.resolve_name(params.service_name))
    if (not use_service):
        rospy.loginfo("[" + rospy.get_name() + "] ~camera_name:=" + params.camera_name)
        rospy.loginfo("[" + rospy.get_name() + "] ~file_name:=" + params.file_name)
    
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
