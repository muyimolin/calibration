#!/usr/bin/env python
import roslib
roslib.load_manifest("camera_cb_detector")
import math
import operator

import cv
import cv2
import cv_bridge
import numpy
import numpy.linalg

import image_geometry
import sensor_msgs.msg
import geometry_msgs.msg
import calibration_msgs.msg

# Helper functions

def _pdist(p1, p2):
    """
    Distance bwt two points. p1 = (x, y), p2 = (x, y)
    """
    return math.sqrt(math.pow(p1[0] - p2[0], 2) + math.pow(p1[1] - p2[1], 2))

# Supported calibration patterns
class Patterns:
    Chessboard, Circles, ACircles = range(3)

class CalibrationException(Exception):
    pass


class CalibrationPlate(object):
    def __init__(self, name, n_cols = 0, n_rows = 0, dim_x = 1.0, dim_y = 1.0):
        self.name = name
        self.n_cols = n_cols
        self.n_rows = n_rows
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.num_pts = self.n_cols * self.n_rows
        self.br = cv_bridge.CvBridge()
    
    def mk_gray(self, msg):
        """
        Convert a message into a bgr8 OpenCV bgr8 *monochrome* image.
        Deal with bayer images by converting to color, then to monochrome.
        """
        # TODO OpenCV supports converting Bayer->monochrome directly now
        # TODO Convert to one-channel monochrome instead, let other code
        #      (e.g. downsample_and_detect) expand to bgr8.
        if 'bayer' in msg.encoding:
            converter = {
                "bayer_rggb8" : cv.CV_BayerBG2BGR,
                "bayer_bggr8" : cv.CV_BayerRG2BGR,
                "bayer_gbrg8" : cv.CV_BayerGR2BGR,
                "bayer_grbg8" : cv.CV_BayerGB2BGR }[msg.encoding]
            msg.encoding = "mono8"
            raw = self.br.imgmsg_to_cv(msg)
            rgb = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC3)
            mono = cv.CreateMat(raw.rows, raw.cols, cv.CV_8UC1)

            cv.CvtColor(raw, rgb, converter)
            cv.CvtColor(rgb, mono, cv.CV_BGR2GRAY)
            cv.CvtColor(mono, rgb, cv.CV_GRAY2BGR)
        else:
            rgb = self.br.imgmsg_to_cv(msg, "bgr8")

        return rgb
        
class Chessboard(CalibrationPlate):
    def __init__(self, *args):
        CalibrationPlate.__init__(self, *args)
        
    def get_corners(self, img, refine=True):
        """
        Get corners for a particular chessboard for an image
        """
        w, h = cv.GetSize(img)
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
        (ok, corners) = cv.FindChessboardCorners(mono, (self.n_cols, self.n_rows), cv.CV_CALIB_CB_ADAPTIVE_THRESH | cv.CV_CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        
        # If any corners are within BORDER pixels of the screen edge, reject the detection by setting ok to false
        # NOTE: This may cause problems with very low-resolution cameras, where 8 pixels is a non-negligible fraction
        # of the image size. See http://answers.ros.org/question/3155/how-can-i-calibrate-low-resolution-cameras
        BORDER = 8
        if not all([(BORDER < x < (w - BORDER)) and (BORDER < y < (h - BORDER)) for (x, y) in corners]):
            ok = False
            
        if refine and ok:
            # Use a radius of half the minimum distance between corners. This should be large enough to snap to the
            # correct corner, but not so large as to include a wrong corner in the search window.
            min_distance = float("inf")
            for row in range(self.n_rows):
                for col in range(self.n_cols - 1):
                    index = row*self.n_cols + col
                    min_distance = min(min_distance, _pdist(corners[index], corners[index + 1]))
            for row in range(self.n_rows - 1):
                for col in range(self.n_cols):
                    index = row*self.n_cols + col
                    min_distance = min(min_distance, _pdist(corners[index], corners[index + self.n_cols]))
            radius = int(math.ceil(min_distance * 0.5))
            corners = cv.FindCornerSubPix(mono, corners, (radius,radius), (-1,-1),
                                          ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
            
        return (ok, corners)
    
    def detect(self, msg):
        """
        Downsample the input image to approximately VGA resolution and detect the
        calibration target corners in the full-size image.

        Combines these apparently orthogonal duties as an optimization. Checkerboard
        detection is too expensive on large images, so it's better to do detection on
        the smaller display image and scale the corners back up to the correct size.

        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale)).
        """
        rgb = self.mk_gray(msg)
        # Scale the input image down to ~VGA size
        (width, height) = cv.GetSize(rgb)
        scale = math.sqrt( (width*height) / (640.*480.) )
        if scale > 1.0:
            scrib = cv.CreateMat(int(height / scale), int(width / scale), cv.GetElemType(rgb))
            cv.Resize(rgb, scrib)
        else:
            scrib = cv.CloneMat(rgb)
        # Due to rounding, actual horizontal/vertical scaling may differ slightly
        x_scale = float(width) / scrib.cols
        y_scale = float(height) / scrib.rows

        (ok, downsampled_corners) = self.get_corners(scrib, refine = True)

        # Scale corners back to full size image
        if scale > 1.0:
            # Refine up-scaled corners in the original full-res image
            # TODO Does this really make a difference in practice?
            corners_unrefined = [(c[0]*x_scale, c[1]*y_scale) for c in downsampled_corners]
            # TODO It's silly that this conversion is needed, this function should just work
            #      on the one-channel mono image
            mono = cv.CreateMat(rgb.rows, rgb.cols, cv.CV_8UC1)
            cv.CvtColor(rgb, mono, cv.CV_BGR2GRAY)
            radius = int(math.ceil(scale))
            corners = cv.FindCornerSubPix(mono, corners_unrefined, (radius,radius), (-1,-1),
                                          ( cv.CV_TERMCRIT_EPS+cv.CV_TERMCRIT_ITER, 30, 0.1 ))
        else:
            corners = downsampled_corners

        return (ok, corners, rgb, (x_scale, y_scale))
    
    def mk_pattern_msg(self, ok, corners):
        msg = calibration_msgs.msg.CalibrationPattern()
        for y in range(self.n_rows):
            for x in range(self.n_cols):
                point = geometry_msgs.msg.Point()
                point.x = x*self.dim_x
                point.y = y*self.dim_y
                point.z = 0
                msg.object_points.append(point)
        if (corners):
            for i in range(len(corners)):
                point = geometry_msgs.msg.Point()
                point.x = corners[i][0]
                point.y = corners[i][1]
                point.z = 0
                msg.image_points.append(point)
        if (corners and ok):
            msg.success = ok
        else:
            msg.success = False    
        return msg
        
        
class Circles(CalibrationPlate):
    def __init__(self, *args):
        CalibrationPlate.__init__(self, *args)
        
    def get_corners(self, img):
        """
        Get circle centers for a symmetric or asymmetric grid
        """
        w, h = cv.GetSize(img)
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
        mono_arr = numpy.array(mono)
        (ok, corners) = cv2.findCirclesGrid(mono_arr, (self.n_cols, self.n_rows), cv2.CALIB_CB_SYMMETRIC_GRID)

        # In symmetric case, findCirclesGrid does not detect the target if it's turned sideways. So we try
        # again with dimensions swapped - not so efficient.
        # TODO Better to add as second board? Corner ordering will change.
        if not ok:
            (ok, corners) = cv2.findCirclesGrid(mono_arr, (self.n_rows, self.n_cols), cv2.CALIB_CB_SYMMETRIC_GRID)

        # For some reason findCirclesGrid returns centers as [[x y]] instead of (x y) like FindChessboardCorners
        if corners is not None:
            corners = [(x, y) for [[x, y]] in corners]
    
        return (ok, corners)
    
    def detect(self, msg):
        """
        Detect the calibration target corners in the full-size image.
        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale)).
        """
        rgb = self.mk_gray(msg)
        x_scale = 1
        y_scale = 1
        # Circle grid detection is fast even on large images
        (ok, corners) = self.get_corners(rgb)

        return (ok, corners, rgb, (x_scale, y_scale))

    def mk_pattern_msg(self, ok, corners):
        msg = calibration_msgs.msg.CalibrationPattern()
        for y in range(self.n_rows):
            for x in range(self.n_cols):
                point = geometry_msgs.msg.Point()
                point.x = x*self.dim_x
                point.y = y*self.dim_y
                point.z = 0
                msg.object_points.append(point)
        if (corners):
            for i in range(len(corners)):
                point = geometry_msgs.msg.Point()
                point.x = corners[i][0]
                point.y = corners[i][1]
                point.z = 0
                msg.image_points.append(point)
        if (corners and ok):
            msg.success = ok
        else:
            msg.success = False    
        return msg
         
class ACircles(CalibrationPlate):
    def __init__(self, *args):
        CalibrationPlate.__init__(self, *args)
        
    def get_corners(self, img):
        """
        Get circle centers for a symmetric or asymmetric grid
        """
        w, h = cv.GetSize(img)
        mono = cv.CreateMat(h, w, cv.CV_8UC1)
        cv.CvtColor(img, mono, cv.CV_BGR2GRAY)
        mono_arr = numpy.array(mono)
        (ok, corners) = cv2.findCirclesGrid(mono_arr, (self.n_cols, self.n_rows), cv2.CALIB_CB_ASYMMETRIC_GRID)

        # For some reason findCirclesGrid returns centers as [[x y]] instead of (x y) like FindChessboardCorners
        if corners is not None:
            corners = [(x, y) for [[x, y]] in corners]

        return (ok, corners)
    
    def detect(self, msg):
        """
        Detect the calibration target corners in the full-size image.
        Returns (scrib, corners, downsampled_corners, board, (x_scale, y_scale)).
        """
        rgb = self.mk_gray(msg)
        x_scale = 1
        y_scale = 1
        # Circle grid detection is fast even on large images
        (ok, corners) = self.get_corners(rgb)

        return (ok, corners, rgb, (x_scale, y_scale))
    
    def mk_pattern_msg(self, ok, corners):
        msg = calibration_msgs.msg.CalibrationPattern()
        for y in range(self.n_rows):
            for x in range(self.n_cols):
                point = geometry_msgs.msg.Point()
                point.x = (2*x+y%2)*self.dim_x
                point.y = y*self.dim_y
                point.z = 0
                msg.object_points.append(point)
        if (corners):
            for i in range(len(corners)):
                point = geometry_msgs.msg.Point()
                point.x = corners[i][0]
                point.y = corners[i][1]
                point.z = 0
                msg.image_points.append(point)
        if (corners and ok):
            msg.success = ok
        else:
            msg.success = False    
        return msg
    
