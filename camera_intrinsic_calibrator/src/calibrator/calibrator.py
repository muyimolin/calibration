#!/usr/bin/env python

import roslib
import cv
import cv2
import cv_bridge
import numpy
import numpy.linalg
import math

from sensor_msgs.msg import *
from calibration_msgs.msg import *

def lmin(seq1, seq2):
    """ Pairwise minimum of two sequences """
    return [min(a, b) for (a, b) in zip(seq1, seq2)]

def lmax(seq1, seq2):
    """ Pairwise maximum of two sequences """
    return [max(a, b) for (a, b) in zip(seq1, seq2)]

def mean(seq):
    return sum(seq) / len(seq)

def cvmat_iterator(cvmat):
    for i in range(cvmat.rows):
        for j in range(cvmat.cols):
            yield cvmat[i,j]
        
class CalibrationException(Exception):
    pass
        
class Calibrator:
    def __init__(self, info):
        self._info = info
        self._width = info.width
        self._height = info.height
        self.size = (info.width, info.height)

    def mk_camera_info_msg(self, d, k, r, p):
        """
        Return CameraInfo message for given calibration matrices
        """
        msg = CameraInfo()
        msg.header = self._info.header
        (msg.width, msg.height) = self.size
        if d.rows > 5:
            msg.distortion_model = "rational_polynomial"
        else:
            msg.distortion_model = "plumb_bob"
        msg.D = [d[i,0] for i in range(d.rows)]
        msg.K = list(cvmat_iterator(k))
        msg.R = list(cvmat_iterator(r))
        msg.P = list(cvmat_iterator(p))
        msg.binning_x = self._info.binning_x
        msg.binning_y = self._info.binning_y
        msg.roi = self._info.roi
        print msg
        return msg
        
    def mk_object_points(self, object_points):
        total_num = len(object_points)*len(object_points[0])
        opts = cv.CreateMat(total_num, 3, cv.CV_32FC1)
        idx = 0
        for points in object_points:
            for j in range(len(points)):
                opts[idx + j, 0] = points[j][0]
                opts[idx + j, 1] = points[j][1]
                opts[idx + j, 2] = 0
            idx += len(points)
        return opts
    
    def mk_image_points(self, good_corners):
        total_num = len(good_corners)*len(good_corners[0])
        ipts = cv.CreateMat(total_num, 2, cv.CV_32FC1)
        idx = 0
        for corners in good_corners:
            for i in range(len(corners)):
                ipts[idx + i, 0] = corners[i][0]
                ipts[idx + i, 1] = corners[i][1]
            idx += len(corners)
        
        image_points = cv.Reshape(ipts, 2)

        return image_points
    
    def mk_point_counts(self, object_points):
        npts = cv.CreateMat(len(object_points), 1, cv.CV_32SC1)
        for i, points in enumerate(object_points):
            npts[i, 0] = len(points)
        return npts
        
    def get_parameters(self, corners, object_points):
        """
        Return list of parameters [X, Y, size, skew] describing the checkerboard view.
        """
        
        def get_outside_corners(corners, object_points):
            """
            Return the four corners of the board as a whole, as (up_left, up_right, down_right, down_left).
            """
            Xs = [ x for (x, y) in object_points]
            Ys = [ y for (x, y) in object_points]
            xdim = Xs.count(0.0)
            ydim = Ys.count(0.0)
            
            if len(corners) != xdim * ydim:
                raise CalibrationException("Invalid number of corners! %d corners. X: %d, Y: %d" % 
                                           (len(corners), xdim, ydim))

            up_left    = numpy.array( corners[0] )
            up_right   = numpy.array( corners[ydim - 1] )
            down_right = numpy.array( corners[-1] )
            down_left  = numpy.array( corners[-ydim] )

            return (up_left, up_right, down_right, down_left)
        
        def get_skew(corners, object_points):
            """
            Get skew for given checkerboard detection. 
            Scaled to [0,1], which 0 = no skew, 1 = high skew
            Skew is proportional to the divergence of three outside corners from 90 degrees.
            """
            # TODO Using three nearby interior corners might be more robust, outside corners occasionally
            # get mis-detected
            up_left, up_right, down_right, _ = get_outside_corners(corners, object_points)

            def angle(a, b, c):
                """
                Return angle between lines ab, bc
                """
                ab = a - b
                cb = c - b
                return math.acos(numpy.dot(ab,cb) / (numpy.linalg.norm(ab) * numpy.linalg.norm(cb)))

            skew = min(1.0, 2. * abs((math.pi / 2.) - angle(up_left, up_right, down_right)))
            return skew
        
        def get_area(corners, object_points):
            """
            Get 2d image area of the detected checkerboard.
            The projected checkerboard is assumed to be a convex quadrilateral, and the area computed as
            |p X q|/2; see http://mathworld.wolfram.com/Quadrilateral.html.
            """
            (up_left, up_right, down_right, down_left) = get_outside_corners(corners, object_points)
            a = up_right - up_left
            b = down_right - up_right
            c = down_left - down_right
            p = b + c
            q = a + b
            return abs(p[0]*q[1] - p[1]*q[0]) / 2.
        
        # Compute some parameters for this chessboard
        
        Xs = [x for (x, y) in corners]
        Ys = [y for (x, y) in corners]
        area = get_area(corners, object_points)
        border = math.sqrt(area)
        # For X and Y, we "shrink" the image all around by approx. half the board size.
        # Otherwise large boards are penalized because you can't get much X/Y variation.
        p_x = min(1.0, max(0.0, (mean(Xs) - border / 2) / (self._width  - border)))
        p_y = min(1.0, max(0.0, (mean(Ys) - border / 2) / (self._height - border)))
        p_size = math.sqrt(area / (self._width * self._height))
        skew = get_skew(corners, object_points)
        params = [p_x, p_y, p_size, skew]
        return params
        
class MonoCalibrator(Calibrator):
    def __init__(self, info, flags=0, param_ranges = None):
        Calibrator.__init__(self, info)
        
        self.calibrated = False
        self.calib_flags = flags
        
        self.br = cv_bridge.CvBridge()
        
        # self.db is list of (parameters, image) samples for use in calibration. parameters has form
        # (X, Y, size, skew) all normalized to [0,1], to keep track of what sort of samples we've taken
        # and ensure enough variety.
        self.db = []
        # For each db sample, we also record the detected corners.
        self.good_corners = []
        # Set to true when we have sufficiently varied samples to calibrate
        self.goodenough = False
        self.param_names = ["X", "Y", "Size", "Skew"]
        self.param_ranges = [0.7, 0.7, 0.4, 0.5]
        if(param_ranges and len(param_ranges)==4):
            self.param_ranges = param_ranges
            
        
    def is_good_sample(self, params):
        """
        Returns true if the checkerboard detection described by params should be added to the database.
        """
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a-b) for (a,b) in zip(p1, p2)])

        db_params = [sample[0] for sample in self.db]
        d = min([param_distance(params, p) for p in db_params])
        #print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        return d > 0.2
    
    def compute_progress(self):
        if not self.db:
            return None, False

        # Find range of checkerboard poses covered by samples in database
        all_params = [sample[0] for sample in self.db]
        min_params = reduce(lmin, all_params)
        max_params = reduce(lmax, all_params)
        # Don't reward small size or skew
        min_params = [min_params[0], min_params[1], 0., 0.]

        # For each parameter, judge how much progress has been made toward adequate variation
        progress = [min((hi - lo) / r, 1.0) for (lo, hi, r) in zip(min_params, max_params, self.param_ranges)]
        # If we have lots of samples, allow calibration even if not all parameters are green
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])
        return zip(self.param_names, min_params, max_params, progress), self.goodenough
    
    
    def handle_msg(self, msg):
        """
        Detects the calibration target and, if found and provides enough new information,
        adds it to the sample database.

        Returns a calibrationPattern message and progress info.
        """
                
        if msg.success:
            # Add sample to database only if it's sufficiently differnt from previous samples
            corners = [ (point.x, point.y) for point in msg.image_points ]
            object_points = [(point.x, point.y) for point in msg.object_points]
            params = self.get_parameters(corners, object_points)
            if self.is_good_sample(params):
                self.db.append((params, corners, object_points))
                print "*** Added sample %d, p_x = %.3f, p_y = %.3f, p_size = %.3f, skew = %.3f" % tuple([len(self.db)] + params)
        
        return(self.compute_progress())
        
    def do_calibration(self):
        if not self.goodenough:
            print "Can not calibrate yet!"
            return
        #append all things in db
        good_corners = [corners for (params, corners, object_points) in self.db ]
        good_points  = [object_points for (params, corners, object_points) in self.db ]
        
        intrinsics = cv.CreateMat(3, 3, cv.CV_64FC1)
        if self.calib_flags & cv2.CALIB_RATIONAL_MODEL:
            distortion = cv.CreateMat(8, 1, cv.CV_64FC1) # rational polynomial
        else:
            distortion = cv.CreateMat(5, 1, cv.CV_64FC1) # plumb bob
            
        cv.SetZero(intrinsics)
        cv.SetZero(distortion)
            
        # If FIX_ASPECT_RATIO flag set, enforce focal lengths have 1/1 ratio
        intrinsics[0,0] = 1.0
        intrinsics[1,1] = 1.0
        
        opts = self.mk_object_points(good_points)
        ipts = self.mk_image_points(good_corners)
        npts = self.mk_point_counts(good_points)
                        
        cv.CalibrateCamera2(opts, 
                            ipts, 
                            npts,
                            self.size, intrinsics,
                            distortion,
                            cv.CreateMat(len(good_corners), 3, cv.CV_32FC1),
                            cv.CreateMat(len(good_corners), 3, cv.CV_32FC1),
                            flags = self.calib_flags)
            
        self.intrinsics = intrinsics
        self.distortion = distortion
        
        # R is identity matrix for monocular calibration
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.R)
        self.P = cv.CreateMat(3, 4, cv.CV_64FC1)
        cv.SetZero(self.P)
        
        ncm = cv.GetSubRect(self.P, (0, 0, 3, 3))
        cv.GetOptimalNewCameraMatrix(self.intrinsics, self.distortion, self.size, 0, ncm)
        
        self.calibrated = True
        
        return (self.distortion, self.intrinsics, self.R, self.P)
            
            
class StereoCalibrator(Calibrator):
    def __init__(self, info, flags=0, param_ranges = None):
        Calibrator.__init__(self, info)
            
        if not param_ranges:
            param_ranges = [0.4, 0.7, 0.4, 0.5]
        
        # Create monocalibrators for each camera
        self.l = MonoCalibrator(info, flags, param_ranges)
        self.r = MonoCalibrator(info, flags, param_ranges)
        
        self.calibrated = False
        self.db = []
        
    def is_good_sample(self, lparams, rparams):
        """
        Returns true if the checkerboard detection described by params should be added to the database.
        """
        if not self.db:
            return True

        def param_distance(p1, p2):
            return sum([abs(a-b) for (a,b) in zip(p1, p2)])

        db_lparams = [ sample[0] for sample in self.db]
        db_rparams = [ sample[1] for sample in self.db]
        d1 = min([param_distance(lparams, p) for p in db_lparams])
        d2 = min([param_distance(rparams, p) for p in db_rparams])
        d = max(d1, d2)
        #print "d = %.3f" % d #DEBUG
        # TODO What's a good threshold here? Should it be configurable?
        return d > 0.2
        
    def compute_progress(self, lprogress, rprogress):
        if not self.db:
            return None, False
        if not lprogress or not rprogress:
            return None, False
        
        # Separate parameters out
        [l_param_names, l_min_params, l_max_params, l_progress] = zip(*lprogress)
        [r_param_names, r_min_params, r_max_params, r_progress] = zip(*rprogress)

        # Compute worst-case coverage
        min_params = lmax(l_min_params, r_min_params)
        max_params = lmin(l_max_params, r_max_params)
        progress = lmin(l_progress, r_progress)
        
        # If we have lots of samples, allow calibration even if not all parameters are green
        self.goodenough = (len(self.db) >= 40) or all([p == 1.0 for p in progress])
        return zip(l_param_names, min_params, max_params, progress), self.goodenough
        
    def handle_msg(self, lmsg, rmsg):
        (lprogress, lgood) = self.l.handle_msg(lmsg)
        (rprogress, rgood) = self.r.handle_msg(rmsg)
        epierror = -1
        
        # If we have detected both corners, add to database 
        if lmsg.success and rmsg.success:
            lcorners = [ (point.x, point.y) for point in lmsg.image_points ]
            lobject_points = [(point.x, point.y) for point in lmsg.object_points]
            rcorners = [ (point.x, point.y) for point in rmsg.image_points ]
            robject_points = [(point.x, point.y) for point in rmsg.object_points]
            lparams = self.get_parameters(lcorners, lobject_points)
            rparams = self.get_parameters(rcorners, robject_points)
            if(self.is_good_sample(lparams, rparams)):
                self.db.append( (lparams, rparams, lcorners, rcorners, lobject_points, robject_points) )
                print "*** Added stereo sample %d***" % len(self.db)
                
        return [(self.compute_progress(lprogress, rprogress)), (lprogress, lgood), (rprogress, rgood)]
        
    def do_calibration(self):
        if not self.goodenough:
            print "Can not calibrate yet!"
            return
        #append all things in db
        good_l_corners = [lcorners for (lparams, rparams, lcorners, rcorners, lobject_points, robject_points) in self.db ]
        good_l_points  = [lobject_points for (lparams, rparams, lcorners, rcorners, lobject_points, robject_points) in self.db ]
        good_r_corners = [rcorners for (lparams, rparams, lcorners, rcorners, lobject_points, robject_points) in self.db ]
        good_r_points  = [robject_points for (lparams, rparams, lcorners, rcorners, lobject_points, robject_points) in self.db ]
        
        # Perform monocular calibrations
        self.l.do_calibration()
        self.r.do_calibration()
        
##        print "left intrinsics"
##        print numpy.asarray(self.l.intrinsics)
##        print "left distortion"
##        print numpy.asarray(self.l.distortion)
##        print "left projection"
##        print numpy.asarray(self.l.P)
##        
##        print "right intrinsics"
##        print numpy.asarray(self.r.intrinsics)
##        print "right distortion"
##        print numpy.asarray(self.r.distortion)
##        print "right projection"
##        print numpy.asarray(self.r.P)
        
        flags = cv2.CALIB_FIX_INTRINSIC
        
        self.T = cv.CreateMat(3, 1, cv.CV_64FC1)
        self.R = cv.CreateMat(3, 3, cv.CV_64FC1)
        cv.SetIdentity(self.T)
        cv.SetIdentity(self.R)
        
        opts = self.mk_object_points(good_l_points)
        l_ipts = self.mk_image_points(good_l_corners)
        r_ipts = self.mk_image_points(good_r_corners)
        npts = self.mk_point_counts(good_l_points)
                        
        cv.StereoCalibrate(opts, 
                           l_ipts,
                           r_ipts,
                           npts,
                           self.l.intrinsics, self.l.distortion,
                           self.r.intrinsics, self.r.distortion,
                           self.size,
                           self.R,                            # R
                           self.T,                            # T
                           cv.CreateMat(3, 3, cv.CV_32FC1),   # E
                           cv.CreateMat(3, 3, cv.CV_32FC1),   # F
                           (cv.CV_TERMCRIT_ITER + cv.CV_TERMCRIT_EPS, 1, 1e-5),
                           flags)
                        
##        print "after cal left intrinsics"
##        print numpy.asarray(self.l.intrinsics)
##        print "after cal left distortion"
##        print numpy.asarray(self.l.distortion)
##        
##        print "after cal right intrinsics"
##        print numpy.asarray(self.r.intrinsics)
##        print "after cal right distortion"
##        print numpy.asarray(self.r.distortion)
##        
##        print "after cal R"
##        print numpy.asarray(self.R)
##        print "after cal T"
##        print numpy.asarray(self.T)
                        
        cv.StereoRectify(self.l.intrinsics,
                 self.r.intrinsics,
                 self.l.distortion,
                 self.r.distortion,
                 self.size,
                 self.R,
                 self.T,
                 self.l.R, self.r.R, self.l.P, self.r.P)
                
##        print "after rectify left intrinsics"
##        print numpy.asarray(self.l.intrinsics)
##        print "after rectify left distortion"
##        print numpy.asarray(self.l.distortion)
##        
##        print "after rectify right intrinsics"
##        print numpy.asarray(self.r.intrinsics)
##        print "after rectify right distortion"
##        print numpy.asarray(self.r.distortion)
##        
##        print "after rectify left R"
##        print numpy.asarray(self.l.R)
##        print "after rectify left P"
##        print numpy.asarray(self.l.P)
##        print "after rectify right R"
##        print numpy.asarray(self.r.R)
##        print "after rectify right P"
##        print numpy.asarray(self.r.P)
                        
        return (self.l.distortion, self.l.intrinsics, self.l.R, self.l.P, self.r.distortion, self.r.intrinsics, self.r.R, self.r.P)
    
