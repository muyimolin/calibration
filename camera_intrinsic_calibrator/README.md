# Package Summary

Provide a node that performs an intrinsic calibration of cameras and saves the result

*   Maintainer: [Allison Thackston](http://bender.jsc.nasa.gov/confluence/display/~athackst)
*   Author: [Allison Thackston](http://bender.jsc.nasa.gov/confluence/display/~athackst)

# mono_calibrator

## Topics

### subscriptions
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">features</td><td colspan="1" class="confluenceTd">calibration_msgs/CalibrationPattern</td><td class="confluenceTd">The description of the calibration plate and locations in an image</td></tr><tr><td colspan="1" class="confluenceTd">camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/CameraInfo</td><td colspan="1" class="confluenceTd">The information associated with the camera to calibrate</td></tr></tbody></table></div>

### service requests
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">camera/set_camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/SetCameraInfo</td><td class="confluenceTd">The output calibration</td></tr></tbody></table></div>

## Usage

### roslaunch

To run, you will need to set image and camera. The roslaunch file starts up the mono_calibrator, image_cb_detector, image_cb_annotator and optionally the viewer.
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ roslaunch camera_intrinsic_calibrator mono_calibrator.launch image:=&lt;image_raw&gt; camera_ns:=&lt;/camera&gt;</pre>
</div></div><div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">image</td><td class="confluenceTd"> </td><td class="confluenceTd">Name of the image topic to detect the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">camera_ns</td><td colspan="1" class="confluenceTd"> </td><td colspan="1" class="confluenceTd"> </td></tr><tr><td colspan="1" class="confluenceTd">camera</td><td colspan="1" class="confluenceTd">(camera_ns)/driver</td><td colspan="1" class="confluenceTd">The name of the camera node</td></tr><tr><td colspan="1" class="confluenceTd">features</td><td colspan="1" class="confluenceTd">(camera_ns)/features</td><td colspan="1" class="confluenceTd">The name of the features topic</td></tr><tr><td colspan="1" class="confluenceTd">camera_info</td><td colspan="1" class="confluenceTd">(camera_ns)/camera_info</td><td colspan="1" class="confluenceTd">The name of the camera_info topic</td></tr><tr><td colspan="1" class="confluenceTd">use_service</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to use the camera service to save the calibration or a file</td></tr><tr><td colspan="1" class="confluenceTd">camera_name</td><td colspan="1" class="confluenceTd">camera</td><td colspan="1" class="confluenceTd">The camera name for the calibration file – used if not using service</td></tr><tr><td colspan="1" class="confluenceTd">file_name</td><td colspan="1" class="confluenceTd">camera.yaml</td><td colspan="1" class="confluenceTd">The calibration file name – used if not using service</td></tr><tr><td colspan="1" class="confluenceTd">viewer</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to automatically launch the image viewer</td></tr><tr><td class="confluenceTd">marker_size</td><td class="confluenceTd">5</td><td class="confluenceTd">The size of the markers for the detected calibration plate</td></tr><tr><td class="confluenceTd">marker_width</td><td class="confluenceTd">2</td><td class="confluenceTd">The width of the makers for the detected calibration plate</td></tr><tr><td class="confluenceTd">scaling</td><td class="confluenceTd">0.5</td><td class="confluenceTd">The scale to use for the annotator/viewer</td></tr><tr><td colspan="1" class="confluenceTd">num_cols</td><td colspan="1" class="confluenceTd">8</td><td colspan="1" class="confluenceTd">Number of columns in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">num_rows</td><td colspan="1" class="confluenceTd">6</td><td colspan="1" class="confluenceTd">Number of rows in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">dim</td><td colspan="1" class="confluenceTd">1</td><td colspan="1" class="confluenceTd">The dimensions between points on the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">pattern</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

0: Checkerboard

1: Circles

2: ACircles
</td></tr><tr><td colspan="1" class="confluenceTd">flags</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

1: CV_CALIB_ADAPTIVE_THRESH

4: CV_CALIB_CB_FILTER_QUADS

2: CV_CALIB_CB_NORMALIZE_IMAGE

8: CV_CALIB_CB_FAST_CHECK
</td></tr><tr><td colspan="1" class="confluenceTd">autostart</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to automatically start the calibration</td></tr></tbody></table></div>

### rosrun

Parameters can be set by either commad line options or rosparams.  Command line options override rosparam values.
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ rosrun camera_intrinsic_calibrator mono_calibratory.py</pre>
</div></div>

#### command line options
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">--autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">--use_service</td><td class="confluenceTd">True</td><td class="confluenceTd">

Whether to use the camera service function to save calibration
</td></tr><tr><td class="confluenceTd">--camera_name</td><td class="confluenceTd">camera</td><td class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">--file_name</td><td colspan="1" class="confluenceTd">camera.yaml</td><td colspan="1" class="confluenceTd">The name of the calibration file if not using the service</td></tr></tbody></table></div>

#### rosparams
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">~autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">~use_service</td><td class="confluenceTd">True</td><td class="confluenceTd">Whether to use the camera service function to save calibration</td></tr><tr><td class="confluenceTd">~camera_name</td><td class="confluenceTd">camera</td><td class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td class="confluenceTd">~file_name</td><td class="confluenceTd">camera.yaml</td><td class="confluenceTd">The name of the calibration file if not using the service</td></tr></tbody></table></div>

### action-lib

#### goal

bool fix_principal_point

bool fix_aspect_ratio

bool zero_tangent_dist

uint16 num_coeffs

float64[] param_ranges

#### result

sensor_msgs/CameraInfo result

std_msgs/Header header

uint32 seq

time stamp

string frame_id

uint32 height

uint32 width

string distortion_model

float64[] D

float64[9] K

float64[9] R

float64[12] P

uint32 binning_x

uint32 binning_y

sensor_msgs/RegionOfInterest roi

uint32 x_offset

uint32 y_offset

uint32 height

uint32 width

bool do_rectify

#### feedback

camera_intrinsic_calibrator/CalibrationCoverage feedback

camera_intrinsic_calibrator/Progress x

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress y

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress size

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress skew

float64 min

float64 max

float64 percent

bool goodenough

# stereo_calibrator

## Topics

### subscriptions
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">left/features</td><td colspan="1" class="confluenceTd">calibration_msgs/CalibrationPattern</td><td class="confluenceTd">The description of the calibration plate and locations in the left image</td></tr><tr><td colspan="1" class="confluenceTd">right/features</td><td colspan="1" class="confluenceTd">calibration_msgs/CalibrationPattern</td><td colspan="1" class="confluenceTd">The description of the calibration plate and locations in the right image</td></tr><tr><td colspan="1" class="confluenceTd">left/camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/CameraInfo</td><td colspan="1" class="confluenceTd">The information associated with the left camera to calibrate</td></tr><tr><td colspan="1" class="confluenceTd">right/camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/CameraInfo</td><td colspan="1" class="confluenceTd">The information associated with the right camera to calibrate</td></tr></tbody></table></div>

### service requests
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">left_camera/set_camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/SetCameraInfo</td><td class="confluenceTd">The output calibration for the left camera</td></tr><tr><td colspan="1" class="confluenceTd">right_camera/set_camera_info</td><td colspan="1" class="confluenceTd">sensor_msgs/SetCameraInfo</td><td colspan="1" class="confluenceTd">The output calibration for the right camera</td></tr></tbody></table></div>

## Usage

### roslaunch

Topic names should match standard camera names, but can be overwritten on the command line.  The roslaunch file starts up stereo_calibrator, image_cb_detector, image_cb_annotator and optionally the viewer.
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ roslaunch camera_intrinsic_calibrator stereo_calibrator.launch right:=&lt;image_raw&gt; left:=&lt;image_raw&gt; left_ns:=&lt;/camera/left&gt; right_ns:=&lt;/camera/right&gt;</pre>
</div></div><div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">left_ns</td><td class="confluenceTd"> </td><td class="confluenceTd">The left camera namespace</td></tr><tr><td colspan="1" class="confluenceTd">left</td><td colspan="1" class="confluenceTd"> </td><td colspan="1" class="confluenceTd">The name of the left image topic to use for calibration</td></tr><tr><td colspan="1" class="confluenceTd">right_ns</td><td colspan="1" class="confluenceTd"> </td><td colspan="1" class="confluenceTd">The right camera namespace</td></tr><tr><td colspan="1" class="confluenceTd">right</td><td colspan="1" class="confluenceTd"> </td><td colspan="1" class="confluenceTd">Name of the right image topic to detect the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">viewer</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to automatically launch the image viewer</td></tr><tr><td colspan="1" class="confluenceTd">autostart</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to automatically start the calibration</td></tr><tr><td colspan="1" class="confluenceTd">approximate</td><td colspan="1" class="confluenceTd">0.0</td><td colspan="1" class="confluenceTd">The approximate time matching parameter between the two cameras (0 for exact timestamp matching)</td></tr><tr><td class="confluenceTd">marker_size</td><td class="confluenceTd">5</td><td class="confluenceTd">The size of the markers for the detected calibration plate</td></tr><tr><td class="confluenceTd">marker_width</td><td class="confluenceTd">2</td><td class="confluenceTd">The width of the makers for the detected calibration plate</td></tr><tr><td class="confluenceTd">scaling</td><td class="confluenceTd">0.5</td><td class="confluenceTd">The scale to use for the annotator/viewer</td></tr><tr><td colspan="1" class="confluenceTd">num_cols</td><td colspan="1" class="confluenceTd">8</td><td colspan="1" class="confluenceTd">Number of columns in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">num_rows</td><td colspan="1" class="confluenceTd">6</td><td colspan="1" class="confluenceTd">Number of rows in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">dim</td><td colspan="1" class="confluenceTd">1</td><td colspan="1" class="confluenceTd">The dimensions between points on the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">pattern</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

0: Checkerboar

1: Circles
2: ACircles</td></tr><tr><td colspan="1" class="confluenceTd">flags</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

1: CV_CALIB_ADAPTIVE_THRESH

4: CV_CALIB_CB_FILTER_QUADS

2: CV_CALIB_CB_NORMALIZE_IMAGE

8: CV_CALIB_CB_FAST_CHECK
</td></tr><tr><td colspan="1" class="confluenceTd">use_service</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether or not to use the camera service to save the calibration or a file</td></tr><tr><td colspan="1" class="confluenceTd">left_driver</td><td colspan="1" class="confluenceTd">(left_ns)/driver</td><td colspan="1" class="confluenceTd">The name of the left camera node</td></tr><tr><td colspan="1" class="confluenceTd">left_features</td><td colspan="1" class="confluenceTd">(left_ns)/features</td><td colspan="1" class="confluenceTd">The name of the left features topic</td></tr><tr><td colspan="1" class="confluenceTd">left_camera_info</td><td colspan="1" class="confluenceTd">(left_ns)/camera_info</td><td colspan="1" class="confluenceTd">The name of the left camera_info topic</td></tr><tr><td colspan="1" class="confluenceTd">left_camera_name</td><td colspan="1" class="confluenceTd">left</td><td colspan="1" class="confluenceTd">The unique id/name for the left camera in the calibration file – used if not using service</td></tr><tr><td colspan="1" class="confluenceTd">left_file_name</td><td colspan="1" class="confluenceTd">left.yaml</td><td colspan="1" class="confluenceTd">The calibration file name for the left camera – used if not using service</td></tr><tr><td colspan="1" class="confluenceTd">right_driver</td><td colspan="1" class="confluenceTd">(right_ns)/driver</td><td colspan="1" class="confluenceTd">The name of the right camera node</td></tr><tr><td colspan="1" class="confluenceTd">right_features</td><td colspan="1" class="confluenceTd">(right_ns)/features</td><td colspan="1" class="confluenceTd">The name of the right features topic</td></tr><tr><td colspan="1" class="confluenceTd">right_camera_info</td><td colspan="1" class="confluenceTd">(right_ns)/camera_info</td><td colspan="1" class="confluenceTd">The name of the right camera_info topic</td></tr><tr><td colspan="1" class="confluenceTd">right_camera_name</td><td colspan="1" class="confluenceTd">right</td><td colspan="1" class="confluenceTd">The unique id/name for the right camera in the calibration file – used if not using service</td></tr><tr><td colspan="1" class="confluenceTd">right_file_name</td><td colspan="1" class="confluenceTd">right.yaml</td><td colspan="1" class="confluenceTd">The calibration file name for the right camera – used if not using service</td></tr></tbody></table></div>

### rosrun

Parameters can be set by either commad line options or rosparams.  Command line options override rosparam values.
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ rosrun camera_intrinsic_calibrator stereo_calibratory.py</pre>
</div></div>

#### command line options
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">--autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">--use_service</td><td class="confluenceTd">True</td><td class="confluenceTd">

Whether to use the camera service function to save calibration
</td></tr><tr><td colspan="1" class="confluenceTd">--approximate</td><td colspan="1" class="confluenceTd">0.0</td><td colspan="1" class="confluenceTd">The approximate time matching parameter between the two cameras (0 for exact timestamp matching)</td></tr><tr><td class="confluenceTd">--left_camera_name</td><td class="confluenceTd">left</td><td class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">--left_file_name</td><td colspan="1" class="confluenceTd">left.yaml</td><td colspan="1" class="confluenceTd">The name of the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">--right_camera_name</td><td colspan="1" class="confluenceTd">right</td><td colspan="1" class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">--right_file_name</td><td colspan="1" class="confluenceTd">right.yaml</td><td colspan="1" class="confluenceTd">The name of the calibration file if not using the service</td></tr></tbody></table></div>

#### rosparams
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">~autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">~use_service</td><td class="confluenceTd">True</td><td class="confluenceTd">Whether to use the camera service function to save calibration</td></tr><tr><td colspan="1" class="confluenceTd">~approximate</td><td colspan="1" class="confluenceTd">0.0</td><td colspan="1" class="confluenceTd">The approximate time matching parameter between the two cameras (0 for exact timestamp matching)</td></tr><tr><td class="confluenceTd">~left_camera_name</td><td class="confluenceTd">left</td><td class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td class="confluenceTd">~left_file_name</td><td class="confluenceTd">left.yaml</td><td class="confluenceTd">The name of the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">~right_camera_name</td><td colspan="1" class="confluenceTd">right</td><td colspan="1" class="confluenceTd">The name of the camera to use in the calibration file if not using the service</td></tr><tr><td colspan="1" class="confluenceTd">~right_file_name</td><td colspan="1" class="confluenceTd">right.yaml</td><td colspan="1" class="confluenceTd">The name of the calibration file if not using the service</td></tr></tbody></table></div>

### action-lib

#### goal

bool fix_principal_point

bool fix_aspect_ratio

bool zero_tangent_dist

uint16 num_coeffs

float64[] param_ranges

#### result

sensor_msgs/CameraInfo left_result

std_msgs/Header header

uint32 seq

time stamp

string frame_id

uint32 height

uint32 width

string distortion_model

float64[] D

float64[9] K

float64[9] R

float64[12] P

uint32 binning_x

uint32 binning_y

sensor_msgs/RegionOfInterest roi

uint32 x_offset

uint32 y_offset

uint32 height

uint32 width

bool do_rectify

sensor_msgs/CameraInfo right_result

std_msgs/Header header

uint32 seq

time stamp

string frame_id

uint32 height

uint32 width

string distortion_model

float64[] D

float64[9] K

float64[9] R

float64[12] P

uint32 binning_x

uint32 binning_y

sensor_msgs/RegionOfInterest roi

uint32 x_offset

uint32 y_offset

uint32 height

uint32 width

bool do_rectify

#### feedback

camera_intrinsic_calibrator/CalibrationCoverage stereo_feedback

camera_intrinsic_calibrator/Progress x

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress y

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress size

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress skew

float64 min

float64 max

float64 percent

bool goodenough

camera_intrinsic_calibrator/CalibrationCoverage left_feedback

camera_intrinsic_calibrator/Progress x

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress y

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress size

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress skew

float64 min

float64 max

float64 percent

bool goodenough

camera_intrinsic_calibrator/CalibrationCoverage right_feedback

camera_intrinsic_calibrator/Progress x

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress y

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress size

float64 min

float64 max

float64 percent

camera_intrinsic_calibrator/Progress skew

float64 min

float64 max

float64 percent

bool goodenough

