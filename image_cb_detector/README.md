# Package Summary

Provide a node that extracts calibration plate locations from ROS images.

*   Maintainer: [Vincent Rabaud](https://github.com/vrabaud)
*   Author: [Allison Thackston](https://github.com/athackst)

# Topics

## subscriptions
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">image_raw</td><td colspan="1" class="confluenceTd">sensor_msgs/Image</td><td class="confluenceTd">The image topic to use to find the calibration plate</td></tr></tbody></table></div>

## publications
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th colspan="1" class="confluenceTh">Type</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">features</td><td colspan="1" class="confluenceTd">calibration_msgs/CalibrationPattern</td><td class="confluenceTd">The output description of the calibration plate and locations in the image</td></tr><tr><td class="confluenceTd">calibration_image</td><td colspan="1" class="confluenceTd">sensor_msgs/Image</td><td class="confluenceTd">The converted image used for calibration</td></tr></tbody></table></div>

# Usage

There are a variety of ways to run the image_cb_detector.  If running stand alone, the preferred method is to use roslaunch.  It can also be started directly using rosrun and through the action interface

## roslaunch
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ roslaunch image_cb_detector image_cb_detector.launch</pre>
</div></div><div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">image</td><td class="confluenceTd"> </td><td class="confluenceTd">Name of the image topic to detect the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">autostart</td><td colspan="1" class="confluenceTd">True</td><td colspan="1" class="confluenceTd">Whether to start detecting the board automatically on startup</td></tr><tr><td colspan="1" class="confluenceTd">num_cols</td><td colspan="1" class="confluenceTd">8</td><td colspan="1" class="confluenceTd">Number of columns in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">num_rows</td><td colspan="1" class="confluenceTd">6</td><td colspan="1" class="confluenceTd">Number of rows in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">dim</td><td colspan="1" class="confluenceTd">1</td><td colspan="1" class="confluenceTd">The dimensions between points on the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">pattern</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

0: Checkerboar

1: Circles
2: ACircles</td></tr><tr><td colspan="1" class="confluenceTd">flags</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

1: CV_CALIB_ADAPTIVE_THRESH

4: CV_CALIB_CB_FILTER_QUADS

2: CV_CALIB_CB_NORMALIZE_IMAGE

8: CV_CALIB_CB_FAST_CHECK
</td></tr><tr><td colspan="1" class="confluenceTd">marker_size</td><td colspan="1" class="confluenceTd">5</td><td colspan="1" class="confluenceTd">The size of the markers for the detected calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">marker_width</td><td colspan="1" class="confluenceTd">2</td><td colspan="1" class="confluenceTd">The width of the makers for the detected calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">scaling</td><td colspan="1" class="confluenceTd">0.5</td><td colspan="1" class="confluenceTd">The scale to use for the annotator/viewer</td></tr><tr><td class="confluenceTd">viewer</td><td class="confluenceTd">True</td><td class="confluenceTd">Whether or not to automatically launch image_view</td></tr></tbody></table></div>

## rosrun
<div class="code panel pdl" style="border-width: 1px;"><div class="codeContent panelContent pdl">
<pre class="theme: Confluence; brush: java; gutter: false" style="font-size:12px;">~$ rosrun image_cb_detector calibration_detector.py</pre>
</div></div>

#### command line options
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">--autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">--cols</td><td class="confluenceTd">8</td><td class="confluenceTd">

Number of columns in calibration plate
</td></tr><tr><td class="confluenceTd">--rows</td><td class="confluenceTd">6</td><td class="confluenceTd">Number of rows in calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">--dim</td><td colspan="1" class="confluenceTd">1</td><td colspan="1" class="confluenceTd">The dimensions between points on the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">--pattern</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

0: Checkerboard

1: Circles

2: ACircles
</td></tr><tr><td colspan="1" class="confluenceTd">--flags</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

1: CV_CALIB_ADAPTIVE_THRESH

4: CV_CALIB_CB_FILTER_QUADS

2: CV_CALIB_CB_NORMALIZE_IMAGE

8: CV_CALIB_CB_FAST_CHECK
</td></tr></tbody></table></div>

#### rosparams
<div class="table-wrap"><table class="confluenceTable"><tbody><tr><th class="confluenceTh">Name</th><th class="confluenceTh">Default Value</th><th class="confluenceTh">Description</th></tr><tr><td class="confluenceTd">~autostart</td><td class="confluenceTd">False</td><td class="confluenceTd">Whether to start looking for a specific calibration plate on starting</td></tr><tr><td class="confluenceTd">~num_cols</td><td class="confluenceTd">8</td><td class="confluenceTd">Number of columns in calibration plate</td></tr><tr><td class="confluenceTd">~num_rows</td><td class="confluenceTd">6</td><td class="confluenceTd">Number of rows in calibration plate</td></tr><tr><td class="confluenceTd">~dim</td><td class="confluenceTd">1</td><td class="confluenceTd">The dimensions between points on the calibration plate</td></tr><tr><td colspan="1" class="confluenceTd">~pattern</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

0: Checkerboard

1: Circles
2: ACircles</td></tr><tr><td colspan="1" class="confluenceTd">~flags</td><td colspan="1" class="confluenceTd">0</td><td colspan="1" class="confluenceTd">

1: CV_CALIB_ADAPTIVE_THRESH

4: CV_CALIB_CB_FILTER_QUADS

2: CV_CALIB_CB_NORMALIZE_IMAGE

8: CV_CALIB_CB_FAST_CHECK
</td></tr></tbody></table></div>

## action-lib

#### goal


    uint16 num_x
    uint16 num_y
    float32 spacing_x
    float32 spacing_y
    uint8 pattern
    uint8 flags
    # Specify how many times we want to upsample the image.
    #  This is often useful for detecting small checkerboards far away
    float32 width_scaling
    float32 height_scaling
    # Configure openCV's subpixel corner detector
    uint32 subpixel_window
    int32  subpixel_zero_zone
    uint8 CHECKERBOARD = 0
    uint8 CIRCLES      = 1
    uint8 ACIRCLES     = 2
