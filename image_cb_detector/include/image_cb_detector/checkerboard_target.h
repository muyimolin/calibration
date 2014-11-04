/**************************************************************************
** Copyright (c) 2014 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 24 Oct 2014
**
** Developed jointly by NASA/JSC and Oceaneering Space Systems
**
** Licensed under the NASA Open Source Agreement v1.3 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://opensource.org/licenses/NASA-1.3
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
**************************************************************************/

#ifndef CHECKERBOARD_TARGET_H_
#define CHECKERBOARD_TARGET_H_

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <image_cb_detector/calibration_target_base.h>

namespace calibration_target
{
class Checkerboard : public calibration_target_base::CalibrationTarget
{
public:
    Checkerboard() {}

    void initialize(std::string name);
    void detect(const sensor_msgs::ImageConstPtr & image);
    calibration_msgs::CalibrationPattern getFeatures();
    sensor_msgs::Image getImage();

protected:
    //! Convert image message into a 8-bit 1 channel monochrome OpenCV image
    bool convertImage(const sensor_msgs::ImageConstPtr & image);
    double calcDistance(cv::Point2f a, cv::Point2f b);

private:
    ros::NodeHandle nh;
    int numX, numY;
    double dimX, dimY;
    int flags;
    cv::Size boardSize;
    bool success;
    std::vector<cv::Point2f> corners;
    cv_bridge::CvImageConstPtr mono8;
    std_msgs::Header header;
    calibration_msgs::CalibrationPattern features;

};
}

#endif // CHECKERBOARD_TARGET_H_
