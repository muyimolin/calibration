/**************************************************************************
** Copyright (c) 2014 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 10 Nov 2014
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

#ifndef CIRCLES_TARGET_H
#define CIRCLES_TARGET_H

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <image_cb_detector/calibration_target_base.h>
#include <image_cb_detector/image_converter.h>
#include <dynamic_reconfigure/server.h>
#include <image_cb_detector/CirclesConfig.h>

namespace calibration_target
{
class Circles : public calibration_target_base::CalibrationTarget
{
public:
    Circles() {}

    void initialize(std::string name);
    void detect(const sensor_msgs::ImageConstPtr & image);
    calibration_msgs::CalibrationPattern getFeatures();
    sensor_msgs::Image getImage();
protected:
    void reconfigureCallback(image_cb_detector::CirclesConfig &config, uint32_t level);

private:
    ros::NodeHandle nh;
    int flags;
    cv::Size boardSize;
    cv::Ptr<cv::FeatureDetector> blobDetector;
    cv::SimpleBlobDetector::Params params;
    bool success;
    std::vector<cv::Point2f> centers;
    cv_bridge::CvImageConstPtr mono8;
    std_msgs::Header header;
    calibration_msgs::CalibrationPattern features;

    // Dynamic reconfigure
    typedef dynamic_reconfigure::Server<image_cb_detector::CirclesConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigureServer;
    boost::recursive_mutex config_mutex;
    bool configured;

};
}
#endif // CIRCLES_TARGET_H
