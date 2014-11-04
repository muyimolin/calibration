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

#ifndef CALIBRATION_DETECTOR_SERVER_H
#define CALIBRATION_DETECTOR_SERVER_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <pluginlib/class_loader.h>
#include <image_cb_detector/calibration_target_base.h>
#include <actionlib/server/simple_action_server.h>
#include <image_cb_detector/SelectAction.h>
#include <sensor_msgs/Image.h>
#include <calibration_msgs/CalibrationPattern.h>

class CalibrationDetectorServer
{
public:
    CalibrationDetectorServer(std::string name);
    virtual ~CalibrationDetectorServer();

    void goalCallback();
    void preemptCallback();
    void imageCallback(const sensor_msgs::ImageConstPtr & image);
protected:
    ros::NodeHandle nh;
    pluginlib::ClassLoader<calibration_target_base::CalibrationTarget> target_loader;
    actionlib::SimpleActionServer<image_cb_detector::SelectAction> action;

    ros::Publisher featurePub;
    ros::Publisher imagePub;
    ros::Subscriber imageSub;

private:
    boost::shared_ptr<calibration_target_base::CalibrationTarget> target;
};

#endif // CALIBRATION_DETECTOR_SERVER_H
