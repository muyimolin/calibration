/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

//! \author Vijay Pradeep

#include <laser_cb_detector/laser_cb_detector.h>
#include <ros/console.h>
#include <cv_bridge/CvBridge.h>
//#include <highgui.h>

using namespace std;
using namespace laser_cb_detector;

LaserCbDetector::LaserCbDetector() : configured_(false) {}

bool LaserCbDetector::configure(const ConfigGoal& config)
{
  config_ = config;
  return true;
}

bool LaserCbDetector::detect(const calibration_msgs::DenseLaserSnapshot& snapshot,
                             calibration_msgs::CalibrationPattern& result)
{
  // ***** Convert the snapshot into an image, based on intensity window in config *****
  if(!bridge_.fromIntensity(snapshot, config_.min_intensity, config_.max_intensity))
    return false;
  IplImage* image = bridge_.toIpl();

  if (config_.flip_horizontal)
  {
    ROS_DEBUG("Flipping image");
    cvFlip(image, NULL, 1);
  }
  else
    ROS_DEBUG("Not flipping image");

  sensor_msgs::CvBridge cvbridge_;
  sensor_msgs::Image::Ptr ros_image = cvbridge_.cvToImgMsg(image);

  if(detect(ros_image, result)){
    if (config_.flip_horizontal){
      for(int i=0; i < result.image_points.size(); i++)
        result.image_points[i].x = image->width - result.image_points[i].x - 1;
    }
    return true;
  }else
    return false;
}

bool LaserCbDetector::detect(const sensor_msgs::ImageConstPtr& ros_image,
                             calibration_msgs::CalibrationPattern& result)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvShare(ros_image, "mono8")->image;
  }
  catch (cv_bridge::Exception error)
  {
    ROS_ERROR("error");
    return false;
  }

  // ***** Resize the image based on scaling parameters in config *****
  const int scaled_width  = (int) (.5 + image.cols  * config_.width_scaling);
  const int scaled_height = (int) (.5 + image.rows * config_.height_scaling);
  cv::Mat image_scaled;
  cv::resize(image, image_scaled, cv::Size(scaled_width, scaled_height), 0, 0, CV_INTER_LINEAR);

  // ***** Allocate vector for found corners *****
  vector<cv::Point2f> cv_corners;
  cv_corners.resize(config_.num_x*config_.num_y);

  // ***** Do the actual checkerboard extraction *****
  cv::Size board_size(config_.num_x, config_.num_y);
  int found = cv::findChessboardCorners( image_scaled, board_size, cv_corners, CV_CALIB_CB_ADAPTIVE_THRESH) ;

  if(found)
  {
    ROS_DEBUG("Found CB");
    //ROS_INFO("Found checkerboard. Subpixel window: %i", config_.subpixel_window);


    cv::Size subpixel_window(config_.subpixel_window,
                                       config_.subpixel_window);
    cv::Size subpixel_zero_zone(config_.subpixel_zero_zone,
                                       config_.subpixel_zero_zone);

    // Subpixel fine-tuning stuff
    cv::cornerSubPix( image_scaled, cv_corners,
                       subpixel_window,
                       subpixel_zero_zone,
                       cv::TermCriteria(CV_TERMCRIT_ITER,20,1e-2));
  }
  else
    ROS_DEBUG("Didn't find CB");

  // ***** Downscale the detected corners and generate the CalibrationPattern message *****
  result.header.stamp    = ros_image->header.stamp;
  result.header.frame_id = ros_image->header.frame_id;

  result.object_points.resize(config_.num_x * config_.num_y);
  for (unsigned int i=0; i < config_.num_y; i++)
  {
    for (unsigned int j=0; j < config_.num_x; j++)
    {
      result.object_points[i*config_.num_x + j].x = j*config_.spacing_x;
      result.object_points[i*config_.num_x + j].y = i*config_.spacing_y;
      result.object_points[i*config_.num_x + j].z = 0.0;
    }
  }

  result.image_points.resize(cv_corners.size());

  for (size_t i=0; i < cv_corners.size(); i++)
  {
    result.image_points[i].x = cv_corners[i].x / config_.width_scaling;
    result.image_points[i].y = cv_corners[i].y / config_.height_scaling;
  }

  result.success = found;

  return true;
}


bool LaserCbDetector::getImage(const calibration_msgs::DenseLaserSnapshot& snapshot, sensor_msgs::Image& ros_image)
{
  if(!bridge_.fromIntensity(snapshot, config_.min_intensity, config_.max_intensity))
  {
    ROS_ERROR("Error building IplImage from DenseLaserSnapshot's intensity data");
    return false;
  }
  IplImage* image = bridge_.toIpl();

  if(!sensor_msgs::CvBridge::fromIpltoRosImage(image, ros_image, "mono8"))
  {
    ROS_ERROR("Error converting IplImage to a ROS sensor_msgs::Image");
    return false;
  }

  return true;
}

