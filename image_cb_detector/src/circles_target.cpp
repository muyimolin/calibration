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

#include <image_cb_detector/circles_target.h>

namespace enc = sensor_msgs::image_encodings;

using namespace calibration_target;
using namespace cv;

void Circles::initialize(std::string name)
{
    nh = ros::NodeHandle(name);

    configured = false;
    // Setup dynamic reconfigure server
    reconfigureServer.reset(new ReconfigureServer(config_mutex, nh));
    ReconfigureServer::CallbackType f = boost::bind(&Circles::reconfigureCallback, this, _1, _2);
    reconfigureServer->setCallback(f);
}

void Circles::reconfigureCallback(image_cb_detector::CirclesConfig &config, uint32_t level)
{
    ROS_INFO("[%s][num_x]: %i", nh.getNamespace().c_str(), config.num_x);
    ROS_INFO("[%s][num_y]: %i", nh.getNamespace().c_str(), config.num_y);
    ROS_INFO("[%s][dim_x]: %f", nh.getNamespace().c_str(), config.dim_x);
    ROS_INFO("[%s][dim_y]: %f", nh.getNamespace().c_str(), config.dim_y);

    boardSize = cv::Size(config.num_x, config.num_y);
    centers.resize(boardSize.area());
    features.object_points.resize(boardSize.area());
    for (int i = 0; i < boardSize.height; ++i)
    {
        for (int j = 0; j < boardSize.width; ++j)
        {
            features.object_points.at(i*boardSize.width + j).x = j * config.dim_x;
            features.object_points.at(i*boardSize.width + j).y = i * config.dim_y;
            features.object_points.at(i*boardSize.width + j).z = 0.0;
        }
    }

    ROS_INFO("[%s][min_area]: %f", nh.getNamespace().c_str(), config.min_area);
    ROS_INFO("[%s][max_area]: %f", nh.getNamespace().c_str(), config.max_area);
    ROS_INFO("[%s][min_dist]: %f", nh.getNamespace().c_str(), config.min_dist);

    params.minArea = config.min_area;
    params.maxArea = config.max_area;
    params.minDistBetweenBlobs = config.min_dist;

    blobDetector = new SimpleBlobDetector(params);

    flags = 0;
    if(config.clustering)
    {
        flags = CALIB_CB_CLUSTERING;
    }

    ROS_INFO("[%s][flags]: %i", nh.getNamespace().c_str(), flags);
    configured = true;
}

void Circles::detect(const sensor_msgs::ImageConstPtr &image)
{
    //! update features
    header = image->header;

    //! convert the image
    mono8 = ImageConverter::toMono8(image);
    if(mono8 && configured)
    {
        boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex);
        //! find the centers
        int found = cv::findCirclesGrid(mono8->image, boardSize, centers, CALIB_CB_SYMMETRIC_GRID | flags, blobDetector);

        if (found)
        {
            ROS_DEBUG("[%s] found circles", ros::this_node::getName().c_str());
            success = true;
        }
        else
        {
            ROS_DEBUG("[%s] didn't find checkerboard", ros::this_node::getName().c_str());
            success = false;
        }
    }
}

calibration_msgs::CalibrationPattern Circles::getFeatures()
{
    boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex);
    //! fill out the header
    features.header = header;

    //! resize image points in case it changes on no detection
    features.image_points.resize(centers.size());

    for(size_t i = 0; i < centers.size(); ++i)
    {
        features.image_points.at(i).x = centers.at(i).x;
        features.image_points.at(i).y  = centers.at(i).y;
    }

    features.success = success;
    return features;
}

sensor_msgs::Image Circles::getImage()
{
    sensor_msgs::Image calImage;
    mono8->toImageMsg(calImage);
    return calImage;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_cb_detector, circles_target, calibration_target::Circles, calibration_target_base::CalibrationTarget)
