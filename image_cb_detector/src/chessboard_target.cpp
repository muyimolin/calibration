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

#include <image_cb_detector/chessboard_target.h>

namespace enc = sensor_msgs::image_encodings;

using namespace calibration_target;
using namespace cv;

void Chessboard::initialize(std::string name)
{
    nh = ros::NodeHandle(name);
    configured = false;
    // Setup dynamic reconfigure server
    reconfigureServer.reset(new ReconfigureServer(config_mutex, nh));
    ReconfigureServer::CallbackType f = boost::bind(&Chessboard::reconfigureCallback, this, _1, _2);
    reconfigureServer->setCallback(f);
}

void Chessboard::reconfigureCallback(image_cb_detector::ChessboardConfig &config, uint32_t level)
{
    ROS_INFO("[%s][num_x]: %i", nh.getNamespace().c_str(), config.num_x);
    ROS_INFO("[%s][num_y]: %i", nh.getNamespace().c_str(), config.num_y);
    ROS_INFO("[%s][dim_x]: %f", nh.getNamespace().c_str(), config.dim_x);
    ROS_INFO("[%s][dim_y]: %f", nh.getNamespace().c_str(), config.dim_y);
    flags = 0;
    if(config.adaptive_threshold)
    {
        flags = flags | CALIB_CB_ADAPTIVE_THRESH;
    }
    if(config.normalize)
    {
        flags = flags | CALIB_CB_NORMALIZE_IMAGE;
    }
    if(config.filter_quads)
    {
        flags = flags | CALIB_CB_FILTER_QUADS;
    }
    if(config.fast)
    {
        flags = flags | CALIB_CB_FAST_CHECK;
    }
    ROS_INFO("[%s][flags]: %i", nh.getNamespace().c_str(), flags);

    boardSize = cv::Size(config.num_x, config.num_y);
    corners.resize(boardSize.area());
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
    configured = true;
}

double Chessboard::calcDistance(cv::Point2f a, cv::Point2f b)
{
    return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y-b.y)*(a.y-b.y));
}

void Chessboard::detect(const sensor_msgs::ImageConstPtr &image)
{
    //! update features
    header = image->header;

    //! convert the image
    mono8 = ImageConverter::toMono8(image);
    if(mono8 && configured)
    {
        boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex);
        //! find the corners
        int found = cv::findChessboardCorners(mono8->image, boardSize, corners, flags);

        if (found)
        {
            ROS_DEBUG("[%s] found checkerboard", ros::this_node::getName().c_str());
            //! find the minimum distance between corners, and set half that size to subpixel window
            double minDist = std::max(image->width, image->height);
            for(int i = 0; i < boardSize.height; ++i)
            {
                for (int j = 0; j < boardSize.width-1; ++j)
                {
                    minDist = std::min(minDist, calcDistance(corners.at(i*boardSize.width + j), corners.at(i*boardSize.width + j + 1)));
                }
            }
            for (int i = 0; i < boardSize.height-1; ++i)
            {
                for (int j = 0; j < boardSize.width; ++j)
                {
                    minDist = std::min(minDist, calcDistance(corners.at(i*boardSize.width + j), corners.at(i*boardSize.width + j + boardSize.width)));
                }
            }
            cv::cornerSubPix(mono8->image, corners, cv::Size( std::ceil(minDist/2),  std::ceil(minDist/2)), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 30, 1e-2));
            success = true;
        }
        else
        {
            ROS_DEBUG("[%s] didn't find checkerboard", ros::this_node::getName().c_str());
            success = false;
        }
    }
}

calibration_msgs::CalibrationPattern Chessboard::getFeatures()
{
    boost::lock_guard<boost::recursive_mutex> scoped_lock(config_mutex);
    //! fill out the header
    features.header = header;

    //! resize image points in case it changes on no detection
    features.image_points.resize(corners.size());

    for(size_t i = 0; i < corners.size(); ++i)
    {
        features.image_points.at(i).x = corners.at(i).x;
        features.image_points.at(i).y  = corners.at(i).y;
    }

    features.success = success;
    return features;
}

sensor_msgs::Image Chessboard::getImage()
{
    sensor_msgs::Image calImage;
    mono8->toImageMsg(calImage);
    return calImage;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_cb_detector, chessboard_target, calibration_target::Chessboard, calibration_target_base::CalibrationTarget)
