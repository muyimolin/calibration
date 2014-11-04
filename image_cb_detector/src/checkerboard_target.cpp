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

#include <image_cb_detector/checkerboard_target.h>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

using namespace calibration_target;
using namespace cv;

void Checkerboard::initialize(std::string name)
{
    nh = ros::NodeHandle(name);
    nh.param<int>("num_y", numY, 6);
    nh.param<int>("num_x", numX, 8);
    nh.param<double>("dim_x", dimX, 1.);
    nh.param<double>("dim_y", dimY, 1.);
    nh.param<int>("flags", flags, 0);

    ROS_INFO("[%s][num_y]: %i", ros::names::resolve(name).c_str(), numY);
    ROS_INFO("[%s][num_x]: %i", ros::names::resolve(name).c_str(), numX);
    ROS_INFO("[%s][dim_x]: %f", ros::names::resolve(name).c_str(), dimX);
    ROS_INFO("[%s][dim_y]: %f", ros::names::resolve(name).c_str(), dimY);
    ROS_INFO("[%s][flags]: %i", ros::names::resolve(name).c_str(), flags);

    boardSize = cv::Size(numX, numY);
    corners.resize(numX * numY);
    features.object_points.resize(numX * numY);
    for (int i = 0; i < numY; ++i)
    {
        for (int j = 0; j < numX; ++j)
        {
            features.object_points.at(i*numX + j).x = j * dimX;
            features.object_points.at(i*numX + j).y = i * dimY;
            features.object_points.at(i*numX + j).z = 0.0;
        }
    }
}

bool Checkerboard::convertImage(const sensor_msgs::ImageConstPtr &image)
{
    try
    {
        if (image->encoding.find("FC1") != std::string::npos)
        {
            cv_bridge::CvImagePtr cvImgCopy = cv_bridge::toCvCopy(image);
            cv::Mat convertImg(cvImgCopy->image.rows, cvImgCopy->image.cols, CV_8UC1);

            // find max
            double max = 0;
            for(int i = 0; i < cvImgCopy->image.rows; i++)
            {
                float* Di = cvImgCopy->image.ptr<float>(i);
                for(int j = 0; j < cvImgCopy->image.cols; j++)
                {
                    if(Di[j]> max)
                    {
                        max = Di[j];
                    }
                }
            }
            // scale
            if (max > 0)
            {
                for(int i = 0; i < cvImgCopy->image.rows; i++)
                {
                    float* Di = cvImgCopy->image.ptr<float>(i);
                    char* Ii = convertImg.ptr<char>(i);
                    for(int j = 0; j < cvImgCopy->image.cols; j++)
                    {
                        Ii[j] = (char) (255/max*(Di[j]));
                    }
                }
            }
            else
            {
                for(int i = 0; i < cvImgCopy->image.rows; i++)
                {
                    float* Di = cvImgCopy->image.ptr<float>(i);
                    char* Ii = convertImg.ptr<char>(i);
                    for(int j = 0; j < cvImgCopy->image.cols; j++)
                    {
                        Ii[j] = (char) (Di[j]);
                    }
                }
            }

            cv_bridge::CvImagePtr ptr = boost::make_shared< cv_bridge::CvImage>();
            ptr->header = image->header;
            ptr->encoding = sensor_msgs::image_encodings::MONO8;
            ptr->image = convertImg;
            mono8 = ptr;
        }
        else if(image->encoding.find("bayer") != std::string::npos)
        {
            cv_bridge::CvImagePtr cvImgCopy = cv_bridge::toCvCopy(image);
            int convert_type;
            if (image->encoding == "bayer_rggb8") convert_type = CV_BayerBG2GRAY;
            if (image->encoding == "bayer_bggr8") convert_type = CV_BayerRG2GRAY;
            if (image->encoding == "bayer_gbrg8") convert_type = CV_BayerGR2GRAY;
            if (image->encoding == "bayer_grbg8") convert_type = CV_BayerGR2GRAY;
            cv::Mat mono(cvImgCopy->image.rows, cvImgCopy->image.cols, CV_8UC1);
            cv::cvtColor(cvImgCopy->image, mono, convert_type);

            cv_bridge::CvImagePtr ptr = boost::make_shared< cv_bridge::CvImage>();
            ptr->header = image->header;
            ptr->encoding = sensor_msgs::image_encodings::MONO8;
            ptr->image = mono;
            mono8 = ptr;
        }

        else
        {
            mono8 = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
        }
    }
    catch (cv_bridge::Exception error)
    {
        ROS_ERROR("[%s] error: %s", ros::this_node::getName().c_str(), error.what());
        return false;
    }
    return true;
}

double Checkerboard::calcDistance(cv::Point2f a, cv::Point2f b)
{
    return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y-b.y)*(a.y-b.y));
}

void Checkerboard::detect(const sensor_msgs::ImageConstPtr &image)
{
    //! update features
    header = image->header;

    //! convert the image
    if(convertImage(image))
    {
        //! find the corners
        int found = cv::findChessboardCorners(mono8->image, boardSize, corners, flags);

        if (found)
        {
            ROS_DEBUG("[%s] found checkerboard", ros::this_node::getName().c_str());
            //! find the minimum distance between corners, and set half that size to subpixel window
            double minDist = std::max(image->width, image->height);
            for(int i = 0; i < numY; ++i)
            {
                for (int j = 0; j < numX-1; ++j)
                {
                    minDist = std::min(minDist, calcDistance(corners.at(i*numX + j), corners.at(i*numX + j + 1)));
                }
            }
            for (int i = 0; i < numY-1; ++i)
            {
                for (int j = 0; j < numX; ++j)
                {
                    minDist = std::min(minDist, calcDistance(corners.at(i*numX + j), corners.at(i*numX+j + numX)));
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

calibration_msgs::CalibrationPattern Checkerboard::getFeatures()
{
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

sensor_msgs::Image Checkerboard::getImage()
{
    sensor_msgs::Image calImage;
    mono8->toImageMsg(calImage);
    return calImage;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(image_cb_detector, checkerboard_target, calibration_target::Checkerboard, calibration_target_base::CalibrationTarget)
