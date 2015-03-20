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

#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <cv_bridge/cv_bridge.h>
#include <boost/make_shared.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace ImageConverter
{
inline cv_bridge::CvImageConstPtr toMono8(const sensor_msgs::ImageConstPtr &image)
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
        return ptr;
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
        return ptr;
    }

    else
    {
        return (cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8));
    }
    return cv_bridge::CvImagePtr();
}

}

#endif // IMAGE_CONVERTER_H
