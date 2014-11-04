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

#ifndef CALIBRATION_TARGET_BASE_H_
#define CALIBRATION_TARGET_BASE_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <calibration_msgs/CalibrationPattern.h>

namespace calibration_target_base {
  /**
   * @class CalibrationPlate
   * @brief Provides an interface for adding calibration plates
   */
  class CalibrationTarget{
    public:
      /**
       * @brief Initialization function for the CalibrationPlate
       * @param name The name of this calibration plate
       */
      virtual void initialize(std::string name) = 0;

      /**
       * @brief given an image, detect the calibration plate
       * @param image The image to use for detection
       */
      virtual void detect(const sensor_msgs::ImageConstPtr & image) = 0;

      /**
       * @brief get the latest calibration detection
       */
      virtual calibration_msgs::CalibrationPattern getFeatures() = 0;

      /**
       * @brief get the modified image used for calibration
       */
      virtual sensor_msgs::Image getImage() = 0;
  
      /**
       * @brief  Virtual destructor for the interface
       */
      virtual ~CalibrationTarget(){}

    protected:
      CalibrationTarget(){}
  };
}

#endif //CALIBRATION_TARGET_BASE_H_
