/**************************************************************************
** Copyright (c) 2014 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 28 Oct 2014
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
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_transform_publisher/Save.h>
#include <dynamic_transform_publisher/Update.h>
#include <geometry_msgs/Transform.h>
#include <yaml-cpp/yaml.h>

class TransformSender
{
public:
    TransformSender() {}
    virtual ~TransformSender() {}

    void set(double x, double y, double z, double qx, double qy, double qz, double qw, std::string parent_frame, std::string child_frame)
    {
        transform.setOrigin( tf::Vector3(x, y, z));
        transform.setRotation( tf::Quaternion(qx, qy, qz, qw));
        parentFrame = parent_frame;
        childFrame = child_frame;
    }

    void send(ros::Time time)
    {
        if (parentFrame != "" && childFrame != "")
        {
            br.sendTransform(tf::StampedTransform(transform, time, parentFrame, childFrame));
        }
    }

    bool update(dynamic_transform_publisher::UpdateRequest &request, dynamic_transform_publisher::UpdateResponse &response)
    {
        set(request.transform.translation.x, request.transform.translation.y, request.transform.translation.z
            , request.transform.rotation.x, request.transform.rotation.y, request.transform.rotation.z, request.transform.rotation.w
            , request.parent_frame, request.child_frame);
        return true;
    }

    bool save(dynamic_transform_publisher::SaveRequest &request, dynamic_transform_publisher::SaveResponse &response)
    {
        YAML::Emitter out;
         out << YAML::BeginMap;
         out << YAML::Key << "parent_frame" << YAML::Value << parentFrame;
         out << YAML::Key << "child_frame" << YAML::Value <<childFrame;
         out << YAML::Key << "transform" << YAML::Value;
         out << YAML::BeginMap;
         out << YAML::Key << "translation" << YAML::Value;
         out << YAML::BeginMap;
         out << YAML::Key << "x" << YAML::Value << transform.getOrigin().x();
         out << YAML::Key << "y" << YAML::Value << transform.getOrigin().y();
         out << YAML::Key << "z" << YAML::Value << transform.getOrigin().z();
         out << YAML::EndMap;
         out << YAML::Key << "rotation" << YAML::Value;
         out << YAML::BeginMap;
         out << YAML::Key << "x" << YAML::Value << transform.getRotation().x();
         out << YAML::Key << "y" << YAML::Value << transform.getRotation().y();
         out << YAML::Key << "z" << YAML::Value << transform.getRotation().z();
         out << YAML::Key << "w" << YAML::Value << transform.getRotation().w();
         out << YAML::EndMap;
         out << YAML::EndMap;
         out << YAML::EndMap;

         std::ofstream fout;
         fout.open(request.file_name.c_str());
         fout << out.c_str();
         fout.close();
         return true;
    }

private:
    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string parentFrame;
    std::string childFrame;

};


int main(int argc, char** argv){
  ros::init(argc, argv, "dynamic_tf_publisher");

  ros::NodeHandle nh("~");
  TransformSender ts;

  ros::ServiceServer updateSrv = nh.advertiseService("update", &TransformSender::update, &ts);
  ros::ServiceServer saveSrv   = nh.advertiseService("save", &TransformSender::save, &ts);
  double rate;
  nh.param<double>("rate", rate, 30);
  ros::Duration sleeper = ros::Duration(1/rate);

  std::string parentFrame, childFrame;
  double x, y, z, qx, qy, qz, qw;
  if (nh.getParam("parent_frame", parentFrame) &&
      nh.getParam("child_frame", childFrame) &&
      nh.getParam("transform/translation/x", x) &&
      nh.getParam("transform/translation/y", y) &&
      nh.getParam("transform/translation/z", z) &&
      nh.getParam("transform/rotation/x", qx) &&
      nh.getParam("transform/rotation/y", qy) &&
      nh.getParam("transform/rotation/z", qz) &&
      nh.getParam("transform/rotation/w", qw))
  {
      ts.set(x, y, z, qx, qy, qz, qw, parentFrame, childFrame);
  }

  while(nh.ok())
  {
      ts.send(ros::Time::now() + sleeper);
      sleeper.sleep();
      ros::spinOnce();
  }

  return 0;
};
