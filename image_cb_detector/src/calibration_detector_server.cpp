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

#include <image_cb_detector/calibration_detector_server.h>

CalibrationDetectorServer::CalibrationDetectorServer(std::string name)
    :nh()
    ,target_loader("image_cb_detector", "calibration_target_base::CalibrationTarget")
    ,action(name, false)
{
    featurePub = nh.advertise<calibration_msgs::CalibrationPattern>("features", 1);
    ROS_INFO("[%s] Publishing to: %s", ros::this_node::getName().c_str(), ros::names::remap("features").c_str());
    imagePub   = nh.advertise<sensor_msgs::Image>("calibration_image", 1);
    ROS_INFO("[%s] Publishing to: %s", ros::this_node::getName().c_str(), ros::names::remap("calibration_image").c_str());

    imageSub = nh.subscribe("image_raw", 1, &CalibrationDetectorServer::imageCallback, this);
    ROS_INFO("[%s] Subscribing to: %s", ros::this_node::getName().c_str(), ros::names::remap("image_raw").c_str());

    action.registerGoalCallback( boost::bind(&CalibrationDetectorServer::goalCallback, this) );
    action.registerPreemptCallback( boost::bind(&CalibrationDetectorServer::preemptCallback, this) );

    action.start();
}

CalibrationDetectorServer::~CalibrationDetectorServer()
{
    ROS_WARN("[%s] Shutting down", ros::this_node::getName().c_str());
    action.shutdown();
    imageSub.shutdown();
    featurePub.shutdown();
    imagePub.shutdown();
}

void CalibrationDetectorServer::goalCallback()
{
    image_cb_detector::SelectGoalConstPtr currGoal =  action.acceptNewGoal();
    try
    {
        target = target_loader.createInstance(currGoal->calibration_target_type);
        target->initialize(currGoal->calibration_target_name);
    }
    catch(pluginlib::PluginlibException& e)
    {
        ROS_ERROR("[%s] Could not load target plugin. Error %s", ros::this_node::getName().c_str(), e.what());
        action.setAborted();
    }
}

void CalibrationDetectorServer::preemptCallback()
{
    ROS_INFO("[%s] Premempted", ros::this_node::getName().c_str());
    action.setPreempted();
}

void CalibrationDetectorServer::imageCallback(const sensor_msgs::ImageConstPtr &image)
{
    if (!action.isActive())
    {
        return;
    }
    target->detect(image);
    imagePub.publish(target->getImage());
    featurePub.publish(target->getFeatures());
}

/**************************************************************************
** main portion
**************************************************************************/
#include <signal.h>
#include <ros/xmlrpc_manager.h>

// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_detector_action", ros::init_options::NoSigintHandler);
    signal(SIGINT, mySigIntHandler);

    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

    ros::NodeHandle pnh("~");

    bool autostart;
    pnh.param<bool>("autostart", autostart, false);

    std::string target_type;
    pnh.param<std::string>("target_type", target_type, "image_cb_detector/checkerboard_target");

    std::string target_name;
    pnh.param<std::string>("target_name", target_name, ros::this_node::getName());

    try
    {
        CalibrationDetectorServer calibrationAction(ros::this_node::getName());
        if(autostart)
        {
            ROS_INFO("[%s] Autostarting %s %s", ros::this_node::getName().c_str(), target_type.c_str(), target_name.c_str());
            ros::Publisher goalPub = pnh.advertise<image_cb_detector::SelectActionGoal>("goal", 1, true);
            image_cb_detector::SelectActionGoal goalMsg;
            goalMsg.goal.calibration_target_type = target_type;
            goalMsg.goal.calibration_target_name = target_name;
            goalMsg.header.stamp = ros::Time::now();
            goalMsg.header.frame_id = "autostart";
            goalMsg.goal_id.stamp = ros::Time::now();
            goalMsg.goal_id.id = "autostart";
            goalPub.publish(goalMsg);
        }
        while (!g_request_shutdown)
        {
            ros::spinOnce();
        }
    }
    catch (std::exception& e)
    {
        ROS_FATAL("[%s] CalibrationDetectorServer caught exception.  Aborting. %s", ros::this_node::getName().c_str(), e.what());
        ROS_BREAK();
    }

    ros::shutdown();
    return 0;
}

