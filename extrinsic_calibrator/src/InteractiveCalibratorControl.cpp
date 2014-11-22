/**************************************************************************
** Copyright (c) 2014 United States Government as represented by the
** National Aeronotics and Space Administration.  All Rights Reserved
**
** Author: Allison Thackston
** Created: 29 Oct 2014
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
#include <deque>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_listener.h>
#include <dynamic_transform_publisher/Update.h>
#include <dynamic_transform_publisher/Save.h>

using namespace visualization_msgs;

class InteractiveCalibratorControl
{
public:
    InteractiveCalibratorControl(std::string parent_frame, std::string child_frame, std::string file_name, tf::Transform initial)
        : server(ros::this_node::getName())
        , baseFrame(parent_frame)
    {
        initialPose = initial;
        currentPose = initialPose;
        ros::NodeHandle nh;
        updater = nh.serviceClient<dynamic_transform_publisher::Update>(nh.resolveName("dynamic_tf_publisher")+"/update");
        ROS_INFO("updater: %s/update", nh.resolveName("dynamic_tf_publisher").c_str());
        if(!updater.exists())
        {
            ROS_ERROR("Could not find updater service");
            ros::shutdown();
        }
        updateSrv.request.child_frame = child_frame;
        updateSrv.request.parent_frame = parent_frame;

        saver = nh.serviceClient<dynamic_transform_publisher::Save>(nh.resolveName("dynamic_tf_publisher")+"/save");
        ROS_INFO("saver: %s/save", nh.resolveName("dynamic_tf_publisher").c_str());
        if(!saver.exists())
        {
            ROS_ERROR("Could not find save service");
            ros::shutdown();
        }
        fileName = file_name;

        //visualization_msgs::InteractiveMarker marker;
        marker.header.frame_id = parent_frame;
        marker.name = "InteractiveCalibratorMarker";
        marker.description = "6-DOF manual calibration";
        marker.pose = geometry_msgs::Pose();
        marker.scale = 0.1;

        visualization_msgs::InteractiveMarkerControl markerControl;
        markerControl.name="rotate_x";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 1;
        markerControl.orientation.y = 0;
        markerControl.orientation.z = 0;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name="rotate_y";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 0;
        markerControl.orientation.y = 0;
        markerControl.orientation.z = 1;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name="rotate_z";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 0;
        markerControl.orientation.y = 1;
        markerControl.orientation.z = 0;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name="move_x";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 1;
        markerControl.orientation.y = 0;
        markerControl.orientation.z = 0;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name="move_y";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 0;
        markerControl.orientation.y = 0;
        markerControl.orientation.z = 1;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name="move_z";
        markerControl.orientation.w = 1;
        markerControl.orientation.x = 0;
        markerControl.orientation.y = 1;
        markerControl.orientation.z = 0;
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(markerControl);
        markerControl.name = "menu_control";
        markerControl.description = "";
        markerControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
        marker.controls.push_back(markerControl);

        server.insert(marker, boost::bind(&InteractiveCalibratorControl::processFeedback, this, _1));
        menu.insert("Reset", boost::bind(&InteractiveCalibratorControl::processReset, this, _1));
        menu.insert("Save", boost::bind(&InteractiveCalibratorControl::processSave, this, _1));
        menu.insert("Undo", boost::bind(&InteractiveCalibratorControl::processUndo, this, _1));

        menu.apply(server, marker.name);
        server.applyChanges();
    }

    virtual ~InteractiveCalibratorControl()  {}

    void processReset(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {

        ROS_DEBUG_STREAM( "got reset on menu item " << feedback->menu_entry_id << " clicked." );
        poseTFToTransformMsg(initialPose, updateSrv.request.transform);
        if(updater.call(updateSrv))
        {
            ROS_INFO("Reset tf");
        }
        server.setPose(marker.name, geometry_msgs::Pose(), marker.header);
        server.applyChanges();
    }

    void processSave(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_DEBUG_STREAM( "got save on menu item " << feedback->menu_entry_id << " clicked." );
        if (fileName == "")
        {
            std::stringstream name;
            name << ros::this_node::getName() << ros::Time::now() <<".yaml";
            saveSrv.request.file_name = name.str();
        }
        else
        {
            saveSrv.request.file_name = fileName;
        }
        if(saver.call(saveSrv))
        {
            ROS_INFO("saved tf in %s", saveSrv.request.file_name.c_str());
        }
    }

    void processUndo(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        ROS_DEBUG_STREAM("got undo on menu item " << feedback->menu_entry_id << " clicked.");
        if (poseHistory.size() > 0)
        {
            currentPose = poseHistory.back();
            poseTFToTransformMsg(currentPose, updateSrv.request.transform);
            if(updater.call(updateSrv))
            {

                ROS_INFO_STREAM("undo tf to (" <<currentPose.getOrigin().x() << ", " << currentPose.getOrigin().y() << ", " << currentPose.getOrigin().z() << ") "
                                 << "("<<currentPose.getRotation().x() << ", " << currentPose.getRotation().y() << ", " <<currentPose.getRotation().z() << ", " << currentPose.getRotation().w() << ")");
            }
            poseHistory.pop_back();
        }
        else
        {
            ROS_INFO_STREAM("end of undo list");
        }
    }


    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
    {
        if(poseHistory.size() == 0 || !isEqual(currentPose, poseHistory.back()))
        {
            poseHistory.push_back(currentPose);
        }
        //! update dynamic transform
        ROS_DEBUG_STREAM("old transform: (" <<initialPose.getOrigin().x() << ", " << initialPose.getOrigin().y() << ", " << initialPose.getOrigin().z() << ") "
                        << "("<<initialPose.getRotation().x() << ", " << initialPose.getRotation().y() << ", " <<initialPose.getRotation().z() << ", " << initialPose.getRotation().w() << ")");
        ROS_DEBUG_STREAM( feedback->marker_name << " is now at "
                         << "("<< feedback->pose.position.x << ", " << feedback->pose.position.y << ", " << feedback->pose.position.z  << ") "
                         << "("<< feedback->pose.orientation.x << ", " <<feedback->pose.orientation.y << ", " << feedback->pose.orientation.z << ", " << feedback->pose.orientation.w << ")");

        tf::Transform t;
        tf::poseMsgToTF(feedback->pose, t);
        currentPose = initialPose*t;
        ROS_DEBUG_STREAM("new transform: (" <<currentPose.getOrigin().x() << ", " << currentPose.getOrigin().y() << ", " << currentPose.getOrigin().z() << ") "
                        << "("<<currentPose.getRotation().x() << ", " << currentPose.getRotation().y() << ", " <<currentPose.getRotation().z() << ", " << currentPose.getRotation().w() << ")");

        poseTFToTransformMsg(currentPose, updateSrv.request.transform);

        if(updater.call(updateSrv))
        {

            ROS_INFO_STREAM("updated tf to (" <<currentPose.getOrigin().x() << ", " << currentPose.getOrigin().y() << ", " << currentPose.getOrigin().z() << ") "
                             << "("<<currentPose.getRotation().x() << ", " << currentPose.getRotation().y() << ", " <<currentPose.getRotation().z() << ", " << currentPose.getRotation().w() << ")");
        }

        while (poseHistory.size() > 100)
        {
            poseHistory.pop_front();
        }

    }

    void poseTFToTransformMsg(const tf::Pose& pose, geometry_msgs::Transform &transform)
    {
        transform.translation.x = pose.getOrigin().x();
        transform.translation.y = pose.getOrigin().y();
        transform.translation.z = pose.getOrigin().z();
        transform.rotation.x = pose.getRotation().x();
        transform.rotation.y = pose.getRotation().y();
        transform.rotation.z = pose.getRotation().z();
        transform.rotation.w = pose.getRotation().w();
    }

    bool isEqual(const tf::Transform &a, const tf::Transform &b)
    {
        return (a.getOrigin() == b.getOrigin() && a.getRotation() == b.getRotation());
    }

private:
    interactive_markers::InteractiveMarkerServer server;
    std::string baseFrame;
    tf::Transform initialPose;
    visualization_msgs::InteractiveMarker marker;
    interactive_markers::MenuHandler menu;


    ros::ServiceClient updater;
    dynamic_transform_publisher::Update updateSrv;

    ros::ServiceClient saver;
    dynamic_transform_publisher::Save saveSrv;
    std::string fileName;
    tf::Transform currentPose;
    std::deque<tf::Transform> poseHistory;
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_marker");

    ros::NodeHandle nh("~");
    std::string parent_frame, child_frame, file_name;
    nh.getParam("parent_frame", parent_frame);
    nh.getParam("child_frame", child_frame);
    nh.param<std::string>("file_name", file_name, "");

    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform(parent_frame, child_frame, now-ros::Duration(1.0), ros::Duration(5.0));
        listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
        ROS_INFO_STREAM("found transform for " << parent_frame<<"->"<<child_frame<<": (" <<transform.getOrigin().x() << ", " << transform.getOrigin().y() << ", " << transform.getOrigin().z() << ") "
                        << "("<<transform.getRotation().x() << ", " << transform.getRotation().y() << ", " <<transform.getRotation().z() << ", " << transform.getRotation().w() << ")");

    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    InteractiveCalibratorControl control(parent_frame, child_frame, file_name, transform);
    ros::spin();
}
