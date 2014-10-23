#!/usr/bin/env python
#
# Copyright (c) 2014 United States Government as represented by the
# National Aeronotics and Space Administration.  All Rights Reserved
#
# Author: Allison Thackston
# Created: 22 Oct 2014
#
# Developed jointly by NASA/JSC and Oceaneering Space Systems
#
# Licensed under the NASA Open Source Agreement v1.3 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://opensource.org/licenses/NASA-1.3
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import roslib; roslib.load_manifest('dynamic_transform_publisher')
import rospy
import yaml
import tf
from dynamic_transform_publisher.srv import *
from geometry_msgs.msg import *

class TransformSender:
    def __init__(self):
        self._transform = Transform()
        self._parent_frame = None
        self._child_frame = None
        self._br = tf.TransformBroadcaster()
        pass
    
    def set(self, x, y, z, qx, qy, qz, qw, parent_frame, child_frame):
        self._transform = Transform((x, y, z), (qx, qy, qz, qw))
        self._parent_frame = parent_frame
        self._child_frame = child_frame
    
    def send(self, time):
        if self._parent_frame and self._child_frame:
            self._br.sendTransform(self._transform.translation, self._transform.rotation, time, self._parent_frame, self._child_frame)
    
    def update(self, req):
        self.set(req.transform.translation.x, req.transform.translation.y, req.transform.translation.z,
                 req.transform.rotation.x, req.transform.rotation.y, req.transform.rotation.z, req.transform.rotation.w,
            req.parent_frame, req.child_frame)
        return UpdateResponse()
    
    def save(self, req):
        data = dict(
            parent_frame = self._parent_frame,
            child_frame = self._child_frame,
            transform = dict( translation = dict( x = self._transform.translation[0],
                                                  y = self._transform.translation[1],
                                                  z = self._transform.translation[2]),
                              rotation =  dict( x = self._transform.rotation[0],
                                                y = self._transform.rotation[1],
                                                z = self._transform.rotation[2],
                                                w = self._transform.rotation[3])))
                                                
        with open(req.file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        
        return SaveResponse()


if __name__ == '__main__':
    rospy.init_node('dynamic_transform_publisher')
    
    tf_sender = TransformSender()
    updater = rospy.Service('~update', Update, tf_sender.update)
    saver = rospy.Service('~save', Save, tf_sender.save)
    rate = rospy.Rate(rospy.get_param("~rate", 30))
    
    # check to see if transform parameters have been set
    try:
        parent_frame = rospy.get_param('~parent_frame')
        child_frame = rospy.get_param('~child_frame')
        transform = rospy.get_param('~transform')
        tf_sender.set(transform['translation']['x'], transform['translation']['y'], transform['translation']['z'],
                      transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'], transform['rotation']['w'],
                      parent_frame, child_frame)
        rospy.loginfo("Starting with initial transform: %s->%s {(%f, %f, %f) (%f, %f, %f, %f)}" % (parent_frame, child_frame, transform['translation']['x'], transform['translation']['y'], transform['translation']['z'],
                      transform['rotation']['x'], transform['rotation']['y'], transform['rotation']['z'], transform['rotation']['w']))
    except:
        rospy.loginfo("Starting with no initial transform")
    
    while not rospy.is_shutdown():
        tf_sender.send(rospy.Time.now())
        rate.sleep()
