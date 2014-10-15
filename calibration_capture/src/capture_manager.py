#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Oceaneering Space Systems / NASA
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Oceaneering Space Systems or NASA nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Allison Thackston

import roslib; roslib.load_manifest('calibration_capture')
import rospy
import yaml
import threading
import actionlib
from urdf_python.urdf import *
from calibration_capture.msg import *
from calibration_msgs.msg import *
from capture_executive.config_manager import *
from capture_executive.sensor_managers import *
from capture_executive.robot_measurement_cache import RobotMeasurementCache


class CaptureManagerServer:
    def __init__(self, config_dir, system, robot_desc):
        
        # Load Configs
        self.cam_config        = yaml.load(open(config_dir + "/cam_config.yaml"))
        self.chain_config      = yaml.load(open(config_dir + "/chain_config.yaml"))
        self.laser_config      = yaml.load(open(config_dir + "/laser_config.yaml"))
        self.controller_config = yaml.load(open(config_dir + "/controller_config.yaml"))
        
        # Not all robots have all the things
        if self.cam_config == None:
            self.cam_config = dict()
        if self.chain_config == None:
            self.chain_config = dict()
        if self.laser_config == None:
            self.laser_config = dict()
        if self.controller_config == None:
            self.controller_config = dict()
            
        # parse urdf and get list of links
        links = URDF().parse(robot_desc).links.keys()
        
        # load system config
        system = yaml.load(open(system))
        
        # remove cams that are not on urdf
        for cam in self.cam_config.keys():
            try:
                link = system['sensors']['rectified_cams'][cam]['frame_id']
                if not link in links:
                    rospy.logwarn( 'URDF does not contain link [%s]. Removing camera [%s]' % (link, cam))
                    del self.cam_config[cam]
            except:
                rospy.logwarn( 'System description does not contain camera [%s]' % cam)
                del self.cam_config[cam]
                
        # remove lasers that are not on urdf
        for laser in self.laser_config.keys():
            try:
                link = system['sensors']['tilting_lasers'][laser]['frame_id']
                if not link in links:
                    rospy.logwarn( 'URDF does not contain link [%s]. Removing laser [%s]' % (link, laser) )
                    del self.laser_config[laser]
            except:
                rospy.logwarn( 'System description does not contain laser [%s]' % laser )
                del self.laser_config[laser]
                
        # remove chains that are not on urdf
        for chain in self.chain_config.keys():
            try:
                link = system['sensors']['chains'][chain]['tip']
                if not link in links:
                    rospy.logwarn( 'URDF does not contain link [%s]. Removing chain [%s]' % (link, chain) )
                    del self.chain_config[chain]
                    continue
                link = system['sensors']['chains'][chain]['root']
                if not link in links:
                    rospy.logwarn( 'URDF does not contain link [%s]. Removing chain [%s]' % (link, chain) )
                    del self.chain_config[chain]      
                    continue      
            except:
                rospy.logwarn( 'System description does not contain chain [%s]' % chain )
                del self.chain_config[chain]
                
        # Construct Configuration Manager (manages configuration of nodes in the pipeline)
        self.config_manager = ConfigManager(self.cam_config,
                                            self.chain_config,
                                            self.laser_config,
                                            self.controller_config)
                                            
        # Construct a manager for each sensor stream (Don't enable any of them)
        self.cam_managers   = [ (cam_id,   CamManager(  cam_id,   self.add_cam_measurement) )   for cam_id   in self.cam_config.keys() ]
        self.chain_managers = [ (chain_id, ChainManager(chain_id, self.add_chain_measurement) ) for chain_id in self.chain_config.keys() ]
        self.laser_managers = [ (laser_id, LaserManager(laser_id, self.add_laser_measurement) ) for laser_id in self.laser_config.keys() ]
        
        # Create measurement cache
        self.cache = RobotMeasurementCache()
        
        # Set up goals
        self.goal_sample_id = None
        self.goal_target_id = None
        self.goal_chain_id  = None
        
        # Set up publisher
        self.result = rospy.Publisher("robot_measurement", RobotMeasurement)
        
        # Subscribe to topic containing stable intervals
        self.request_interval_sub = rospy.Subscriber("intersected_interval", calibration_msgs.msg.Interval, self.request_callback)

        self.interval_status_sub = rospy.Subscriber(
              "intersected_interval_status",
              calibration_msgs.msg.IntervalStatus, self.status_callback)
            
        # Set up action
        self._server = actionlib.SimpleActionServer('capture_manager', CaptureManagerAction, None, False)
        self._server.register_goal_callback(self.goal_callback)
        self._server.register_preempt_callback(self.preempt_callback)
        self._server.start()

        
        
    def goal_callback(self):
        rospy.loginfo("[%s] Setting calibration configuration" % rospy.get_name())
        goal = self._server.accept_new_goal()

        # get ids
        cam_ids = [x.id for x in goal.camera_measurements]
        laser_ids = [x.id for x in goal.laser_measurements]
        chain_ids = [x.id for x in goal.joint_measurements]
        
        self.goal_sample_id = goal.sample_id
        self.goal_target_id = goal.target_id
        self.goal_chain_id  = goal.chain_id
                
        # Set up the pipeline
        self.config_manager.reconfigure(goal)
        rospy.logdebug("[%s] Set up pipelines" % rospy.get_name())
        
        # Set up cache
        self.cache.clear()
        self.cache.reconfigure(cam_ids, chain_ids, laser_ids)
        rospy.logdebug("[%s] Configurating cache" % rospy.get_name())

        # Set up the sensor managers
        enable_list = []
        disable_list = []
        for cam_id, cam_manager in self.cam_managers:
            if cam_id in cam_ids:
                enable_list.append(cam_id)
                cam_manager.enable()
            else:
                disable_list.append(cam_id)
                cam_manager.disable()
                
        rospy.logdebug("[%s] Set up camera managers" % rospy.get_name())

        for chain_id, chain_manager in self.chain_managers:
            if chain_id in chain_ids:
                enable_list.append(chain_id)
                chain_manager.enable()
            else:
                disable_list.append(chain_id)
                chain_manager.disable()
                
        rospy.logdebug("[%s] Set up chain managers" % rospy.get_name())

        for laser_id, laser_manager in self.laser_managers:
            if laser_id in laser_ids:
                enable_list.append(laser_id)
                laser_manager.enable()
            else:
                disable_list.append(laser_id)
                laser_manager.disable()
                
        rospy.logdebug("[%s] Set up laser managers" % rospy.get_name())
        
        if self._server.is_preempt_requested():
            rospy.loginfo("[%s] preempt detected during accept" % rospy.get_name())
            self._server.set_preempted()
    
    def preempt_callback(self):
        rospy.loginfo("[%s] preemting..." % rospy.get_name())
        self._server.set_preempted()
        self.goal_sample_id = None
        self.goal_target_id = None
        self.goal_chain_id  = None
        
        #disable all the things
        for cam_id, cam_manager in self.cam_managers:
            cam_manager.disable()
                
        rospy.loginfo("[%s] Disabled camera managers" % rospy.get_name())

        for chain_id, chain_manager in self.chain_managers:
            chain_manager.disable()
                
        rospy.loginfo("[%s] Disabled chain managers" % rospy.get_name())

        for laser_id, laser_manager in self.laser_managers:
                laser_manager.disable()
                
        rospy.loginfo("[%s] Disabled laser managers" % rospy.get_name())      
        
    def request_callback(self, msg):
        rospy.logdebug("[%s] in request_callback" % rospy.get_name())
        # See if the interval is big enough to care
        # TODO: make this a property
        if (msg.end - msg.start) < rospy.Duration(1,0):
            return
        if(self._server.is_active() and self.goal_sample_id):
            rospy.logdebug("[%s] server active" % rospy.get_name())
            m = self.cache.request_robot_measurement(msg.start, msg.end)
            if isinstance(m, basestring):
                self.message = m
            else:
                m.sample_id = self.goal_sample_id
                m.target_id = self.goal_target_id
                m.chain_id  = self.goal_chain_id
                msg = CaptureManagerResult()
                msg.result = m
                # Set succeeded
                self._server.set_succeeded(msg)
                # Publish RobotMeasurement
                self.result.publish(m)
                # Reset ids
                self.goal_sample_id = None
                self.goal_target_id = None
                self.goal_chain_id  = None
                
        
    def status_callback(self, msg):
        if(self._server.is_active() and self.goal_sample_id):
            self.interval_status=msg
            
    def add_cam_measurement(self, cam_id, msg):
        if (self._server.is_active() and self.goal_sample_id):
            self.cache.add_cam_measurement(cam_id, msg)
            
    def add_chain_measurement(self, chain_id, msg):
        if(self._server.is_active() and self.goal_sample_id):
            rospy.logdebug("[%s] Adding chain measurement" % rospy.get_name())
            self.cache.add_chain_measurement(chain_id, msg)
            rospy.logdebug("[%s] Finished adding chain measurement" % rospy.get_name())
        
    def add_laser_measurement(self, laser_id, msg, interval_start, interval_end):
        if(self._server.is_active() and self.goal_sample_id):
            self.cache.add_laser_measurement(laser_id, msg, interval_start, interval_end)
            
if __name__=='__main__':

    config_dir        = rospy.get_param("~config_dir", None)
    system            = rospy.get_param("~system", None)
    robot_description = rospy.get_param('robot_description', None)
    
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-c", "--config", dest= "config_dir", type="string", default=config_dir, help="Directory containing configuration files")
    parser.add_option("-s", "--system", dest= "system", type="string", default=system, help="System file")
    parser.add_option("-r", "--robot_desc", dest = "robot_description", type="string", default=robot_description, help="The urdf of the robot")
    parser.add_option("-d", "--debug", action="store_false", dest="debug", default=False, help="Set log level to debug")
    
    options, args = parser.parse_args()
    if options.debug:
        rospy.init_node("capture_manager", log_level=rospy.DEBUG)
    else:
        rospy.init_node("capture_manager")
    rospy.loginfo("[%s] Starting..." % rospy.get_name())
    
    config_dir        = options.config_dir
    system            = options.system
    robot_description = options.robot_description
    
    if not config_dir:
        rospy.logfatal("Config dir not set")
        sys.exit(-1)
        
    if not system:
        rospy.logfatal("System file not set")
        sys.exit(-1)
        
    if not robot_description:
        rospy.logfatal("Robot description not set")
        sys.exit(-1)
    
    rospy.loginfo("[%s] [config_dir] %s" % (rospy.get_name(), str(config_dir)))
    rospy.loginfo("[%s] [system] %s" % (rospy.get_name(), str(system)))
            
    CaptureManagerServer(config_dir, system, robot_description)
    
    rospy.spin()
