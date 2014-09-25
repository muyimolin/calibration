#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Oceaneering Space Systems, NASA
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
import actionlib
import os
import string
import thread, time

from actionlib_msgs.msg import *
from calibration_capture.msg import *

def create_goal(config, name):
    goal = CaptureManagerGoal()
    
    if 'camera_measurements' in config:
        for measurement in config['camera_measurements']:
            m = Measurement()
            m.id = measurement['cam_id']
            m.config = measurement['config']
            goal.camera_measurements.append(m)
                
    if 'joint_measurements' in config:
        for measurement in config['joint_measurements']:
            m = Measurement()
            m.id = measurement['chain_id']
            m.config = measurement['config']
            goal.joint_measurements.append(m)
        
    if 'laser_measurements' in config:
        for measurement in config['laser_measurements']:
            m = Measurement()
            m.id = measurement['laser_id']
            m.config = measurement['config']
            goal.laser_measurements.append(m)
        
    if 'joint_commands' in config:
        for command in config['joint_commands']:
            c = Command()
            c.controller = command['controller']
            for segment in command['segments']:
                c.segments.time_from_start = rospy.Duration.from_sec(segment['duration'])
                c.segments.positions = segment['positions']
            goal.joint_commands.append(c)

        
    goal.sample_id = config['sample_id']
    goal.target_id = config['target']['target_id']
    goal.chain_id  = config['target']['chain_id']
    
    return goal

def input_thread(resp):
    while not rospy.is_shutdown():
        resp.append(raw_input(">>>"))

if __name__=='__main__':
    rospy.init_node("capture_executive_client")

    rospy.logdebug("Starting executive...")
    
    timeout = rospy.get_param("~timeout", 40.0)

    try:
        samples_dir = rospy.get_param('~samples_dir')
    except:
        rospy.logfatal('samples_dir not set, exiting')
        sys.exit(-1)
        
    client = actionlib.SimpleActionClient('capture_manager', CaptureManagerAction)
    rospy.loginfo("Waiting for capture manager")
    client.wait_for_server()
    rospy.loginfo("Found capture manager")

    # setup our samples
    sample_steps = list()
    sample_names = dict()
    sample_options = dict()
    sample_success = dict()
    sample_failure = dict()
    for directory in os.listdir(samples_dir):
        try:
            sample_options[directory] = yaml.load(open(samples_dir + '/' + directory + '/config.yaml'))
            sample_steps.append(directory)
        except IOError:
            continue
        sample_names[directory] = [x for x in os.listdir(samples_dir + '/' + directory + '/') if x.endswith('.yaml') and x != 'config.yaml']
        sample_names[directory].sort()
        sample_success[directory] = 0
        sample_failure[directory] = 0
    sample_steps.sort()

    for step in sample_steps:
        rospy.logdebug("%s Samples: \n - %s" % (sample_options[step]['group'], "\n - ".join(sample_names[step])))
    
    print "Ready to begin collecting, press <enter> to continue"
    resp = []
    thread.start_new_thread(input_thread, (resp,))
    r =rospy.Rate(10)
    while not rospy.is_shutdown() and len(resp) == 0:
        r.sleep()

    try:
        for step in sample_steps:
            keep_collecting = True
            full_paths = [samples_dir + '/' + step + '/' + x for x in sample_names[step] ]
            # TODO: Should we pre-command the robot to the first point?
##            cur_config = yaml.load(open(full_paths[0]))
##            rospy.loginfo("cur_config: %s" % full_paths[0])
##            goal = create_goal(cur_config, os.path.basename(full_paths[0]))
##            client.send_goal(goal)
            # Set up prompt
            # This will give users time to set up the scene
            del resp[:]
            print "------------------------------------------------------------------"
            print sample_options[step]["prompt"]
            print "Type <enter> to continue, type N to exit this step, type Q to quit"
            print "------------------------------------------------------------------"
            while not rospy.is_shutdown():
                if len(resp) > 0:
                    break
                r.sleep()
            if "Q" in resp or "q" in resp:
                exit(0)
            if "N" in resp or "n" in resp:
                print "!!!!!!!!!!!!!!!!!! Skipping step !!!!!!!!!!!!!!!!!!"
                continue
            # loop through samples in a quitable loop
            for cur_sample_path in full_paths:
                active = False
                del resp[:]
                while not rospy.is_shutdown() and keep_collecting:
                    if not active:
                        print "****************************************************"
                        print "On %s sample [%s]" % (sample_options[step]["group"], cur_sample_path)
                        cur_config = yaml.load(open(cur_sample_path))
                        client.send_goal(create_goal(cur_config, os.path.basename(cur_sample_path)))
                        active = True
                        max_time = rospy.get_rostime() + rospy.Duration.from_sec(timeout)
                        print "Type S to go to next sample, Type N to exit this step, Type Q to quit"
                        print "******************************************************"
                    if client.get_state() == GoalStatus.SUCCEEDED:
                        print "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                        print " Successfully Captured a %s Sample, press <enter> to continue, N to go to next step " % sample_options[step]["group"]
                        print " +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++"
                        while not rospy.is_shutdown():
                            if len(resp) > 0:
                                break
                            r.sleep()
                        sample_success[step] += 1
                        keep_collecting = sample_options[step]["repeat"]
                        active = False
                    elif client.get_state() == GoalStatus.ACTIVE:
                        if rospy.get_rostime() > max_time and active:
                        #we have failed some how
                            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                            print " Timeout Capturing a %s Sample, press <enter to continue> " % sample_options[step]["group"]
                            print " ---------------------------------------------------------"
                            active = False
                            while not rospy.is_shutdown():
                                if len(resp) > 0:
                                    break
                                r.sleep()
                            client.cancel_goal()
                            active = False
                            if not sample_options[step]["repeat"]:
                                sample_failure[step] += 1
                                keep_collecting = sample_options[step]["repeat"]
                    elif client.get_state() == GoalStatus.PENDING:
                        #do nothing
                        pass
                    else:
                        #we have failed some how
                            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                            print " Failed To Capture a %s Sample, press <enter to continue> " % sample_options[step]["group"]
                            print " ---------------------------------------------------------"
                            active = False
                            while not rospy.is_shutdown():
                                if len(resp) > 0:
                                    break
                                r.sleep()
                            if not sample_options[step]["repeat"]:
                                sample_failure[step] += 1
                                keep_collecting = sample_options[step]["repeat"]
                          
                    if "Q" in resp or "q" in resp:
                        exit(0)
                    if "N" in resp or "n" in resp:
                        print "!!!!!!!!!!!!!! Skipping to next step !!!!!!!!!!!!!!!!!!!!!!!"
                        keep_collecting = False
                    if "S" in resp or "s" in resp:
                        print " ......Moving on to next sample..... "
                        client.cancel_goal()
                        active = False
                        if not sample_options[step]["repeat"]:
                                sample_failure[step] += 1
                                keep_collecting = sample_options[step]["repeat"]
                    
                    del resp[:]            
                    r.sleep()
            print sample_options[step]["finish"]
                        
                    
    except EOFError:
        print "Exiting"    

    print "Calibration data collection has completed!"
    for step in sample_steps:
        if sample_options[step]["repeat"]:
            print "%s Samples: %u" % (sample_options[step]["group"], sample_success[step])
        else:
            print "%s Samples: %u/%u" % (sample_options[step]["group"], sample_success[step], sample_success[step] + sample_failure[step])
    print ""
    print "You can now kill this node, along with any other calibration nodes that are running."


