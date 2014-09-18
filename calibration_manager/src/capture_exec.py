#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

import roslib; roslib.load_manifest('calibration_manager')
import rospy
import yaml
import actionlib

from calibration_manager.msg import *

if __name__=='__main__':
    rospy.init_node("capture_executive_client")

    rospy.logdebug("Starting executive...")

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

    try:
        for step in sample_steps:
            keep_collecting = True
            full_paths = [samples_dir + '/' + step + '/' + x for x in sample_names[step] ]
            print "full_paths"
            print full_paths
            cur_config = yaml.load(open(full_paths[0]))
            rospy.loginfo("cur_config: %s" % full_paths[0])
            client.send_goal(create_goal(cur_config))
            # wait for result in a quitable loop
            while not rospy.is_shutdown() and keep_collecting:
                print
                print sample_options[step]["prompt"]
                print "Press <enter> to continue, type N to exit this step, type Q to quit"
                resp = raw_input(">>>")
                if string.upper(resp) == "N":
                    print sample_options[step]["finish"]
                    keep_collecting = False
                elif string.upper(resp) == "Q":
                    exit(0)
                else:
                    for cur_sample_path in full_paths:
                        print "On %s sample [%s]" % (sample_options[step]["group"], cur_sample_path)
                        cur_config = yaml.load(open(cur_sample_path))
                        client.send_goal(create_goal(cur_config))
                        success = client.wait_for_result(rospy.Duration.from_sec(40.0))
                        if not success:
                            print "--------------- Failed To Capture a %s Sample -----------------" % sample_options[step]["group"]
                            if not sample_options[step]["repeat"]:
                                sample_failure[step] += 1
                        else:
                            print "++++++++++++++ Successfully Captured a %s Sample ++++++++++++++" % sample_options[step]["group"]
                            sample_success[step] += 1
                            pub.publish(m_robot)
                        print "Succeeded on %u %s samples" % (sample_success[step], sample_options[step]["group"])
                        if rospy.is_shutdown():
                            break
                    keep_collecting = sample_options[step]["repeat"]
    except EOFError:
        print "Exiting"    

    time.sleep(1)

    print "Calibration data collection has completed!"
    for step in sample_steps:
        if sample_options[step]["repeat"]:
            print "%s Samples: %u" % (sample_options[step]["group"], sample_success[step])
        else:
            print "%s Samples: %u/%u" % (sample_options[step]["group"], sample_success[step], sample_success[step] + sample_failure[step])
    print ""
    print "You can now kill this node, along with any other calibration nodes that are running."


