#!/usr/bin/env python

import roslib; roslib.load_manifest('calibration_manager')
import rospy
import yaml
import threading
import actionlib
roslib.load_manifest('calibration_manager')
from urdf_python.urdf import *
from calibration_manager.msg import *
from capture_executive.config_manager import *
from capture_executive.sensor_managers import *
from capture_executive.robot_measurement_cache import RobotMeasurementCache


class CaptureManagerServer:
    def __init__(self, config_dir, system, robot_desc):
        self.lock = threading.Lock()
        
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
        
        # set up publisher
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
        
    def goal_callback(self):
        goal = self._server.accept_new_goal()
        rospy.loginfo("[%s] Setting calibration configuration" % rospy.get_name())
        
        # fill in only valid cameras
        camera_measurements = goal.camera_measurements
        next_configuration["camera_measurements"] = []
        cam_ids = []
        for (i, cam) in enumerate(camera_measurements):
            if self.cam_config.has_key(cam.id):
                next_configuration["camera_measurements"].append({'cam_id': cam.id, 'config': cam.config})
                cam_ids.append(cam.id)
            else:
                rospy.logdebug("Not capturing measurement for camera: %s"%(cam.id))
                
        # fill in only valid lasers
        laser_measurements = goal.laser_measurements
        next_configuration["laser_measurements"] = []
        laser_ids = []
        for (i, laser) in enumerate(laser_measurements):
            if self.laser_config.has_key(laser.id):
                next_configuration["laser_measurements"].append({'laser_id': laser.id, 'config': laser.config})
                laser_ids.append(laser.id)
            else:
                rospy.logdebug("Not capturing measurement for laser: %s"%(laser.id))
                
        # fill in only valid chains
        joint_measurements = goal.joint_measurements
        next_configuration["joint_measurements"] = []
        chain_ids = []
        for (i, chain) in enumerate(joint_measurements):
            if self.chain_config.has_key(chain.id):
                next_configuration["joint_measurements"].append({'chain_id': chain.id, 'config': chain.config})
                chain_ids.append(chain.id)
            else:
                rospy.logdebug("Not capturing measurement for chain: %s"%(chain.id))
                
        # fill in valid joint_commands
        joint_commands = next_configuration.joint_commands
        next_configuration["joint_commands"] = []
        for(i, command) in enumerate(joint_commands):
            if self.controller_config.has_key(command.controller):
                next_configuration["joint_commands"].append({'controller': command.controller, 'segments': {'duration': command.segments.duration, 'positions': command.segments.positions}})
            else:
                rospy.debug("Not commanding controller: %s" % (command.controller))
                
        # Set up the pipeline
        self.config_manager.reconfigure(next_configuration)
        
        # Set up cache
        self.cache.reconfigure(cam_ids, chain_ids, laser_ids)
        
        # Set up the sensor managers
        for cam_id, cam_manager in self.cam_managers:
            if cam_id in cam_ids:
                enable_list.append(cam_id)
                cam_manager.enable()
            else:
                disable_list.append(cam_id)
                cam_manager.disable()

        for chain_id, chain_manager in self.chain_managers:
            if chain_id in chain_ids:
                enable_list.append(chain_id)
                chain_manager.enable()
            else:
                disable_list.append(chain_id)
                chain_manager.disable()

        for laser_id, laser_manager in self.laser_managers:
            if laser_id in laser_ids:
                enable_list.append(laser_id)
                laser_manager.enable()
            else:
                disable_list.append(laser_id)
                laser_manager.disable()
        
        self.m_robot = None
    
    def preempt_callback():
        self._server.set_preempted()
        self.lock.aquire()
        self.cache.clear()
        self.locl.release()        
        
    def request_callback(self, msg):
        # See if the interval is big enough to care
        if (msg.end - msg.start) < rospy.Duration(1,0):
            return
        
        if(self._server.is_active() and not self.m_robot):
            self.lock.aquire()
            m = self.cache.request_robot_measurement(msg.start, msg.end)
            if isinstance(m, basestring):
                self.message = m
            else:
                self.m_robot = m
                self._server.set_succeeded(self.m_robot)
                result.publish(self.m_robot)
            self.lock.release()
                
        
    def status_callback(self, msg):
        if(self._server.is_active() and not self.m_robot):
            self.lock.aquire()
            self.interval_status=msg
            self.lock.release()
            
    def add_cam_measurement(self, cam_id, msg):
        if (msg.success and 
            self._servier.is_active() and not self.m_robot):
            self.lock.aquire()
            self.cache.add_cam_measurement(cam_id, msg)
            self.lock.release()
            
    def add_chain_measurement(self, chain_id, msg):
        if(self._server.is_active() and not self.m_robot):
            self.lock.aquire()
            self.cache.add_chain_measurement(chain_id, msg)
            self.lock.release()
        
    def add_laser_measurement(self, laser_id, msg, interval_start, interval_end):
        if(self._server.is_active() and not self.m_robot):
            self.lock.aquire
            self.cache.add_laser_measurement(laser_id, msg, interval_start, interval_end)
            self.lock.release()
            
if __name__=='__main__':
    rospy.init_node("capture_manager")
    rospy.loginfo("[%s] Starting..." % rospy.get_name())
    
    config_dir        = rospy.get_param("~config_dir", None)
    system            = rospy.get_param("~system", None)
    robot_description = rospy.get_param('robot_description', None)
    
    from optparse import OptionParser
    parser = OptionParser("%prog", description=None)
    parser.add_option("-c", "--config", dest= "config_dir", type="string", default=config_dir, help="Directory containing configuration files")
    parser.add_option("-s", "--system", dest= "system", type="string", default=system, help="System file")
    parser.add_option("-r", "--robot_desc", dest = "robot_description", type="string", default=robot_description, help="The urdf of the robot")
    
    options, args = parser.parse_args()
    
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
    
    rospy.loginfo("[%s] Settings: " % rospy.get_name() +
              "\n config_dir  = " + str(config_dir) +
              "\n system  = " + str(system) )
            
    CaptureManagerServer(config_dir, system, robot_description)
    
    rospy.spin()
