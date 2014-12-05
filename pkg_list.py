import os

def get_build_deps(path):
    return []

def get_pkg(path):
    d               = {}
    d['DEBNAME']    = 'calibration'
    prefix          = os.path.join('opt/nasa/r2/stacks', d['DEBNAME'].replace('-', '_'))
    d['DEBMAINT']   = 'Allison Thackston [allison.thackston@nasa.gov]'
    d['CHOWN']      = [['/{}'.format(prefix), 'vanguard:ros']]
    d['CHMODLIST']  = [['/{}'.format(prefix), 'a-w']]
    d['DEBDEPENDS'] = ('ros-fuerte-ros,  ros-fuerte-ros-comm, ros-fuerte-std-msgs, ros-fuerte-common, ros-fuerte-common-msgs, ros-fuerte-geometry, ros-fuerte-image-common, ros-fuerte-image-pipeline, ros-fuerte-perception-pcl, ros-fuerte-vision-opencv, ros-fuerte-actionlib, ros-fuerte-pluginlib')
    d['DEBDESC']    = 'Stack containing the Calibration.'

    def group(srcpre, dstpre, srcs):
        return [(os.path.join(srcpre, s), os.path.join(dstpre, s)) for s in srcs]

    def makeGroup(src, dest):
        return group(src, dest, [f for f in os.listdir(src)])

    def makeGroupSuffix(src, dest, suffix):
        return group(src, dest, [f for f in os.listdir(src) if f.endswith(suffix)])

    def addLibraryPackage(prefix, packageName):
        path = os.path.join(prefix, packageName)
        packageFiles = [(os.path.join(packageName, 'manifest.xml'),                        os.path.join(path, 'manifest.xml'))]
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'lib'),                  os.path.join(path, 'lib'),                  '.so')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'include', packageName), os.path.join(path, 'include', packageName), '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),                  os.path.join(path, 'src'),                  '.cpp')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),                  os.path.join(path, 'src'),                  '.c')
        packageFiles += [('ROS_NOBUILD',                                                   os.path.join(path, 'ROS_NOBUILD'))]
        return packageFiles

    def addRosSrvPackage(prefix, packageName):
        path = os.path.join(prefix, packageName)
        packageFiles = [(os.path.join(packageName, 'manifest.xml'),          os.path.join(path, 'manifest.xml'))]
        packageFiles += makeGroup(os.path.join(packageName, 'bin'),    os.path.join(path, 'bin'))
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'launch'), os.path.join(path, 'launch'), '.launch')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),    os.path.join(path, 'src'),    '.cpp')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),    os.path.join(path, 'src'),    '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'srv_gen/cpp/include', packageName),   os.path.join(path, 'srv_gen/cpp/include', packageName),   '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'srv'),            os.path.join(path, 'src', packageName, 'srv'),            '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'srv'),                                os.path.join(path, 'srv'),                                '.srv')
        packageFiles += [('ROS_NOBUILD',                                     os.path.join(path, 'ROS_NOBUILD'))]
        return packageFiles

    def addRosMsgPackage(prefix, packageName):
        path = os.path.join(prefix, packageName)
        packageFiles = [(os.path.join(packageName, 'manifest.xml'),          os.path.join(path, 'manifest.xml'))]
        packageFiles += makeGroup(os.path.join(packageName, 'bin'),    os.path.join(path, 'bin'))
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),    os.path.join(path, 'src'),    '.cpp')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src'),    os.path.join(path, 'src'),    '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'lib'),                                os.path.join(path, 'lib'),                                '.so')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'msg_gen/cpp/include', packageName),   os.path.join(path, 'msg_gen/cpp/include', packageName),   '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'msg'),            os.path.join(path, 'src', packageName, 'msg'),            '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'msg'),                                os.path.join(path, 'msg'),                                '.msg')        
        packageFiles += [('ROS_NOBUILD',                                     os.path.join(path, 'ROS_NOBUILD'))]
        return packageFiles

    def addMsgsPackage(prefix, packageName):
        path = os.path.join(prefix, packageName)
        packageFiles = [(os.path.join(packageName, 'manifest.xml'),                                os.path.join(path, 'manifest.xml'))]
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'msg_gen/cpp/include', packageName),   os.path.join(path, 'msg_gen/cpp/include', packageName),   '.h')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'msg'),            os.path.join(path, 'src', packageName, 'msg'),            '.py')
        packageFiles += makeGroupSuffix(os.path.join(packageName, 'msg'),                                os.path.join(path, 'msg'),                                '.msg')
        
        packageFiles += [('ROS_NOBUILD',                                                           os.path.join(path, 'ROS_NOBUILD'))]
        return packageFiles

    # stack
    d['DEBFILES'] = [('stack.xml', os.path.join(prefix, 'stack.xml'))]

    with open('ROS_NOBUILD', 'w') as f:
        f.write('0')

    # packages
    packageName = 'calibration_capture'
    path = os.path.join(prefix, packageName)
    d['DEBFILES'] += [(os.path.join(packageName, 'manifest.xml'),                                     os.path.join(path, 'manifest.xml'))]
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'launch'),                             os.path.join(path, 'launch'), '.launch')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg_gen/cpp/include', packageName),   os.path.join(path, 'msg_gen/cpp/include', packageName),   '.h')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'msg'),            os.path.join(path, 'src', packageName, 'msg'),            '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg'),                                os.path.join(path, 'msg'),                                '.msg')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', 'capture_executive'),           os.path.join(path, 'src', 'capture_executive'),           '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', 'urdf_python'),                 os.path.join(path, 'src', 'urdf_python'),                 '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src'),                                os.path.join(path, 'src'),                                '.py')
    d['DEBFILES'] += [('ROS_NOBUILD',                                                      os.path.join(path, 'ROS_NOBUILD'))]

    packageName = 'calibration_estimation'
    path = os.path.join(prefix, packageName)
    d['DEBFILES'] += [(os.path.join(packageName, 'manifest.xml'),                                     os.path.join(path, 'manifest.xml'))]
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src'),                                os.path.join(path, 'src'),                                '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'sensors'),        os.path.join(path, 'src', packageName, 'sensors'),        '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', 'urdf_python'),                 os.path.join(path, 'src', 'urdf_python'),                 '.py')
    d['DEBFILES'] += [('ROS_NOBUILD',                                                      os.path.join(path, 'ROS_NOBUILD'))]    

    packageName = 'calibration_launch'
    path = os.path.join(prefix, packageName)
    d['DEBFILES'] += [(os.path.join(packageName, 'manifest.xml'),                                     os.path.join(path, 'manifest.xml'))]
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'capture_data'),                       os.path.join(path, 'capture_data'),                       '.launch')
    d['DEBFILES'] += [('ROS_NOBUILD',                                                      os.path.join(path, 'ROS_NOBUILD'))]    

    d['DEBFILES'] += addMsgsPackage(prefix, 'calibration_msgs')

    packageName = 'camera_intrinsic_calibrator'
    path = os.path.join(prefix, packageName)
    d['DEBFILES'] += [(os.path.join(packageName, 'manifest.xml'),                                     os.path.join(path, 'manifest.xml'))]
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'launch'),                             os.path.join(path, 'launch'), '.launch')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg'),                                os.path.join(path, 'msg'),                                '.msg')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg_gen/cpp/include', packageName),   os.path.join(path, 'msg_gen/cpp/include', packageName),   '.h')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'msg'),            os.path.join(path, 'src', packageName, 'msg'),            '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', 'calibrator'),                  os.path.join(path, 'src', 'calibrator'),                  '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src'),                                os.path.join(path, 'src'),                                '.py')
    d['DEBFILES'] += [('ROS_NOBUILD',                                                      os.path.join(path, 'ROS_NOBUILD'))]

    d['DEBFILES'] += addRosSrvPackage(prefix, 'dynamic_transform_publisher')

    packageName = 'image_cb_detector'
    path = os.path.join(prefix, packageName)
    d['DEBFILES'] += [(os.path.join(packageName, 'manifest.xml'),                                     os.path.join(path, 'manifest.xml'))]
    d['DEBFILES'] += [(os.path.join(packageName, 'calibration_targets.xml'),                   os.path.join(path, 'calibration_targets.xml'))]
    d['DEBFILES'] += makeGroup(os.path.join(packageName, 'bin'),                           os.path.join(path, 'bin'))
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'cfg'),                     os.path.join(path, 'cfg'),                      '.cfg')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'cfg'),                     os.path.join(path, 'cfg'),                      '.cfgc')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'cfg'), os.path.join(path, 'src', packageName, 'cfg'),  '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'include', packageName),    os.path.join(path, 'include', packageName),     '.h')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'launch'),                             os.path.join(path, 'launch'), '.launch')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'lib'),                                os.path.join(path, 'lib'),                                '.so')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg'),                                os.path.join(path, 'msg'),                                '.msg')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'msg_gen/cpp/include', packageName),   os.path.join(path, 'msg_gen/cpp/include', packageName),   '.h')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName),                   os.path.join(path, 'src', packageName),                   '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', packageName, 'msg'),            os.path.join(path, 'src', packageName, 'msg'),            '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src', 'camera_calibrator'),           os.path.join(path, 'src', 'camera_calibrator'),           '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src'),                                os.path.join(path, 'src'),                                '.py')
    d['DEBFILES'] += makeGroupSuffix(os.path.join(packageName, 'src'),                                os.path.join(path, 'src'),                                '.cpp')
    d['DEBFILES'] += [('ROS_NOBUILD',                                                      os.path.join(path, 'ROS_NOBUILD'))]

    d['DEBFILES'] += addRosMsgPackage(prefix,  'interval_intersection')
    d['DEBFILES'] += addRosMsgPackage(prefix,  'joint_states_settler')
    d['DEBFILES'] += addRosMsgPackage(prefix,  'laser_cb_detector')
    d['DEBFILES'] += addRosMsgPackage(prefix,  'monocam_settler')
    d['DEBFILES'] += addLibraryPackage(prefix,  'settlerlib')

    return d
