import yaml
from sensor_msgs.msg import CameraInfo

def writeCalibration(file_name, camera_name, cam_info):
    data = dict(
        image_width = cam_info.width,
        image_height = cam_info.height,
        camera_name = camera_name,
        camera_matrix = dict (
            rows = 3,
            cols = 3,
            data = [cam_info.K]
            ),
        distortion_model = cam_info.distortion_model,
        distortion_coefficients = dict (
            rows = 1,
            cols = len(cam_info.D),
            data = cam_info.D,
            ),
        rectification_matrix = dict (
            rows = 3,
            cols = 3,
            data = cam_info.R
            ),
        projection_matrix = dict (
            rows = 3,
            cols = 4,
            data = cam_info.P
            )
        )
    
    with open(file_name, 'w') as outfile:
        outfile.write( yaml.dump(data, default_flow_style=False))
        outfile.close()
