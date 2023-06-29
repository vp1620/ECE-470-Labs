## *********************************************************
##
## File autogenerated for the image_proc package
## by the dynamic_reconfigure package.
## Please do not edit.
##
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'name': 'Default', 'type': '', 'state': True, 'cstate': 'true', 'id': 0, 'parent': 0, 'parameters': [{'name': 'debayer', 'type': 'int', 'default': 0, 'level': 0, 'description': 'Debayering algorithm', 'min': 0, 'max': 3, 'srcline': 291, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'edit_method': "{'enum': [{'name': 'Bilinear', 'type': 'int', 'value': 0, 'srcline': 9, 'srcfile': '/home/ur3/catkin_alaynam3/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Debayer.cfg', 'description': 'Fast algorithm using bilinear interpolation', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'EdgeAware', 'type': 'int', 'value': 1, 'srcline': 11, 'srcfile': '/home/ur3/catkin_alaynam3/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Debayer.cfg', 'description': 'Edge-aware algorithm', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'EdgeAwareWeighted', 'type': 'int', 'value': 2, 'srcline': 13, 'srcfile': '/home/ur3/catkin_alaynam3/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Debayer.cfg', 'description': 'Weighted edge-aware algorithm', 'ctype': 'int', 'cconsttype': 'const int'}, {'name': 'VNG', 'type': 'int', 'value': 3, 'srcline': 15, 'srcfile': '/home/ur3/catkin_alaynam3/src/lab2andDriver/drivers/camera_calibration/image_pipeline/image_proc/cfg/Debayer.cfg', 'description': 'Slow but high quality Variable Number of Gradients algorithm', 'ctype': 'int', 'cconsttype': 'const int'}], 'enum_description': 'Debayering algorithm'}", 'ctype': 'int', 'cconsttype': 'const int'}], 'groups': [], 'srcline': 246, 'srcfile': '/opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure/parameter_generator_catkin.py', 'class': 'DEFAULT', 'parentclass': '', 'parentname': 'Default', 'field': 'default', 'upper': 'DEFAULT', 'lower': 'groups'}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

Debayer_Bilinear = 0
Debayer_EdgeAware = 1
Debayer_EdgeAwareWeighted = 2
Debayer_VNG = 3
