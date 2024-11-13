import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/giselly/ros2_ws/src/safety_detectors/install/safety_detectors'
