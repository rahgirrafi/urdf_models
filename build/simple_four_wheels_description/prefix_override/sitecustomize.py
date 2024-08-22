import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rafi/ros2/urdf_ws/install/simple_four_wheels_description'
