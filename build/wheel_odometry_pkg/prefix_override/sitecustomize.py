import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pola-nasser13/wheel_odometry_ws/install/wheel_odometry_pkg'
