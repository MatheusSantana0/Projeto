import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matheus/Documents/Projeto/ros2_ws/install/go2_main'
