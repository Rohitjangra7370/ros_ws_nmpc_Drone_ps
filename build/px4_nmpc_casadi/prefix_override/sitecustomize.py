import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/olympusforge/Drone_PS/px4_nmpc_ws/install/px4_nmpc_casadi'
