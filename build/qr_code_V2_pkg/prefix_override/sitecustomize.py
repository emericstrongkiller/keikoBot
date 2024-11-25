import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oriane/vrx_ws/src/Aquabot-Competitor/keikoBot/install'
