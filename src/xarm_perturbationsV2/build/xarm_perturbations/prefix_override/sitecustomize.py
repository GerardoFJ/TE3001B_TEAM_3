import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/TE3001B_TEAM_3/src/xarm_perturbationsV2/install/xarm_perturbations'
