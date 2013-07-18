
from controller import Robot
import PuPy
import os.path
import numpy as np

# gait params
gait_params = {
    'frequency' : (1.0, 1.0, 1.0, 1.0),
    'offset'    : ( -0.23, -0.23, -0.37, -0.37),
    'amplitude' : ( 0.56, 0.56, 0.65, 0.65),
    'phase'     : (0.0, 0.0, 0.5, 0.5)
}

# gaits
gait = PuPy.Gait(gait_params)

# receiver callback
def recv1(robot, epoch, current_time, msg):
    print "recv1", msg

def recv2(robot, epoch, current_time, msg):
    print "recv2", msg

# actor
actor = PuPy.ConstantGaitControl(gait)


# robot
r = PuPy.robotBuilder(Robot, actor, event_period_ms=50, event_handlers=[recv1, recv2])
r.run()


