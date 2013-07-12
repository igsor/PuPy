
from controller import Robot
import PuPy
import os.path

# gait params
gait_params = {
    'bound_1Hz': {
            'frequency' : (1.0, 1.0, 1.0, 1.0),
            'offset'    : ( -0.23, -0.23, -0.37, -0.37),
            'amplitude' : ( 0.56, 0.56, 0.65, 0.65),
            'phase'     : (0.0, 0.0, 0.5, 0.5)
        },
    'bound_left_1Hz': {
            'frequency' : (1.0, 1.0, 1.0, 1.0),
            'offset'    : ( -0.23, -0.23, -0.37,-0.37),
            'amplitude' : ( 0.38, 0.65, 0.47, 0.65),
            'phase'     : (0.1141, 0.0, 0.611155, 0.5)
        },
    'bound_right_1Hz': {
            'frequency' : (1.0, 1.0, 1.0, 1.0),
            'offset'    : ( -0.23, -0.23, -0.37, -0.37),
            'amplitude' : ( 0.65, 0.38, 0.65, 0.47),
            'phase'     : (0.0, 0.1141, 0.5, 0.611155)
        }
}

# gaits
gaits = [PuPy.Gait(gait_params[g], name=g) for g in gait_params]

# actor
#actor = PuPy.RandomGaitControl(gaits)
actor = PuPy.ConstantGaitControl(gaits[2])
observer = PuPy.RobotCollector(actor, expfile='/tmp/puppy_sim.hdf5')

# robot
r = PuPy.robotBuilder(Robot, observer, sampling_period_ms=20, noise_ctrl=None, noise_obs=None)

# run
r.run()
