
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

# Multidimensional collector
class MyCollector(PuPy.PuppyCollector):
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        if len(epoch) > 0:
            epoch['random'] = np.random.normal(size=(100,10))
        return super(MyCollector, self).__call__(epoch, time_start_ms, time_end_ms, step_size)


# actor
actor = PuPy.ConstantGaitControl(gait)
observer = MyCollector(actor, expfile='/tmp/puppy_sim.hdf5')


# robot
r = PuPy.robotBuilder(Robot, observer, sampling_period_ms=20, noise_ctrl=None, noise_obs=None)

# run
r.run()
