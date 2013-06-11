
from controller import Supervisor
import PuPy

# checks
checks = []
checks.append(PuPy.RevertTumbled(grace_time_ms=2000))
checks.append(PuPy.RevertMaxIter(max_duration_ms=20*50*2*20))

# set up supervisor
s = PuPy.supervisorBuilder(Supervisor, 20, checks)

# run
s.run()
