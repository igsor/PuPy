
from controller import Supervisor
import PuPy

# emitter checks
class EmitterCheck(PuPy.SupervisorCheck):
    def __call__(self, supervisor):
        supervisor.emitter.send('Emitting ' + str(supervisor.numIter))

checks = []
checks.append(EmitterCheck())

# set up supervisor
PuPy.supervisorBuilder(Supervisor, 100, checks).run()
