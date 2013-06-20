
import random
from math import sin,pi
import time

class Gait(object):
    """Motor target generator, using predefined gait_switcher.
    
    The motor signal follows the parametrised sine
    
        :math:`A  \sin(2 \pi f x + p) + B`
    
    with the parameters A, f, p, B
    
    ``params``
        :py:meth:`dict` holding the parameters:
        
         keys:   amplitude, frequency, phase, offset
         
         values: 4-tuples holding the parameter values. The order is
                 front-left, front-right, rear-left, rear-right
    
    """
    def __init__(self, params, name=None):
        self.params = params
        if name is None: name = 'Unknown gait'
        self.name = name
    def __iter__(self):
        return self.iter(0, 20)
    def iter(self, time_start_ms, step):
        """Return the motor target sequence in the interval [*time_start_ms*, *time_end_ms*]."""
        params = zip(self.params['amplitude'], self.params['frequency'], self.params['phase'], self.params['offset'])
        current_time_ms = time_start_ms
        while True:
            current_time_ms += step
            yield [A * sin(2.0 * pi * (freq * current_time_ms / 1e3 - phase)) + offset for A, freq, phase, offset in params]
    def __str__(self):
        return self.name

class PuppyActor(object):
    """Template class for an actor, used in :py:class:`WebotsPuppyMixin`.
    
    The actor is called after every control period, when a new sequence
    of motor targets is required. It is expected to return an iterator
    which in every step produces a 4-tuple, representing the targets
    of the motors.
    The order is front-left, front-right, rear-left, rear-right.
    
    ``epoch``
        The sensor measurements in the previous control period. They
        are returned as dict, with the sensor name as key and a
        numpy array of observations as value.
        
        Note that the motor targets are one-step ahead in the sense that
        they are applied but have not yet been executed.
        
        .. versionchanged:: 1365
            There's some discussion about this point. After commit
            1365 (10eb3eed-6697-4d8c-9aac-32ebf1d36239), the behaviour
            is different: The sensor readings are the product of
            applying the target. May again be changed later (after more
            discussion).
        
        
        Further, note that the :py:meth:`dict` may be empty (this is
        guaranteed at least once in the simulator initialization).
    
    ``time_start_ms``
        The (simulated) time from which on the motor target will be
        applied. *time_start_ms* is weakly positive and strictly
        monotonic increasing (meaning that it is zero only in the very
        first call).
    
    ``time_end_ms``
        The (simulated) time up to which the motor target must at least
        be defined.
    
    ``step_size_ms``
        The motor period, i.e. the number of milliseconds pass until
        the next motor target is applied.
    
    If the targets are represented by a list, it must at least have
    
        .. :math:`\\frac{ \\text{time_end_ms} - \\text{time_start_ms} }{ \\text{step_size_ms} }`
    
        (*time_end_ms* - *time_start_ms*) / *step_size_ms*
    
    items and it has to be returned as iterator, as in
    
        >>> iter(myList)
    
    """
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        raise NotImplementedError()

class RandomGaitControl(PuppyActor):
    """From a list of available gaits, randomly select one."""
    def __init__(self, gaits):
        self.gaits = gaits[:]
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        gait = random.choice(self.gaits)
        print gait
        return gait.iter(time_start_ms, step_size)

class ConstantGaitControl(PuppyActor):
    """Given a gait, always apply it."""
    def __init__(self, gait):
        self.gait = gait
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        return self.gait.iter(time_start_ms, step_size)

class SequentialGaitControl(PuppyActor):
    """Execute a predefined sequence of gaits.
    
    Note that it's assumed that *gait_iter* does not terminate
    permaturely.
    """
    def __init__(self, gait_iter):
        self.gait_iter = gait_iter
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        gait = self.gait_iter.next()
        return gait.iter(time_start_ms, step_size)

class _PuppyCollector_h5py(PuppyActor):
    """Collect sensor readouts and store them in a file.
    HDF5 is written through the h5py module.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    ``actor``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``actor``
        is required. It must match the :py:class:`PuppyActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    
    """
    def __init__(self, actor, expfile, headers=None):
        # set actor
        self.actor = actor
        
        # create experiment storage
        import h5py
        self.fh = h5py.File(expfile,'a')
        name = str(len(self.fh.keys()))
        self.grp = self.fh.create_group(name)
        amngr = h5py.AttributeManager(self.grp)
        amngr.create('time', time.time())
        if headers is not None:
            for k in headers:
                amngr.create(k, headers[k])
        
        print "Using storage", name
    
    def __del__(self):
        self.fh.close()
    
    # if RevertTumbling is used:
    #  last epoch will not be written since it is not necessarily complete;
    #  grace time deals with this (> one epoch)
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        # write epoch to dataset
        for k in epoch:
            if k not in self.grp:
                #maxshape = tuple([None] * len(epoch[k].shape))
                #self.grp.create_dataset(k, shape=epoch[k].shape, data=epoch[k], chunks=True, maxshape=maxshape)
                self.grp.create_dataset(k, shape=epoch[k].shape, data=epoch[k], chunks=True, maxshape=None)
            else:
                N = epoch[k].shape[0]
                K = self.grp[k].shape[0]
                self.grp[k].resize(size=N+K, axis=0)
                self.grp[k][K:] = epoch[k]
        
        self.fh.flush()
        return self.actor(epoch, time_start_ms, time_end_ms, step_size)

class _PuppyCollector_pytables(PuppyActor):
    """Collect sensor readouts and store them in a file.
    HDF5 is written through the PyTables module.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    ``actor``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``actor``
        is required. It must match the :py:class:`PuppyActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    
    """
    def __init__(self, actor, expfile, headers=None):
        # set actor
        self.actor = actor
        
        # create experiment storage
        import tables
        self.fh = tables.File(expfile,'a')
        name = 'exp' + str(len(self.fh.root._v_groups))
        self.grp = self.fh.create_group(self.fh.root, name)
        
        self.grp._f_setattr('time', time.time())
        if headers is not None:
            for k in headers: self.grp._f_setattr(k, headers[k])
        
        print "Using storage", name
    
    def __del__(self):
        self.fh.close()
    
    # if RevertTumbling is used:
    #  last epoch will not be written since it is not necessarily complete;
    #  grace time deals with this (> one epoch)
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size):
        # write epoch to dataset
        for k in epoch:
            if k not in self.grp:
                self.fh.create_earray(self.grp, k, chunkshape=epoch[k].shape, obj=epoch[k])
            else:
                self.grp._v_children[k].append(epoch[k])
        
        self.fh.flush()
        return self.actor(epoch, time_start_ms, time_end_ms, step_size)

class PuppyCollector(_PuppyCollector_h5py):
    """Collect sensor readouts and store them in a file.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    .. note::
        This class abstracts interface from implementation. Internally,
        either the HDF5 interface from [PyTables]_ or [h5py]_ may be
        used.
    
    ``actor``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``actor``
        is required. It must match the :py:class:`PuppyActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    """
    pass
