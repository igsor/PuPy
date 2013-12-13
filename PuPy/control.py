"""
"""
import random
from math import sin, pi
import time
import numpy as np
import json
import PuPy
import os
import warnings

def load_gaits(filename=PuPy.__file__[:-17]+'data'+os.sep+'puppy_gaits.json', names=None):
    """
    Returns the gaits stored in the given json file.
    
    ``filename``
        string containing the name of the json file where the gaits are stored
        (default is PuPy/data/puppy_gaits.json).
    
    ``names``
         If the gait names are given as a list of strings, only the desired gaits are returned as a list.
         Otherwise a dictionary containing all gaits are returned. 
    """
    gaits = json.load(open(filename, 'r'))
    if names is not None:
        gaits = [Gait(gaits[name], name) for name in names]
    else:
        gaits = dict([(name, Gait(gaits[name], name)) for name in gaits])          
    return gaits

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
        if name is None:
            name = 'Unknown gait'
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
    
    def copy(self):
        return Gait(self.params.copy(), self.name)
    
    def __str__(self):
        return self.name


class NoneChild(object):
    """Dummy class for actor's child."""
    def __call__(self, *args, **kwargs):
        return None
    def _get_initial_targets(self, *args, **kwargs):
        return None

class RobotActor(object):
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
        
        Note that the motor targets are the ones that have been applied,
        i.e. those that lead up to the sensor measurements. Imagine this
        cycle::
        
            trg[i] -> move robot -> sensors[i] -> trg[i+1] -> ...
        
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
    
    ``child``
        Every :py:class:`RobotActor` has a ``child`` member, which can be
        another :py:class:`RobotActor`. The ``child`` is called in the end
        of the __call__() method,
        e.g. return self.child(epoch, time_start_ms, time_end_ms, step_size_ms).
            
    """
    def __init__(self, child=None):
        self.child = child
        if self.child is None:
            self.child = NoneChild()
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        raise NotImplementedError()
    
    def get_from_child(self, name):
        """return member ``name`` from first child class in the hierarchy that has this member."""
        if hasattr(self.child, name):
            return getattr(self.child, name)
        elif hasattr(self.child, 'get_from_child'):
            return self.child.get_from_child(name)
        else:
            return None
            
    def signal(self, event):
        """send signal ``event`` to all child classes."""
        if hasattr(self.child, 'signal'):
            self.child.signal(event)
        self._signal(event)
    
    def _signal(self, event):
        """Template method for subclasses. Overload this method to enable signal receiving."""
        pass
    
    def _get_initial_targets(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        """Template method for subclasses. Use this to return a motor targets-iterator at time 0."""
        return self.child._get_initial_targets(epoch, time_start_ms, time_end_ms, step_size_ms)
        

class PuppyActor(RobotActor):
    """Deprecated alias for :py:class:`RobotActor`."""
    pass

class RandomGaitControl(RobotActor):
    """From a list of available gaits, randomly select one."""
    def __init__(self, gaits):
        super(RandomGaitControl, self).__init__()
        self.gaits = gaits[:]
        self.gait = None
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        self.gait = random.choice(self.gaits)
        print self.gait
        return self.gait.iter(time_start_ms, step_size_ms)
    
    def _get_initial_targets(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        return self.__call__(epoch, time_start_ms, time_end_ms, step_size_ms)

class ConstantGaitControl(RobotActor):
    """Given a gait, always apply it."""
    def __init__(self, gait):
        super(ConstantGaitControl, self).__init__()
        self.gait = gait
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        return self.gait.iter(time_start_ms, step_size_ms)
    
    def _get_initial_targets(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        return self.__call__(epoch, time_start_ms, time_end_ms, step_size_ms)

class SequentialGaitControl(RobotActor):
    """Execute a predefined sequence of gaits.
    
    Note that it's assumed that *gait_iter* does not terminate
    permaturely.
    """
    def __init__(self, gait_iter):
        super(SequentialGaitControl, self).__init__()
        self.gait_iter = gait_iter
        self.gait = None
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        self.gait = self.gait_iter.next()
        return self.gait.iter(time_start_ms, step_size_ms)
    
    def _get_initial_targets(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        return self.__call__(epoch, time_start_ms, time_end_ms, step_size_ms)

class _RobotCollector_h5py(RobotActor):
    """Collect sensor readouts and store them in a file.
    HDF5 is written through the h5py module.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    ``child``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``child``-actor
        is required. It must match the :py:class:`RobotActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    
    """
    def __init__(self, child, expfile, headers=None, vars=None):
        super(_RobotCollector_h5py, self).__init__(child)
        
        self.headers = headers
        self.vars = vars
        
        # create experiment storage
        import h5py
        self.fh = h5py.File(expfile,'a')
        self._create_group(str(len(self.fh.keys())))
        
    def _create_group(self, grp_name):
        self.grp_name = grp_name
        self.grp = self.fh.create_group(self.grp_name)
        self.set_header('time', time.ctime())
        if self.headers is not None:
            for k in self.headers:
                self.set_header(k, self.headers[k])
        print "Using storage", self.grp_name
    
    def set_header(self, name, data):
        """Add custom header ``data`` to the current group. The data is
        stored under key ``name``. If the key is already in use, the
        value will be overwritten.
        """
        import h5py
        amngr = h5py.AttributeManager(self.grp)
        if name in amngr:
            del amngr[name]
        amngr.create(name, data)
    
    def __del__(self):
        self.fh.close()
    
    # if RevertTumbling is used:
    #  last epoch will not be written since it is not necessarily complete;
    #  grace time deals with this (> one epoch)
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        # write epoch to dataset
        keys = epoch.keys()
        if self.vars is None:
            vars = keys
        else:
            vars = self.vars
        for k in vars:
            if k not in keys:
                warnings.warn('logging of %s requested but not present in epoch. skipping...'%k)
                continue
            if k not in self.grp:
                maxshape = tuple([None] * len(epoch[k].shape))
                self.grp.create_dataset(k, data=epoch[k], chunks=True, maxshape=maxshape)
            else:
                N = epoch[k].shape[0]
                K = self.grp[k].shape[0]
                self.grp[k].resize(size=N+K, axis=0)
                self.grp[k][K:] = epoch[k]
        
        self.fh.flush()
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)
    
    def _signal(self, event):
        if isinstance(event, str) and event=='new_episode':
            # start a new episode, i.e. create a new group in expfile
            # and store all new epochs there:
            self._create_group(str(int(self.grp_name)+1))

class _RobotCollector_pytables(RobotActor):
    """Collect sensor readouts and store them in a file.
    HDF5 is written through the PyTables module.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    ``child``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``child``-actor
        is required. It must match the :py:class:`RobotActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    
    """
    def __init__(self, child, expfile, headers=None):
        super(_RobotCollector_pytables, self).__init__(child)
        
        # create experiment storage
        import tables
        self.fh = tables.File(expfile,'a')
        name = 'exp' + str(len(self.fh.root._v_groups))
        self.grp = self.fh.create_group(self.fh.root, name)
        
        self.grp._f_setattr('time', time.time())
        if headers is not None:
            for k in headers:
                self.grp._f_setattr(k, headers[k])
        
        print "Using storage", name
    
    def __del__(self):
        self.fh.close()
    
    def set_header(self, name, data):
        """Add custom header ``data`` to the current group. The data is
        stored under key ``name``. If the key is already in use, the
        value will be overwritten.
        """
        self.grp._f_setattr(name, data)
    
    # if RevertTumbling is used:
    #  last epoch will not be written since it is not necessarily complete;
    #  grace time deals with this (> one epoch)
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        # write epoch to dataset
        for k in epoch:
            if k not in self.grp:
                self.fh.create_earray(self.grp, k, chunkshape=epoch[k].shape, obj=epoch[k])
            else:
                self.grp._v_children[k].append(epoch[k])
        
        self.fh.flush()
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)
    
    def _signal(self, event):
        if isinstance(event, str) and event=='new_episode':
            # start a new episode, i.e. create a new group in expfile
            # and store all new epochs there:
            warnings.warn("starting new episode not yet implemented in ``_RobotCollector_pytables``.")

class RobotCollector(_RobotCollector_h5py):
    """Collect sensor readouts and store them in a file.
    
    The data is stored in the [HDF5]_ format. For each simulation run,
    there's a group, identified by a running number. Within each group,
    the sensor data is stored in exclusive datasets, placed under the
    sensor's name.
    
    .. note::
        This class abstracts interface from implementation. Internally,
        either the HDF5 interface from [PyTables]_ or [h5py]_ may be
        used.
    
    ``child``
        The :py:class:`PuppyCollector` works as intermediate actor, it
        does not implement a policy itself. For this, another ``child``-actor
        is required. It must match the :py:class:`RobotActor` interface.
    
    ``expfile``
        Path to the file into which the experimental data should be
        stored.
    
    ``headers``
        Additional headers, stored with the current experiment.
        A *dict* is expected. Default is None (no headers).
    """
    pass

class PuppyCollector(RobotCollector):
    """Deprecated alias for :py:class:`RobotCollector`."""
    pass


class GaitNameCollector(RobotActor):
    """A collector that records the name of the current gait."""
    def __init__(self, child, sampling_period_ms, ctrl_period_ms, gait_names=None, **kwargs):
        super(GaitNameCollector, self).__init__(child, **kwargs)
        self.sampling_period_ms = sampling_period_ms
        self.ctrl_period_ms = ctrl_period_ms
        if gait_names is not None:
            self.max_name_len = max([len(gait) for gait in gait_names])
        else:
            self.max_name_len = 20
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        if time_start_ms:
            gait_name = self.get_from_child('gait').name
            epoch['gait'] = np.repeat(np.array(gait_name, dtype='|S'+str(self.max_name_len)), self.ctrl_period_ms/self.sampling_period_ms)
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)


class GaitIndexCollector(RobotActor):
    """
    A collector that records the index of the current gait.
    The mapping between indices and names of the gaits is stored in a dictionary in the
    header of the log file (requires a collector with method 'set_header(name, data)').
    """
    def __init__(self, child, sampling_period_ms, ctrl_period_ms, gait_names=None, **kwargs):
        super(GaitIndexCollector, self).__init__(child, **kwargs)
        self.sampling_period_ms = sampling_period_ms
        self.ctrl_period_ms = ctrl_period_ms
        if gait_names is None:
            self.gait_names = []
        else:
            self.gait_names = gait_names
            self._set_header(self.gait_names)
            
    def _set_header(self, gait_names):
        set_header = self.get_from_child('set_header')
        if set_header is not None:
            set_header('gait_names', np.array(gait_names))
        
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        if time_start_ms:
            gait_name = self.get_from_child('gait').name
            if gait_name not in self.gait_names:
                self.gait_names.append(gait_name)
                self._set_header(self.gait_names)
            gait_idx = np.nonzero(np.array(self.gait_names)==gait_name)[0][0]    
            epoch['gait_idx'] = np.repeat([gait_idx], self.ctrl_period_ms/self.sampling_period_ms)
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)


class GaitParametersCollector(RobotActor):
    """A collector that records the motor parameters."""
    def __init__(self, child, sampling_period_ms, ctrl_period_ms, **kwargs):
        super(GaitParametersCollector, self).__init__(child, **kwargs)
        self.sampling_period_ms = sampling_period_ms
        self.ctrl_period_ms = ctrl_period_ms
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        if time_start_ms:
            current_params = self.get_from_child('gait').params
            epoch['frequency_FL'] = np.repeat([current_params['frequency'][0]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['frequency_FR'] = np.repeat([current_params['frequency'][1]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['frequency_HL'] = np.repeat([current_params['frequency'][2]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['frequency_HR'] = np.repeat([current_params['frequency'][3]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['offset_FL'] = np.repeat([current_params['offset'][0]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['offset_FR'] = np.repeat([current_params['offset'][1]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['offset_HL'] = np.repeat([current_params['offset'][2]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['offset_HR'] = np.repeat([current_params['offset'][3]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['amplitude_FL'] = np.repeat([current_params['amplitude'][0]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['amplitude_FR'] = np.repeat([current_params['amplitude'][1]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['amplitude_HL'] = np.repeat([current_params['amplitude'][2]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['amplitude_HR'] = np.repeat([current_params['amplitude'][3]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['phase_FL'] = np.repeat([current_params['phase'][0]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['phase_FR'] = np.repeat([current_params['phase'][1]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['phase_HL'] = np.repeat([current_params['phase'][2]], self.ctrl_period_ms/self.sampling_period_ms)
            epoch['phase_HR'] = np.repeat([current_params['phase'][3]], self.ctrl_period_ms/self.sampling_period_ms)
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)

class TumbleCollector(RobotActor):
    """A collector that records when Puppy tumbles."""
    def __init__(self, child, sampling_period_ms, ctrl_period_ms, **kwargs):
        super(TumbleCollector, self).__init__(child, **kwargs)
        self.sampling_period_ms = sampling_period_ms
        self.ctrl_period_ms = ctrl_period_ms
        self._tumbled = np.zeros([self.ctrl_period_ms/self.sampling_period_ms,])
        self.event_handler = self._get_event_handler()
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        if time_start_ms:
            epoch['tumble'] = self._tumbled
            self._tumbled = np.zeros([self.ctrl_period_ms/self.sampling_period_ms,])
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)
    
    def _get_event_handler(self):
        def func(robot, epoch, current_time, msg):
            if msg=='tumbled':
                self._tumbled[current_time % (self.ctrl_period_ms/self.sampling_period_ms)] = 1
        return func

class ResetCollector(RobotActor):
    """A collector that records when Puppy was reset (respawned)."""
    def __init__(self, child, sampling_period_ms, ctrl_period_ms, **kwargs):
        super(ResetCollector, self).__init__(child, **kwargs)
        self.sampling_period_ms = sampling_period_ms
        self.ctrl_period_ms = ctrl_period_ms
        self._reset = np.zeros([self.ctrl_period_ms/self.sampling_period_ms,])
        self.event_handler = self._get_event_handler()
    
    def __call__(self, epoch, time_start_ms, time_end_ms, step_size_ms):
        if time_start_ms:
            epoch['reset'] = self._reset
            self._reset = np.zeros([self.ctrl_period_ms/self.sampling_period_ms,])
        return self.child(epoch, time_start_ms, time_end_ms, step_size_ms)
    
    def _get_event_handler(self):
        def func(robot, epoch, current_time, msg):
            if msg=='reset':
                self._reset[current_time % (self.ctrl_period_ms/self.sampling_period_ms)] = 1
        return func
