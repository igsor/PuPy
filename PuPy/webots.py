
from fractions import gcd
import operator
import Queue
import warnings
import numpy as np
import sys

# todo: doctest
# todo: observer callback? other callbacks?
# todo: supervisor revert name (string)
# todo: action name might be interesting (e.g. when observing gaits)

class WebotsPuppyMixin(object):
    """Webots Puppy controller. It samples all sensors and periodically
    consults an ``actor`` for control decisions.
    
    ``actor``
        A function which determines the motor targets for the next
        control period. See *PuppyActor* for specifics.
        
        The function must return an interator which is valid for at
        least *ctrl_period_ms* / *motor_period_ms* steps. In each step,
        it must return a list of four motor targets.
        
        The actor's interface is defined by :py:class:`PuppyActor`. Note
        however, that the interface is organized such that the class
        structure may be obsolete.
        
    ``sampling_period_ms``
        The period according to which sensors are sampled.
        In milliseconds.
    
    ``motor_period_ms``
        The period according to which motor targets are set. In
        milliseconds. Usually, the same as *sampling_period_ms*
        (the default). If not, it's advised that it's a multiple of the
        sampling period, otherwise the observations per control decision
        may become funny.
    
    ``ctrl_period_ms``
        The period of control actions. In milliseconds.
        Must be a larger than or equal to the motor period and, if
        larger, a multiple thereof.
    
    ``noise_ctrl``
        Additive zero-mean gaussian noise on the motor targets.
        Additional to whatever *webots* does. Either a scalar of 4-tuple
        is expected, which represents the noise variances for all of the
        motors or each individual one, respectively. Use None to discard
        the noise (default).
    
    ``noise_obs``
        Additive zero-mean gaussian noise on the motor targets.
        Additional to whatever *webots* does. Either a scalar of dict
        is expected, which represents the noise variances for all of the
        sensors or each individual one, respectively. In the latter
        approach, the dict keys have to correspond to the sensor name.
        Use None to discard the noise (default).
    
    ``event_period_ms``
        The period in milliseconds that is used for polling the
        receiver. Should optimally be a multiple of the control or
        sampling period or the Supervisor's sampling period.
    
    ``event_handler``
        Register a callback that is executed whenever a new message
        arrives at the receiver. The handler is executed for every
        message individually (even if there are several). It must only
        take one argument, the message.
    
    """
    def __init__(self, actor, sampling_period_ms=20, ctrl_period_ms=2000, motor_period_ms=None, noise_ctrl=None, noise_obs=None, event_period_ms=20, event_handler=None):
        # action
        self.actor = actor
        
        # init time constants
        self.sampling_period = sampling_period_ms
        self.ctrl_period = ctrl_period_ms
        if motor_period_ms is None: motor_period_ms = sampling_period_ms
        self.motor_period = motor_period_ms
        
        # check assumptions on periods
        assert self.ctrl_period >= self.motor_period
        assert self.ctrl_period % self.motor_period == 0
        if self.motor_period > self.sampling_period:
            if self.motor_period % self.sampling_period != 0:
                warnings.warn('The motor period is not a multiple of the sampling period')
        elif self.motor_period < self.sampling_period:
            if self.sampling_period % self.motor_period != 0:
                warnings.warn('The sampling period is not a multiple of the motor period')
        
        # init motors
        self.motors = [self.getServo(s) for s in self._s_hip]
        
        # init noise
        self.motor_noise = noise_ctrl
        self.observer_noise = noise_obs
        
        # init sensors
        self.getAccelerometer(self._s_accel).enable(self.sampling_period)
        self.getGyro(self._s_gyro).enable(self.sampling_period)
        self.getCompass(self._s_compass).enable(self.sampling_period)
        self.getGPS(self._s_gps).enable(self.sampling_period)
        for s in self._s_hip + self._s_knee:
            self.getServo(s).enablePosition(self.sampling_period)
        for t in self._s_touch:
            self.getTouchSensor(t).enable(self.sampling_period)
        
        # init sensor readout function
        self.readout = self.__readout_gen()
        
        # init supervisor receiver
        self.receiver = self.getReceiver('fromSupervisorReceiver')
        self.event_period = event_period_ms
        self.event_handler = event_handler
        self.receiver.enable(self.event_period)
    
    # Sensor names
    _s_accel = 'accelerometer'
    _s_gyro = 'gyro'
    _s_compass = 'compass'
    _s_gps = 'puppyGPS'
    
    _s_target = ('trg0', 'trg1', 'trg2', 'trg3')
    _s_hip = ('hip0', 'hip1', 'hip2', 'hip3')
    _s_knee = ('knee0', 'knee1', 'knee2', 'knee3')
    _s_touch = ('touch0', 'touch1', 'touch2', 'touch3')
    
    def __readout_gen(self):
        """Produce a sensor readout generator."""
        single_dim, three_dim = [], []
        for s in self._s_hip + self._s_knee:
            single_dim.append(self.getServo(s).getPosition)
        
        for t in self._s_touch:
            single_dim.append(self.getTouchSensor(t).getValue)
        
        three_dim.append(self.getAccelerometer(self._s_accel).getValues)
        three_dim.append(self.getGyro(self._s_gyro).getValues)
        three_dim.append(self.getCompass(self._s_compass).getValues)
        three_dim.append(self.getGPS(self._s_gps).getValues)
        
        while True:
            singles = map(lambda f:f(), single_dim)
            three = map(lambda (x,z,y):[x,y,z], map(lambda f:f(), three_dim))
            yield singles + reduce(operator.add, three)
    
    def readout_labels(self):
        """Return a list of sensor labels where the order is the same
        as in the sensor readouts."""
        f = lambda lbl: reduce(operator.add, map(lambda i:(i+'_x', i+'_y', i+'_z'), lbl))
        return \
            self._s_target + self._s_hip + self._s_knee + self._s_touch + \
            f((self._s_accel, self._s_gyro, self._s_compass, self._s_gps))
    
    def run(self):
        """Main controller loop. Runs infinitely unless aborted by
        webots.
        
        The controller operates on a sense, think, act cycle. In every
        step - according to the sampling period - the sensors are read
        out (sense). This information is passed to the control decision
        machine (think) and its result used for updating the motors
        (act).
        
        The ``actor`` is provided with all sensor readings, starting
        from the previous call up to the current one. The very last
        readout is equal to the current state of the robot.
        The motor targets (denoted by trg) are the ones that have been
        applied in the previous step. This can be interpreted as the
        target that caused the sensor readings. Also note, that the
        motor targets returned by the ``actor`` will be effective
        immediately, i.e. the first target will be set right after the
        ``actor`` was called.
        
        .. versionchanged:: 1365
            After version 1365 (10eb3eed-6697-4d8c-9aac-32ebf1d36239),
            the behaviour of the main loop was changed: There's an
            initialization of the motor target and the target executed
            before the measurement (so long, it was executed after). We
            need to discuss and test this to be more specific about the
            behaviour.
        
        """
        sys.stdout.flush()
        current_time = 0
        # gcd is associative and commutative
        loop_wait = gcd(self.motor_period, self.sampling_period)
        loop_wait = gcd(self.event_period, loop_wait)
        epoch = Queue.deque(maxlen=self.ctrl_period/self.sampling_period)
        # first epoch targets
        motor_targets = self.actor(dict(), current_time, current_time + self.ctrl_period, self.motor_period)
        
        ## NOTE: if act after sense, we need these two lines as initial target
        current_target = motor_targets.next()
        for servo, trg in zip(self.motors, current_target): servo.setPosition(trg)
        
        while True:
            # advance
            if self.step(loop_wait) == -1: break # otherwise 1st sensor read-outs are nan
            current_time += loop_wait
            
            # first serve receiver callback
            if current_time % self.event_period == 0 and self.event_handler is not None:
                # check messages in receiver
                while self.receiver.getQueueLength() > 0:
                    self.event_handler(self.receiver.getData())
                    self.receiver.nextPacket()
            
            ## NOTE ##
            # act here to have the target which will be applied in the next step
            
            # sense
            # update observations
            if current_time % self.sampling_period == 0:
                sensors_current = self.readout.next()
                epoch.append( current_target + sensors_current )
            
            # think
            # next action
            if current_time % self.ctrl_period == 0:
                ep = dict(zip(self.readout_labels(), map(np.array, zip(*epoch))))
                if self.observer_noise is not None: self._add_observer_noise(ep)
                
                motor_targets = self.actor(ep, current_time, current_time + self.ctrl_period, self.motor_period)
                epoch.clear()
            
            ## NOTE ##
            # act here to have the target which was applied in the last step (= target that lead to current observation)
            # in this case, the target initialization must be uncommented
            # the two act schemes are shifted, i.e. (act before sense)[1:] == (act after sense)[:-1]
            
            # act
            # set motor target
            if current_time % self.motor_period == 0:
                current_target = motor_targets.next()
                if self.motor_noise is not None: current_target = self._add_motor_noise(current_target)
                
                for servo, trg in zip(self.motors, current_target):
                    servo.setPosition(trg)
        
        # teardown
        self._post_run_hook(current_time)
        return self
    
    def _post_run_hook(self, current_time_ms):
        """Cleanup after the main loop has terminated.
        
        The main loop may exit (e.g. if the simulation is reverted) and
        the robot instance closed in the process. Since the destructor
        is not called by default, this hook provides a method to clean
        up any remaining resources and execute post-run code.
        
        The default action is to delete the actor, thus envoking its
        destructor (if any).
        
        ``current_time``
            Time when the main loop was interrupted, in milliseconds.
            
        """
        del self.actor
    
    def _add_observer_noise(self, ep):
        """Add observer noise, according to *observer_noise*.
        """
        if isinstance(self.observer_noise, dict):
            # dict case, individual noise per sensor
            for k in self.observer_noise: # len(obs_noise) <= len(ep)
                if k in ep and k not in ('trg0','trg1','trg2','trg3'):
                    ep[k] += np.random.normal(scale=self.observer_noise[k], size=ep[k].shape)
                else:
                    # scalar case, same noise for all sensors
                    for k in ep:
                        if k in ('trg0','trg1','trg2','trg3'): continue
                        ep[k] += np.random.normal(scale=self.observer_noise, size=ep[k].shape)
        return ep
    
    def _add_motor_noise(self, current_target):
        """Add motor noise, according to *motor_noise*.
        """
        if isinstance(self.motor_noise, int) or isinstance(self.motor_noise, float):
            # scalar case, same noise for all motors
            noise = np.random.normal(scale=self.motor_noise, size=(4,))
        else:
            # list case, individual noise per motor
            noise = [np.random.normal(scale=sig) for sig in self.motor_noise]
        current_target = map(operator.add, current_target, noise)
        return current_target

class WebotsSupervisorMixin(object):
    """Webots supervisor 'controller'. It actively probes the simulation,
    performs checks and reverts the simulation if necessary.
    
    ``sampling_period_ms``
        The period in milliseconds which the supervisor uses to observe
        the robot and possibly executes actions.
    
    ``checks``
        A list of callables, which are executed in order in every
        sampling step. A check's interface must be compliant with
        the one of :py:class:`SupervisorCheck`.
    
    """
    def __init__(self, sampling_period_ms, checks=[]):
        self.checks = checks
        self.loop_wait = sampling_period_ms
        self.emitter = self.getEmitter('toRobotEmitter')
        
    def run(self):
        """Supervisor's main loop. Call this function upon script
        initialization, such that the supervisor becomes active.
        
        The routine runs its checks and reverts the simulation if
        indicated. The iterations counter is made available to the
        checks through *WebotsSupervisorMixin.numIter*.
        
        Note, that reverting the simulation restarts the whole simulation
        which also reloads the supervisor.
        
        """
        self.numIter = 0
        while True:
            
            # run checks
            for check in self.checks:
                check(self)
            
            # advance
            if self.step(self.loop_wait) == -1: break
            self.numIter += self.loop_wait
        
        self._post_run_hook()
        return self
    
    def _pre_revert_hook(self, reason):
        """A custom function (default empty) that is executed just
        before the simulation is reverted.
        
        ``reason``
            A string identifying the check which requested the revert.
        
        """
        pass
    
    def _post_run_hook(self):
        """Cleanup after the main loop has terminated.
        
        The main loop may exit (e.g. if the simulation is reverted) and
        the supervisor instance closed in the process. Since the
        destructor is not called by default, this hook provides a
        method to clean up any remaining resources and execute
        post-run code.
        
        """
        pass

class RecordingSupervisor(WebotsSupervisorMixin):
    """Record the simulation into a webots animation file stored at
    ``animFilename``. The file extension should be 'wva'.
    
    .. todo::
        Webots (PRO 6.4.4) crashes when the simulation is stopped
    
    """
    def __init__(self, animFilename, *args, **kwargs):
        super(RecordingSupervisor, self).__init__(*args, **kwargs)
        self.startAnimation(animFilename)
        
    def _post_run_hook(self):
        """Stop the animation.
        """
        self.stopAnimation()

class SupervisorCheck(object):
    """A template for supervisor's checks."""
    def __call__(self, supervisor):
        """Evalute the check and implement the consequences.
        
        ``supervisor``
            The supervisor instance. For communication back to the
            robot, an *emitter* is available through
            *supervisor.emitter*.
        
        """
        raise NotImplementedError()

class RevertCheck(SupervisorCheck):
    """A template for supervisor's revert checks."""
    def __call__(self, supervisor):
        """Evalute the check, call ``revert`` iff the simulation should be
        reverted.
        
        ``robot``
            The supervisor instance
        
        """
        raise NotImplementedError()
    
    def revert(self, supervisor):
        """Revert the simulation.
        """
        print "Revert simulation (%s)" % (str(self))
        supervisor._pre_revert_hook(str(self))
        supervisor.simulationRevert()

class RevertTumbled(RevertCheck):
    """Revert the simulation if the robot has tumbled.
    
    ``grace_time_ms``
        Let the robot run after tumbling for some time before the
        simulation is reverted. In milliseconds.
    
    """
    def __init__(self, grace_time_ms=2000):
        self.grace_time = grace_time_ms
        self._queue = Queue.deque(maxlen=5)
    
    def __call__(self, supervisor):
        if supervisor.getFromDef('puppy').getOrientation()[4] < 0.15:
            self._queue.append(supervisor.numIter)
            if len(self._queue) > 1 and self._queue[-1] - self._queue[0] < 15*supervisor.loop_wait:
                
                # let grace period pass
                supervisor.step(self.grace_time)
                
                # revert
                self._queue.clear()
                self.revert(supervisor)
    
    def __str__(self):
        return "Tumbled"

class RevertMaxIter(RevertCheck):
    """Revert the simulation if a maximum duration is exceeded.
    
    ``max_duration_ms``
        Maximum time a simulation may run, in milliseconds. After
        this limit, the simulation is reverted.
    
    """
    def __init__(self, max_duration_ms):
        self.max_iter = max_duration_ms
    
    def __call__(self, supervisor):
        if supervisor.numIter > self.max_iter:
            self.revert(supervisor)
    
    def __str__(self):
        return "MaxIter"

class RespawnCheck(SupervisorCheck):
    """A template for supervisor's respawn checks.
    
    ``reset_policy``
        Where to respawn the robot (0=center [0,0], 1=current position,
        2=random position). Instead of literals, use
        *RespawnCheck._reset_center*, *RespawnCheck._reset_current*
        and *RespawnCheck._reset_random*.
        In case of random respawn the *arena_size* needs to be provided.
    
    ``arena_size``
        Size of the arena as list [min_x, max_x, min_z, max_z].
    
    """
    _reset_center = 0
    _reset_current = 1
    _reset_random = 2
    
    def __init__(self, reset_policy=_reset_center, arena_size=[0,0,0,0]):
        self.reset_policy = reset_policy
        self.arena_size = arena_size
    
    def __call__(self, supervisor):
        """Evalute the check, call ``respawn`` iff the supervisor should be
        respawned (reset to a new position).
        
        ``supervisor``
            The supervisor instance
        
        """
        raise NotImplementedError()
    
    def __str__(self):
        raise NotImplementedError()
    
    def respawn(self, supervisor):
        """Reset the robots position.
        """
        robotDef = supervisor.getFromDef('puppy')
        posCurr = robotDef.getPosition()
        rot = [0, 1, 0, float(np.random.rand()*2.*np.pi)]
        if self.reset_policy==self._reset_center:
            new_pos = [0,0]
        elif self.reset_policy==self._reset_current:
            new_pos = posCurr[::2]
        elif self.reset_policy==self._reset_random:
            new_pos = [np.random.randint(self.arena_size[0], self.arena_size[1]),
                       np.random.randint(self.arena_size[2], self.arena_size[3])]
        
        robotDef.getField('rotation').setSFRotation(rot)
        robotDef.getField('translation').setSFVec3f([new_pos[0], 0.13, new_pos[1]]) # when tumbling, remain on same position
        supervisor.emitter.send('reset')
        print "Respawn robot (%s)" % (str(self))

class RespawnTumbled(RespawnCheck):
    """Respawn the robot if it has tumbled.
    
    ``grace_time_ms``
        Let the robot run for some time after tumbling before it is
        respawned. In milliseconds
    
    """
    def __init__(self, grace_time_ms=2000, **kwargs):
        RespawnCheck.__init__(self, **kwargs)
        self.grace_time = grace_time_ms
        self._queue = Queue.deque(maxlen=5)
    
    def __call__(self, supervisor):
        if supervisor.getFromDef('puppy').getOrientation()[4] < 0.15:
            self._queue.append(supervisor.numIter)
            if len(self._queue) > 1 and self._queue[-1] - self._queue[0] < 15*supervisor.loop_wait:
                
                # let grace period pass
                supervisor.step(self.grace_time)
                
                # revert
                self._queue.clear()
                supervisor.emitter.send('tumbled')
                self.respawn(supervisor)
    
    def __str__(self):
        return "Tumbled"

class RespawnOutOfArena(RespawnCheck):
    """Respawn the robot if it comoes too close to the arena boundary.
    
    ``distance``
        The robot's distance to the arena boundary.
    
    ``arena_size``
        Size of the arena as list [min_x, max_x, min_z, max_z].
    
    """
    def __init__(self, distance=2000, arena_size=[0,0,0,0], **kwargs):
        RespawnCheck.__init__(self, arena_size=arena_size, **kwargs)
        self.distance = distance
    
    def __call__(self, supervisor):
        posCurr = supervisor.getFromDef('puppy').getPosition()
        if (posCurr[0]>self.arena_size[1]-self.distance or posCurr[0]<self.arena_size[0]+self.distance 
         or posCurr[2]>self.arena_size[3]-self.distance or posCurr[2]<self.arena_size[2]+self.distance):
            supervisor.emitter.send('out_of_arena')
            self.respawn(supervisor)
    
    def __str__(self):
        return "Out-Of-Arena"

class QuitMaxIter(SupervisorCheck):
    """Quit webots if a time limit is exceeded. (in milliseconds!)
    
    ``max_duration_ms``
        Maximum running time, in milliseconds.
    
    """
    def __init__(self, max_duration_ms):
        self.max_iter = max_duration_ms
    
    def __call__(self, supervisor):
        if supervisor.numIter > self.max_iter:
            print "Quit simulation (%s)" % (str(self))
            supervisor.simulationQuit(0)
    
    def __str__(self):
        return "MaxIter"

def mixin(cls_mixin, cls_base):
    """Create a class which derives from two base classes.
    
    The intention of this function is to set up a working *webots*
    controller, using webots's Robot class and the Puppy mixin from
    this library.
    
    ``mixin``
        The mixin class, e.g.
        *WebotsPuppyMixin* or *WebotsSupervisorMixin*
    
    ``base``
        The base class, e.g.
        *Robot* or *Supervisor*
    
    """
    class WebotsMixin(cls_mixin, cls_base):
        def __init__(self, *args, **kwargs):
            cls_base.__init__(self)
            cls_mixin.__init__(self, *args, **kwargs)
    
    return WebotsMixin

def builder(cls_mixin, cls_base, *args, **kwargs):
    """Set up a mixin class and return an instance of it.
    """
    cls = mixin(cls_mixin, cls_base)
    return cls(*args, **kwargs)

def robotBuilder(cls_base, *args, **kwargs):
    """Return an instance of a webots puppy robot.
    """
    return builder(WebotsPuppyMixin, cls_base, *args, **kwargs)

def supervisorBuilder(cls_base, *args, **kwargs):
    """Return an instance of a webots puppy supervisor.
    """
    return builder(WebotsSupervisorMixin, cls_base, *args, **kwargs)

