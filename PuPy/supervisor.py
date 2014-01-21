"""
"""
import Queue

class SupervisorCheck(object):
    """ A template class for supervisor's checks. """
    def __init__(self, **kwargs):
        pass
    
    def __call__(self, supervisor):
        """Evalute the check and trigger the action if check applies.
        The action should be called with the ``supervisor`` and a
        message string as arguments, e.g. self.action(supervisor, msg)
        
        ``supervisor``
            The supervisor instance. For communication back to the
            robot, an *emitter* is available through
            *supervisor.emitter*.
                    
        """
        raise NotImplementedError()


class SupervisorAction(object):
    """ A template class for supervisor's check actions. """
    
    def __init__(self, **kwargs):
        pass
    
    def action(self, supervisor, msg):
        """Overload in subclass for implementing an action"""
        raise NotImplementedError()




class TumbledCheck(SupervisorCheck):
    """Check whethre Puppy has tumbled."""
    def __init__(self, **kwargs):
        super(TumbledCheck, self).__init__(**kwargs)
        self._queue = Queue.deque(maxlen=5)
    
    def __call__(self, supervisor):
        if supervisor.getFromDef('puppy').getOrientation()[4] < 0.15:
            self._queue.append(supervisor.num_iter)
            if len(self._queue) > 1 and self._queue[-1] - self._queue[0] < 15*supervisor.loop_wait:
                self._queue.clear()
                supervisor.emitter.send('tumbled')
                self.action(supervisor, "Tumbled")


class MaxIterCheck(SupervisorCheck):
    """Checks whether a maximum duration is exceeded.
    
    ``max_duration_ms``
        Maximum time a simulation may run, in milliseconds. After
        this limit, the simulation is reverted.
    
    """
    def __init__(self, **kwargs):
        self.max_iter = kwargs.pop('max_duration_ms')
        super(MaxIterCheck, self).__init__(**kwargs)
    
    def __call__(self, supervisor):
        if supervisor.num_iter > self.max_iter:
            supervisor.emitter.send('max_iter')
            self.action(supervisor, "MaxIter")


class OutOfArenaCheck(SupervisorCheck):
    """Check whether the robot comes too close to the arena boundary.
    
    ``distance``
        The threshold distance from the robot to the arena boundary 
        when to trigger the action.
    
    ``arena_size``
        Size of the arena as list [min_x, max_x, min_z, max_z].
    
    """
    def __init__(self, **kwargs):
        self.arena_size = kwargs.pop('arena_size', (0, 0, 0, 0))
        self.distance = kwargs.pop('distance', 5)
        super(OutOfArenaCheck, self).__init__(**kwargs)
    
    def __call__(self, supervisor):
        pos_curr = supervisor.getFromDef('puppy').getPosition()
        if (pos_curr[0]>self.arena_size[1]-self.distance or pos_curr[0]<self.arena_size[0]+self.distance 
         or pos_curr[2]>self.arena_size[3]-self.distance or pos_curr[2]<self.arena_size[2]+self.distance):
            supervisor.emitter.send('out_of_arena')
            self.action(supervisor, "Out-Of-Arena")


class ReceiverCheck(SupervisorCheck):
    """Listens on the supervisor's receiver channel for messages of the
    robot (on demand messages).
    Note that since messages are popped from the receiver stack, this
    check doesn't interoperate with other to-supervisor communication
    checks (messages will be lost).
    As a work-around, :py:class:``ReceiverCheck`` gets a list of
    :py:class:`SupervisorAction` that are connected with a message string
    to identify which action has to be triggered.
    
    ``actions``
        list of tuples/lists having a message string and an instance of a
        :py:class:`SupervisorAction`, that will be triggered when the
        message was received.
        E.g. ['revert on demand', PuPy.RevertAction(grace_time=2000),
              'quit on demand', PuPy.QuitAction()]
    """
    def __init__(self, actions, **kwargs):
        super(ReceiverCheck, self).__init__(**kwargs)
        
        self.actions = {}
        for msg, supervisor_action in actions:
            self.actions[msg] = supervisor_action.action
    
    def __call__(self, supervisor):
        while supervisor.receiver.getQueueLength() > 0:
            msg = supervisor.receiver.getData()
            supervisor.receiver.nextPacket()
            if msg in self.actions:
                action_func = self.actions[msg]
                action_func(supervisor, msg)




class NotifyAction(SupervisorAction):
    """Print a notification if some condition holds.
    
    The ``timeout`` controls how often the notification is printed:
    
    * ``timeout`` = 0: The message is shown every time the check is run.
                       (the default)
    * ``timeout`` < 0: The message is shown only once.
    * ``timeout`` > 0: The message is suppressed for the respective
                       number of calls.
    """
    def __init__(self, **kwargs):
        self.timeout = kwargs.pop('timeout', 0)
        super(NotifyAction, self).__init__(**kwargs)
        self.notification_counter = 0

    def action(self, supervisor, msg=None):
        if msg is None:
            msg = str(self)
        
        if self.notification_counter == 0:
            if self.timeout < 0:
                self.notification_counter = -1
            else:
                self.notification_counter = self.timeout
        else:
            self.notification_counter -= 1
            return
        
        supervisor.emitter.send(msg)
        print "Notification:", msg


class RevertAction(SupervisorAction):
    """A template for supervisor's revert checks."""
    def __init__(self, **kwargs):
        self.grace_time = kwargs.pop('grace_time_ms', 2000)
        super(RevertAction, self).__init__(**kwargs)
    
    def action(self, supervisor, msg):
        """Revert the simulation.
        """
        # let grace period pass
        supervisor.step(self.grace_time)
        
        # revert
        print "Revert simulation (%s)" % msg
        supervisor.emitter.send('revert')
        supervisor._pre_revert_hook(msg)
        supervisor.simulationRevert()


class RespawnAction(SupervisorAction):
    """A template for supervisor's respawn action.
    
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
    
    def __init__(self, **kwargs):
        self.reset_policy = kwargs.pop('reset_policy', self._reset_center)
        self.arena_size = kwargs.pop('arena_size', (0, 0, 0, 0))
        super(RespawnAction, self).__init__(**kwargs)
    
    def action(self, supervisor, msg):
        """Reset the robots position.
        """
        import numpy as np
        robot_def = supervisor.getFromDef('puppy')
        pos_curr = robot_def.getPosition()
        rot = [0, 1, 0, float(np.random.rand()*2.*np.pi)]
        if self.reset_policy == self._reset_center:
            new_pos = [0, 0]
        elif self.reset_policy == self._reset_current:
            new_pos = pos_curr[::2]
        elif self.reset_policy == self._reset_random:
            new_pos = [np.random.randint(self.arena_size[0], self.arena_size[1]),
                       np.random.randint(self.arena_size[2], self.arena_size[3])]
        
        print "Respawn robot (%s)" % msg
        robot_def.getField('rotation').setSFRotation(rot)
        robot_def.getField('translation').setSFVec3f([new_pos[0], 0.13, new_pos[1]]) # when tumbling, remain on same position
        supervisor.emitter.send('reset')


class QuitAction(SupervisorAction):
    """Quit webots"""
    def action(self, supervisor, msg):
        print "Quit simulation (%s)" % (msg)
        supervisor.simulationQuit(0)




def supervisorCheckMixin(cls_check, cls_action):
    class SupervisorCheckMixin(cls_check, cls_action):
        """Trivial class to correctly set up the mixin."""
        def __init__(self, **kwargs):
            cls_check.__init__(self, **kwargs)
            cls_action.__init__(self, **kwargs)
    return SupervisorCheckMixin


def supervisorCheckBuilder(cls_check, cls_action, **kwargs):
    """Return a mixin of a :py:class:`SupervisorCheck` class with the
    functionality specified by the argument classes.
    ``cls_check``
        subclass of :py:class:`SupervisorCheck` implementing the ``check`` method
        (and potentially ``_post_action_hook``).
    
    ``cls_action``
        subclass of :py:class:`SupervisorAction` implementing the ``action`` method.
    """
    cls = supervisorCheckMixin(cls_check, cls_action)
    return cls(**kwargs)


# for back-compatibility:
def RespawnOnDemand(**kwargs):
    return ('respawn_on_demand', RespawnAction(**kwargs))
def RevertOnDemand(**kwargs):
    return ('revert_on_demand', RevertAction(**kwargs))
def QuitOnDemand():
    return ('quit_on_demand', QuitAction())
NotifyTumbled = supervisorCheckMixin(TumbledCheck, NotifyAction)
RevertTumbled = supervisorCheckMixin(TumbledCheck, RevertAction)
RevertMaxIter = supervisorCheckMixin(MaxIterCheck, RevertAction)
RevertOutOfArena = supervisorCheckMixin(OutOfArenaCheck, RevertAction)
RespawnTumbled = supervisorCheckMixin(TumbledCheck, RespawnAction)
RespawnOutOfArena = supervisorCheckMixin(OutOfArenaCheck, RespawnAction)
RespawnMaxIter = supervisorCheckMixin(MaxIterCheck, RespawnAction)
QuitMaxIter = supervisorCheckMixin(MaxIterCheck, QuitAction)
QuitTumbled = supervisorCheckMixin(TumbledCheck, QuitAction)

