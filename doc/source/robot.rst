
Robot
=====

.. module:: PuPy

.. contents::


Introduction
------------

The :py:class:`WebotsPuppyMixin` class connects the Puppy robot model to the
Webots' programming interface. It is designed such that it simplifies
and unifies the development of a custom Puppy controller. Just like the
:doc:`supervisor`, the :py:class:`WebotsPuppyMixin`
has to be instantiated through a builder. At creation, the class requires
an ``actor`` callback (which implements the controller) and some timings.
The timings define the length of different epochs, namely the intervalls
at which the ``actor`` is executed and how many sensor readings will be
available. The callback must be compliant with :py:class:`RobotActor`. Note
that although the interface is specified in a class structure, it is designed
in a way that also fits a plain function.

Similar to :py:class:`WebotsSupervisorMixin`, the
:py:meth:`WebotsPuppyMixin.run` method implements the main,
infinite control loop. Within the loop, a sense-think-act cycle is
running. Sensor data are automatically read, so the sense-part is taken
care of. Since the think and act parts are problem-dependent, they have
to be implemented by the ``actor``. Each call to the ``actor`` must
return an iterator for the next of motor targets. These are incrementally
enforced on the motors, such that the ``actor`` may indeed define the robot's
behaviour. To do so, the sensor readings between two calls are supplied.
To see how this works exactly, consult the documentation of
:py:class:`WebotsPuppyMixin` and :py:class:`RobotActor`.

The :py:class:`WebotsPuppyMixin` abstracts from the Webots API in the sense
that it takes care of initialization and readout of Puppy's sensors. Thus,
the ``actor`` may focus on the pure controller implementation. Some simple
controllers have been prepared, for them see the :doc:`control` page.

Example
-------

In the controller script, Webots' :py:mod:`controller` module will be present,
so it is imported together with the :py:mod:`PuPy` module.

>>> from controller import Robot
>>> import PuPy

As a simple illustrative walking pattern, a :py:class:`Gait` is initialized.
Details are not relevant right here (see the :doc:`control` page for that),
this just specifies a way of moving around.

>>> gait = PuPy.Gait({
>>>     'frequency' : (1.0, 1.0, 1.0, 1.0),
>>>     'offset'    : ( -0.23, -0.23, -0.37, -0.37),
>>>     'amplitude' : ( 0.56, 0.56, 0.65, 0.65),
>>>     'phase'     : (0.0, 0.0, 0.5, 0.5)
>>> })

With this specification, an :py:class:`actor <RobotActor>` can be set up.
Again, it's not important what the actor concretely does but its mere
existence: It implements the act-step in the sense-think-act cycle. Again,
for details see :doc:`control`.

>>> actor = PuPy.ConstantGaitControl(gait)

When everything is ready, the robot instance is created through the respective
builder, as already mentioned. The first argument is Webots' :py:class:`Robot`
class, followed by arguments of :py:class:`WebotsPuppyMixin` (``actor`` in this
case).

>>> r = PuPy.robotBuilder(Robot, actor)

The instance is ready, its main loop awaits execution. Let's do this as
the last script line:

>>> r.run()

When the simulation terminates (i.e. reverts or quits), the main loop
will actually be broken and the above call returns. Make sure that if
there's code below this point, it doesn't prevent termination. There's a
one second timeframe before the script gets killed by Webots.

Reference
---------

.. autoclass:: WebotsPuppyMixin
    :members: run

