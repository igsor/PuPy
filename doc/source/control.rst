
Control
=======

.. module:: PuPy

.. contents::

Introduction
------------

Given the :py:class:`supervisor <WebotsSupervisorMixin>` and
:py:class:`robot <WebotsPuppyMixin>`, a working controller can be set up.
As described on the :doc:`robot` page, the this can be achieved by
implementing an ``actor``, compliant with the :py:class:`PuppyActor`
interface. Although this task is generally problem dependent, some
typical actors have been prepared. The most common control setup is
based on :py:class:`Gaits <Gait>`. 

.. todo::
    - How to set up a controller
    - Explain basic controllers

Example
-------

This example is very similar to the one at :doc:`robot`; make sure, you
understand the robot aspects.

Again, it's first required to import some modules:

>>> from controller import Robot
>>> import PuPy

In this example, the robot movement is based on some predefined walking
patterns. A pattern is defined through a :py:class:`Gait`, which
generates a specific sinewave. The parameters of the wave have to be
passed to the :py:class:`Gait` constructor. Since there are four motors,
there are four values for each parameter.

>>> boundLeft = PuPy.Gait({
>>>     'frequency' : (1.0, 1.0, 1.0, 1.0),
>>>     'offset'    : ( -0.23, -0.23, -0.37,-0.37),
>>>     'amplitude' : ( 0.38, 0.65, 0.47, 0.65),
>>>     'phase'     : (0.1141, 0.0, 0.611155, 0.5)
>>> })

One pattern would be quite boring, so another one is defined:

>>> boundRight = PuPy.Gait({
>>>     'frequency' : (1.0, 1.0, 1.0, 1.0),
>>>     'offset'    : ( -0.23, -0.23, -0.37, -0.37),
>>>     'amplitude' : ( 0.65, 0.38, 0.65, 0.47),
>>>     'phase'     : (0.0, 0.1141, 0.5, 0.611155)
>>> })

The gaits only implements a sequence of motor targets, not an actor. One
might just apply one constant pattern (there's :py:class:`ConstantGaitControl`
for that) but may also come up with other gait-based controllers. Here,
the goal is to randomly switch between the two gaits. For this, the
respective controller is initialized with the two gaits:

>>> actor = PuPy.RandomGaitControl([boundLeft, boundRight])

This now is an actor, as it implements the :py:class:`PuppyActor`
interface - of course, :py:class:`RandomGaitControl` is designed to do
so. Yet, it might also be interesting to store sensor readouts of the
simulation in a file for later inspection. For this purpose, there exists
the :py:class:`PuppyCollector`. It is a transparent actor, meaning that
it simulates an actor towards the :py:class:`WebotsPuppyMixin` but lets
a 'true' actor do the work. It just stores the provided measurements in
a file.

This transparency can also be observed in the initialization, when the
previously defined actor is passed as argument, together with a target
file.

>>> observer = PuPy.PuppyCollector(actor, expfile='/tmp/puppy_sim.hdf5')

When the actor is initialized, it is passed to the constructor of
:py:class:`WebotsPuppyMixin` through the :py:func:`robot builder <robotBuilder>`.
Then, the robot controller can be executed and its effect observed in
webots or from the data log.

>>> r = PuPy.robotBuilder(Robot, observer, sampling_period_ms=20)
>>> r.run()

Once some data has been gathered and the simulation is stopped, the
data file may be inspected. Note that the code below is not part of the
control script anymore but a seperate python instance
(e.g. in a script or interactively).

>>> import h5py, pylab
>>> f = h5py.File('/tmp/puppy_sim.hdf5','r')
>>> pylab.plot(f['0']['trg0'][:200], 'k', label='FL')
>>> pylab.plot(f['0']['trg1'][:200], 'r', label='FR')
>>> pylab.plot(f['0']['trg2'][:200], 'g', label='HL')
>>> pylab.plot(f['0']['trg3'][:200], 'b', label='HR')
>>> pylab.title('Motor targets')
>>> pylab.xlabel('time')
>>> pylab.show()



Reference
---------

.. autoclass:: PuppyActor
    :members:

.. autoclass:: Gait
    :members:

.. autoclass:: RandomGaitControl
    :members:
    :show-inheritance:

.. autoclass:: ConstantGaitControl
    :members:
    :show-inheritance:

.. autoclass:: SequentialGaitControl
    :members:
    :show-inheritance:

.. autoclass:: PuppyCollector
    :members:
