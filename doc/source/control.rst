
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



>>> from controller import Robot
>>> import PuPy

>>> gait = PuPy.Gait({
>>>     'frequency' : (1.0, 1.0, 1.0, 1.0),
>>>     'offset'    : ( -0.23, -0.23, -0.37, -0.37),
>>>     'amplitude' : ( 0.56, 0.56, 0.65, 0.65),
>>>     'phase'     : (0.0, 0.0, 0.5, 0.5)
>>> })

>>> actor = PuPy.ConstantGaitControl(gait)
>>> observer = PuPy.PuppyCollector(actor, expfile='/tmp/puppy_sim.hdf5')

>>> r = PuPy.robotBuilder(Robot, observer, sampling_period_ms=20)

>>> r.run()


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
