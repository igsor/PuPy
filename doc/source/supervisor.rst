
Supervisor
==========

.. module:: PuPy

.. contents::


Introduction
------------

Each simulation run is observed and possibly influenced by a Supervisor.
It has complete access of all the Robot's and environment's properties
and can exclusively execute certain actions (such as start, stop or
record the simulation). To implement such a supervisor, one needs to set up
a script and link it to the webots world (see :doc:`webots`). Within the
script, the module :py:mod:`controller` can be imported (at runtime only),
specifically its class :py:class:`controller.Supervisor`. It provides the
basic Webots interface.

The mixin :py:class:`WebotsSupervisorMixin` extends the base functionality
of :py:class:`controller.Supervisor` by a :py:meth:`WebotsSupervisorMixin.run`
method. Within this method, the infinite control loop is executed. It only
returns shortly before the simulation is terminated. In the loop, a number
of user-defined checks is executed periodically. These checks are the
main tool to customize the supervisor, since they are also allowed to
invoke supervisor actions. The checks are executed one after another
with no guaranteed order. They can be interpreted as a list of 
`Strategies <http://en.wikipedia.org/wiki/Strategy_pattern>`_. A check
should evaluate its condition (generally speaking, execute its code) and
return if no action has to be taken. When reverting or restarting,
it is advised to go through the predefined structures to ensure
consistency.

For the typical use cases, checks have been predefined. They all are
successors of :py:class:`SupervisorCheck` and further divide into
:py:class:`RevertCheck` and :py:class:`RespawnCheck`, based on their
effect. However, note that checks are defined in a way that also allows
them to be functions instead of classes, thus this hierarchy is optional.

Example
-------

The supervisor facility is intended to be used from within the
webots simulator [Webots]_. The simulator automatically appends the
controller module to Python's search path, such that the
:py:class:`Supervisor` class can be imported. Of course,
the :py:mod:`PuPy` module is also made available:

>>> from controller import Supervisor
>>> import PuPy

The supervisor is capable of executing some checks from time to time
and react correspondingly. Two checks are added, one that limits the
time of the experiment and another one to catch a tumbled robot. Both
checks revert the simulation if the respective condition is met:

>>> checks = []
>>> checks.append(PuPy.RevertTumbled(grace_time_ms=2000))
>>> checks.append(PuPy.RevertMaxIter(max_duration_ms=20*50*2*20))

With these checks, the supervisor can be created. For this, one has to
go through the respective builder. This is because the webots module
is not available within PuPy. In the case of the supervisor,
:py:func:`supervisorBuilder` needs to be called with ``Supervisor`` as
its first argument and the :py:class:`WebotsSupervisorMixin`'s arguments
following.

>>> s = PuPy.supervisorBuilder(Supervisor, 20, checks)

With this, there's a supervisor instance ``s`` which now can be started.
The call to :py:meth:`WebotsSupervisorMixin.run` will only exit when the
simulation is somehow aborted.

>>> s.run()


Reference
---------

.. autoclass:: WebotsSupervisorMixin
    :members:

.. autoclass:: SupervisorCheck
    :members:
    :special-members:

.. autoclass:: RevertCheck
    :members:
    :show-inheritance:

.. autoclass:: RevertMaxIter
    :members:
    :show-inheritance:

.. autoclass:: RevertTumbled
    :members:
    :show-inheritance:

.. autoclass:: RespawnCheck
    :members:
    :show-inheritance:

.. autoclass:: RespawnTumbled
    :members:
    :show-inheritance:

.. autoclass:: RespawnOutOfArena
    :members:
    :show-inheritance:

.. autoclass:: QuitMaxIter
    :members:
    :show-inheritance:

