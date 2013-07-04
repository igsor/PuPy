
Supervisor
==========

.. module:: PuPy



.. contents::


Introduction
------------

.. todo::
    - What the supervisor is used for
    - What can be achieved with it
    - Typical use cases
    - Explain the supervisor and the check facility
    - Links to patterns (strategy, chain of responsibility, ...)


Example
-------

The supervisor facility is intended to be used from within the
webots simulator [Webots]_. The simulator automatically appends the
controller module to `Python's <http://www.python.org>`_ search path,
such that the :py:class:`Supervisor` class can be imported. Of course,
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
:py:func:`supervisorBuilder` needs to be called with *Supervisor* as
its first argument and the :py:class:`WebotsSupervisorMixin`'s arguments
following.

>>> s = PuPy.supervisorBuilder(Supervisor, 20, checks)

With this, there's a supervisor instance which now can be started. The
call to :py:meth:`WebotsSupervisorMixin.run` will only exit when the
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

