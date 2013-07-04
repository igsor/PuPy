
Examples
========

A simple example
----------------

.. literalinclude:: ../../test/webotsSupervisorController.py

.. literalinclude:: ../../test/webotsRobotController.py

Try out the code by storing the excerpts on your disk and execute::

    world_builder -c <pth/to/robot> -s <pth/to/supervisor> -t styrofoam /tmp/webots_test

A custom collector
------------------

It's basically the same example as above. The main difference is that 
the observation file contains a new dataset 'random' (with just random values).
It has the same length as the other datasets but ten columns
(instead of the usual single one).

.. literalinclude:: ../../test/collector.py

Supervisor to Robot communication
---------------------------------

The robot will walk unspectacularly but a message will be printed to the
console every 100ms. The message is 'Emitting ``<nr>``', with ``<nr>``
a multiple of 100.

.. literalinclude:: ../../test/commSupervisor.py

.. literalinclude:: ../../test/commRobot.py


