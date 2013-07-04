
Webots ecosystem
================

.. module:: PuPy

.. contents::

Introduction
------------

The [Webots]_ simulator already offers Python bindings. The reference manual
can be found `here <http://www.cyberbotics.com/reference.pdf>`_.

Besides the models of the environment and of course the robot, Webots requires
two controllers which implement the robot and simulation logic. The robot model
is not part of this library but assumed to be available. The focus of this
module lies on the controllers.
The robot controller implements the robot's logic. If there's no such
controller, the robot will just remain on its place. It's the controller's
responsibility to define motor targets (possibly with respect to sensory
readouts) on behalf of which the simulator will act. 
The Supervisor controller offers a global view (thus it may access some
information, the robot may not) and allows to start, restart, revert or
quit the simulation. As hinted, it is used to supervise the experiment.
Both controllers have to be implemented in a seperate file and correctly
linked to a webots world (see `Supporting material`_) in order to be executed.

.. warning::
    When running, the Supervisor and Robot controllers are executed as
    seperate processes. When the simulation is reverted (or quit, naturally),
    both processes are terminated. This introduces all the common issues
    with concurrency and inter process communication. Keep this in mind
    and consider restarting instead of reverting a simulation. Also make
    use of the intrinsic communication channels.

Webots offers Python bindings, but the respective module is only available
within a Webots execution. Since we don't want to force this module to
be within Pythons default module search path, the PuPy module must
not directly depend on it. Due to this, a bit more complex approach must
be taken. The library offers two classes (:py:class:`WebotsPuppyMixin`, :py:class:`WebotsSupervisorMixin`)
which have to be mixed into Webot's Robot and Supervisor class at runtime.
To do so more easily, a `builder <http://en.wikipedia.org/wiki/Builder_pattern>`_
is offered for both (:py:func:`robotBuilder`, :py:func:`supervisorBuilder`).

For example, in the controller script, setting up a working instance is as
easy as this:

>>> from controller import Robot
>>> import PuPy
>>> robot = PuPy.robotBuilder(Robot, <args>)

with ``<args>`` the arguments to :py:class:`WebotsPuppyMixin`. The example
works analogous for the Supervisor. If you subclass :py:class:`WebotsPuppyMixin`
or :py:class:`WebotsSupervisorMixin`, the working instance can be set up
through the :py:func:`builder` function.

Reference
---------

.. autofunction:: mixin

.. autofunction:: builder

.. autofunction:: robotBuilder

.. autofunction:: supervisorBuilder

Supporting material
-------------------

When experimenting with several controllers and environments, webots
requires a world file (and some others) for each configuration. Usually,
these differ only in some aspects while most of the content is identical.
Also, the controller is part of the configuration, making it very annoying
to keep track of subtle differences, and keeping track of working versions.
To support development, two small scripts are provided which take care of
this problem.

.. todo::
    Webots world file structure

Splitter
~~~~~~~~

Starting with a world (including the robot and environment), it is first
split into its parts and each one stored with a certain name. This way,
information about the robot or world can be extracted and later identified.
For example, you can store several versions of the robot without redundant
data. This script extracts the desired information from the world file
and stores it in an own data structure. Specify what parts should be saved
and under what name.

.. note::
    The script requires the robot to be named *puppy*.

.. literalinclude:: ../../starter/world_splitter.sh


Builder
~~~~~~~~

You have to define a robot and supervisor controller and choose an environment.
The script will set up a simulator world from these arguments and start webots.
This way, controllers (and environments) may be mixed quickly without having
several files with almost the same content. The controller scripts are
symlinked, so they can be edited while the world is active in webots. Naturally,
the changes take effect after reverting the simulation. 

.. note::
    This script is written for common Linux systems (it requires bash and symlinks).
    Other plattforms may not be supported.

.. note::
    The supplementary files (terrain, plugins, prototypes, etc.) must be installed
    in the same location as the script itself. If this behaviour is not desired,
    set the ``$BASE`` variable to the correct directory. You can generate some of
    these files with the `Splitter`_ script above. Also check out the `Downloads`_ section.

.. note::
    If you wish to create a snapshot of an active controller, the `Splitter`_ script
    may be useful. It copies the controllers to some location, usually outside
    of your common working directory.

.. literalinclude:: ../../starter/world_builder.sh


Downloads
---------

.. todo::
    - webots source downloads (Puppy, worlds)
    - supporting material downloads
