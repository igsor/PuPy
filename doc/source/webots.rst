
Webots ecosystem
================

.. contents::

Introduction
------------

.. todo::
    - Link to webots page ([Webots]_)
    - Link to the API
    - Python bindings
    - Lifetime and availability of libraries
    - Explanation and justification of the mixin concept
    - Links to builder pattern and perhaps mixin
    - How supervisor and robot join in
    - A warning about restarting the simulation (process management)

Reference
---------

.. module:: PuPy

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
