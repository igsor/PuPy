.. Puppy Python Controller documentation master file, created by
   sphinx-quickstart on Thu May 16 12:02:16 2013.

Puppy Python Controller (PuPy)
==============================

There's this robot, Puppy, a four-legged artificial dog. Although it is
actually built and can be controlled in the real-world, some applications
require its simulated version. This may for example be to prevent the motors
from damage or to do hundreds of experiments in short time. A popular choice
for simulating such robots, is [Webots]_. Webots already provides a Python
interface to implement arbitrary simulation controllers. The PuPy module
builds on top of that but also abstracts from the concrete Puppy robot model.
The focus lies on implementing various controllers, while the robot model is
assumed to be already available. This way, it provides a simple facility
to further abstract from the simulation setup and offers an easy way to
implement a new controller for Puppy. The idea is to offer an abstraction
such that most of the controllers are easy to implement. More
involved or in some sense special controllers may be off the grid and therefore
still require more complex implementation. Yet, especially when starting work
on Puppy, this module should make your life much easier.

The module is written in pure Python (version 2.7). Since [Webots]_
requires two controllers - one for the supervisor and one for the
robot - the module's core consists of two classes: an overlay for the
:py:class:`Robot <PuPy.WebotsPuppyMixin>` and
:py:class:`Supervisor <PuPy.WebotsSupervisorMixin>`. These are the main
abstractions from the Webots interface. Especially the
:py:class:`Robot <PuPy.WebotsPuppyMixin>` provides a natural way of
building a controller in the well-known sense-think-act cycle. It also
deals with the communication to the simulator, exonerate the programmer
with simulation-related issues. It offers a way to build a controller
within a pure Python/NumPy environment. For the
:py:class:`Supervisor <PuPy.WebotsSupervisorMixin>`, typical use cases have
been prepared, also abstracting from the Webots interface. With these
core functions, it should be possible to implement a simple controller
within minutes and a minimal amount of code.

To run the module, some more libraries are required. See the :doc:`download`
page for more information.

Contents
--------

.. toctree::
    :maxdepth: 1
    
    webots
    supervisor
    robot
    control
    
    examples
    download
    license
    resources
    
    todopg


Indices and tables
------------------

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

