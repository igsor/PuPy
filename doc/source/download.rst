
.. _downloads:

Download
========

.. contents::

Installation
------------

Using `Pip Installs Python (Pip) <http://www.pip-installer.org/en/latest/index.html>`_,
simply type::

    pip install http://www.igsor.net/research/PuPy/_downloads/latest.tar.gz

if you want to use the package from the webpage. If you have downloaded it yourself, use::

    pip install path/to/PuPy.tar.gz

If you're using `distutils <http://docs.python.org/distutils/>`_, type::
    
    tar -xzf path/to/PuPy.tgz       # extract files.
    cd PuPy*                        # change into PuPy directory.
    sudo python setup.py install    # install using distutils (as root).
    #rm -R .                        # remove source. If desired, uncomment this line.
    #cd .. && rmdir PuPy*           # remove working directory. If desired, uncomment this line but be careful.

The project is also available on git, with the package and all supplementary data:

    git clone https://github.com/igsor/PuPy


Make sure all dependencies are installed on your system. This obviously includes
[Python]_, as well as [SciPy]_ and [NumPy]_. Furthermore, either [h5py]_ or [PyTables]_
is required. Since this module is tightly bound to [Webots]_ you should
have it available as well.

If you wish to store the library outside of the system's paths, a neat way to
link the library to Python is to use a path configuration file (``.pth``). Place
a file ``<modname>.pth`` in python's default module search path for site packages.
The file must contain nothing else than the actual path to the library. Note that
the module's source must be within a subfolder of the specified path and the subfolder
must have the same name as the module (i.e. ``<modname>``). Normally, ``<modname>``
would be *PuPy*.

To get the site package path, type:

>>> import site
>>> site.getsitepackages()
['/usr/local/lib/python2.7/dist-packages', '/usr/lib/python2.7/dist-packages']


If in use, it also makes sense to install the additional scripts from
the :doc:`webots page <webots>` in a system's default executable search
path. Get these by typing into a bash::

    tux@localhost:~$ echo $PATH
    /usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin


``/usr/local/sbin`` should be a fair choice. It is advised to keep the
script together with the other data, thus only a symlink is created (as root)::

    root@localhost:~# ln -s </pth/to/world_builder.sh> /usr/local/sbin/webots_builder

The same procedure may be repeated for ``world_splitter.sh``.

.. _available_downloads:

Available downloads
-------------------

PuPy library and documentation by Nico Schmidt and Matthias Baumgartner:

- :download:`PuPy-1.0 <_downloads/PuPy-1.0.tar.gz>` (latest)

- :download:`This documentation (pdf) <_downloads/PuPy-1.0-doc.pdf>`

Supplementary scripts and the Puppy webots model by Matej Hoffmann and Stefan Hutter:

- :download:`Webots data and supplementary scripts <_downloads/additionals.tar.gz>`

The logo by Karin Baumgartner:

- :download:`PuPy Logo (svg) <_downloads/logo.svg>`

- :download:`PuPy Logo (pdf) <_downloads/logo.pdf>`

License
-------

This project is released under the terms of the 3-clause BSD License and CC BY. See the section
:ref:`license` for details.
