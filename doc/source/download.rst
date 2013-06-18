
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
    #cd .. && rmdir PuPy*           # remove working directory. If desired, uncomment this line.

Make sure, `scipy <http://www.scipy.org/>`_ and `numpy <http://numpy.scipy.org/>`_ are
installed on your system.

.. todo::
    Also dependent on webots (some parts), h5py or pytables
    Also, some further scripts (for webots) are supplied, don't forget to document their installation.

Getting started
---------------

Again, make sure that besides the `PuPy <http://www.igsor.net/research/PuPy/>`_, the packages `scipy <http://www.scipy.org/>`_
and `numpy <http://numpy.scipy.org/>`_ are installed on your system.

.. todo::
    Explanations and examples

Available downloads
-------------------

- :download:`PuPy-0.1 dev <_downloads/PuPy-0.1dev.tar.gz>` (latest)

.. sth:
    - :download:`This documentation (html) <dist/aiLib-0.1doc-html.tar.gz>` (current)
    - :download:`This documentation (pdf) <dist/aiLib-0.1doc.pdf>` (current)

License
-------

This project is released under the terms of the 3-clause BSD License. See the section
:ref:`license` for details.
