ViSP Python Documentation
============================

.. currentmodule:: visp

.. toctree::
    :maxdepth: 2
    :hidden:

    rst/coming_from_cpp.rst
    rst/python_api/python_api.rst
    rst/tutorials/tutorials.rst
    rst/dev/dev.rst
    api.rst
    rst/known_issues.rst


Welcome to the ViSP Python binding documentation!

  ViSP is a modular C++ library that allows fast development of visual servoing applications.
  ViSP is developed and maintained by the `Inria Rainbow (former Lagadic) team located <https://team.inria.fr/rainbow/>`_ at Inria Rennes.



Introduction
----------------------------

This documentation is specifically aimed at developers choosing to use ViSP in Python.

Other, more general resources, are available:

* If you are using C++, please see `the dedicated documentation <https://visp-doc.inria.fr/doxygen/visp-daily/>`_

* The ViSP wiki can be found `here <https://github.com/lagadic/visp/wiki>`_

* The ViSP source code available on `GitHub <https://github.com/lagadic/visp>`_

* Results and demonstrations can be seen on the `ViSP YouTube channel <https://www.youtube.com/@VispTeam>`_


.. note::

  This documentation does not cover the full capabilities of ViSP. Please see the C++ documentation, which contains:

  * `Tutorials <https://visp-doc.inria.fr/doxygen/visp-daily/tutorial_users.html>`_ on:

    * The core concepts: linear algebra, image processing, etc.
    * Visual servoing with 3D, 2D or photometric features
    * Object pose estimation and tracking

      * With the model-based tracker (MBT) :py:class:`visp.mbt.MbGenericTracker`
      * With MegaPose, a deep learning approach to pose estimation :py:class:`visp.dnn_tracker.MegaPose`

  * `Examples <https://visp-doc.inria.fr/doxygen/visp-daily/examples.html>`_

    * Demonstrating basic feature usage
    * Servoing on specific robotics platforms
    * Tracking


.. warning::

  There are still issues with these generated bindings: see :ref:`Known issues`.


Getting started
^^^^^^^^^^^^^^^^^^^^^^^

If you are transitioning from C++, please have a look at the :ref:`CPP guide` to understand the differences between the Python and C++ versions.

For general ViSP + Python guidance, see the :ref:`Python API guide`.

For tutorials on specific features: see :ref:`Tutorials`.

Finally, if you wish to browse the full ViSP class documentation, go to the :ref:`API reference`.


Customizing, extending and contributing to the bindings
--------------------------------------------------------

If you wish to contribute, extend or modify the bindings for your own needs, please read :ref:`Development guide`
