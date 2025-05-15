Robot interfaces
================

This example shows how to connect to the Franka Robotics Panda robot. First it gets the current joint positions,
before moving to a secure home position.

.. literalinclude:: /examples/franka-connexion.py
  :language: python

To run this example we suppose that you successfully build and install libfranka third-party before
building ViSP Python bindings in a Conda environment (prefered) or a Python virtual environment as explained
`here <https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-python-bindings.html#py_bindings_build>`_.

If your robot has "192.168.30.10" IP, you may run this example with:

::

  (visp-conda-ws) $ python -i ${VISP_WS}/visp/modules/python/examples/franka-connexion.py --ip 192.168.30.10
