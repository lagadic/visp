================================================
Display an image
================================================

Introduction
===========================

Goal
---------------------------

In this tutorial you will learn how to :

- Display an image in a window.
- Use the `Display <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display>`_ class.

Prerequisites
---------------------------

You should first read the `Getting started with images <tutorial-image-getting-started.html>`_ tutorial.

Display an image
===========================

Code
---------------------------

The following `example <examples/code-image-display.html>`_ reads the ``monkey.jpeg`` file as an `ImageRGBa <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.ImageRGBa.html#visp.core.ImageRGBa>`_ object and displays it :

.. literalinclude:: /examples/image/tutorial-image-display.py
	:language: python
	:linenos:

You can run the example with :

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-display.py

Result
---------------------------

The image is displayed in a window :

.. image:: images/result-image-display.png
	:alt: Image
	:align: center

Click anywhere in the window to close it and allow the program to finish.

Explanation
---------------------------

We first import the classes required to read and use the image:

.. code-block:: python

	from visp.core import ImageRGBa
	from visp.io import ImageIo

We then import the `Display <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display>`_ class and the
`get_display() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.display_utils.get_display.html#visp.display_utils.get_display>`_ function :

.. code-block:: python

	from visp.core import Display
	from visp.python.display_utils import get_display

.. note::

  The `get_display() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.display_utils.get_display.html#visp.display_utils.get_display>`_ function
  detects the available GUI backend and returns a display object that can be
  used by the application. See the `GUI module overview <https://visp.inria.fr/gui/>`_
  for more information.

We then import the image from the disk as the `ImageRGBa <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.ImageRGBa.html#visp.core.ImageRGBa>`_ object I :

.. code-block:: python

	# Image path
	path = sys.path[0] + "/monkey.jpeg"

	# Read the image 
	I = ImageRGBa()
	try:
	  ImageIo.read(I, path)
	except:
	  print(f"Cannot read image {path}")
	  sys.exit()

.. note::

	 For more information about reading and writing images, see the `Read an write an image <tutorial-image-io.html>`_ tutorial.

Next, we create a `Display <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display>`_ object d using the `get_display() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.display_utils.get_display.html#visp.display_utils.get_display>`_ method, and initialize it with the dimensions of the image :

.. code-block:: python

	d = get_display()
	d.init(I)

We then initialize it to the dimensions of the image we want to display,
and set the title of the display using `setTitle() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.setTitle>`_.

.. code-block:: python

	d.init(I)
	Display.setTitle(I, "monkey.jpeg")

The image is then drawn in the display with the `display() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.display>`_ method. The `flush() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.flush>`_ method finally show the window on screen :

.. code-block:: python

	Display.display(I)
	Display.flush(I)

The `flush() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.flush>`_ function does not stop the program execution. We therefore call the `getClick() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.getClick>`_ method, to
pause the program until the user clicks in the display :

.. code-block:: python

	print("A click to quit...")
	Display.getClick(I)

Other Options
===========================

Change scaling factor
---------------------

Depending on the image dimensions and your screen resolution, the display window
may be too large to fit on the screen.

The following `example <examples/code-image-display-scaled-default.html>`_ creates a 2160 x 3840 image with a red circle in its center and displays it.
Run it with :

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-display-scaled-default.py

If your screen resolution is lower than the image resolution, the window may
not fit entirely on the screen. For example, the image below shows the result
on a 1920 x 1080 display :

.. image:: images/result-image-display-scaled-default.png
	:alt: Image
	:align: center

To scale the image automatically so that it fits on the screen, call `setDownScalingFactor() <https://visp-doc.inria.fr/doxygen/visp-python-daily/_autosummary/visp.core.Display.html#visp.core.Display.setDownScalingFactor>`_ method after initializing the display :

.. code-block:: python

	d.setDownScalingFactor(Display.SCALE_AUTO)

You can test this behavior with the following `example <examples/code-image-display-scaled-auto.html>`_ :

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-display-scaled-auto.py

You can also specify a fixed downscaling factor. For example, the following
code divides the image width and height by five:

.. code-block:: python

	d.setDownScalingFactor(Display.SCALE_5)


You can test this behavior with the following `example <examples/code-image-display-scaled-manu.html>`_ :

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-display-scaled-manu.py

Display using Matplotlib
------------------------

.. note::

	This option requires the Matplotlib Python package.

You can also use Matplotlib to display the image. First, import
``matplotlib.pyplot`` :

.. code-block:: python

	import matplotlib.pyplot as plt

Then display the image in a figure :

.. code-block:: python

	plt.imshow(I)
	plt.axis('off')
	plt.title("monkey.jpeg")
	plt.show()

The following `example <examples/code-image-display-matplotlib.html>`_ shows this approach :

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-display-matplotlib.py

The result should look similar to this window :

.. image:: images/result-image-display-matplotlib.png
	:alt: Image
	:align: center

Next Tutorial
===========================

You are now ready to learn how to `Read an write an image <tutorial-image-io.html>`_.