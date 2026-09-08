============================
Read and write an image file
============================

Introduction
===========================

Objective
---------------------------

In this tutorial you will learn how to:

- Read an image from a file
- Write an image to a file.
- Use the `ImageIo <file:///home/sjourdro/visp-ws/visp-build-bindings/doc/python/_autosummary/visp.io.ImageIo.html>`_ class.

Prerequisites
---------------------------

You should first read the `Getting started with images <tutorial-image-getting-started.html>`_ tutorial.

Read and write an image
===========================

Code
---------------------------

The following `example <examples/code-image-io.html>`_ reads the ``monkey.jpeg`` file, converts it into a grayscale image, and saves the result as ``grayscale_monkey.jpeg``:

.. literalinclude:: /examples/image/tutorial-image-io.py
	:language: python
	:linenos:

You can run the example with:

.. code-block:: bash

	python3 $VISP_WS/visp/modules/python/examples/image/tutorial-image-io.py

Result
---------------------------

The example displays the input image followed by the converted grayscale image: 

.. list-table::

  * - .. image:: images/result-image-io-input.png

    - .. image:: images/result-image-io-output.png


The input and output images are available in:

.. code-block:: text

   $VISP_WS/visp/modules/python/examples/image

Explanation
---------------------------

We first import the classes required to create, convert and display images:

.. code-block:: python

	import sys

	from visp.core import ImageGray, ImageRGBa, ImageConvert
	from visp.core import Display
	from visp.python.display_utils import get_display

We also import `ImageIo <file:///home/sjourdro/visp-ws/visp-build-bindings/doc/python/_autosummary/visp.io.ImageIo.html>`_, which we use to read and write image files:

.. code-block:: python

	from visp.io import ImageIo

We define a helper function to display the images that we manipulate:

.. note::

  For a more detailed explanation of this function, see the `Display an image in a window <tutorial-image-display.html>`_ tutorial.

.. code-block:: python

	def display(I, title):
	  d = get_display()
	  d.init(I)
	  Display.setTitle(I, title)
	  Display.display(I)
	  Display.flush(I)

	  print("A click to quit...")
	  Display.getClick(I)

We then read the image ``monkey.jpeg`` using the `ImageIo.read() <file:///home/sjourdro/visp-ws/visp-build-bindings/doc/python/_autosummary/visp.io.ImageIo.html#visp.io.ImageIo.read>`_ method.

We use an exception handler so that we can stop the program in case the reading fails because the remaining operations require a valid input image:

.. code-block:: python

	inputPath = sys.path[0] + "/monkey.jpeg"
	I = ImageRGBa()
	try:
	  ImageIo.read(I, inputPath)
	  print(f"Successfully loaded image: {inputPath}")
	except Exception as e:
	  print(e)
	  sys.exit()

We display the loaded image with the helper function defined previously:

.. code-block:: python

	display(I, "Loaded image")

We convert the input image into a grayscale image with:

.. code-block:: python

	Igray = ImageGray()
	ImageConvert.convert(I, Igray)

We write the grayscale image to the ``grayscale_monkey.jpeg`` file with `ImageIo.write() <file:///home/sjourdro/visp-ws/visp-build-bindings/doc/python/_autosummary/visp.io.ImageIo.html#visp.io.ImageIo.write>`_.

The exception handler is non-blocking this time, because further processing does not depend on the output file:

.. code-block:: python

	outputPath = sys.path[0] + "/grayscale_monkey.jpeg"
	try:
	  ImageIo.write(Igray, outputPath)
	  print(f"Image successfully written to '{outputPath}'")
	except Exception as e:
	  print(e)

Finally, we display the grayscale image:

.. code-block:: python

	display(Igray, "Written image")

Next Tutorial
===========================

You are now ready to learn how to `Insert basic drawings in an image <tutorial-image-drawings.html>`_.