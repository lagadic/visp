.. _tutorial-image-io:

============================
Read and write an image file
============================

Introduction
===========================

Goal
---------------------------

In this tutorial you will learn how to:

- Read an image from a file
- Write an image to a file.
- Use the :py:class:`~visp.io.ImageIo` class.

Prerequisites
---------------------------

You should first read the :ref:`Getting started with images <tutorial-image-getting-started>` tutorial.

Read and write an image file
============================

Code
---------------------------

The following :ref:`example <code-image-io>` reads the ``monkey.jpeg`` file, converts it into a grayscale image,
and saves the result as ``grayscale_monkey.jpeg``:

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

We first import the classes required to create, convert and display images.
We also import :py:class:`~visp.io.ImageIo`,
which we use to read and write image files:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :end-at: from visp.io import ImageIo

We define a helper function to display the images that we manipulate:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Function displaying an image
  :end-at:   Display.getClick(I)

.. note::

  For a more detailed explanation of how displays work in ViSP, see the :ref:`Display an image <tutorial-image-display>`
  tutorial.

We then read the image ``monkey.jpeg`` using the :py:meth:`~visp.io.ImageIo.read` method.
We use an exception handler so that we can stop the program in case the reading fails because the remaining operations
require a valid input image:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Read the image
  :end-at:   sys.exit()

We display the loaded image with the helper function defined previously:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Display the loaded image
  :end-at: display(I, "Loaded image")

We convert the input image into a grayscale image with:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Convert the image into a grayscale image
  :end-at: ImageConvert.convert(I, Igray)

We write the grayscale image to the ``grayscale_monkey.jpeg`` file with :py:meth:`~visp.io.ImageIo.write`.
The exception handler is non-blocking this time, because further processing does not depend on the output file:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Write the image
  :end-at:   print(e)


Finally, we display the grayscale image:

.. literalinclude:: /examples/image/tutorial-image-io.py
  :language: python
  :start-at: # Display the written image

Next Tutorial
===========================

You are now ready to learn how to :ref:`Insert basic drawings in an image <tutorial-image-drawings>`.
