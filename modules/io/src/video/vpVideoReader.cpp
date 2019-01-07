/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Read videos and image sequences.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
\file vpVideoReader.cpp
\brief Read videos and image sequences
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpVideoReader.h>

#include <cctype>
#include <fstream>
#include <iostream>
#include <limits> // numeric_limits

/*!
Basic constructor.
*/
vpVideoReader::vpVideoReader()
  : vpFrameGrabber(), imSequence(NULL),
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    capture(), frame(),
#endif
    formatType(FORMAT_UNKNOWN), initFileName(false), isOpen(false), frameCount(0), firstFrame(0), lastFrame(0),
    firstFrameIndexIsSet(false), lastFrameIndexIsSet(false), frameStep(1), frameRate(0.)
{
}

/*!
Basic destructor.
*/
vpVideoReader::~vpVideoReader()
{
  if (imSequence != NULL) {
    delete imSequence;
  }
}

/*!
It enables to set the path and the name of the file(s) which as/have to be
read.

If you want to read a video file, \f$ filename \f$ corresponds to the path to
the file (example : /local/video.mpeg).

If you want to read a sequence of images, \f$ filename \f$ corresponds to the
path followed by the image name template. For example, if you want to read
different images named image0001.jpeg, image0002.jpg, ... and located in the
folder /local/image, \f$ filename \f$ will be "/local/image/image%04d.jpg".

\param filename : Path to a video file or file name template of a image
sequence.
*/
void vpVideoReader::setFileName(const char *filename)
{
  if ((!filename) || (*filename == '\0')) {
    vpERROR_TRACE("filename empty ");
    throw(vpImageException(vpImageException::noFileNameError, "filename empty "));
  }

  if (strlen(filename) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError, "Not enough memory to initialize the file name"));
  }

  strcpy(this->fileName, filename);

  formatType = getFormat(fileName);

  if (formatType == FORMAT_UNKNOWN) {
    throw(vpException(vpException::badValue, "Filename extension not supported"));
  }

  // checking image name format
  if (isImageExtensionSupported()) {
    std::string format = vpIoTools::getName(fileName);
    if (!checkImageNameFormat(format)) {
      throw(vpException(vpException::badValue, "Format of image name wasn't recognized: %s", format.c_str()));
    }
  }

  initFileName = true;
}

/*!
It enables to set the path and the name of the file(s) which as/have to be
read.

If you want to read a video file, \f$ filename \f$ corresponds to the path to
the file (example : /local/video.mpeg).

If you want to read a sequence of images, \f$ filename \f$ corresponds to the
path followed by the image name template. For example, if you want to read
different images named image0001.jpeg, image0002.jpg, ... and located in the
folder /local/image, \f$ filename \f$ will be "/local/image/image%04d.jpg".

\param filename : Path to a video file or file name template of a image
sequence.
*/
void vpVideoReader::setFileName(const std::string &filename) { setFileName(filename.c_str()); }

/*!
  Open video stream and get first and last frame indexes.
*/
void vpVideoReader::getProperties()
{
  if (!initFileName) {
    throw(vpImageException(vpImageException::noFileNameError, "The generic filename has to be set"));
  }

  if (isImageExtensionSupported()) {
    imSequence = new vpDiskGrabber;
    imSequence->setGenericName(fileName);
    imSequence->setStep(frameStep);
    if (firstFrameIndexIsSet) {
      imSequence->setImageNumber(firstFrame);
    }
    frameRate = -1.;
  } else if (isVideoExtensionSupported()) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    capture.open(fileName);

    if (!capture.isOpened()) {
      throw(vpException(vpException::ioError, "Could not open the video %s with OpenCV", fileName));
    }
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    width = (unsigned int)capture.get(cv::CAP_PROP_FRAME_WIDTH);
    height = (unsigned int)capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    frameRate = (double)capture.get(cv::CAP_PROP_FPS);
#else
    width = (unsigned int)capture.get(CV_CAP_PROP_FRAME_WIDTH);
    height = (unsigned int)capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    frameRate = capture.get(CV_CAP_PROP_FPS);
#endif

#else
    throw(vpException(vpException::fatalError, "To read video files ViSP should be build with opencv "
                                               "3rd >= 2.1.0 party libraries."));
#endif
  } else if (formatType == FORMAT_UNKNOWN) {
    // vpERROR_TRACE("The format of the file does not correspond to a readable
    // format.");
    throw(vpException(vpException::fatalError, "The format of the file does "
                                               "not correspond to a readable "
                                               "format supported by ViSP."));
  }

  findFirstFrameIndex();
  isOpen = true;
  findLastFrameIndex();
}

/*!
Sets all the parameters needed to read the video or the image sequence.

Grab the first frame and stores it in the image \f$ I \f$.

\param I : The image where the frame is stored.
*/
void vpVideoReader::open(vpImage<vpRGBa> &I)
{
  getProperties();

  frameCount = firstFrame;
  if (!getFrame(I, firstFrame)) {
    throw(vpException(vpException::ioError, "Could not read the video first frame"));
  }

  // Rewind to the first frame since open() should not increase the frame
  // counter
  frameCount = firstFrame;

  if (isVideoExtensionSupported()) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    capture.set(cv::CAP_PROP_POS_FRAMES, firstFrame - 1);
#else
    capture.set(CV_CAP_PROP_POS_FRAMES, firstFrame - 1);
#endif
    frameCount--;
#endif
  }
}

/*!
Sets all the parameters needed to read the video or the image sequence.

Grab the first frame and stores it in the image \f$ I \f$.

\param I : The image where the frame is stored.
*/
void vpVideoReader::open(vpImage<unsigned char> &I)
{
  getProperties();

  frameCount = firstFrame;
  if (!getFrame(I, firstFrame)) {
    throw(vpException(vpException::ioError, "Could not read the video first frame"));
  }

  // Rewind to the first frame since open() should not increase the frame
  // counter
  frameCount = firstFrame;

  if (isVideoExtensionSupported()) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    capture.set(cv::CAP_PROP_POS_FRAMES, firstFrame - 1);
#else
    capture.set(CV_CAP_PROP_POS_FRAMES, firstFrame - 1);
#endif
    frameCount--;
#endif
  }
}

/*!
Grabs the current (k) image in the stack of frames and increments the frame
counter in order to grab the next image (k+1) during the next use of the
method. If open() was not called previously, this method opens the video
reader.

This method enables to use the class as frame grabber.

\param I : The image where the frame is stored.
*/
void vpVideoReader::acquire(vpImage<vpRGBa> &I)
{
  if (!isOpen) {
    open(I);
  }

  // getFrame(I,frameCount);
  if (imSequence != NULL) {
    imSequence->setStep(frameStep);
    imSequence->acquire(I);
    frameCount = imSequence->getImageNumber();
    if (frameCount + frameStep > lastFrame) {
      imSequence->setImageNumber(frameCount);
    } else if (frameCount + frameStep < firstFrame) {
      imSequence->setImageNumber(frameCount);
    }
  }
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  else {
    capture >> frame;
    if (frameStep == 1) {
      frameCount++;
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      frameCount = (long)capture.get(cv::CAP_PROP_POS_FRAMES);
      if (frameStep > 0) {
        if (frameCount + frameStep <= lastFrame) {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount - 1);
        }
      } else if (frameStep < 0) {
        if (frameCount + frameStep >= firstFrame) {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(cv::CAP_PROP_POS_FRAMES, firstFrame - 1);
        }
      }
#else
      frameCount = (long)capture.get(CV_CAP_PROP_POS_FRAMES);
      if (frameStep > 0) {
        if (frameCount + frameStep <= lastFrame) {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount - 1);
        }
      } else if (frameStep < 0) {
        if (frameCount + frameStep >= firstFrame) {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(CV_CAP_PROP_POS_FRAMES, firstFrame - 1);
        }
      }
#endif
    }

    if (frame.empty())
      setLastFrameIndex(frameCount - frameStep);
    else
      vpImageConvert::convert(frame, I);
  }
#endif
}

/*!
Grabs the kth image in the stack of frames and increments the frame counter in
order to grab the next image (k+1) during the next use of the method.

This method enables to use the class as frame grabber.

\param I : The image where the frame is stored.
*/
void vpVideoReader::acquire(vpImage<unsigned char> &I)
{
  if (!isOpen) {
    open(I);
  }

  if (imSequence != NULL) {
    imSequence->setStep(frameStep);
    imSequence->acquire(I);
    frameCount = imSequence->getImageNumber();
    if (frameCount + frameStep > lastFrame) {
      imSequence->setImageNumber(frameCount);
    } else if (frameCount + frameStep < firstFrame) {
      imSequence->setImageNumber(frameCount);
    }
  }
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  else {
    capture >> frame;
    if (frameStep == 1) {
      frameCount++;
    } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      frameCount = (long)capture.get(cv::CAP_PROP_POS_FRAMES);
      if (frameStep > 0) {
        if (frameCount + frameStep <= lastFrame) {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount - 1);
        }
      } else if (frameStep < 0) {
        if (frameCount + frameStep >= firstFrame) {
          capture.set(cv::CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(cv::CAP_PROP_POS_FRAMES, firstFrame - 1);
        }
      }
#else
      frameCount = (long)capture.get(CV_CAP_PROP_POS_FRAMES);
      if (frameStep > 0) {
        if (frameCount + frameStep <= lastFrame) {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount - 1);
        }
      } else if (frameStep < 0) {
        if (frameCount + frameStep >= firstFrame) {
          capture.set(CV_CAP_PROP_POS_FRAMES, frameCount + frameStep - 1);
        } else {
          capture.set(CV_CAP_PROP_POS_FRAMES, firstFrame - 1);
        }
      }
#endif
    }

    if (frame.empty())
      setLastFrameIndex(frameCount - frameStep);
    else
      vpImageConvert::convert(frame, I);
  }
#endif
}

/*!
Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.

\warning For the video files this method is not precise, and returns the
nearest key frame from the expected frame. But this method enables to position
the reader where you want. Then, use the acquire method to grab the following
images one after one.

\param I : The vpImage used to stored the frame.
\param frame_index : The index of the frame which has to be read.

\return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<vpRGBa> &I, long frame_index)
{
  if (imSequence != NULL) {
    try {
      imSequence->acquire(I, frame_index);
      width = I.getWidth();
      height = I.getHeight();
      frameCount = imSequence->getImageNumber();
      imSequence->setImageNumber(frameCount); // to not increment vpDiskGrabber next image
      if (frameCount + frameStep > lastFrame) {
        imSequence->setImageNumber(frameCount);
      } else if (frameCount + frameStep < firstFrame) {
        imSequence->setImageNumber(frameCount);
      }
    } catch (...) {
      vpERROR_TRACE("Couldn't find the %u th frame", frame_index);
      return false;
    }
  } else {
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    if (!capture.set(cv::CAP_PROP_POS_FRAMES, frame_index)) {
      vpERROR_TRACE("Couldn't find the %ld th frame", frame_index);
      return false;
    }

    capture >> frame;
    frameCount = frame_index + frameStep; // next index
    capture.set(cv::CAP_PROP_POS_FRAMES, frameCount);
    if (frame.empty()) {
      // New trial that makes things working with opencv 3.0.0
      capture >> frame;
      if (frame.empty()) {
        setLastFrameIndex(frameCount - frameStep);
        return false;
      } else {
        vpImageConvert::convert(frame, I);
      }
    } else
      vpImageConvert::convert(frame, I);
#elif defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
    if (!capture.set(CV_CAP_PROP_POS_FRAMES, frame_index)) {
      vpERROR_TRACE("Couldn't find the %ld th frame", frame_index);
      return false;
    }

    capture >> frame;
    frameCount = frame_index + frameStep; // next index
    capture.set(CV_CAP_PROP_POS_FRAMES, frameCount);
    if (frame.empty())
      setLastFrameIndex(frameCount - frameStep);
    else
      vpImageConvert::convert(frame, I);
#endif
  }
  return true;
}

/*!
Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.

\warning For the video files this method is not precise, and returns the
nearest key frame from the expected frame. But this method enables to position
the reader where you want. Then, use the acquire method to grab the following
images one after one.

\param I : The vpImage used to stored the frame.
\param frame_index : The index of the frame which has to be read.

\return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<unsigned char> &I, long frame_index)
{
  if (imSequence != NULL) {
    try {
      imSequence->acquire(I, frame_index);
      width = I.getWidth();
      height = I.getHeight();
      frameCount = imSequence->getImageNumber();
      imSequence->setImageNumber(frameCount); // to not increment vpDiskGrabber next image
      if (frameCount + frameStep > lastFrame) {
        imSequence->setImageNumber(frameCount);
      } else if (frameCount + frameStep < firstFrame) {
        imSequence->setImageNumber(frameCount);
      }
    } catch (...) {
      vpERROR_TRACE("Couldn't find the %u th frame", frame_index);
      return false;
    }
  } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    if (!capture.set(cv::CAP_PROP_POS_FRAMES, frame_index)) {
      vpERROR_TRACE("Couldn't find the %ld th frame", frame_index);
      return false;
    }
    capture >> frame;
    if (frame.empty()) {
      // New trial that makes things working with opencv 3.0.0
      capture >> frame;
      if (frame.empty()) {
        setLastFrameIndex(frameCount - frameStep);
        return false;
      } else {
        vpImageConvert::convert(frame, I);
      }
    } else {
      vpImageConvert::convert(frame, I);
    }
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    if (!capture.set(CV_CAP_PROP_POS_FRAMES, frame_index)) {
      vpERROR_TRACE("Couldn't find the %ld th frame",
                    frame_index); // next index
      return false;
    }
    capture >> frame;
    frameCount = (long)capture.get(CV_CAP_PROP_POS_FRAMES);
    if (frameStep > 1) {
      frameCount += frameStep - 1; // next index
      capture.set(CV_CAP_PROP_POS_FRAMES, frameCount);
    } else if (frameStep < -1) {
      frameCount += frameStep - 1; // next index
      capture.set(CV_CAP_PROP_POS_FRAMES, frameCount);
    }
    if (frame.empty())
      setLastFrameIndex(frameCount - frameStep);
    else
      vpImageConvert::convert(frame, I);
#endif
  }
  return true;
}

/*!
Gets the format of the file(s) which has/have to be read.

\return Returns the format.
*/
vpVideoReader::vpVideoFormatType vpVideoReader::getFormat(const char *filename)
{
  std::string sfilename(filename);

  std::string ext = vpVideoReader::getExtension(sfilename);

  if (ext.compare(".PGM") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".pgm") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".PPM") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".ppm") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".JPG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".JPEG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpeg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".PNG") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".png") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".TIFF") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".tiff") == 0)
    return FORMAT_TIFF;
  else if (ext.compare(".BMP") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".bmp") == 0)
    return FORMAT_BMP;
  else if (ext.compare(".DIB") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".dib") == 0)
    return FORMAT_DIB;
  else if (ext.compare(".PBM") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".pbm") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".SR") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".sr") == 0)
    return FORMAT_PBM;
  else if (ext.compare(".RAS") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".ras") == 0)
    return FORMAT_RASTER;
  else if (ext.compare(".JP2") == 0)
    return FORMAT_JPEG2000;
  else if (ext.compare(".jp2") == 0)
    return FORMAT_JPEG2000;
  else if (ext.compare(".AVI") == 0)
    return FORMAT_AVI;
  else if (ext.compare(".avi") == 0)
    return FORMAT_AVI;
  else if (ext.compare(".MPEG") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".mpeg") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".MPG") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".mpg") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".MPEG4") == 0)
    return FORMAT_MPEG4;
  else if (ext.compare(".mpeg4") == 0)
    return FORMAT_MPEG4;
  else if (ext.compare(".MP4") == 0)
    return FORMAT_MPEG4;
  else if (ext.compare(".mp4") == 0)
    return FORMAT_MPEG4;
  else if (ext.compare(".MOV") == 0)
    return FORMAT_MOV;
  else if (ext.compare(".mov") == 0)
    return FORMAT_MOV;
  else if (ext.compare(".OGV") == 0)
    return FORMAT_OGV;
  else if (ext.compare(".ogv") == 0)
    return FORMAT_OGV;
  else if (ext.compare(".WMV") == 0)
    return FORMAT_WMV;
  else if (ext.compare(".wmv") == 0)
    return FORMAT_WMV;
  else if (ext.compare(".FLV") == 0)
    return FORMAT_FLV;
  else if (ext.compare(".flv") == 0)
    return FORMAT_FLV;
  else if (ext.compare(".MKV") == 0)
    return FORMAT_MKV;
  else if (ext.compare(".mkv") == 0)
    return FORMAT_MKV;
  else
    return FORMAT_UNKNOWN;
}

// return the extension of the file including the dot
std::string vpVideoReader::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size() - 1);
  return ext;
}

/*!
Get the last frame index (update the lastFrame attribute).
*/
void vpVideoReader::findLastFrameIndex()
{
  if (!isOpen) {
    vpERROR_TRACE("Use the open method before");
    throw(vpException(vpException::notInitialized, "file not yet opened"));
  }

  if (imSequence != NULL) {
    if (!lastFrameIndexIsSet) {
      std::string imageNameFormat = vpIoTools::getName(std::string(fileName));
      std::string dirName = vpIoTools::getParent(std::string(fileName));
      if (dirName == "") {
        dirName = ".";
      }
      std::vector<std::string> files = vpIoTools::getDirFiles(dirName);
      lastFrame = 0;
      for (size_t i = 0; i < files.size(); i++) {
        // Checking that file name satisfies image format,
        // specified by imageNameFormat, and extracting imageIndex
        long imageIndex = extractImageIndex(files[i], imageNameFormat);
        if ((imageIndex != -1) && (imageIndex > lastFrame)) {
          lastFrame = imageIndex;
        }
      }
    }
  }

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
  else if (!lastFrameIndexIsSet) {
    lastFrame = (long)capture.get(cv::CAP_PROP_FRAME_COUNT);
    if (lastFrame <= 2) // with tutorial/matching/video-postcard.mpeg it
                        // return 2 with OpenCV 3.0.0
    {
      // std::cout << "Warning: Problem with cv::CAP_PROP_FRAME_COUNT. We set
      // video last frame to an arbitrary value (1000)." << std::endl;
      lastFrame = 100000; // Set lastFrame to an arbitrary value
    }
  }
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  else if (!lastFrameIndexIsSet) {
    lastFrame = (long)capture.get(CV_CAP_PROP_FRAME_COUNT);
    if (lastFrame <= 2) // with tutorial/matching/video-postcard.mpeg it
                        // return 2 with OpenCV 2.4.10
    {
      // std::cout << "Warning: Problem with CV_CAP_PROP_FRAME_COUNT. We set
      // video last frame to an arbitrary value (1000)." << std::endl;
      lastFrame = 100000; // Set lastFrame to an arbitrary value
    }
  }
#endif
}

/*!
Get the first frame index (update the firstFrame attribute).
*/
void vpVideoReader::findFirstFrameIndex()
{
  if (imSequence != NULL) {
    if (!firstFrameIndexIsSet) {
      std::string imageNameFormat = vpIoTools::getName(std::string(fileName));
      std::string dirName = vpIoTools::getParent(std::string(fileName));
      if (dirName == "") {
        dirName = ".";
      }
      std::vector<std::string> files = vpIoTools::getDirFiles(dirName);
      firstFrame = -1;
      for (size_t i = 0; i < files.size(); i++) {
        // Checking that file name satisfies image format, specified by
        // imageNameFormat, and extracting imageIndex
        long imageIndex = extractImageIndex(files[i], imageNameFormat);
        if ((imageIndex != -1) && (imageIndex < firstFrame || firstFrame == -1)) {
          firstFrame = imageIndex;
        }
      }
      imSequence->setImageNumber(firstFrame);
    }
  }
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  else if (!firstFrameIndexIsSet) {
    firstFrame = 1L;
  }
#endif
}

/*!
Return true if the image file extension is supported, false otherwise.
*/
bool vpVideoReader::isImageExtensionSupported()
{
  return (formatType == FORMAT_PGM || formatType == FORMAT_PPM || formatType == FORMAT_JPEG ||
          formatType == FORMAT_PNG || formatType == FORMAT_TIFF || formatType == FORMAT_BMP ||
          formatType == FORMAT_DIB || formatType == FORMAT_PBM || formatType == FORMAT_RASTER ||
          formatType == FORMAT_JPEG2000);
}

/*!
Return true if the video file extension is supported, false otherwise.
*/
bool vpVideoReader::isVideoExtensionSupported()
{
  return (formatType == FORMAT_AVI || formatType == FORMAT_MPEG || formatType == FORMAT_MPEG4 ||
          formatType == FORMAT_MOV || formatType == FORMAT_OGV || formatType == FORMAT_WMV ||
          formatType == FORMAT_FLV || formatType == FORMAT_MKV);
}

/*!

   Operator that allows to capture a grey level image.
   \param I : The captured image.

   \code
#include <visp3/io/vpVideoReader.h>

int main()
{
  vpImage<unsigned char> I;
  vpVideoReader reader;

  // Initialize the reader.
  reader.setFileName("./image/image%04d.jpeg");
  reader.setFirstFrameIndex(10);
  reader.setLastFrameIndex(20);
  reader.open(I);

  while (! reader.end() )
  reader >> I;
}
   \endcode
 */
vpVideoReader &vpVideoReader::operator>>(vpImage<unsigned char> &I)
{
  this->acquire(I);
  return *this;
}

/*!

   Operator that allows to capture a grey level image.
   \param I : The captured image.

   \code
#include <visp3/io/vpVideoReader.h>

int main()
{
  vpImage<vpRGBa> I;
  vpVideoReader reader;

  // Initialize the reader.
  reader.setFileName("./image/image%04d.jpeg");
  reader.setFirstFrameIndex(10);
  reader.setLastFrameIndex(20);
  reader.open(I);

  while (! reader.end() )
  reader >> I;
}
   \endcode
 */
vpVideoReader &vpVideoReader::operator>>(vpImage<vpRGBa> &I)
{
  this->acquire(I);
  return *this;
}

/*!
  Checks imageName format and extracts its index.

  Format must contain substring "%0xd", defining the length of image index.
  For example, format can be "img%04d.jpg". Then "img0001.jpg" and
  "img0000.jpg" satisfy it, while "picture001.jpg" and "img001.jpg" don't.

  \param imageName : name from which to extract
  \param format : format of image name
  \return extracted index on success, -1 otherwise.
*/
long vpVideoReader::extractImageIndex(const std::string &imageName, const std::string &format)
{
  size_t indexBegin = format.find_last_of('%');
  size_t indexEnd = format.find_first_of('d', indexBegin);
  size_t suffixLength = format.length() - indexEnd - 1;

  // Extracting index
  if (imageName.length() <= suffixLength + indexBegin) {
    return -1;
  }
  size_t indexLength = imageName.length() - suffixLength - indexBegin;
  std::string indexSubstr = imageName.substr(indexBegin, indexLength);
  std::istringstream ss(indexSubstr);
  long index = 0;
  ss >> index;
  if (ss.fail() || index < 0 || !ss.eof()) {
    return -1;
  }

  // Checking that format with inserted index equals imageName
  char nameByFormat[FILENAME_MAX];
  sprintf(nameByFormat, format.c_str(), index);
  if (std::string(nameByFormat) != imageName) {
    return -1;
  }
  return index;
}

/*!
  Checks image name template, for example "img%04d.jpg"
  \return true if it is correct, false otherwise
*/
bool vpVideoReader::checkImageNameFormat(const std::string &format)
{
  size_t indexBegin = format.find_last_of('%');
  size_t indexEnd = format.find_first_of('d', indexBegin);
  if (indexBegin == std::string::npos || indexEnd == std::string::npos) {
    return false;
  }
  for (size_t i = indexBegin + 1; i < indexEnd; i++) {
    if (!std::isdigit(format[i])) {
      return false;
    }
  }
  return true;
}
