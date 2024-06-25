/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpVideoReader.cpp
 * \brief Read videos and image sequences
 */

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpVideoReader.h>

#include <cctype>
#include <fstream>
#include <iostream>

BEGIN_VISP_NAMESPACE

/*!
  Basic constructor.
*/
vpVideoReader::vpVideoReader()
  : vpFrameGrabber(), m_imSequence(nullptr),
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  m_capture(), m_frame(), m_lastframe_unknown(false),
#endif
  m_formatType(FORMAT_UNKNOWN), m_videoName(), m_frameName(), m_initFileName(false), m_isOpen(false), m_frameCount(0),
  m_firstFrame(0), m_lastFrame(0), m_firstFrameIndexIsSet(false), m_lastFrameIndexIsSet(false), m_frameStep(1),
  m_frameRate(0.)
{ }

/*!
Basic destructor.
*/
vpVideoReader::~vpVideoReader()
{
  if (m_imSequence != nullptr) {
    delete m_imSequence;
  }
}

/*!
  It enables to set the path and the name of the video which has to be
  read.

  If you want to read a video file, `filename` corresponds to the path to
  the file (e.g.: `/local/video.mpeg`).

  If you want to read a sequence of images, `filename` corresponds rather to the
  path followed by the image name template. For example, if you want to read
  different images named `image0001.jpeg`, `image0002.jpg`... and located in the
  folder `/local/image`, `filename` will be `/local/image/image%04d.jpg`.

  \param filename : Path to a video file or file name template of a image
  sequence.
*/
void vpVideoReader::setFileName(const std::string &filename)
{
  if (filename.empty()) {
    throw(vpImageException(vpImageException::noFileNameError, "filename empty "));
  }

  m_videoName = filename;
  m_frameName = filename;

  m_formatType = getFormat(filename);

  if (m_formatType == FORMAT_UNKNOWN) {
    throw(vpException(vpException::badValue, "Filename extension not supported"));
  }

  // checking image name format
  if (isImageExtensionSupported()) {
    std::string format = vpIoTools::getName(m_videoName);
    if (!checkImageNameFormat(format)) {
      throw(vpException(vpException::badValue, "Format of image name wasn't recognized: %s", format.c_str()));
    }
  }

  m_initFileName = true;
}

/*!
  Open video stream and get first and last frame indexes.
*/
void vpVideoReader::getProperties()
{
  if (m_isOpen) {
    return;
  }

  if (!m_initFileName) {
    throw(vpImageException(vpImageException::noFileNameError, "The generic filename has to be set"));
  }

  if (isImageExtensionSupported()) {
    m_imSequence = new vpDiskGrabber;
    m_imSequence->setGenericName(m_videoName.c_str());
    m_imSequence->setStep(m_frameStep);
    if (m_firstFrameIndexIsSet) {
      m_imSequence->setImageNumber(m_firstFrame);
    }
    m_frameRate = -1.;
  }
  else if (isVideoExtensionSupported()) {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
    m_capture.open(m_videoName.c_str());

    if (!m_capture.isOpened()) {
      throw(vpException(vpException::ioError, "Could not open the video %s with OpenCV", m_videoName.c_str()));
    }
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    width = (unsigned int)m_capture.get(cv::CAP_PROP_FRAME_WIDTH);
    height = (unsigned int)m_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
    m_frameRate = (double)m_capture.get(cv::CAP_PROP_FPS);
#else
    width = (unsigned int)m_capture.get(CV_CAP_PROP_FRAME_WIDTH);
    height = (unsigned int)m_capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    m_frameRate = m_capture.get(CV_CAP_PROP_FPS);
#endif

#else
    throw(vpException(vpException::fatalError, "To read video files ViSP should be build with opencv "
                      "3rd >= 2.1.0 party libraries."));
#endif
  }
  else if (m_formatType == FORMAT_UNKNOWN) {
    throw(vpException(vpException::fatalError, "The format of the file does "
                      "not correspond to a readable "
                      "format supported by ViSP."));
  }

  findFirstFrameIndex();
  m_isOpen = true;
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

  m_frameCount = m_firstFrame;
  if (!getFrame(I, m_firstFrame)) {
    throw(vpException(vpException::ioError, "Could not read the video first frame"));
  }

  // Rewind to the first frame since open() should not increase the frame
  // counter
  m_frameCount = m_firstFrame;

  if (isVideoExtensionSupported()) {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    m_capture.set(cv::CAP_PROP_POS_FRAMES, m_firstFrame - 1);
#else
    m_capture.set(CV_CAP_PROP_POS_FRAMES, m_firstFrame - 1);
#endif
    m_frameCount--;
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

  m_frameCount = m_firstFrame;
  if (!getFrame(I, m_firstFrame)) {
    throw(vpException(vpException::ioError, "Could not read the video first frame"));
  }

  // Rewind to the first frame since open() should not increase the frame
  // counter
  m_frameCount = m_firstFrame;

  if (isVideoExtensionSupported()) {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)

#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    m_capture.set(cv::CAP_PROP_POS_FRAMES, m_firstFrame - 1);
#else
    m_capture.set(CV_CAP_PROP_POS_FRAMES, m_firstFrame - 1);
#endif
    m_frameCount--;
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
  if (!m_isOpen) {
    open(I);
  }
  if (m_imSequence != nullptr) {
    m_imSequence->setStep(m_frameStep);
    bool skip_frame = false;
    do {
      try {
        m_imSequence->acquire(I);
        skip_frame = false;
      }
      catch (...) {
        skip_frame = true;
      }
    } while (skip_frame && m_imSequence->getImageNumber() < m_lastFrame);
    m_frameCount = m_imSequence->getImageNumber();
    m_frameName = m_imSequence->getImageName();
    if (m_frameCount + m_frameStep > m_lastFrame) {
      m_imSequence->setImageNumber(m_frameCount);
    }
    else if (m_frameCount + m_frameStep < m_firstFrame) {
      m_imSequence->setImageNumber(m_frameCount);
    }
  }
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  else {
    m_capture >> m_frame;
    if (m_frameStep == 1) {
      ++m_frameCount;
    }
    else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      m_frameCount = (long)m_capture.get(cv::CAP_PROP_POS_FRAMES);
      if (m_frameStep > 0) {
        if (m_frameCount + m_frameStep <= m_lastFrame) {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount - 1);
        }
      }
      else if (m_frameStep < 0) {
        if (m_frameCount + m_frameStep >= m_firstFrame) {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_firstFrame - 1);
        }
      }
#else
      m_frameCount = (long)m_capture.get(CV_CAP_PROP_POS_FRAMES);
      if (m_frameStep > 0) {
        if (m_frameCount + m_frameStep <= m_lastFrame) {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount - 1);
        }
      }
      else if (m_frameStep < 0) {
        if (m_frameCount + m_frameStep >= m_firstFrame) {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_firstFrame - 1);
        }
      }
#endif
    }

    if (m_frame.empty()) {
      std::cout << "Warning: Unable to decode image " << m_frameCount - m_frameStep << std::endl;
      if (m_lastframe_unknown) {
        // Set last frame to this image index
        setLastFrameIndex(m_frameCount - m_frameStep);
      }
    }
    else {
      vpImageConvert::convert(m_frame, I);
    }
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
  if (!m_isOpen) {
    open(I);
  }

  if (m_imSequence != nullptr) {
    m_imSequence->setStep(m_frameStep);
    bool skip_frame = false;
    do {
      try {
        m_imSequence->acquire(I);
        skip_frame = false;
      }
      catch (...) {
        skip_frame = true;
      }
    } while (skip_frame && m_imSequence->getImageNumber() < m_lastFrame);
    m_frameCount = m_imSequence->getImageNumber();
    m_frameName = m_imSequence->getImageName();
    if (m_frameCount + m_frameStep > m_lastFrame) {
      m_imSequence->setImageNumber(m_frameCount);
    }
    else if (m_frameCount + m_frameStep < m_firstFrame) {
      m_imSequence->setImageNumber(m_frameCount);
    }
  }
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  else {
    m_capture >> m_frame;
    if (m_frameStep == 1) {
      ++m_frameCount;
    }
    else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
      m_frameCount = (long)m_capture.get(cv::CAP_PROP_POS_FRAMES);
      if (m_frameStep > 0) {
        if (m_frameCount + m_frameStep <= m_lastFrame) {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount - 1);
        }
      }
      else if (m_frameStep < 0) {
        if (m_frameCount + m_frameStep >= m_firstFrame) {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(cv::CAP_PROP_POS_FRAMES, m_firstFrame - 1);
        }
      }
#else
      m_frameCount = (long)m_capture.get(CV_CAP_PROP_POS_FRAMES);
      if (m_frameStep > 0) {
        if (m_frameCount + m_frameStep <= m_lastFrame) {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount - 1);
        }
      }
      else if (m_frameStep < 0) {
        if (m_frameCount + m_frameStep >= m_firstFrame) {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount + m_frameStep - 1);
        }
        else {
          m_capture.set(CV_CAP_PROP_POS_FRAMES, m_firstFrame - 1);
        }
      }
#endif
    }

    if (m_frame.empty()) {
      std::cout << "Warning: Unable to decode image " << m_frameCount - m_frameStep << std::endl;
    }
    else {
      vpImageConvert::convert(m_frame, I);
    }
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
  if (m_imSequence != nullptr) {
    try {
      m_imSequence->acquire(I, frame_index);
      width = I.getWidth();
      height = I.getHeight();
      m_frameCount = m_imSequence->getImageNumber();
      m_imSequence->setImageNumber(m_frameCount); // to not increment vpDiskGrabber next image
      if (m_frameCount + m_frameStep > m_lastFrame) {
        m_imSequence->setImageNumber(m_frameCount);
      }
      else if (m_frameCount + m_frameStep < m_firstFrame) {
        m_imSequence->setImageNumber(m_frameCount);
      }
    }
    catch (...) {
      // Couldn't find the %u th frame", frame_index
      return false;
    }
  }
  else {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    if (!m_capture.set(cv::CAP_PROP_POS_FRAMES, frame_index)) {
      // Couldn't find the %ld th frame", frame_index
      return false;
    }

    m_capture >> m_frame;
    m_frameCount = frame_index + m_frameStep; // next index
    m_capture.set(cv::CAP_PROP_POS_FRAMES, m_frameCount);
    if (m_frame.empty()) {
      // New trial that makes things working with opencv 3.0.0
      m_capture >> m_frame;
      if (m_frame.empty()) {
        setLastFrameIndex(m_frameCount - m_frameStep);
        return false;
      }
      else {
        vpImageConvert::convert(m_frame, I);
      }
    }
    else
      vpImageConvert::convert(m_frame, I);
#else
    if (!m_capture.set(CV_CAP_PROP_POS_FRAMES, frame_index)) {
      // Couldn't find the %ld th frame", frame_index
      return false;
    }

    m_capture >> m_frame;
    m_frameCount = frame_index + m_frameStep; // next index
    m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount);
    if (m_frame.empty())
      setLastFrameIndex(m_frameCount - m_frameStep);
    else
      vpImageConvert::convert(m_frame, I);
#endif
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
  if (m_imSequence != nullptr) {
    try {
      m_imSequence->acquire(I, frame_index);
      width = I.getWidth();
      height = I.getHeight();
      m_frameCount = m_imSequence->getImageNumber();
      m_imSequence->setImageNumber(m_frameCount); // to not increment vpDiskGrabber next image
      if (m_frameCount + m_frameStep > m_lastFrame) {
        m_imSequence->setImageNumber(m_frameCount);
      }
      else if (m_frameCount + m_frameStep < m_firstFrame) {
        m_imSequence->setImageNumber(m_frameCount);
      }
    }
    catch (...) {
      // Couldn't find the %u th frame", frame_index
      return false;
    }
  }
  else {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    if (!m_capture.set(cv::CAP_PROP_POS_FRAMES, frame_index)) {
      // Couldn't find the %ld th frame", frame_index
      return false;
    }
    m_capture >> m_frame;
    if (m_frame.empty()) {
      // New trial that makes things working with opencv 3.0.0
      m_capture >> m_frame;
      if (m_frame.empty()) {
        setLastFrameIndex(m_frameCount - m_frameStep);
        return false;
      }
      else {
        vpImageConvert::convert(m_frame, I);
      }
    }
    else {
      vpImageConvert::convert(m_frame, I);
    }
#else
    if (!m_capture.set(CV_CAP_PROP_POS_FRAMES, frame_index)) {
      // Couldn't find the %ld th frame", frame_index
      return false;
    }
    m_capture >> m_frame;
    m_frameCount = (long)m_capture.get(CV_CAP_PROP_POS_FRAMES);
    if (m_frameStep > 1) {
      m_frameCount += m_frameStep - 1; // next index
      m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount);
    }
    else if (m_frameStep < -1) {
      m_frameCount += m_frameStep - 1; // next index
      m_capture.set(CV_CAP_PROP_POS_FRAMES, m_frameCount);
    }
    if (m_frame.empty())
      setLastFrameIndex(m_frameCount - m_frameStep);
    else
      vpImageConvert::convert(m_frame, I);
#endif
#endif
  }
  return true;
}

/*!
  Gets the format of the file(s) which has/have to be read.

  \return Returns the format.
*/
vpVideoReader::vpVideoFormatType vpVideoReader::getFormat(const std::string &filename) const
{
  std::string ext = vpVideoReader::getExtension(filename);

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
  else if (ext.compare(".MTS") == 0)
    return FORMAT_MTS;
  else if (ext.compare(".mts") == 0)
    return FORMAT_MTS;
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
  if (!m_isOpen) {
    throw(vpException(vpException::notInitialized, "File not yet opened. Use the open() method before"));
  }

  if (m_imSequence != nullptr) {
    if (!m_lastFrameIndexIsSet) {
      std::string imageNameFormat = vpIoTools::getName(m_videoName);
      std::string dirName = vpIoTools::getParent(m_videoName);
      if (dirName == "") {
        dirName = ".";
      }
      std::vector<std::string> files = vpIoTools::getDirFiles(dirName);
      m_lastFrame = 0;
      for (size_t i = 0; i < files.size(); ++i) {
        // Checking that file name satisfies image format,
        // specified by imageNameFormat, and extracting imageIndex
        long imageIndex = vpIoTools::getIndex(files[i], imageNameFormat);
        if ((imageIndex != -1) && (imageIndex > m_lastFrame)) {
          m_lastFrame = imageIndex;
        }
      }
    }
  }

#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
  else if (!m_lastFrameIndexIsSet) {
    m_lastFrame = (long)m_capture.get(cv::CAP_PROP_FRAME_COUNT);
    if (m_lastFrame <= 2) {
      // With visp/tutorial/detection/matching/video-postcard.mpeg that is MPEG-2 it return 2 with OpenCV 3.0.0
      // With visp-images/video/cube.mpeg that is MPEG-1 it return 1 with OpenCV 4.1.1
      // We set video last frame to an arbitrary value 100000 and set a flag
      m_lastframe_unknown = true;
      m_lastFrame = 100000; // Set lastFrame to an arbitrary value
    }
  }
#else
  else if (!m_lastFrameIndexIsSet) {
    m_lastFrame = (long)m_capture.get(CV_CAP_PROP_FRAME_COUNT);
    if (m_lastFrame <= 2) {
      // With visp/tutorial/detection/matching/video-postcard.mpeg that is MPEG-2 it return 2 with OpenCV 3.0.0
      // With visp-images/video/cube.mpeg that is MPEG-1 it return 1 with OpenCV 4.1.1
      // We set video last frame to an arbitrary value 100000 and set a flag
      m_lastframe_unknown = true;
      m_lastFrame = 100000; // Set lastFrame to an arbitrary value
    }
  }
#endif
#endif
}

/*!
  Get the first frame index (update the firstFrame attribute).
*/
void vpVideoReader::findFirstFrameIndex()
{
  if (m_imSequence != nullptr) {
    if (!m_firstFrameIndexIsSet) {
      std::string imageNameFormat = vpIoTools::getName(m_videoName);
      std::string dirName = vpIoTools::getParent(m_videoName);
      if (dirName == "") {
        dirName = ".";
      }
      std::vector<std::string> files = vpIoTools::getDirFiles(dirName);
      m_firstFrame = -1;
      for (size_t i = 0; i < files.size(); ++i) {
        // Checking that file name satisfies image format, specified by
        // imageNameFormat, and extracting imageIndex
        long imageIndex = vpIoTools::getIndex(files[i], imageNameFormat);

        if ((imageIndex != -1) && (imageIndex < m_firstFrame || m_firstFrame == -1)) {
          m_firstFrame = imageIndex;
        }
      }
      m_imSequence->setImageNumber(m_firstFrame);
    }
  }
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)

  else if (!m_firstFrameIndexIsSet) {
    m_firstFrame = 1L;
  }
#endif
}

/*!
  Return true if the image file extension is supported, false otherwise.
*/
bool vpVideoReader::isImageExtensionSupported() const
{
  return (m_formatType == FORMAT_PGM || m_formatType == FORMAT_PPM || m_formatType == FORMAT_JPEG ||
          m_formatType == FORMAT_PNG || m_formatType == FORMAT_TIFF || m_formatType == FORMAT_BMP ||
          m_formatType == FORMAT_DIB || m_formatType == FORMAT_PBM || m_formatType == FORMAT_RASTER ||
          m_formatType == FORMAT_JPEG2000);
}

/*!
  Return true if the video file extension is supported, false otherwise.
*/
bool vpVideoReader::isVideoExtensionSupported() const
{
  return (m_formatType == FORMAT_AVI || m_formatType == FORMAT_MPEG || m_formatType == FORMAT_MPEG4 ||
          m_formatType == FORMAT_MOV || m_formatType == FORMAT_OGV || m_formatType == FORMAT_WMV ||
          m_formatType == FORMAT_FLV || m_formatType == FORMAT_MKV || m_formatType == FORMAT_MTS);
}

/*!
  Operator that allows to capture a grey level image.
  \param I : The captured image.

  \code
  #include <visp3/io/vpVideoReader.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  Checks image name template, for example "img%04d.jpg"
  \return true if it is correct, false otherwise
*/
bool vpVideoReader::checkImageNameFormat(const std::string &format) const
{
  size_t indexBegin = format.find_last_of('%');
  size_t indexEnd = format.find_first_of('d', indexBegin);
  if (indexBegin == std::string::npos || indexEnd == std::string::npos) {
    return false;
  }
  for (size_t i = indexBegin + 1; i < indexEnd; ++i) {
    if (!std::isdigit(format[i])) {
      return false;
    }
  }
  return true;
}

/*!
 * Indicate if the video is an encoded single video file.
 *
 * \return true if the video format corresponds to an encoded single video file like one of the following
 * avi, mpeg, mp4, mts, mov, ogv, wmv, flv, mkv. Return false, if the video is a sequence of successive
 * images (png, jpeg, ppm, pgm...).
 */
bool vpVideoReader::isVideoFormat() const
{
  // Video format
  switch (m_formatType) {
  case FORMAT_AVI:
  case FORMAT_MPEG:
  case FORMAT_MPEG4:
  case FORMAT_MTS:
  case FORMAT_MOV:
  case FORMAT_OGV:
  case FORMAT_WMV:
  case FORMAT_FLV:
  case FORMAT_MKV:
    return true;
  default:
    return false;
  }
}

END_VISP_NAMESPACE
