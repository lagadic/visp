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
 * Write image sequences.
 */

/*!
  \file vpVideoWriter.cpp
  \brief Write image sequences.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpVideoWriter.h>

#if defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgproc/imgproc.hpp>
#endif

BEGIN_VISP_NAMESPACE

/*!
  Basic constructor.
*/
vpVideoWriter::vpVideoWriter()
  :
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  m_writer(), m_framerate(25.0),
#endif
  m_formatType(FORMAT_UNKNOWN), m_videoName(), m_frameName(), m_initFileName(false), m_isOpen(false), m_frameCount(0),
  m_firstFrame(0), m_width(0), m_height(0), m_frameStep(1)
{
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  m_fourcc = cv::VideoWriter::fourcc('P', 'I', 'M', '1');
#else
  m_fourcc = CV_FOURCC('P', 'I', 'M', '1'); // default is a MPEG-1 codec
#endif
#endif
}

/*!
  Basic destructor.
*/
vpVideoWriter::~vpVideoWriter() { }

/*!
  It enables to set the path and the name of the video or sequence of images
  which will be saved.

  If you want to write a sequence of images, `filename` corresponds to
  the path followed by the image name template. For example, if you want to
  write different images named `image0001.jpeg`, `image0002.jpg`, ... and located
  in the folder `/local/image`, `filename` will be `/local/image/image%04d.jpg`.

  \note The function open() will create recursively the folders to host the
  video or the sequence of images.

  \param filename : filename template of an image sequence.
*/
void vpVideoWriter::setFileName(const std::string &filename)
{
  if (filename.empty()) {
    throw(vpImageException(vpImageException::noFileNameError, "Filename empty in video writer"));
  }

  m_videoName = filename;
  m_frameName = filename;

  m_formatType = getFormat(filename);

  if (m_formatType == FORMAT_UNKNOWN) {
    throw(vpException(vpException::badValue, "Filename extension not supported in video writer"));
  }

  m_initFileName = true;
}

/*!
  Sets all the parameters needed to write the video or the image sequence.

  This function will also create recursively the folders to host the
  video or the sequence of images.

  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage<vpRGBa> &I)
{
  if (!m_initFileName) {
    throw(vpImageException(vpImageException::noFileNameError, "The generic filename has to be set in video writer"));
  }

  vpIoTools::makeDirectory(vpIoTools::getParent(m_videoName));

  if (m_formatType == FORMAT_PGM || m_formatType == FORMAT_PPM || m_formatType == FORMAT_JPEG ||
      m_formatType == FORMAT_PNG) {
    m_width = I.getWidth();
    m_height = I.getHeight();
  }
  else if (m_formatType == FORMAT_AVI || m_formatType == FORMAT_MPEG || m_formatType == FORMAT_MPEG4 ||
          m_formatType == FORMAT_MOV) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
    m_writer = cv::VideoWriter(m_videoName, m_fourcc, m_framerate,
                               cv::Size(static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight())));

    if (!m_writer.isOpened()) {
      throw(vpException(vpException::fatalError, "Could not encode the video with OpenCV"));
    }
#else
    throw(vpException(vpException::fatalError, "To encode video files ViSP should be build with OpenCV >= 2.1.0"));
#endif
  }

  m_frameCount = m_firstFrame;

  m_isOpen = true;
}

/*!
  Sets all the parameters needed to write the video or the image sequence.

  This function will also create recursively the folders to host the
  video or the sequence of images.

  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage<unsigned char> &I)
{
  if (!m_initFileName) {
    throw(vpImageException(vpImageException::noFileNameError, "The generic filename has to be set in video writer"));
  }

  vpIoTools::makeDirectory(vpIoTools::getParent(m_videoName));

  if (m_formatType == FORMAT_PGM || m_formatType == FORMAT_PPM || m_formatType == FORMAT_JPEG ||
      m_formatType == FORMAT_PNG) {
    m_width = I.getWidth();
    m_height = I.getHeight();
  }
  else if (m_formatType == FORMAT_AVI || m_formatType == FORMAT_MPEG || m_formatType == FORMAT_MPEG4 ||
          m_formatType == FORMAT_MOV) {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
    m_writer = cv::VideoWriter(m_videoName, m_fourcc, m_framerate,
                               cv::Size(static_cast<int>(I.getWidth()), static_cast<int>(I.getHeight())));

    if (!m_writer.isOpened()) {
      throw(vpException(vpException::fatalError, "Could not encode the video with OpenCV"));
    }
#else
    throw(vpException(vpException::fatalError, "To encode video files ViSP should be build with OpenCV >= 2.1.0"));
#endif
  }

  m_frameCount = m_firstFrame;

  m_isOpen = true;
}

/*!
  Saves the image as a frame of the video or as an image belonging to the
  image sequence.

  Each time this method is used, the frame counter is incremented and thus the
  file name change for the case of an image sequence.

  \param I : The image which has to be saved.
*/
void vpVideoWriter::saveFrame(vpImage<vpRGBa> &I)
{
  if (!m_isOpen) {
    throw(vpException(vpException::notInitialized, "The video has to be open first with video writer open() method"));
  }

  if (m_formatType == FORMAT_PGM || m_formatType == FORMAT_PPM || m_formatType == FORMAT_JPEG ||
      m_formatType == FORMAT_PNG) {
    char name[FILENAME_MAX];
    snprintf(name, FILENAME_MAX, m_videoName.c_str(), m_frameCount);
    vpImageIo::write(I, name);
    m_frameName = std::string(name);
  }
  else {
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
    cv::Mat matFrame;
    vpImageConvert::convert(I, matFrame);
    m_writer << matFrame;
#endif
  }

  m_frameCount += m_frameStep;
}

/*!
  Saves the image as a frame of the video or as an image belonging to the
  image sequence.

  Each time this method is used, the frame counter is incremented and thus the
  file name change for the case of an image sequence.

  \param I : The image which has to be saved.
*/
void vpVideoWriter::saveFrame(vpImage<unsigned char> &I)
{
  if (!m_isOpen) {
    throw(vpException(vpException::notInitialized, "The video has to be open first with video writer open() method"));
  }

  if (m_formatType == FORMAT_PGM || m_formatType == FORMAT_PPM || m_formatType == FORMAT_JPEG ||
      m_formatType == FORMAT_PNG) {
    char name[FILENAME_MAX];
    snprintf(name, FILENAME_MAX, m_videoName.c_str(), m_frameCount);
    vpImageIo::write(I, name);
    m_frameName = std::string(name);
  }
  else {
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    cv::Mat matFrame, rgbMatFrame;
    vpImageConvert::convert(I, matFrame);
    cv::cvtColor(matFrame, rgbMatFrame, cv::COLOR_GRAY2BGR);
    m_writer << rgbMatFrame;
#else
    cv::Mat matFrame, rgbMatFrame;
    vpImageConvert::convert(I, matFrame);
    cv::cvtColor(matFrame, rgbMatFrame, CV_GRAY2BGR);
    m_writer << rgbMatFrame;
#endif
#endif
  }

  m_frameCount += m_frameStep;
}

/*!
  Deallocates parameters use to write the video or the image sequence.
*/
void vpVideoWriter::close()
{
  if (!m_isOpen) {
    throw(vpException(vpException::notInitialized, "Cannot close video writer: not yet opened"));
  }
}

/*!
  Gets the format of the file(s) which has/have to be written.

  \return Returns the format.
*/
vpVideoWriter::vpVideoFormatType vpVideoWriter::getFormat(const std::string &filename)
{
  std::string ext = vpVideoWriter::getExtension(filename);

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
  else
    return FORMAT_UNKNOWN;
}

// return the extension of the file including the dot
std::string vpVideoWriter::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size() - 1);
  return ext;
}

/*!
  Enables to set the first frame index.

  \param first_frame : The first frame index. Value should be positive.
*/
void vpVideoWriter::setFirstFrameIndex(int first_frame)
{
  if (first_frame < 0) {
    throw(vpException(vpException::fatalError, "Video writer first frame index cannot be negative"));
  }
  m_firstFrame = first_frame;
}

END_VISP_NAMESPACE
