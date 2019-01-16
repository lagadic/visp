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
 * Write image sequences.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpVideoWriter.cpp
  \brief Write image sequences.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/io/vpVideoWriter.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020200
#include <opencv2/imgproc/imgproc.hpp>
#endif

/*!
  Basic constructor.
*/
vpVideoWriter::vpVideoWriter()
  :
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    writer(), fourcc(0), framerate(0.),
#endif
    formatType(FORMAT_UNKNOWN), initFileName(false), isOpen(false), frameCount(0), firstFrame(0), width(0), height(0)
{
  initFileName = false;
  firstFrame = 0;
  frameCount = 0;
  isOpen = false;
  width = height = 0;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
  framerate = 25.0;
  fourcc = cv::VideoWriter::fourcc('P', 'I', 'M', '1');
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
  framerate = 25.0;
  fourcc = CV_FOURCC('P', 'I', 'M', '1'); // default is a MPEG-1 codec
#endif
}

/*!
  Basic destructor.
*/
vpVideoWriter::~vpVideoWriter() {}

/*!
  It enables to set the path and the name of the files which will be saved.

  If you want to write a sequence of images, \f$ filename \f$ corresponds to
  the path followed by the image name template. For exemple, if you want to
  write different images named image0001.jpeg, image0002.jpg, ... and located
  in the folder /local/image, \f$ filename \f$ will be
  "/local/image/image%04d.jpg".

  \param filename : filename template of an image sequence.
*/
void vpVideoWriter::setFileName(const char *filename)
{
  if (!filename || *filename == '\0') {
    vpERROR_TRACE("filename empty ");
    throw(vpImageException(vpImageException::noFileNameError, "filename empty "));
  }

  if (strlen(filename) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError, "Not enough memory to intialize the file name"));
  }

  strcpy(this->fileName, filename);

  formatType = getFormat(fileName);

  if (formatType == FORMAT_UNKNOWN) {
    throw(vpException(vpException::badValue, "Filename extension not supported"));
  }

  initFileName = true;
}

/*!
  It enables to set the path and the name of the files which will be saved.

  If you want to write a sequence of images, \f$ filename \f$ corresponds to
  the path followed by the image name template. For exemple, if you want to
  write different images named image0001.jpeg, image0002.jpg, ... and located
  in the folder /local/image, \f$ filename \f$ will be
  "/local/image/image%04d.jpg".

  \param filename : filename template of an image sequence.
*/
void vpVideoWriter::setFileName(const std::string &filename) { setFileName(filename.c_str()); }

/*!
  Sets all the parameters needed to write the video or the image sequence.

  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage<vpRGBa> &I)
{
  if (!initFileName) {
    vpERROR_TRACE("The generic filename has to be set");
    throw(vpImageException(vpImageException::noFileNameError, "filename empty"));
  }

  if (formatType == FORMAT_PGM || formatType == FORMAT_PPM || formatType == FORMAT_JPEG || formatType == FORMAT_PNG) {
    width = I.getWidth();
    height = I.getHeight();
  } else if (formatType == FORMAT_AVI || formatType == FORMAT_MPEG || formatType == FORMAT_MPEG4 ||
             formatType == FORMAT_MOV) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    writer = cv::VideoWriter(fileName, fourcc, framerate, cv::Size((int)I.getWidth(), (int)I.getHeight()));

    if (!writer.isOpened()) {
      // vpERROR_TRACE("Could not open encode the video with opencv");
      throw(vpException(vpException::fatalError, "Could not open encode the video with opencv"));
    }
#else
    throw(vpException(vpException::fatalError, "To encode video files ViSP should be build with "
                                               "opencv 3rd >= 2.1.0 party libraries."));
#endif
  }

  frameCount = firstFrame;

  isOpen = true;
}

/*!
  Sets all the parameters needed to write the video or the image sequence.

  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage<unsigned char> &I)
{
  if (!initFileName) {
    vpERROR_TRACE("The generic filename has to be set");
    throw(vpImageException(vpImageException::noFileNameError, "filename empty"));
  }

  if (formatType == FORMAT_PGM || formatType == FORMAT_PPM || formatType == FORMAT_JPEG || formatType == FORMAT_PNG) {
    width = I.getWidth();
    height = I.getHeight();
  } else if (formatType == FORMAT_AVI || formatType == FORMAT_MPEG || formatType == FORMAT_MPEG4 ||
             formatType == FORMAT_MOV) {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    writer = cv::VideoWriter(fileName, fourcc, framerate, cv::Size((int)I.getWidth(), (int)I.getHeight()));

    if (!writer.isOpened()) {
      vpERROR_TRACE("Could not encode the video with opencv");
      throw(vpException(vpException::ioError, "Could not encode the video with opencv"));
    }
#else
    throw(vpException(vpException::fatalError, "To encode video files ViSP should be build with "
                                               "opencv 3rd >= 2.1.0 party libraries."));
#endif
  }

  frameCount = firstFrame;

  isOpen = true;
}

/*!
  Saves the image as a frame of the video or as an image belonging to the
  image sequence.

  Each time this method is used, the frame counter is incremented and thus the
  file name change for the case of an image sequence.

  \param I : The image which has to be saved
*/
void vpVideoWriter::saveFrame(vpImage<vpRGBa> &I)
{
  if (!isOpen) {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw(vpException(vpException::notInitialized, "file not yet opened"));
  }

  if (formatType == FORMAT_PGM || formatType == FORMAT_PPM || formatType == FORMAT_JPEG || formatType == FORMAT_PNG) {
    char name[FILENAME_MAX];

    sprintf(name, fileName, frameCount);

    vpImageIo::write(I, name);
  } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
    cv::Mat matFrame;
    vpImageConvert::convert(I, matFrame);
    writer << matFrame;
#endif
  }

  frameCount++;
}

/*!
  Saves the image as a frame of the video or as an image belonging to the
  image sequence.

  Each time this method is used, the frame counter is incremented and thus the
  file name change for the case of an image sequence.

  \param I : The image which has to be saved
*/
void vpVideoWriter::saveFrame(vpImage<unsigned char> &I)
{
  if (!isOpen) {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw(vpException(vpException::notInitialized, "file not yet opened"));
  }

  if (formatType == FORMAT_PGM || formatType == FORMAT_PPM || formatType == FORMAT_JPEG || formatType == FORMAT_PNG) {
    char name[FILENAME_MAX];

    sprintf(name, fileName, frameCount);

    vpImageIo::write(I, name);
  } else {
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    cv::Mat matFrame, rgbMatFrame;
    vpImageConvert::convert(I, matFrame);
    cv::cvtColor(matFrame, rgbMatFrame, cv::COLOR_GRAY2BGR);
    writer << rgbMatFrame;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    cv::Mat matFrame, rgbMatFrame;
    vpImageConvert::convert(I, matFrame);
    cv::cvtColor(matFrame, rgbMatFrame, CV_GRAY2BGR);
    writer << rgbMatFrame;
#endif
  }

  frameCount++;
}

/*!
  Deallocates parameters use to write the video or the image sequence.
*/
void vpVideoWriter::close()
{
  if (!isOpen) {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw(vpException(vpException::notInitialized, "file not yet opened"));
  }
}

/*!
  Gets the format of the file(s) which has/have to be written.

  \return Returns the format.
*/
vpVideoWriter::vpVideoFormatType vpVideoWriter::getFormat(const char *filename)
{
  std::string sfilename(filename);

  std::string ext = vpVideoWriter::getExtension(sfilename);

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
