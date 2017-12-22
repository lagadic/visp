/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Write videos and sequences of images.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpVideoWriter.h
  \brief Write videos and sequences of images.
*/

#ifndef vpVideoWriter_H
#define vpVideoWriter_H

#include <string>

#include <visp3/io/vpImageIo.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020200
#include <opencv2/highgui/highgui.hpp>
#elif VISP_HAVE_OPENCV_VERSION >= 0x020000
#include <opencv/highgui.h>
#endif

/*!
  \class vpVideoWriter

  \ingroup group_io_video

  \brief Class that enables to write easily a video file or a sequence of
images.

  This class has its own implementation to write a sequence of PGM and PPM
images.

  This class may benefit from optional 3rd parties:
  - libpng: If installed this optional 3rd party is used to write a sequence
of PNG images. Installation instructions are provided here
https://visp.inria.fr/3rd_png.
  - libjpeg: If installed this optional 3rd party is used to write a sequence
of JPEG images. Installation instructions are provided here
https://visp.inria.fr/3rd_jpeg.
  - OpenCV: If installed this optional 3rd party is used to write a sequence
of images where images could be in TIFF, BMP, DIB, PBM, RASTER, JPEG2000
format. If libpng or libjpeg is not installed, OpenCV is also used to consider
these image formats. OpenCV allows also to consider AVI, MPEG, MPEG4, MOV,
OGV, WMV, FLV, MKV video formats. Installation instructions are provided here
https://visp.inria.fr/3rd_opencv.

  The following example available in tutorial-video-recorder.cpp shows how
this class can be used to record a video from a camera by default in an mpeg
file. \include tutorial-video-recorder.cpp

  The following example shows also how this class can be used to write an
image sequence. The images are stored in the folder "./image" and are named
"image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...

  \code
  #include <visp3/core/vpConfig.h>
  #include <visp3/io/vpVideoWriter.h>

  int main()
  {
  vpImage<vpRGBa> I;

  vpVideoWriter writer;

  //Initialize the writer.
  writer.setFileName("./image/image%04d.jpeg");

  writer.open(I);

  for ( ; ; )
  {
    //Here the code to capture or create an image and stores it in I.

    //Save the image
    writer.saveFrame(I);
  }

  writer.close();

  return 0;
  }
  \endcode

  The other following example explains how to use the class to write directly
an mpeg file.

  \code
#include <visp3/io/vpVideoWriter.h>

int main()
{
  vpImage<vpRGBa> I;

  vpVideoWriter writer;

  // Set up the framerate to 30Hz. Default is 25Hz.
  writer.setFramerate(30);

#if defined VISP_HAVE_OPENCV
  writer.setCodec( CV_FOURCC('P','I','M','1') );
#endif
  writer.setFileName("./test.mpeg");

  writer.open(I);

  for ( ; ; )
  {
    // Here the code to capture or create an image and store it in I.

    // Save the image
    writer.saveFrame(I);
  }

  writer.close();

  return 0;
}
  \endcode
*/

class VISP_EXPORT vpVideoWriter
{
private:
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  cv::VideoWriter writer;
  int fourcc;
  double framerate;
#endif
  //! Types of available formats
  typedef enum {
    FORMAT_PGM,
    FORMAT_PPM,
    FORMAT_JPEG,
    FORMAT_PNG,
    FORMAT_AVI,
    FORMAT_MPEG,
    FORMAT_MPEG4,
    FORMAT_MOV,
    FORMAT_UNKNOWN
  } vpVideoFormatType;

  //! Video's format which has to be writen
  vpVideoFormatType formatType;

  //! Path to the image sequence
  char fileName[FILENAME_MAX];

  //! Indicates if the path to the image sequence is set.
  bool initFileName;

  //! Indicates if the video is "open".
  bool isOpen;

  //! Count the frame number.
  unsigned int frameCount;

  //! The first frame index.
  unsigned int firstFrame;

  //! Size of the frame
  unsigned int width;
  unsigned int height;

public:
  vpVideoWriter();
  ~vpVideoWriter();

  void close();

  /*!
    Gets the current frame index.

    \return Returns the current frame index.
  */
  inline unsigned int getCurrentFrameIndex() const { return frameCount; }

  void open(vpImage<vpRGBa> &I);
  void open(vpImage<unsigned char> &I);
  /*!
    Reset the frame counter and sets it to the first image index.

    By default the first frame index is set to 0.
  */
  inline void resetFrameCounter() { frameCount = firstFrame; }

  void saveFrame(vpImage<vpRGBa> &I);
  void saveFrame(vpImage<unsigned char> &I);

#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  inline void setCodec(const int fourcc_codec) { this->fourcc = fourcc_codec; }
#endif

  void setFileName(const char *filename);
  void setFileName(const std::string &filename);
  /*!
    Enables to set the first frame index.

    \param first_frame : The first frame index.
  */
  inline void setFirstFrameIndex(const unsigned int first_frame) { this->firstFrame = first_frame; }
#if VISP_HAVE_OPENCV_VERSION >= 0x020100
  /*!
      Sets the framerate in Hz of the video when encoding.

      \param frame_rate : the expected framerate.

      By default the framerate is set to 25Hz.
    */
  inline void setFramerate(const double frame_rate) { this->framerate = frame_rate; }
#endif

private:
  vpVideoFormatType getFormat(const char *filename);
  static std::string getExtension(const std::string &filename);
};

#endif
