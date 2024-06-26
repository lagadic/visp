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
 * Write videos and sequences of images.
 */

/*!
  \file vpVideoWriter.h
  \brief Write videos and sequences of images.
*/

#ifndef VP_VIDEO_WRITER_H
#define VP_VIDEO_WRITER_H

#include <string>

#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

BEGIN_VISP_NAMESPACE

/*!
  \class vpVideoWriter

  \ingroup group_io_video

  \brief Class that enables to write easily a video file or a sequence of images.

  This class has its own implementation to write a sequence of PGM and PPM images.

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
  file.

  \include tutorial-video-recorder.cpp

  The following example shows also how this class can be used to write an
  image sequence. The images are stored in the folder "./image" and are named
  "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...

  \code
  #include <visp3/core/vpConfig.h>
  #include <visp3/io/vpVideoWriter.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI)
    vpImage<vpRGBa> I;

    vpVideoWriter writer;

    // Set up the framerate to 30Hz. Default is 25Hz.
    writer.setFramerate(30);

  #if VISP_HAVE_OPENCV_VERSION >= 0x030000
    writer.setCodec(cv::VideoWriter::fourcc('P', 'I', 'M', '1'));
  #else
    writer.setCodec(CV_FOURCC('P', 'I', 'M', '1'));
  #endif

    writer.setFileName("./test.mpeg");

    writer.open(I);

    for (; ; ) {
      // Here the code to capture or create an image and store it in I.

      // Save the image
      writer.saveFrame(I);
    }

    writer.close();
  #endif
    return 0;
  }
  \endcode
*/

class VISP_EXPORT vpVideoWriter
{
private:
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  cv::VideoWriter m_writer;
  int m_fourcc;
  double m_framerate;
#endif
  //! Types of available formats
  typedef enum
  {
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

  //! Video's format which has to be written
  vpVideoFormatType m_formatType;

  //! Path to the video or image sequence
  std::string m_videoName;
  std::string m_frameName;

  //! Indicates if the path to the image sequence is set.
  bool m_initFileName;

  //! Indicates if the video is "open".
  bool m_isOpen;

  //! Count the frame number.
  int m_frameCount;

  //! The first frame index.
  int m_firstFrame;

  //! Size of the frame
  unsigned int m_width;
  unsigned int m_height;

  int m_frameStep;

public:
  vpVideoWriter();
  virtual ~vpVideoWriter();

  void close();

  /*!
    Gets the current frame index.

    \return Returns the current frame index.
  */
  inline unsigned int getCurrentFrameIndex() const { return m_frameCount; }
  /*!
   * Return the name of the file in which the last frame was saved.
   */
  inline std::string getFrameName() const { return m_frameName; }

  void open(vpImage<vpRGBa> &I);
  void open(vpImage<unsigned char> &I);
  /*!
    Reset the frame counter and sets it to the first image index.

    By default the first frame index is set to 0.
  */
  inline void resetFrameCounter() { m_frameCount = m_firstFrame; }

  void saveFrame(vpImage<vpRGBa> &I);
  void saveFrame(vpImage<unsigned char> &I);

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  inline void setCodec(const int fourcc_codec) { m_fourcc = fourcc_codec; }
#endif

  void setFileName(const std::string &filename);
  void setFirstFrameIndex(int first_frame);

  /*!
   * Sets the framerate in Hz of the video when encoding.
   *
   * \param framerate : The expected framerate.
   *
   * By default the framerate is set to 25Hz.
   *
   * \note Framerate can only be set when OpenCV > 2.1.0.
   */
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  inline void setFramerate(const double framerate) { m_framerate = framerate; }
#else
  inline void setFramerate(const double dummy) { (void)dummy; }
#endif
  /*!
   * Set frame step between 2 successive images when a sequence of images is considered.
   * \param frame_step : Step between 2 successive images. The default value is 1.
   */
  inline void setFrameStep(const int frame_step) { m_frameStep = frame_step; }

private:
  vpVideoFormatType getFormat(const std::string &filename);
  static std::string getExtension(const std::string &filename);
};

END_VISP_NAMESPACE

#endif
