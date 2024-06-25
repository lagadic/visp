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
 * Read videos and sequences of images .
 */

/*!
  \file vpVideoReader.h
  \brief Read videos and image sequences
*/

#ifndef VP_VIDEO_READER_H
#define VP_VIDEO_READER_H

#include <string>

#include <visp3/io/vpDiskGrabber.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI)
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

BEGIN_VISP_NAMESPACE

/*!
 * \class vpVideoReader
 *
 * \ingroup group_io_video
 *
 * \brief Class that enables to manipulate easily a video file or a sequence of
 * images. As it inherits from the vpFrameGrabber Class, it can be used like an
 * other frame grabber class.
 *
 * This class has its own implementation to read a sequence of PGM and PPM
 * images.
 *
 * This class may benefit from optional 3rd parties:
 * - libpng: If installed this optional 3rd party is used to read a sequence of
 *   PNG images. Installation instructions are provided here
 *   https://visp.inria.fr/3rd_png.
 * - libjpeg: If installed this optional 3rd party is used to read a sequence
 *   of JPEG images. Installation instructions are provided here
 *   https://visp.inria.fr/3rd_jpeg.
 * - OpenCV: If installed this optional 3rd party is used to read a sequence of
 *   images where images could be in TIFF, BMP, DIB, PBM, RASTER, JPEG2000 format.
 *   If libpng or libjpeg is not installed, OpenCV is also used to consider these
 *   image formats. OpenCV allows also to consider AVI, MPEG, MPEG4, MOV, OGV, WMV,
 *   FLV, MKV video formats. Installation instructions are provided here
 *   https://visp.inria.fr/3rd_opencv.
 *
 * The following example available in tutorial-video-reader.cpp shows how this
 * class is really easy to use. It enables to read a video file named
 * video.mpeg.
 * \include tutorial-video-reader.cpp
 *
 * As shown in the next example, this class allows also to access to a specific
 * frame. But be careful, for video files, the getFrame() method is not precise
 * and returns the nearest intra key frame from the expected frame. You can use
 * the getFrame() method to position the reader in the video and then use the
 * acquire() method to get the following frames one by one.
 * \code
 * #include <visp3/io/vpVideoReader.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #ifdef VISP_HAVE_OPENCV
 *   vpImage<vpRGBa> I;
 *
 *   vpVideoReader reader;
 *
 *   // Initialize the reader.
 *   reader.setFileName("video.mpeg");
 *   reader.open(I);
 *
 *   // Read the nearest key frame from the 3th frame
 *   reader.getFrame(I, 2);
 *
 *   // After positioning the video reader use acquire to read the video frame by frame
 *   reader.acquire(I);
 *
 *   return 0;
 * #endif
 * }
 * \endcode
 *
 * The other following example explains how to use the class to read a
 * sequence of images. The images are stored in the folder "./image" and are
 * named "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ... As explained
 * in setFirstFrameIndex() and setLastFrameIndex() it is also possible to set
 * the first and last image numbers to read a portion of the sequence. If these
 * two functions are not used, first and last image numbers are set automatically
 * to match the first and image images of the sequence.
 *
 * \code
 * #include <visp3/io/vpVideoReader.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<vpRGBa> I;
 *
 *   vpVideoReader reader;
 *
 *   // Initialize the reader.
 *   reader.setFileName("./image/image%04d.jpeg");
 *   reader.setFirstFrameIndex(10);
 *   reader.setLastFrameIndex(20);
 *   reader.open(I);
 *
 *   while (! reader.end() )
 *     reader.acquire(I);
 *
 *   return 0;
 * }
 * \endcode
 *
 * Note that it is also possible to access to a specific frame using getFrame().
 * \code
 * #include <visp3/io/vpVideoReader.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpImage<vpRGBa> I;
 *
 *   vpVideoReader reader;
 *
 *   // Initialize the reader.
 *   reader.setFileName("./image/image%04d.jpeg");
 *   reader.open(I);
 *
 *   // Read the 3th frame
 *   reader.getFrame(I,2);
 *
 *   return 0;
 * }
 * \endcode
*/

class VISP_EXPORT vpVideoReader : public vpFrameGrabber
{
private:
  //! To read sequences of images
  vpDiskGrabber *m_imSequence;
#if defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_VIDEOIO)
  //! To read video files with OpenCV
  cv::VideoCapture m_capture;
  cv::Mat m_frame;
  bool m_lastframe_unknown;
#endif
  //! Types of available formats
  typedef enum
  {
    FORMAT_PGM,
    FORMAT_PPM,
    FORMAT_JPEG,
    FORMAT_PNG,
    // Formats supported by opencv
    FORMAT_TIFF,
    FORMAT_BMP,
    FORMAT_DIB,
    FORMAT_PBM,
    FORMAT_RASTER,
    FORMAT_JPEG2000,
    // Video format
    FORMAT_AVI,
    FORMAT_MPEG,
    FORMAT_MPEG4,
    FORMAT_MTS,
    FORMAT_MOV,
    FORMAT_OGV,
    FORMAT_WMV,
    FORMAT_FLV,
    FORMAT_MKV,
    FORMAT_UNKNOWN
  } vpVideoFormatType;

  //! Video's format which has to be read
  vpVideoFormatType m_formatType;

  //! Path to the video
  std::string m_videoName;
  std::string m_frameName;
  //! Indicates if the path to the video is set.
  bool m_initFileName;
  //! Indicates if the video is "open".
  bool m_isOpen;
  //! Count the frame number when the class is used as a grabber.
  long m_frameCount; // Index of the next image
  //! The first frame index
  long m_firstFrame;
  //! The last frame index
  long m_lastFrame;
  bool m_firstFrameIndexIsSet;
  bool m_lastFrameIndexIsSet;
  //! The frame step
  long m_frameStep;
  double m_frameRate;

public:
  vpVideoReader();
  virtual ~vpVideoReader();

  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<unsigned char> &I);
  void close() { ; }

  /*!
   * \return true if the end of the sequence is reached.
   */
  inline bool end()
  {
    if (m_frameStep > 0) {
      if (m_frameCount + m_frameStep > m_lastFrame)
        return true;
    }
    else if (m_frameStep < 0) {
      if (m_frameCount + m_frameStep < m_firstFrame)
        return true;
    }
    return false;
  }
  bool getFrame(vpImage<vpRGBa> &I, long frame);
  bool getFrame(vpImage<unsigned char> &I, long frame);

  /*!
   * Return the frame rate in Hz used to encode the video stream.
   *
   * If the video is a sequence of images, return -1.
   */
  double getFramerate()
  {
    if (!m_isOpen) {
      getProperties();
    }
    return m_frameRate;
  }

  /*!
   * Get the frame index of the current image. This index is updated at each
   * call of the acquire method. It can be used to detect the end of a file
   * (comparison with getLastFrameIndex()).
   *
   * \return Returns the current frame index.
   *
   * \sa end()
   */
  inline long getFrameIndex() const { return m_frameCount; }

  /*!
   * Return the name of the file in which the last frame was read.
   */
  inline std::string getFrameName() const { return m_frameName; }

  /*!
   * Gets the first frame index.
   *
   * \return Returns the first frame index.
   */
  inline long getFirstFrameIndex()
  {
    if (!m_isOpen) {
      getProperties();
    }
    return m_firstFrame;
  }

  /*!
   * Gets the last frame index.
   *
   * \return Returns the last frame index.
   */
  inline long getLastFrameIndex()
  {
    if (!m_isOpen) {
      getProperties();
    }
    return m_lastFrame;
  }

  /*!
   * Gets the frame step.
   *
   * \return Returns the frame step value.
   */
  inline long getFrameStep() const { return m_frameStep; }

  bool isVideoFormat() const;
  void open(vpImage<vpRGBa> &I);
  void open(vpImage<unsigned char> &I);

  vpVideoReader &operator>>(vpImage<unsigned char> &I);
  vpVideoReader &operator>>(vpImage<vpRGBa> &I);

  /*!
   * Reset the frame counter and sets it to the first image index.
   *
   * By default the first frame index is set to 0.
   *
   * This method is useful if you use the class like a frame grabber (ie with
   * the acquire method).
   */
  inline void resetFrameCounter() { m_frameCount = m_firstFrame; }
  void setFileName(const std::string &filename);

  /*!
   * Enables to set the first frame index if you want to use the class like a
   * grabber (ie with the acquire method).
   *
   * \param first_frame : The first frame index.
   *
   * \sa setLastFrameIndex()
   */
  inline void setFirstFrameIndex(const long first_frame)
  {
    m_firstFrameIndexIsSet = true;
    m_firstFrame = first_frame;
  }

  /*!
   * Enables to set the last frame index.
   *
   * \param last_frame : The last frame index.
   *
   * \sa setFirstFrameIndex()
   */
  inline void setLastFrameIndex(const long last_frame)
  {
    this->m_lastFrameIndexIsSet = true;
    m_lastFrame = last_frame;
  }

  /*!
   * Sets the frame step index.
   * The default frame step is 1
   *
   * \param frame_step : The frame index step.
   *
   * \sa setFrameStep()
   */
  inline void setFrameStep(const long frame_step) { m_frameStep = frame_step; }

private:
  vpVideoFormatType getFormat(const std::string &filename) const;
  static std::string getExtension(const std::string &filename);
  void findFirstFrameIndex();
  void findLastFrameIndex();
  bool isImageExtensionSupported() const;
  bool isVideoExtensionSupported() const;
  bool checkImageNameFormat(const std::string &format) const;
  void getProperties();
};

END_VISP_NAMESPACE

#endif
