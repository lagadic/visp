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
 * Video for linux two framegrabber.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpV4l2Grabber.h
  \brief class for the Video For Linux 2 video device framegrabbing.
  \ingroup libdevice
*/

#ifndef vpV4l2Grabber_hh
#define vpV4l2Grabber_hh

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_V4L2

#include <libv4l2.h> // Video For Linux Two interface
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/videodev2.h> // Video For Linux Two interface

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpRect.h>

/*!
  \class vpV4l2Grabber

  \ingroup group_sensor_camera

  \brief Class that is a wrapper over the Video4Linux2 (V4L2) driver.

  Thus to be enabled, this class needs the optional V4L2 3rd party.
Installation instruction are provided here https://visp.inria.fr/3rd_v4l2.

  Information about Video4Linux can be found on
  http://linuxtv.org/v4lwiki/index.php/Main_Page

  This class was tested with a Pinnacle PCTV Studio/Rave board but
  also with the following webcams (Logitech QuickCam Vision Pro 9000,
  Logitech QuickCam Orbit AF, Logitech QuickCam IM (V-USB39), Dell latitude
E6400 internal webcam).

  If the grabbing fail with a webcam, it means probably that you don't
  have the read/write permission on the /dev/video%%d device. You can
  set the right permissions by "sudo chmod a+rw /dev/video*".

  If the grabbing fails when the camera is attached to a bttv PCI
  card, it means potentially that you have not configured the linux
  bttv kernel module according to your board.

  For that, depending on your linux distribution check the card id in
  - /usr/share/doc/kernel-doc-2.4.20/video4linux/bttv/CARDLIST
  - or
/usr/share/doc/kernel-doc-2.6.20/Documentation/video4linux/CARDLIST.bttv

  For example, the card id of a Pinnacle PCTV Studio/Rave board is 39.
  Once this id is determined, you have to set the bttv driver with, by adding
  \verbatim
  options bttv card=39
  \endverbatim
  in one of theses files :
  - /etc/modules.conf
  - or /etc/modprobe.conf

  This first example available in tutorial-grabber-v4l2.cpp shows how to grab
  and display images from an usb camera.
  \include tutorial-grabber-v4l2.cpp

  This other example shows how to use this grabber with an analogic camera
  attached to a bttv PCI card.
  \code
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpV4l2Grabber.h>

int main()
{
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> I;
  vpV4l2Grabber g;
  g.setInput(2);    // Input 2 on the board
  g.setFramerate(vpV4l2Grabber::framerate_25fps); //  25 fps
  g.setWidth(768);  // Acquired images are 768 width
  g.setHeight(576); // Acquired images are 576 height
  g.setNBuffers(3); // 3 ring buffers to ensure real-time acquisition
  g.open(I);        // Open the grabber

  g.acquire(I);     // Acquire a 768x576 grey image
  vpImageIo::write(I, "image.pgm"); // Save the image on the disk
#endif
}
  \endcode


  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes


  \sa vpFrameGrabber
*/
class VISP_EXPORT vpV4l2Grabber : public vpFrameGrabber
{
public:
  static const unsigned int DEFAULT_INPUT;
  static const unsigned int DEFAULT_SCALE;
  static const __u32 MAX_INPUTS;
  static const __u32 MAX_NORM;
  static const __u32 MAX_FORMAT;
  static const unsigned int MAX_CTRL;
  static const unsigned int MAX_BUFFERS;
  static const unsigned int FRAME_SIZE;

  /*! \enum vpV4l2FramerateType
    Frame rate type for capture.
  */
  typedef enum {
    framerate_50fps, //!< 50 frames per second
    framerate_25fps  //!< 25 frames per second
  } vpV4l2FramerateType;

  /*! \enum vpV4l2FrameFormatType
    Frame format type for capture.
  */
  typedef enum {
    V4L2_FRAME_FORMAT, /*!< a field only */
    V4L2_IMAGE_FORMAT  /*!< an interlaced image */
  } vpV4l2FrameFormatType;

  /*! \enum vpV4l2PixelFormatType
    Pixel format type for capture.
  */
  typedef enum {
    V4L2_GREY_FORMAT,  /*!< 8  Greyscale */
    V4L2_RGB24_FORMAT, /*!< 24  RGB-8-8-8 */
    V4L2_RGB32_FORMAT, /*!< 32  RGB-8-8-8-8 */
    V4L2_BGR24_FORMAT, /*!< 24  BGR-8-8-8 */
    V4L2_YUYV_FORMAT,  /*!< 16  YUYV 4:2:2  */
    V4L2_MAX_FORMAT
  } vpV4l2PixelFormatType;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct ng_video_fmt {
    unsigned int pixelformat; /* VIDEO_* */
    unsigned int width;
    unsigned int height;
    unsigned int bytesperline; /* zero for compressed formats */
  };

  struct ng_video_buf {
    struct ng_video_fmt fmt;
    size_t size;
    unsigned char *data;
    int refcount;
  };
#endif

  // private:
  //#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //  vpV4l2Grabber(const vpV4l2Grabber &)
  //    : fd(-1), device(), cap(), streamparm(), inp(NULL), std(NULL),
  //    fmt(NULL), ctl(NULL),
  //      fmt_v4l2(), fmt_me(), reqbufs(), buf_v4l2(NULL), buf_me(NULL),
  //      queue(0), waiton_cpt(0), index_buffer(0), m_verbose(false),
  //      m_nbuffers(3), field(0), streaming(false),
  //      m_input(vpV4l2Grabber::DEFAULT_INPUT),
  //      m_framerate(vpV4l2Grabber::framerate_25fps),
  //      m_frameformat(vpV4l2Grabber::V4L2_FRAME_FORMAT),
  //      m_pixelformat(vpV4l2Grabber::V4L2_YUYV_FORMAT)
  //  {
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!");
  //  }
  //  vpV4l2Grabber &operator=(const vpV4l2Grabber &){
  //    throw vpException(vpException::functionNotImplementedError,"Not
  //    implemented!"); return *this;
  //  }
  //#endif

public:
  vpV4l2Grabber();
  explicit vpV4l2Grabber(bool verbose);
  vpV4l2Grabber(unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  vpV4l2Grabber(vpImage<unsigned char> &I, unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  vpV4l2Grabber(vpImage<vpRGBa> &I, unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  virtual ~vpV4l2Grabber();

public:
  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<unsigned char> &I, const vpRect &roi);
  void acquire(vpImage<unsigned char> &I, struct timeval &timestamp, const vpRect &roi = vpRect());
  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<vpRGBa> &I, const vpRect &roi);
  void acquire(vpImage<vpRGBa> &I, struct timeval &timestamp, const vpRect &roi = vpRect());
  bool getField();
  vpV4l2FramerateType getFramerate();
  /*!

  Get the pixel format used for capture.

  \return Camera pixel format coding.

  */
  inline vpV4l2PixelFormatType getPixelFormat() { return (this->m_pixelformat); }

  vpV4l2Grabber &operator>>(vpImage<unsigned char> &I);
  vpV4l2Grabber &operator>>(vpImage<vpRGBa> &I);

  /*!
    Activates the verbose mode to print additional information on stdout.
    \param verbose : If true activates the verbose mode.
  */
  void setVerboseMode(bool verbose) { this->m_verbose = verbose; };
  void setFramerate(vpV4l2FramerateType framerate);

  void setInput(unsigned input = vpV4l2Grabber::DEFAULT_INPUT);

  /*!
    Set image width to acquire.

  */
  inline void setWidth(unsigned w) { this->width = w; }
  /*!
    Set image height to acquire.

  */
  inline void setHeight(unsigned h) { this->height = h; }

  void setScale(unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);

  /*!

  Set the number of buffers required for streaming data.

  For non real-time applications the number of buffers should be set to 1. For
  real-time applications to reach 25 fps or 50 fps a good compromise is to set
  the number of buffers to 3.

  \param nbuffers : Number of ring buffers.

  */
  inline void setNBuffers(unsigned nbuffers) { this->m_nbuffers = nbuffers; }

  /*!
    Set the device name.

    \param devname : Device name (like /dev/video0).

  */
  inline void setDevice(const std::string &devname) { sprintf(device, "%s", devname.c_str()); }
  /*!

  Set the pixel format for capture.`If the specified pixel format is
  out of range, we set the V4L2_RGB24_FORMAT.

  \param pixelformat : Camera pixel format coding.

  */
  inline void setPixelFormat(vpV4l2PixelFormatType pixelformat)
  {
    this->m_pixelformat = pixelformat;
    if (this->m_pixelformat >= V4L2_MAX_FORMAT)
      this->m_pixelformat = V4L2_RGB24_FORMAT;
  }

  void close();

private:
  void setFormat();
  /*!
    Set the frame format.

    \param frameformat :
    - vpV4l2Grabber::V4L2_FRAME_FORMAT: capture alternate fields (or frames),
    - vpV4l2Grabber::V4L2_IMAGE_FORMAT: capture interlaced images.
  */
  inline void setFrameFormat(vpV4l2FrameFormatType frameformat) { this->m_frameformat = frameformat; }
  void open();
  void getCapabilities();
  void startStreaming();
  void stopStreaming();
  unsigned char *waiton(__u32 &index, struct timeval &timestamp);
  int queueBuffer();
  void queueAll();
  void printBufInfo(struct v4l2_buffer buf);

  int fd;
  char device[FILENAME_MAX];
  /* device descriptions */
  struct v4l2_capability cap;
  struct v4l2_streamparm streamparm;
  struct v4l2_input *inp;     //[vpV4l2Grabber::MAX_INPUTS];
  struct v4l2_standard *std;  //[vpV4l2Grabber::MAX_NORM];
  struct v4l2_fmtdesc *fmt;   //[vpV4l2Grabber::MAX_FORMAT];
  struct v4l2_queryctrl *ctl; //[vpV4l2Grabber::MAX_CTRL*2];

  /* capture */
  struct v4l2_format fmt_v4l2;
  struct ng_video_fmt fmt_me;
  struct v4l2_requestbuffers reqbufs;
  struct v4l2_buffer *buf_v4l2; //[vpV4l2Grabber::MAX_BUFFERS];
  struct ng_video_buf *buf_me;  //[vpV4l2Grabber::MAX_BUFFERS];
  unsigned int queue;
  unsigned int waiton_cpt;
  __u32 index_buffer; //!< index of the buffer in use

  bool m_verbose;
  unsigned m_nbuffers;
  unsigned int field;
  bool streaming;

  unsigned m_input;
  vpV4l2FramerateType m_framerate;
  vpV4l2FrameFormatType m_frameformat;
  vpV4l2PixelFormatType m_pixelformat;
};

#endif
#endif
