/****************************************************************************
 *
 * $Id: vpV4l2Grabber.h,v 1.14 2008-11-10 16:54:10 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_V4L2

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/videodev2.h> // Video For Linux Two interface

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>




/*!
  \class vpV4l2Grabber

  \ingroup Framegrabber

  \brief Class for the Video4Linux2 video device.

  Information about Video4Linux can be found on
  http://linuxtv.org/v4lwiki/index.php/Main_Page

  This class was tested with a Pinnacle PCTV Studio/Rave board.

  If the grabbing fails, it means potentially that
  you have not configured the linux bttv kernel module according to your board.
  For that, depending on your linux distribution check the card id in
  - /usr/share/doc/kernel-doc-2.4.20/video4linux/bttv/CARDLIST
  - or /usr/share/doc/kernel-doc-2.6.20/Documentation/video4linux/CARDLIST.bttv

  For example, the card id of a Pinnacle PCTV Studio/Rave board is 39.
  Once this id is determined, you have to set the bttv driver with, by adding
  \verbatim
  options bttv card=39
  \endverbatim
  in one of theses files :
  - /etc/modules.conf
  - or /etc/modprobe.conf

  The example below shows how to use this grabber.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpV4l2Grabber.h>
#include <visp/vpImageIo.h>

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
  vpImageIo::writePGM(I, "image.pgm"); // Save the image on the disk
#endif
}
  \endcode
  

  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes


  \sa vpFrameGrabber
*/
class VISP_EXPORT vpV4l2Grabber : public vpFrameGrabber
{
public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;
  static const int MAX_INPUTS;
  static const int MAX_NORM;
  static const int MAX_FORMAT;
  static const int MAX_CTRL;
  static const int MAX_BUFFERS;
  static const int FRAME_SIZE;

  /*! \enum vpV4l2FramerateType
    Frame rate type for capture.
  */
  typedef enum
    {
      framerate_50fps, //!< 50 frames per second
      framerate_25fps  //!< 25 frames per second
    } vpV4l2FramerateType;

  /*! \enum vpV4l2FrameFormatType
    Frame format type for capture.
  */
  typedef enum
    {
      V4L2_FRAME_FORMAT, /*!< a field only */
      V4L2_IMAGE_FORMAT  /*!< an interlaced image */
    } vpV4l2FrameFormatType;

  /*! \enum vpV4l2PixelFormatType
    Pixel format type for capture.
  */
  typedef enum {
    V4L2_GREY_FORMAT, /*!<  */
    V4L2_RGB24_FORMAT, /*!<  */
    V4L2_RGB32_FORMAT, /*!<  */
    V4L2_BGR24_FORMAT, /*!<  */
    V4L2_BGR32_FORMAT /*!<  */
  } vpV4l2PixelFormatType;

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct ng_video_fmt {
    unsigned int   pixelformat;         /* VIDEO_* */
    unsigned int   width;
    unsigned int   height;
    unsigned int   bytesperline;  /* zero for compressed formats */
  };


  struct ng_video_buf {
    struct ng_video_fmt  fmt;
    size_t               size;
    unsigned char        *data;
    int                  refcount;
  };
#endif

public:
  vpV4l2Grabber();
  vpV4l2Grabber(unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  vpV4l2Grabber(vpImage<unsigned char> &I,
		unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  vpV4l2Grabber(vpImage<vpRGBa> &I,
		unsigned input, unsigned scale = vpV4l2Grabber::DEFAULT_SCALE);
  virtual ~vpV4l2Grabber() ;

public:
  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<unsigned char> &I, struct timeval &timestamp) ;
  void acquire(vpImage<vpRGBa> &I) ;
  void acquire(vpImage<vpRGBa> &I, struct timeval &timestamp) ;
  bool getField();
  void setFramerate(vpV4l2FramerateType framerate);
  vpV4l2FramerateType getFramerate();
  void close();

  void setInput(unsigned input = vpV4l2Grabber::DEFAULT_INPUT) ;

  /*!
    Set image width to acquire.

  */
  inline void setWidth(unsigned width)
  {
    this->_width = width;
  }
  /*!
    Set image height to acquire.

  */
  inline void setHeight(unsigned height)
  {
    this->_height = height;
  }

  void setScale(unsigned scale = vpV4l2Grabber::DEFAULT_SCALE) ;

  /*!

  Set the number of buffers required for streaming data.

  For non real-time applications the number of buffers should be set to 1. For
  real-time applications to reach 25 fps or 50 fps a good compromise is to set
  the number of buffers to 3.

  \param nbuffers : Number of ring buffers.

  */
  inline void setNBuffers(unsigned nbuffers)
  {
    this->nbuffers = nbuffers;
  }

  /*!
    Set the device name.

    \param devname : Device name (like /dev/video0).

  */
  inline void setDevice(const char *devname)
  {
    sprintf(device, "%s", devname);
  }


private:
  /*!
    Set the pixel format.

    \param pixelformat :
    - vpV4l2Grabber::V4L2_GREY_FORMAT,
    - vpV4l2Grabber::V4L2_RGB24_FORMAT,
    - vpV4l2Grabber::V4L2_RGB32_FORMAT,
    - vpV4l2Grabber::V4L2_BGR24_FORMAT,
    - vpV4l2Grabber::V4L2_BGR32_FORMAT.
  */
  inline void setPixelFormat(vpV4l2PixelFormatType pixelformat)
  {
    this->pixelformat = pixelformat;
  }
  /*!
    Set the frame format.

    \param frameformat :
    - vpV4l2Grabber::V4L2_FRAME_FORMAT: capture alternate fields (or frames),
    - vpV4l2Grabber::V4L2_IMAGE_FORMAT: capture interlaced images.
  */
  inline void setFrameFormat(vpV4l2FrameFormatType frameformat)
  {
    this->frameformat = frameformat;
  }
  void setFormat();
  void open();
  void getCapabilities();
  void startStreaming();
  void stopStreaming();
  unsigned char * waiton(int &index, struct timeval &timestamp);
  int  queueBuffer();
  void queueAll();
  void printBufInfo(struct v4l2_buffer buf);


  int				fd;
  char				device[FILENAME_MAX];
  /* device descriptions */
  struct v4l2_capability	cap;
  struct v4l2_streamparm	streamparm;
  struct v4l2_input		*inp; //[vpV4l2Grabber::MAX_INPUTS];
  struct v4l2_standard      	*std; //[vpV4l2Grabber::MAX_NORM];
  struct v4l2_fmtdesc		*fmt; //[vpV4l2Grabber::MAX_FORMAT];
  struct v4l2_queryctrl		*ctl; //[vpV4l2Grabber::MAX_CTRL*2];

  /* capture */
  int                           fps;
  struct v4l2_format            fmt_v4l2;
  struct ng_video_fmt           fmt_me;
  struct v4l2_requestbuffers    reqbufs;
  struct v4l2_buffer            *buf_v4l2; //[vpV4l2Grabber::MAX_BUFFERS];
  struct ng_video_buf           *buf_me; //[vpV4l2Grabber::MAX_BUFFERS];
  unsigned int                  queue;
  unsigned int                  waiton_cpt;
  int				index_buffer; //!< index of the buffer in use

  bool		verbose;
  unsigned	nbuffers;
  int           field;
  bool		streaming;

  unsigned      input;
  unsigned      _width;
  unsigned      _height;
  vpV4l2FramerateType framerate;
  vpV4l2FrameFormatType frameformat;
  vpV4l2PixelFormatType pixelformat;
} ;

#endif
#endif

