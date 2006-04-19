/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 2005  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Fabien Spindler
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: fspindle@irisa.fr
#    www  : http://www.irisa.fr/lagadic
#
#----------------------------------------------------------------------------
*/

/*!
  \file vpV4l2Grabber.h
  \brief class for the Video For Linux 2 video device
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
  \brief class for the Video For Linux 2 video device

  \ingroup libdevice

  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes


  \sa vpFrameGrabber
*/
class vpV4l2Grabber : public vpFrameGrabber
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

  enum framerateEnum
    {
      framerate_50fps, //!< 50 frames per second
      framerate_25fps  //!< 25 frames per second
    };
  /*! \enum frameformatEnum
    Frame format for capture.
  */
  typedef enum
    {
      V4L2_FRAME_FORMAT, /*!< a field only */
      V4L2_IMAGE_FORMAT  /*!< an interlaced image */
    } frameformatEnum;

  /*! \enum pixelformatEnum
    Pixel format for capture.
  */
  typedef enum {
    V4L2_GREY_FORMAT, /*!<  */
    V4L2_RGB24_FORMAT, /*!<  */
    V4L2_RGB32_FORMAT, /*!<  */
    V4L2_BGR24_FORMAT, /*!<  */
    V4L2_BGR32_FORMAT /*!<  */
  } pixelformatEnum;

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
  ~vpV4l2Grabber() ;

public:
  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  bool getField();
  void setFramerate(framerateEnum framerate);
  framerateEnum getFramerate();
  void close();



  // fct pour changer le port d'entree
  void setInput(unsigned input = vpV4l2Grabber::DEFAULT_INPUT) ;
  void setScale(unsigned scale = vpV4l2Grabber::DEFAULT_SCALE) ;


private:
  void open();
  void setFormat(int width, int height,
		 frameformatEnum frameformat,
		 pixelformatEnum pixelformat);
  void getCapabilities();
  void startStreaming();
  void stopStreaming();
  unsigned char * waiton(int &index);
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
  int		ncols;
  int		nrows;

  framerateEnum framerate;
  unsigned      input;
  unsigned      scale;
  frameformatEnum frameformat;
} ;

#endif
#endif

