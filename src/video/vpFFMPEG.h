/****************************************************************************
 *
 * $Id: vpImagePoint.h 2359 2009-11-24 15:09:25Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Class that manages the FFMPEG library.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpFFMPEG.h
  \brief Class that manages the FFMPEG library
*/

#ifndef vpFFMPEG_H
#define vpFFMPEG_H

#include <visp/vpImageIo.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>

#ifdef VISP_HAVE_FFMPEG

// Fix for the following compilation error:
// libavutil/common.h:154: error: UINT64_C was not declared in this scope
// libavutil/common.h is no more autosufficient for C++ program because
// stdint.h defines UINT64_C only for C program and not C++ program
#ifdef __cplusplus
#  define __STDC_CONSTANT_MACROS
#  ifdef _STDINT_H
#    undef _STDINT_H
#  endif
// On OS X
#  ifdef _STDINT_H_
#    undef _STDINT_H_
#  endif
#  include <stdint.h>

#  ifndef INT64_C
#    define INT64_C(c) (c ## LL)
#  endif
#  ifndef UINT64_C
#    define UINT64_C(c) (c ## ULL)
#  endif
#endif
// end fix


extern "C"
{
#include <avcodec.h> // requested for CodecID enum
//#include <avformat.h>
//#include <swscale.h>
}

struct AVFormatContext;
struct AVCodecContext;
struct AVCodec;
struct AVFrame;
struct AVFrame;
struct AVFrame;
struct AVPacket;
struct SwsContext;

/*!
  \class vpFFMPEG
  \ingroup Video
  
  \brief This class interfaces the FFmpeg library to enable the reading of video files.
  
  Here an example which explains how to use the class.
  \code
  #include <visp/vpConfig.h>

  #include <visp/vpImage.h>
  #include <visp/vpRGBa.h>
  #include <visp/vpFFMPEG.h>
  
  int main ()
  {
  #ifdef VISP_HAVE_FFMPEG
    vpImage<vpRGBa> I; //The image to stores the frames
    vpFFMPEG ffmpeg;
  
    //Initialization
    ffmpeg.openStream("video.mpeg",vpFFMPEG::COLORED);
    ffmepg.initStream();
  
    //Video reading
    int frameIndex = 0;
    ffmpeg.getFrame(I,frameIndex); //Here the first frame (index 0) is read.
  #endif
  }
  \endcode
  
  If you want to open the video as a gray scaled video, you can use the following example.
  \code
  #include <visp/vpConfig.h>

  #include <visp/vpImage.h>
  #include <visp/vpFFMPEG.h>
  
  int main ()
  {
  #ifdef VISP_HAVE_FFMPEG
    vpImage<unsigned char> I; //The image to stores the frames
    vpFFMPEG ffmpeg;
  
    //Initialization
    ffmpeg.openStream("video.mpeg",vpFFMPEG::GRAY_SCALED);
    ffmepg.initStream();
  
    //Video reading
    int frameIndex = 0;
    ffmpeg.getFrame(I,frameIndex); //Here the first frame (index 0) is read.
  #endif
  }
  \endcode
*/
class VISP_EXPORT vpFFMPEG
{
  public:
    typedef enum
    {
      COLORED,
      GRAY_SCALED,
    }vpFFMPEGColorType;
    
  private:
    //! Video's height and width
    int width, height;
    //! Number of frame in the video.
    unsigned long frameNumber;
    //! FFMPEG variables
    AVFormatContext *pFormatCtx;
    AVCodecContext *pCodecCtx;
    AVCodec *pCodec;
    AVFrame *pFrame;
    AVFrame *pFrameRGB;
    AVFrame *pFrameGRAY;
    AVPacket *packet;
    SwsContext *img_convert_ctx  ;
    unsigned int videoStream;
    int numBytes ;
    uint8_t * buffer ;
    std::vector<int64_t> index;
    //! Indicates if the openStream method was executed
    bool streamWasOpen;
    //! Indicates if the initStream method was executed
    bool streamWasInitialized;
    //! Indicates the video's color output.
    vpFFMPEGColorType color_type;

    //!The file which is the video file
    FILE *f;
    //!Buffers
    uint8_t *outbuf, *picture_buf;
    //!Buffer size
    int outbuf_size;
    //!Size of the data to write in the file
    int out_size;
    //!Bit rate of the video to write    
    unsigned int bit_rate;
    //!Indicates if the openEncoder method was executed
    bool encoderWasOpened;

  public:
    vpFFMPEG();
    ~vpFFMPEG();
  
    /*!
      Gets the video's width.

      \return The value of the video's width.
    */
    inline int getWidth() const {return width;}
    
    /*!
      Gets the video's height.

      \return The value of the video's height.
    */
    inline int getHeight() const {return height;}
    
    /*!
      Gets the video's frame number.

      \return The value of the video's frame number.
    */
    inline unsigned long getFrameNumber() const {return frameNumber;}
    
    /*!
      Sets the bit rate of the video when writing.
      
      \param bit_rate : the expected bit rate.
    */
    inline void setBitRate(const unsigned int bit_rate) {this->bit_rate = bit_rate;}

    bool openStream(const char *filename,vpFFMPEGColorType color_type);
    bool initStream();
    void closeStream();

    bool acquire(vpImage<vpRGBa> &I);
    bool acquire(vpImage<unsigned char> &I);

    bool getFrame(vpImage<vpRGBa> &I, unsigned int frameNumber);
    bool getFrame(vpImage<unsigned char> &I, unsigned int frameNumber);

    bool openEncoder(const char *filename, unsigned int width, unsigned int height, CodecID codec = CODEC_ID_MPEG1VIDEO);
    bool saveFrame(vpImage<vpRGBa> &I);
    bool saveFrame(vpImage<unsigned char> &I);
    bool endWrite();
  
  private:
    void copyBitmap(vpImage<vpRGBa> &I);
    void copyBitmap(vpImage<unsigned char> &I);
    void writeBitmap(vpImage<vpRGBa> &I);
    void writeBitmap(vpImage<unsigned char> &I);
};
#endif
#endif
