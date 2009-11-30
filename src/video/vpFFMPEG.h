/****************************************************************************
 *
 * $Id: vpImagePoint.h 2359 2009-11-24 15:09:25Z nmelchio $
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
 * This file is part of the ViSP toolkit.
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

#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <stdlib.h>
#include <vector>

#ifdef VISP_HAVE_FFMPEG
extern "C"
{
#include <avcodec.h>
#include <avformat.h>
#include <swscale.h>
}

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
  
  #ifdef VISP_HAVE_FFMPEG
  int main ()
  {
    vpImage<vpRGBa> I; //The image to stores the frames
    vpFFMPEG ffmpeg;
  
    //Initialization
    ffmpeg.openStream("video.mpeg",vpFFMPEG::COLORED);
    ffmepg.initStream();
  
    //Video reading
    int frameIndex = 0;
    ffmpeg.getFrame(I,frameIndex); //Here the first frame (index 0) is read.
  }
  
  #else
  int main() {}
  #endif
  \endcode
  
  If you want to open the video as a gray scaled video, you can use the following example.
  \code
  #include <visp/vpConfig.h>

  #include <visp/vpImage.h>
  #include <visp/vpFFMPEG.h>
  
  #ifdef VISP_HAVE_FFMPEG
  int main ()
  {
    vpImage<unsigned char> I; //The image to stores the frames
    vpFFMPEG ffmpeg;
  
    //Initialization
    ffmpeg.openStream("video.mpeg",vpFFMPEG::GRAY_SCALED);
    ffmepg.initStream();
  
    //Video reading
    int frameIndex = 0;
    ffmpeg.getFrame(I,frameIndex); //Here the first frame (index 0) is read.
  }
  
  #else
  int main() {}
  #endif
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
    AVPacket packet;
    SwsContext *img_convert_ctx  ;
    int videoStream, numBytes ;
    uint8_t * buffer ;
    std::vector<int64_t> index;
    //! Indicates if the openStream method was executed
    bool streamWasOpen;
    //! Indicates if the initStream method was executed
    bool streamWasInitialized;
    //! Indicates the video's color output.
    vpFFMPEGColorType color_type;

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

    bool openStream(const char *filename,vpFFMPEGColorType color_type);
    bool initStream();
    void closeStream();

    bool getFrame(vpImage<vpRGBa> &I, unsigned int frameNumber);
    bool getFrame(vpImage<unsigned char> &I, unsigned int frameNumber);
  
  private:
    void copyBitmap(vpImage<vpRGBa> &I);
    void copyBitmap(vpImage<unsigned char> &I);
};
#endif
#endif