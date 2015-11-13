/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
#include <visp3/io/vpFFMPEG.h>

#if VISP_HAVE_OPENCV_VERSION >= 0x020200
#  include <opencv2/highgui/highgui.hpp>
#elif VISP_HAVE_OPENCV_VERSION >= 0x020000
#  include <opencv/highgui.h>
#endif


/*!
  \class vpVideoWriter

  \ingroup group_io_video

  \brief Class that enables to write easily a video file or a sequence of images.
 
  The following example available in tutorial-video-recorder.cpp shows how this
  class can be used to record a video from a camera by default in an mpeg file.
  \include tutorial-video-recorder.cpp

  The following example shows also how this class can be used to write an image sequence.
  The images are stored in the folder "./image" and are named "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...
  
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
  
  The other following example explains how to use the class to write directly an mpeg file.
  
  \code
#include <visp3/io/vpVideoWriter.h>

int main()
{
  vpImage<vpRGBa> I;

  vpVideoWriter writer;

  // Set up the framerate to 30Hz. Default is 25Hz.
  writer.setFramerate(30);

#ifdef VISP_HAVE_FFMPEG
  // Set up the bit rate
  writer.setBitRate(1000000);
  // Set up the codec to use
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
  writer.setCodec(CODEC_ID_MPEG2VIDEO);
#else
  writer.setCodec(AV_CODEC_ID_MPEG2VIDEO);
#endif
#elif defined VISP_HAVE_OPENCV
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
#ifdef VISP_HAVE_FFMPEG
    //!To read video files
    vpFFMPEG *ffmpeg;
    //!The codec to use
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
    CodecID codec;
#else
    AVCodecID codec;
#endif
    //!The bite rate
    unsigned int bit_rate;
    int framerate;
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
	  cv::VideoWriter writer;
	  int fourcc;
	  double framerate;
#endif
    //!Types of available formats
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
    
    //!Video's format which has to be writen
    vpVideoFormatType formatType;
    
    //!Path to the image sequence
    char fileName[FILENAME_MAX];
    
    //!Indicates if the path to the image sequence is set.
    bool initFileName;
    
    //!Indicates if the video is "open".
    bool isOpen;
    
    //!Count the frame number.
    unsigned int frameCount;
    
    //!The first frame index.
    unsigned int firstFrame;
    
    //!Size of the frame
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
    inline unsigned int getCurrentFrameIndex() const {return frameCount;}

    void open (vpImage< vpRGBa > &I);
    void open (vpImage< unsigned char > &I);
    /*!
      Reset the frame counter and sets it to the first image index.

      By default the first frame index is set to 0.
    */
    inline void resetFrameCounter() {frameCount = firstFrame;}

    void saveFrame (vpImage< vpRGBa > &I);
    void saveFrame (vpImage< unsigned char > &I);

#ifdef VISP_HAVE_FFMPEG
    /*!
      Sets the bit rate of the video when encoding.

      \param bitrate : the expected bit rate.

      By default the bit rate is set to 500 000.
    */
    inline void setBitRate(const unsigned int bitrate) {this->bit_rate = bitrate;}

    /*!
      Sets the codec used to encode the video.

      \param codec_id : the expected codec.

      By default codec is set to AV_CODEC_ID_MPEG1VIDEO. But if installed, you can use one of the
      AVCodecID proposed by ffmpeg such as : AV_CODEC_ID_MPEG2VIDEO, AV_CODEC_ID_MPEG2VIDEO_XVMC,
      AV_CODEC_ID_MPEG4, AV_CODEC_ID_H264, ... (More AVCodecID can be found in the ffmpeg documentation).

      Of course to use the codec it must be installed on your computer.
    */
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
    inline void setCodec(const CodecID codec_id) {this->codec = codec_id;}
#else
    inline void setCodec(const AVCodecID codec_id) {this->codec = codec_id;}
#endif
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
    inline void setCodec(const int fourcc_codec) {this->fourcc = fourcc_codec;}
#endif

    void setFileName(const char *filename);
    void setFileName(const std::string &filename);
    /*!
      Enables to set the first frame index.
      
      \param first_frame : The first frame index.
    */
    inline void setFirstFrameIndex(const unsigned int first_frame) {this->firstFrame = first_frame;}
#ifdef VISP_HAVE_FFMPEG
    /*!
      Sets the framerate in Hz of the video when encoding.

      \param frame_rate : the expected framerate.

      By default the framerate is set to 25Hz.
    */
    inline void setFramerate(const int frame_rate) {
      this->framerate = frame_rate;
    }
#elif VISP_HAVE_OPENCV_VERSION >= 0x020100
	/*!
      Sets the framerate in Hz of the video when encoding.

      \param frame_rate : the expected framerate.

      By default the framerate is set to 25Hz.
    */
    inline void setFramerate(const double frame_rate) {
      this->framerate = frame_rate;
    }
#endif

    private:
      vpVideoFormatType getFormat(const char *filename);
      static std::string getExtension(const std::string &filename);
};

#endif
