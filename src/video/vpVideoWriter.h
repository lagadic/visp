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

#include <visp/vpImageIo.h>
#include <visp/vpFFMPEG.h>

/*!
  \class vpVideoWriter

  \ingroup Video

  \brief Class that enables to write easily a video file or a sequence of images.
 
  The following example shows how this class is really easy to use. It enable to write an image sequence. The images are stored in the folder "./image" and are named "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...
  
  \code
  #include <visp/vpConfig.h>
  #include <visp/vpVideoWriter.h>
 
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
  #include <visp/vpConfig.h>
  #include <visp/vpVideoWriter.h>
  
  #ifdef VISP_HAVE_FFMPEG
  int main()
  {
  vpImage<vpRGBa> I;

  vpVideoWriter writer;
  
  //Set up the bit rate
  writer.setBitRate(1000000);
  //Set up the codec to use
  writer.setCodec(CODEC_ID_MPEG1VIDEO);
  
  writer.setFileName("./test.mpeg");
  
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
  #else
  int main() {}
  #endif
  \endcode
*/

class VISP_EXPORT vpVideoWriter
{    
  private:   
#ifdef VISP_HAVE_FFMPEG
    //!To read video files
    vpFFMPEG *ffmpeg;
    //!The codec to use
    CodecID codec;
    //!The bite rate
    unsigned int bit_rate;
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
    
    /*!
      Enables to set the first frame index.
      
      \param firstFrame : The first frame index.
    */
    inline void setFirstFrameIndex(const unsigned int firstFrame) {this->firstFrame = firstFrame;}
    
    /*!
      Reset the frame counter and sets it to the first image index.
      
      By default the first frame index is set to 0.
    */
    inline void resetFrameCounter() {frameCount = firstFrame;}
    
    /*!
      Gets the current frame index.
      
      \return Returns the current frame index.
    */
    inline unsigned int getCurrentFrameIndex() const {return frameCount;}
    
    #ifdef VISP_HAVE_FFMPEG
    /*!
      Sets the bit rate of the video when writing.
      
      \param bit_rate : the expected bit rate.
      
      By default the bit rate is set to 500 000.
    */
    inline void setBitRate(const unsigned int bit_rate) {this->bit_rate = bit_rate;}
    
    /*!
      Sets the codec used to encode the video.
      
      \param codec : the expected codec.
      
      By default the codec is set to CODEC_ID_MPEG1VIDEO. But you can use one of the CodecID proposed by ffmpeg such as : CODEC_ID_MPEG2VIDEO, CODEC_ID_MPEG2VIDEO_XVMC, CODEC_ID_MPEG4, CODEC_ID_H264, ... (More CodecID can be found in the ffmpeg documentation).
      
      Of course to use the codec it must be installed on your computer.
    */
    inline void setCodec(const CodecID codec) {this->codec = codec;}
    #endif
    
    void setFileName(const char *filename);
    void open (vpImage< vpRGBa > &I);
    void open (vpImage< unsigned char > &I);
    void saveFrame (vpImage< vpRGBa > &I);
    void saveFrame (vpImage< unsigned char > &I);
    void close();
    
    private:
      vpVideoFormatType getFormat(const char *filename);
      static std::string getExtension(const std::string &filename);
};

#endif
