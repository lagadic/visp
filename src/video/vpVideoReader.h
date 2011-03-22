/****************************************************************************
 *
 * $Id: vpImagePoint.h 2359 2009-11-24 15:09:25Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Read videos and sequences of images .
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpVideoReader.h
  \brief Read videos and image sequences
*/

#ifndef vpVideoReader_H
#define vpVideoReader_H

#include <visp/vpConfig.h>
#include <visp/vpDiskGrabber.h>
#include <visp/vpFFMPEG.h>

/*!
  \class vpVideoReader

  \ingroup Video

  \brief Class that enables to manipulate easily a video file or a sequence of images. As it inherits from the vpFrameGrabber Class, it can be used like an other frame grabber class.
  
  The following example shows how this class is really easy to use. It enable to read a video file named video.mpeg and located in the folder "./video".
  
  But be careful, for video files, the getFrame method is not precise and returns the nearest key frame from the expected frame. You can use the getFrame method to
  position the reader in the video and then use the acquire method to get the following frames one by one.
  \code
  #include <visp/vpConfig.h>
  #include <visp/vpImage.h>
  #include <visp/vpRGBa.h>
  #include <visp/vpVideoReader.h>
  
  #ifdef VISP_HAVE_FFMPEG
  int main()
  {
  vpImage<vpRGBa> I;

  vpVideoReader reader;
  
  //Initialize the reader.
  reader.setFileName("./video/video.mpeg");
  reader.open(I);
  
  //Read the nearest key frame from the 3th frame
  reader.getFrame(I,2);
  
  //After positionning the video reader use acquire to read the video frame by frame
  reader.acquire(I);

  return 0;
  }
  #else
  int main() {}
  #endif
  \endcode
  
  The other following example explains how to use the class to manipulate a sequence of images. The images are stored in the folder "./image" and are named "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...
  
  \code
  #include <visp/vpConfig.h>
  #include <visp/vpImage.h>
  #include <visp/vpRGBa.h>
  #include <visp/vpVideoReader.h>
  
  int main()
  {
  vpImage<vpRGBa> I;

  vpVideoReader reader;
  
  //Initialize the reader.
  reader.setFileName("./image/image%04d.jpeg");
  reader.open(I);
  
  //Read the 3th frame
  reader.getFrame(I,2);

  return 0;
  }
  \endcode
*/

class VISP_EXPORT vpVideoReader : public vpFrameGrabber
{    
  private:
    //!To read sequences of images
    vpDiskGrabber *imSequence;
#ifdef VISP_HAVE_FFMPEG
    //!To read video files
    vpFFMPEG *ffmpeg;
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
      FORMAT_OGV,
      FORMAT_UNKNOWN
    } vpVideoFormatType;
    
    //!Video's format which has to be read
    vpVideoFormatType formatType;
    
    //!Path to the video
    char fileName[FILENAME_MAX];
    //!Indicates if the path to the video is set.
    bool initFileName;
    //!Indicates if the video is "open".
    bool isOpen;
    //!Count the frame number when the class is used as a grabber.
    long frameCount;
    //!The first frame index
    long firstFrame;
    //!The last frame index
    long lastFrame;

  public:
    vpVideoReader();
    ~vpVideoReader();
    
    /*!
      Enables to set the first frame index if you want to use the class like a grabber (ie with the
      acquire method).
      
      \param firstFrame : The first frame index.
    */
    inline void setFirstFrameIndex(const long firstFrame) {this->firstFrame = firstFrame;}
    
    /*!
      Reset the frame counter and sets it to the first image index.
      
      By default the first frame index is set to 0.
      
      This method is useful if you use the class like a frame grabber (ie with theacquire method).
    */
    inline void resetFrameCounter() {frameCount = firstFrame;}
    
    /*!
      Gets the last frame index.
      
      \return Returns the last frame index.
    */
    inline long getLastFrameIndex() const {return lastFrame;}
    
    void setFileName(const char *filename);
    void open (vpImage< vpRGBa > &I);
    void open (vpImage< unsigned char > &I);
    void acquire(vpImage< vpRGBa > &I);
    void acquire(vpImage< unsigned char > &I);
    bool getFrame(vpImage<vpRGBa> &I, long frame);
    bool getFrame(vpImage<unsigned char> &I, long frame);
    void close(){;}
    
    /*!
      Get the current frame index. This index is updated at each call of the 
      acquire method. It can be used to detect the end of a file (comparison 
      with getLastFrameIndex()).
      
      \return Returns the current frame index.
    */
    inline long getFrameIndex() const { return frameCount;}
    
    
  private:
    vpVideoFormatType getFormat(const char *filename);
    void findLastFrameIndex();
};

#endif
