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
  
  //Read the 3th frame
  reader.getFrame(I,2);

  return 0;
  }
  #else
  int main() {}
  #endif
  \endcode
  
  The other following example explain how to use the class to manipulate a sequence of images. The images are stored in the folder "./image" and are named "image0000.jpeg", "image0001.jpeg", "image0002.jpeg", ...
  
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
  reader.setFileName("./image/image%04d.jpeg");
  reader.open(I);
  
  //Read the 3th frame
  reader.getFrame(I,2);

  return 0;
  }
  #else
  int main() {}
  #endif
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
    unsigned int frameCount;

  public:
    vpVideoReader();
    ~vpVideoReader();
    
    void setFileName(const char *filename);
    void open (vpImage< vpRGBa > &I);
    void open (vpImage< unsigned char > &I);
    void acquire(vpImage< vpRGBa > &I);
    void acquire(vpImage< unsigned char > &I);
    bool getFrame(vpImage<vpRGBa> &I, unsigned int frame);
    bool getFrame(vpImage<unsigned char> &I, unsigned int frame);
    void close(){;}
    
  private:
    vpVideoFormatType getFormat(const char *filename);
};

#endif