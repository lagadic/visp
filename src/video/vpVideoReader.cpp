/****************************************************************************
 *
 * $Id: vpImagePoint.h 2359 2009-11-24 15:09:25Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
 * Read videos and image sequences.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpVideoReader.cpp
  \brief Read videos and image sequences
*/

#include <visp/vpDebug.h>
#include <visp/vpVideoReader.h>

#include <iostream>
#include <fstream>

/*!
  Basic constructor.
*/
vpVideoReader::vpVideoReader()
{
  imSequence = NULL;
  #ifdef VISP_HAVE_FFMPEG
  ffmpeg = NULL;
  #endif
  initFileName = false;
  isOpen = false;
  firstFrame = 0;
  frameCount = 0;
  lastFrame = 0;
}


/*!
  Basic destructor.
*/
vpVideoReader::~vpVideoReader()
{
  if (imSequence != NULL)
  {
    delete imSequence;
  }
  #ifdef VISP_HAVE_FFMPEG
  if (ffmpeg != NULL)
  {
    delete ffmpeg;
  }
  #endif
}


/*!
  It enables to set the path and the name of the file(s) which as/have to be read.
  
  If you want to read a video file, \f$ filename \f$ corresponds to the path to the file (example : /local/video.mpeg).
  
  If you want to read a sequence of images, \f$ filename \f$ corresponds to the path followed by the image name template. For exemple, if you want to read different images named image0001.jpeg, image0002.jpg, ... and located in the folder /local/image, \f$ filename \f$ will be "/local/image/image%04d.jpg". 
  
  \param filename : Path to a video file or file name template of a image sequence.
*/
void vpVideoReader::setFileName(const char *filename)
{
  if (filename == '\0')
  {
    vpERROR_TRACE("filename empty ") ;
    throw (vpImageException(vpImageException::noFileNameError,"filename empty ")) ;
  }
  
  strcpy(this->fileName,filename);
  
  formatType = getFormat(fileName);
  
  initFileName = true;
}


/*!
  Sets all the parameters needed to read the video or the image sequence.
  
  Grab the first frame and stores it in the image \f$ I \f$.
  
  \param I : The image where the frame is stored.
*/
void vpVideoReader::open(vpImage< vpRGBa > &I)
{
  if (!initFileName)
  {
    vpERROR_TRACE("The generic filename has to be set");
    throw (vpImageException(vpImageException::noFileNameError,"filename empty"));
  }
  
  if (formatType == FORMAT_PGM ||
      formatType == FORMAT_PPM ||
      formatType == FORMAT_JPEG ||
      formatType == FORMAT_PNG)
  {
    imSequence = new vpDiskGrabber;
    imSequence->setGenericName(fileName);
    imSequence->setImageNumber(firstFrame);
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    if(!ffmpeg->openStream(fileName, vpFFMPEG::COLORED))
      throw (vpException(vpException::ioError ,"Could not open the video"));
    ffmpeg->initStream();
  }
  
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    vpERROR_TRACE("To read video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  
  frameCount = firstFrame;
  if(!getFrame(I,firstFrame))
  {
    vpERROR_TRACE("Could not read the first frame");
    throw (vpException(vpException::ioError ,"Could not read the first frame"));
  }
  height = I.getHeight();
  width = I.getWidth();
  
  isOpen = true;
  
  findLastFrameIndex();
}


/*!
  Sets all the parameters needed to read the video or the image sequence.
  
  Grab the first frame and stores it in the image \f$ I \f$.
  
  \param I : The image where the frame is stored.
*/
void vpVideoReader::open(vpImage<unsigned char> &I)
{
  if (!initFileName)
  {
    vpERROR_TRACE("The generic filename has to be set");
    throw (vpImageException(vpImageException::noFileNameError,"filename empty"));
  }
  
  if (formatType == FORMAT_PGM ||
      formatType == FORMAT_PPM ||
      formatType == FORMAT_JPEG ||
      formatType == FORMAT_PNG)
  {
    imSequence = new vpDiskGrabber;
    imSequence->setGenericName(fileName);
    imSequence->setImageNumber(firstFrame);
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    if (!ffmpeg->openStream(fileName, vpFFMPEG::GRAY_SCALED))
      throw (vpException(vpException::ioError ,"Could not open the video"));
    ffmpeg->initStream();
  }
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    vpERROR_TRACE("To read video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  
  frameCount = firstFrame;
  if(!getFrame(I,firstFrame))
  {
    vpERROR_TRACE("Could not read the first frame");
    throw (vpException(vpException::ioError ,"Could not read the first frame"));
  }
  height = I.getHeight();
  width = I.getWidth();
  
  isOpen = true;
  
  findLastFrameIndex();
}


/*!
  Grabs the kth image in the stack of frames and increments the frame counter in order to grab the next image (k+1) during the next use of the method.
  
  This method enables to use the class as frame grabber.
  
  \param I : The image where the frame is stored.
*/
void vpVideoReader::acquire(vpImage< vpRGBa > &I)
{
  if (!isOpen)
  {
    vpERROR_TRACE("Use the open method before");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }
  
  //getFrame(I,frameCount);
  if (imSequence != NULL)
    imSequence->acquire(I);
  #ifdef VISP_HAVE_FFMPEG
  else if (ffmpeg !=NULL)
    ffmpeg->acquire(I);
  #endif
  
  frameCount++;
}


/*!
  Grabs the kth image in the stack of frames and increments the frame counter in order to grab the next image (k+1) during the next use of the method.
  
  This method enables to use the class as frame grabber.
  
  \param I : The image where the frame is stored.
*/
void vpVideoReader::acquire(vpImage< unsigned char > &I)
{
  if (!isOpen)
  {
    vpERROR_TRACE("Use the open method before");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }
  
  //getFrame(I,frameCount);
  if (imSequence != NULL)
    imSequence->acquire(I);
  #ifdef VISP_HAVE_FFMPEG
  else if (ffmpeg != NULL)
    ffmpeg->acquire(I);
  #endif
  
  frameCount++;
}


/*!
  Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.
  
  \warning For the video files this method is not precise, and returns the nearest key frame from the expected frame.
  But this method enables to postion the reader where you want. Then, use the acquire method to grab the following images
  one after one.
  
  \param I : The vpImage used to stored the frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<vpRGBa> &I, unsigned int frame)
{
  if (imSequence != NULL)
  {
    try
    {
      imSequence->acquire(I,(unsigned long)frame);
    }
    catch(...)
    {
      vpERROR_TRACE("Couldn't find the %u th frame", frame) ;
      return false;
    }
  }
  #ifdef VISP_HAVE_FFMPEG
  else
  {
    
    if(!ffmpeg->getFrame(I,frame))
    {
      vpERROR_TRACE("Couldn't find the %u th frame", frame) ;
      return false;
    }
  }
  #endif
  return true;
}


/*!
  Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.
  
  \warning For the video files this method is not precise, and returns the nearest key frame from the expected frame.
  But this method enables to postion the reader where you want. Then, use the acquire method to grab the following images
  one after one.
  
  \param I : The vpImage used to stored the frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<unsigned char> &I, unsigned int frame)
{
  if (imSequence != NULL)
  {
    try
    {
      imSequence->acquire(I,(unsigned long)frame);
    }
    catch(...)
    {
      vpERROR_TRACE("Couldn't find the %u th frame", frame) ;
      return false;
    }
  }
  #ifdef VISP_HAVE_FFMPEG
  else
  {
    if(!ffmpeg->getFrame(I,frame))
    {
      vpERROR_TRACE("Couldn't find the %u th frame", frame) ;
      return false;
    }
  }
  #endif
  return true;
}


/*!
  Gets the format of the file(s) which has/have to be read.
  
  \return Returns the format.
*/
vpVideoReader::vpVideoFormatType
vpVideoReader::getFormat(const char *filename)
{
  std::string sfilename(filename);

  int PGM = sfilename.find(".PGM");
  int pgm = sfilename.find(".pgm");
  int PPM = sfilename.find(".PPM");
  int ppm = sfilename.find(".ppm");
  int JPG = sfilename.find(".JPG");
  int jpg = sfilename.find(".jpg");
  int JPEG = sfilename.find(".JPEG");
  int jpeg = sfilename.find(".jpeg");
  int PNG = sfilename.find(".PNG");
  int png = sfilename.find(".png");
  int AVI = sfilename.find(".AVI");
  int avi = sfilename.find(".avi");
  int MPEG = sfilename.find(".MPEG");
  int mpeg = sfilename.find(".mpeg");
  int MPG = sfilename.find(".MPG");
  int mpg = sfilename.find(".mpg");
  int MOV = sfilename.find(".MOV");
  int mov = sfilename.find(".mov");
  
  int size = sfilename.size();

  if ((PGM>0 && PGM<size ) || (pgm>0 && pgm<size))
    return FORMAT_PGM;
  else if ((PPM>0 && PPM<size) || ( ppm>0 && ppm<size))
    return FORMAT_PPM;
  else if ((JPG>0 && JPG<size) || ( jpg>0 && jpg<size) || (JPEG>0 && JPEG<size) || ( jpeg>0 && jpeg<size))
	return FORMAT_JPEG;
  else if ((PNG>0 && PNG<size) || ( png>0 && png<size))
    return FORMAT_PNG;
  else if ((AVI>0 && AVI<size) || ( avi>0 && avi<size))
    return FORMAT_AVI;
  else if ((MPEG>0 && MPEG<size) || ( mpeg>0 && mpeg<size) || (MPG>0 && MPG<size) || (mpg>0 && mpg <size))
    return FORMAT_MPEG;
  else if ((MOV>0 && MOV<size) || ( mov>0 && mov<size))
    return FORMAT_MOV;
  else{ 
    return FORMAT_UNKNOWN;
  } 
}


/*!
  Gets the last frame index
*/
void
vpVideoReader::findLastFrameIndex()
{
  if (!isOpen)
  {
    vpERROR_TRACE("Use the open method before");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }

  if (imSequence != NULL)
  {
    char name[FILENAME_MAX];
    int image_number = firstFrame;
    std::fstream file;
    bool failed;
    do
    {
      sprintf(name,fileName,image_number) ;
      file.open(name, std::fstream::in);
      failed = file.fail();
      if (!failed) file.close();
      image_number++;
    }while(!failed);
      
    lastFrame = image_number - 2;
  }  
    
  #ifdef VISP_HAVE_FFMPEG
  else if (ffmpeg != NULL)
    lastFrame = ffmpeg->getFrameNumber() - 1;
  #endif
}
