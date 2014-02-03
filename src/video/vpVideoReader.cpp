/****************************************************************************
 *
 * $Id: vpImagePoint.h 2359 2009-11-24 15:09:25Z nmelchio $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
  : imSequence(NULL),
#ifdef VISP_HAVE_FFMPEG
    ffmpeg(NULL),
#endif
    formatType(FORMAT_UNKNOWN), initFileName(false), isOpen(false), frameCount(0),
    firstFrame(0), lastFrame(0), firstFrameIndexIsSet(false), lastFrameIndexIsSet(false)
{
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
  
  if (strlen( filename ) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the file name"));
  }

  strcpy(this->fileName,filename);
  
  formatType = getFormat(fileName);
  
  initFileName = true;
}

/*!
  It enables to set the path and the name of the file(s) which as/have to be read.

  If you want to read a video file, \f$ filename \f$ corresponds to the path to the file (example : /local/video.mpeg).

  If you want to read a sequence of images, \f$ filename \f$ corresponds to the path followed by the image name template. For exemple, if you want to read different images named image0001.jpeg, image0002.jpg, ... and located in the folder /local/image, \f$ filename \f$ will be "/local/image/image%04d.jpg".

  \param filename : Path to a video file or file name template of a image sequence.
*/
void vpVideoReader::setFileName(const std::string &filename)
{
  setFileName(filename.c_str());
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
      formatType == FORMAT_PNG ||
      formatType == FORMAT_TIFF ||
      formatType == FORMAT_BMP ||
      formatType == FORMAT_DIB ||
      formatType == FORMAT_PBM ||
      formatType == FORMAT_RASTER ||
      formatType == FORMAT_JPEG2000)
  {
    imSequence = new vpDiskGrabber;
    imSequence->setGenericName(fileName);
    if (firstFrameIndexIsSet)
      imSequence->setImageNumber(firstFrame);
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV ||
           formatType == FORMAT_OGV)
  {
    ffmpeg = new vpFFMPEG;
    if(!ffmpeg->openStream(fileName, vpFFMPEG::COLORED))
      throw (vpException(vpException::ioError ,"Could not open the video"));
    ffmpeg->initStream();
  }
  
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV ||
           formatType == FORMAT_OGV)
  {
    vpERROR_TRACE("To read video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  else if (formatType == FORMAT_UNKNOWN)
  {
    vpERROR_TRACE("The format of the file does not correpsond to a readable format.");
    throw (vpException(vpException::fatalError ,"The format of the file does not correpsond to a readable format."));
  }
  
  findFirstFrameIndex();
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
      formatType == FORMAT_PNG ||
      formatType == FORMAT_TIFF ||
      formatType == FORMAT_BMP ||
      formatType == FORMAT_DIB ||
      formatType == FORMAT_PBM ||
      formatType == FORMAT_RASTER ||
      formatType == FORMAT_JPEG2000)
  {
    imSequence = new vpDiskGrabber;
    imSequence->setGenericName(fileName);
    if (firstFrameIndexIsSet)
      imSequence->setImageNumber(firstFrame);
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV ||
           formatType == FORMAT_OGV)
  {
    ffmpeg = new vpFFMPEG;
    if (!ffmpeg->openStream(fileName, vpFFMPEG::GRAY_SCALED))
      throw (vpException(vpException::ioError ,"Could not open the video"));
    ffmpeg->initStream();
  }
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV ||
           formatType == FORMAT_OGV)
  {
    vpERROR_TRACE("To read video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  else if (formatType == FORMAT_UNKNOWN)
  {
    vpERROR_TRACE("The format of the file does not correpsond to a readable format.");
    throw (vpException(vpException::fatalError ,"The format of the file does not correpsond to a readable format."));
  }
  
  findFirstFrameIndex();
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
  Grabs the current (k) image in the stack of frames and increments the frame counter
  in order to grab the next image (k+1) during the next use of the method. If open()
  was not called priviously, this method opens the video reader.
  
  This method enables to use the class as frame grabber.
  
  \param I : The image where the frame is stored.
*/
void vpVideoReader::acquire(vpImage< vpRGBa > &I)
{
  if (!isOpen) {
    open(I);
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
  if (!isOpen) {
    open(I);
  }
  
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
  But this method enables to position the reader where you want. Then, use the acquire method to grab the following images
  one after one.
  
  \param I : The vpImage used to stored the frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<vpRGBa> &I, long frame)
{
  if (imSequence != NULL)
  {
    try
    {
      imSequence->acquire(I, frame);
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
    
    if(!ffmpeg->getFrame(I, (unsigned int)frame))
    {
      vpERROR_TRACE("Couldn't find the %ld th frame", frame) ;
      return false;
    }
  }
  #endif
  return true;
}


/*!
  Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.
  
  \warning For the video files this method is not precise, and returns the nearest key frame from the expected frame.
  But this method enables to position the reader where you want. Then, use the acquire method to grab the following images
  one after one.
  
  \param I : The vpImage used to stored the frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpVideoReader::getFrame(vpImage<unsigned char> &I, long frame)
{
  if (imSequence != NULL)
  {
    try
    {
      imSequence->acquire(I, frame);
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
    if(!ffmpeg->getFrame(I, (unsigned int)frame))
    {
      vpERROR_TRACE("Couldn't find the %ld th frame", frame) ;
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

  std::string ext = vpVideoReader::getExtension(sfilename);

  if (ext.compare(".PGM") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".pgm") == 0)
    return FORMAT_PGM;
  else if (ext.compare(".PPM") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".ppm") == 0)
    return FORMAT_PPM;
  else if (ext.compare(".JPG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".JPEG") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".jpeg") == 0)
    return FORMAT_JPEG;
  else if (ext.compare(".PNG") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".png") == 0)
    return FORMAT_PNG;
  else if (ext.compare(".AVI") == 0)
    return FORMAT_AVI;
  else if (ext.compare(".avi") == 0)
    return FORMAT_AVI;
  else if (ext.compare(".MPEG") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".mpeg") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".MPG") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".mpg") == 0)
    return FORMAT_MPEG;
  else if (ext.compare(".MOV") == 0)
    return FORMAT_MOV;
  else if (ext.compare(".mov") == 0)
    return FORMAT_MOV;
  else if (ext.compare(".OGV") == 0)
    return FORMAT_OGV;
  else if (ext.compare(".ogv") == 0)
    return FORMAT_OGV;
  else
    return FORMAT_UNKNOWN;
}

// return the extension of the file including the dot
std::string vpVideoReader::getExtension(const std::string &filename)
{
  // extract the extension
  size_t dot = filename.find_last_of(".");
  std::string ext = filename.substr(dot, filename.size()-1);
  return ext;
}


/*!
  Get the last frame index (update the lastFrame attribute).
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
    if (! lastFrameIndexIsSet) {
      char name[FILENAME_MAX];
      int image_number = firstFrame;
      bool failed;
      do
      {
        std::fstream file;
        sprintf(name,fileName,image_number) ;
        file.open(name, std::ios::in);
        failed = file.fail();
        if (!failed) file.close();
        image_number++;
      }while(!failed);

      lastFrame = image_number - 2;
    }
  }

#ifdef VISP_HAVE_FFMPEG
  else if (ffmpeg != NULL) {
    if (! lastFrameIndexIsSet) {
      lastFrame = (long)(ffmpeg->getFrameNumber() - 1);
    }
  }
  #endif
}
/*!
  Get the first frame index (update the firstFrame attribute).
*/
void
vpVideoReader::findFirstFrameIndex()
{
  if (imSequence != NULL)
  {
    if (! firstFrameIndexIsSet) {
      char name[FILENAME_MAX];
      int image_number = 0;
      bool failed;
      do {
        std::fstream file;
        sprintf(name, fileName, image_number) ;
        file.open(name, std::ios::in);
        failed = file.fail();
        if (!failed) file.close();
        image_number++;
      } while(failed);

      firstFrame = image_number - 1;
      imSequence->setImageNumber(firstFrame);
    }
  }

  #ifdef VISP_HAVE_FFMPEG
  else if (ffmpeg != NULL) {
    if (! firstFrameIndexIsSet) {
      firstFrame = (long)(0);
    }
  }
  #endif
}

/*!
  Return the framerate in Hz used to encode the video stream.

  If the video is a sequence of images, return -1.
  */
double vpVideoReader::getFramerate() const
{
  double framerate = -1.;

#ifdef VISP_HAVE_FFMPEG
  if (ffmpeg != NULL)
    framerate = ffmpeg->getFramerate();
#endif
  return framerate;
}
