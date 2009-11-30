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

#include <visp/vpVideoReader.h>

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
  frameCount = 0;
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
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    ffmpeg->openStream(fileName, vpFFMPEG::COLORED);
    ffmpeg->initStream();
  }
  #endif
  
  getFrame(I,0);
  height = I.getHeight();
  width = I.getWidth();
  
  isOpen = true;
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
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    ffmpeg->openStream(fileName, vpFFMPEG::GRAY_SCALED);
    ffmpeg->initStream();
  }
  #endif
  
  getFrame(I,0);
  height = I.getHeight();
  width = I.getWidth();
  
  isOpen = true;
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
    throw (vpException(vpException::notInitialized,"filename empty"));
  }
  
  getFrame(I,frameCount);
  
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
    throw (vpException(vpException::notInitialized,"filename empty"));
  }
  
  getFrame(I,frameCount);
  
  frameCount++;
}


/*!
  Gets the \f$ frame \f$ th frame and stores it in the image  \f$ I \f$.
  
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