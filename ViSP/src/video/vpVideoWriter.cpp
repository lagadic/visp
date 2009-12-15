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
 * Write image sequences.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpVideoWriter.cpp
  \brief Write image sequences.
*/

#include <visp/vpDebug.h>
#include <visp/vpVideoWriter.h>

/*!
  Basic constructor.
*/
vpVideoWriter::vpVideoWriter()
{
  initFileName = false;
  firstFrame = 0;
  frameCount = 0;
  
  #ifdef VISP_HAVE_FFMPEG
  ffmpeg = NULL;
  codec = CODEC_ID_MPEG1VIDEO;
  bit_rate = 500000;
  #endif
}


/*!
  Basic destructor.
*/
vpVideoWriter::~vpVideoWriter()
{
}


/*!
  It enables to set the path and the name of the files which will be saved.
  
  If you want to write a sequence of images, \f$ filename \f$ corresponds to the path followed by the image name template. For exemple, if you want to write different images named image0001.jpeg, image0002.jpg, ... and located in the folder /local/image, \f$ filename \f$ will be "/local/image/image%04d.jpg". 
  
  \param filename : filename template of an image sequence.
*/
void vpVideoWriter::setFileName(const char *filename)
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
  Sets all the parameters needed to write the video or the image sequence.
  
  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage< vpRGBa > &I)
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
    width = I.getWidth();
    height = I.getHeight();
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    ffmpeg->setBitRate(bit_rate);
    if(!ffmpeg->openEncoder(fileName, I.getWidth(), I.getHeight(), codec))
      throw (vpException(vpException::ioError ,"Could not open the video"));
  }
  
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    vpERROR_TRACE("To write video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  
  frameCount = firstFrame;
  
  isOpen = true;
}


/*!
  Sets all the parameters needed to write the video or the image sequence.
  
  \param I : One image with the right dimensions.
*/
void vpVideoWriter::open(vpImage< unsigned char > &I)
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
    width = I.getWidth();
    height = I.getHeight();
  }
  #ifdef VISP_HAVE_FFMPEG
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    ffmpeg = new vpFFMPEG;
    ffmpeg->setBitRate(bit_rate);
    if(!ffmpeg->openEncoder(fileName, I.getWidth(), I.getHeight(), codec))
      throw (vpException(vpException::ioError ,"Could not open the video"));
  }
  
  #else
  else if (formatType == FORMAT_AVI ||
           formatType == FORMAT_MPEG ||
           formatType == FORMAT_MOV)
  {
    vpERROR_TRACE("To write video files the FFmpeg library has to be installed");
    throw (vpException(vpException::fatalError ,"the FFmpeg library is required"));
  }
  #endif
  
  frameCount = firstFrame;
  
  isOpen = true;
}


/*!
  Saves the image as a frame of the video or as an image belonging to the image sequence.
 
  Each time this method is used, the frame counter is incremented and thus the file name change for the case of an image sequence.
 
  \param I : The image which has to be saved
*/
void vpVideoWriter::saveFrame (vpImage< vpRGBa > &I)
{
  if (!isOpen)
  {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }

  
  if (formatType == FORMAT_PGM ||
      formatType == FORMAT_PPM ||
      formatType == FORMAT_JPEG ||
      formatType == FORMAT_PNG)
  {
    char name[FILENAME_MAX];

    sprintf(name,fileName,frameCount);

    vpImageIo::write(I, name);
  }
  
  #ifdef VISP_HAVE_FFMPEG
  else
  {
    ffmpeg->saveFrame(I);
  }
  #endif

  frameCount++;
}


/*!
  Saves the image as a frame of the video or as an image belonging to the image sequence.
 
  Each time this method is used, the frame counter is incremented and thus the file name change for the case of an image sequence.
 
  \param I : The image which has to be saved
*/
void vpVideoWriter::saveFrame (vpImage< unsigned char > &I)
{
  if (!isOpen)
  {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }

  if (formatType == FORMAT_PGM ||
      formatType == FORMAT_PPM ||
      formatType == FORMAT_JPEG ||
      formatType == FORMAT_PNG)
  {
    char name[FILENAME_MAX];

    sprintf(name,fileName,frameCount);

    vpImageIo::write(I, name);
  }
  
  #ifdef VISP_HAVE_FFMPEG
  else
  {
    ffmpeg->saveFrame(I);
  }
  #endif

  frameCount++;
}


/*!
  Deallocates parameters use to write the video or the image sequence.
*/
void vpVideoWriter::close()
{
  if (!isOpen)
  {
    vpERROR_TRACE("The video has to be open first with the open method");
    throw (vpException(vpException::notInitialized,"file not yet opened"));
  }
  #ifdef VISP_HAVE_FFMPEG
  if (ffmpeg != NULL)
  {
    ffmpeg->endWrite();
  }
  #endif
}


/*!
  Gets the format of the file(s) which has/have to be written.
  
  \return Returns the format.
*/
vpVideoWriter::vpVideoFormatType
vpVideoWriter::getFormat(const char *filename)
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

