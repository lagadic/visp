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
  \file vpFFMPEG.cpp
  \brief Class that manages the FFMPEG library
*/

#include <stdio.h>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpFFMPEG.h>
#include <visp/vpImageConvert.h>

#ifdef VISP_HAVE_FFMPEG

/*!
  Basic constructor.
*/
vpFFMPEG::vpFFMPEG()
{
  frameNumber = 0;
  width = -1;
  height = -1;
  streamWasOpen = false;
  streamWasInitialized = false;
}

/*!
  Basic destructor.
*/
vpFFMPEG::~vpFFMPEG()
{
  closeStream();
}

/*!
  Allocates and initializes the parameters depending on the video and the desired color type.
  
  \param filename : Path to the video which has to be read.
  \param color_type : Desired color map used to open the video.
  The parameter can take two values : COLORED and GRAY_SCALED.
  
  \return It returns true if the paramters could be initialized. Else it returns false.
*/
bool vpFFMPEG::openStream(const char *filename, vpFFMPEGColorType color_type)
{
  this->color_type = color_type;
  
  av_register_all();
  if (av_open_input_file (&pFormatCtx, filename, NULL, 0, NULL) != 0)
  {
    vpTRACE("Couldn't open file ");
    return false;
  }

  if (av_find_stream_info (pFormatCtx) < 0)
      return false;
  
  videoStream = -1;
  
  /*
  * Detect streams types
  */
  for (unsigned int i = 0; i < pFormatCtx->nb_streams; i++)
  {
    if(pFormatCtx->streams[i]->codec->codec_type==CODEC_TYPE_VIDEO)
    {
      videoStream = i;
      break;
    }
  }

  if (videoStream != -1)
  {
    pCodecCtx = pFormatCtx->streams[videoStream]->codec;
    pCodec = avcodec_find_decoder(pCodecCtx->codec_id);

    if (pCodec == NULL)
    {
      vpTRACE("unsuported codec");
      return false;		// Codec not found
    }
    
    if (avcodec_open (pCodecCtx, pCodec) < 0)
    {
      vpTRACE("Could not open codec");
      return false;		// Could not open codec
    }

    pFrame = avcodec_alloc_frame();
    
    if (color_type == vpFFMPEG::COLORED)
    {
      pFrameRGB=avcodec_alloc_frame();
    
      if (pFrameRGB == NULL)
        return false;
      
      numBytes = avpicture_get_size (PIX_FMT_RGB24,pCodecCtx->width,pCodecCtx->height);
    }
    
    else if (color_type == vpFFMPEG::GRAY_SCALED)
    {
      pFrameGRAY=avcodec_alloc_frame();
    
      if (pFrameGRAY == NULL)
        return false;
      
      numBytes = avpicture_get_size (PIX_FMT_GRAY8,pCodecCtx->width,pCodecCtx->height);
    }  

    /*
     * Determine required buffer size and allocate buffer
     */
    width = pCodecCtx->width ;
    height = pCodecCtx->height ;
    buffer = (uint8_t *) malloc (sizeof (uint8_t) * numBytes);
  }
  else
  {
    vpTRACE("Didn't find a video stream");
    return false;
  }
  
  if (color_type == vpFFMPEG::COLORED)
    avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);
  
  else if (color_type == vpFFMPEG::GRAY_SCALED)
    avpicture_fill((AVPicture *)pFrameGRAY, buffer, PIX_FMT_GRAY8, pCodecCtx->width, pCodecCtx->height);
  
  streamWasOpen = true;

  return true;
}

/*!
  This method initializes the conversion parameters.
  
  It browses the video and lists all the frame. It sets the number of frame in the video.
  
  \returns It returns true if the method was executed without any problem. Else it returns false.
*/
bool vpFFMPEG::initStream()
{
  if (color_type == vpFFMPEG::COLORED)
    img_convert_ctx= sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,pCodecCtx->height,PIX_FMT_RGB24, SWS_BICUBIC, NULL, NULL, NULL);
  
  else if (color_type == vpFFMPEG::GRAY_SCALED)
    img_convert_ctx= sws_getContext(pCodecCtx->width, pCodecCtx->height, pCodecCtx->pix_fmt, pCodecCtx->width,pCodecCtx->height,PIX_FMT_GRAY8, SWS_BICUBIC, NULL, NULL, NULL);

  int ret = av_seek_frame(pFormatCtx,videoStream, 0, AVSEEK_FLAG_BACKWARD) ;
  if (ret < 0 )
  {
    vpTRACE("Error rewinding stream for full indexing") ;
    return false ;
  }
  avcodec_flush_buffers(pCodecCtx) ;

  int frame_no = 0 ;
  int frameFinished ;

  while (av_read_frame (pFormatCtx, &packet) >= 0)
  {
    if (packet.stream_index == videoStream)
    {
      ret = avcodec_decode_video(pCodecCtx, pFrame, &frameFinished,packet.data, packet.size);

      if (frameFinished)
      {
        if (ret < 0 )
        {
          vpTRACE("Unable to decode video picture");
        }
        index.push_back(packet.pts);
        frame_no++ ;
      }
    }
  }
  
  frameNumber = index.size();
  av_free_packet(&packet);
  
  streamWasInitialized = true;
  
  return true;
}


/*!
  Gets the \f$ frame \f$ th frame from the video and stores it in the image  \f$ I \f$.
  
  \param I : The vpImage used to stored the video's frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpFFMPEG::getFrame(vpImage<vpRGBa> &I, unsigned int frame)
{

  if (frame < frameNumber && streamWasInitialized== true)
  {
    int64_t targetPts = index[frame];
    av_seek_frame(pFormatCtx,videoStream,targetPts, AVSEEK_FLAG_BACKWARD);
  }
  else
  {
    vpTRACE("Couldn't get a frame");
    return false;
  }
  
  avcodec_flush_buffers(pCodecCtx) ;

  int frameFinished ;

  while (av_read_frame (pFormatCtx, &packet) >= 0)
  {
    if (packet.stream_index == videoStream)
    {
      avcodec_decode_video(pCodecCtx, pFrame, &frameFinished, packet.data, packet.size);
      if (frameFinished)
      {
	if (color_type == vpFFMPEG::COLORED)
          sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);
	else if (color_type == vpFFMPEG::GRAY_SCALED)
          sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameGRAY->data, pFrameGRAY->linesize);

          copyBitmap(I);
          break;
        }
      }
    }

    av_free_packet(&packet);
    return true;
}


/*!
  Gets the \f$ frame \f$ th frame from the video and stores it in the image  \f$ I \f$.
  
  \param I : The vpImage used to stored the video's frame.
  \param frame : The index of the frame which has to be read.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpFFMPEG::getFrame(vpImage<unsigned char> &I, unsigned int frame)
{

  if (frame < frameNumber && streamWasInitialized== true)
  {
    int64_t targetPts = index[frame];
    av_seek_frame(pFormatCtx,videoStream,targetPts, AVSEEK_FLAG_BACKWARD);
  }
  else
  {
    vpTRACE("Couldn't get a frame");
    return false;
  }
  
  avcodec_flush_buffers(pCodecCtx) ;

  int frameFinished ;

  while (av_read_frame (pFormatCtx, &packet) >= 0)
  {
    if (packet.stream_index == videoStream)
    {
      avcodec_decode_video(pCodecCtx, pFrame, &frameFinished, packet.data, packet.size);
      if (frameFinished)
      {
        if (color_type == vpFFMPEG::COLORED)
          sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data, pFrameRGB->linesize);
	else if (color_type == vpFFMPEG::GRAY_SCALED)
          sws_scale(img_convert_ctx, pFrame->data, pFrame->linesize, 0, pCodecCtx->height, pFrameGRAY->data, pFrameGRAY->linesize);

          copyBitmap(I);
          break;
        }
      }
    }

    av_free_packet(&packet);
    return true;
}


/*!
  This method enable to fill the vpImage bitmap thanks to the selected frame.
*/
void vpFFMPEG::copyBitmap(vpImage<vpRGBa> &I)
{
  I.resize(height,width);
  
  unsigned char* line;
  unsigned char* beginOutput = (unsigned char*)I.bitmap;
  unsigned char* output = NULL;

  if (color_type == COLORED)
  {
    unsigned char* input = (unsigned char*)pFrameRGB->data[0];
    int widthStep = pFrameRGB->linesize[0];
    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + 4 * width * i;
      for(int j=0 ; j < width ; j++)
      {
        *(output++) = *(line);
        *(output++) = *(line+1);
        *(output++) = *(line+2);
        *(output++) = 0;

        line+=3;
      }
    //go to the next line
    input+=widthStep;
    }
  }
  
  else if (color_type == GRAY_SCALED)
  {
    unsigned char* input = (unsigned char*)pFrameGRAY->data[0];
    int widthStep = pFrameGRAY->linesize[0];
    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + 4 * width * i;
      for(int j=0 ; j < width ; j++)
        {
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);
          *output++ = *(line);;

          line++;
        }
      //go to the next line
      input+=widthStep;
    }
  }
}

/*!
  This method enable to fill the vpImage bitmap thanks to the selected frame.
*/
void vpFFMPEG::copyBitmap(vpImage<unsigned char> &I)
{
  I.resize(height,width);
  
  unsigned char* line;
  unsigned char* beginOutput = (unsigned char*)I.bitmap;
  unsigned char* output = NULL;

  if (color_type == GRAY_SCALED)
  {
    unsigned char* input = (unsigned char*)pFrameGRAY->data[0];
    int widthStep = pFrameGRAY->linesize[0];
    for(int i=0 ; i < height ; i++)
    {
      line = input;
      output = beginOutput + width * i;
      for(int j=0 ; j < width ; j++)
      {
        *(output++) = *(line);

        line++;
      }
    //go to the next line
    input+=widthStep;
    }
  }
  
  if (color_type == COLORED)
  {
    unsigned char* input = (unsigned char*)pFrameRGB->data[0];
    int widthStep = pFrameRGB->linesize[0];
    for (int i = 0  ; i < height ; i++)
    {
      vpImageConvert::RGBToGrey(input + i*widthStep, beginOutput + i*width,width,1,false);
    }
  }
}

/*!
  Deallocates all the FFMPEG parameters.
*/
void vpFFMPEG::closeStream()
{
  if (streamWasOpen)
  {
    av_free(buffer);
    
    if (color_type == vpFFMPEG::COLORED)
      av_free(pFrameRGB);
    
    else if (color_type == vpFFMPEG::GRAY_SCALED)
      av_free(pFrameGRAY);

    // Free the YUV frame
    av_free(pFrame);

    // Close the codec
    avcodec_close(pCodecCtx);

    // Close the video file
    av_close_input_file(pFormatCtx);
  }
  streamWasOpen = false;
}

#endif