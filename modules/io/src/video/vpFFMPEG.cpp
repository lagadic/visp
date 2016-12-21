/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/io/vpFFMPEG.h>
#include <visp3/core/vpImageConvert.h>

#ifdef VISP_HAVE_FFMPEG

extern "C"
{
//#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

/*!
  Basic constructor.
*/
vpFFMPEG::vpFFMPEG()
  : width(-1), height(-1), frameNumber(0), pFormatCtx(NULL), pCodecCtx(NULL),
    pCodec(NULL), pFrame(NULL), pFrameRGB(NULL), pFrameGRAY(NULL), packet(NULL),
    img_convert_ctx(NULL), videoStream(0), numBytes(0), buffer(NULL), index(),
    streamWasOpen(false), streamWasInitialized(false), color_type(COLORED),
    f(NULL), outbuf(NULL), picture_buf(NULL), outbuf_size(0), out_size(0),
    bit_rate(500000), encoderWasOpened(false),
    framerate_stream(-1), framerate_encoder(25)
{
  packet = new AVPacket;
}

/*!
  Basic destructor.
*/
vpFFMPEG::~vpFFMPEG()
{
  closeStream();
  delete packet;
}

/*!
  Allocates and initializes the parameters depending on the video and the desired color type.
  One the stream is opened, it is possible to get the video encoding framerate getFramerate(),
  and the dimension of the images using getWidth() and getHeight().
  
  \param filename : Path to the video which has to be read.
  \param colortype : Desired color map used to open the video.
  The parameter can take two values : COLORED and GRAY_SCALED.
  
  \return It returns true if the paramters could be initialized. Else it returns false.
*/
bool vpFFMPEG::openStream(const char *filename, vpFFMPEGColorType colortype)
{
  this->color_type = colortype;
  
  av_register_all();
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(53,0,0) // libavformat 52.84.0
  if (av_open_input_file (&pFormatCtx, filename, NULL, 0, NULL) != 0)
#else
  if (avformat_open_input (&pFormatCtx, filename, NULL, NULL) != 0) // libavformat 53.4.0
#endif
  {
    vpTRACE("Couldn't open file ");
    return false;
  }

#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(53,21,0) // libavformat 53.21.0
  if (av_find_stream_info (pFormatCtx) < 0)
#else 
  if (avformat_find_stream_info (pFormatCtx, NULL) < 0)
#endif
      return false;
  
  videoStream = 0;
  bool found_codec = false;
  
  /*
  * Detect streams types
  */
  for (unsigned int i = 0; i < pFormatCtx->nb_streams; i++)
  {
#if LIBAVUTIL_VERSION_INT < AV_VERSION_INT(51,0,0)
    if(pFormatCtx->streams[i]->codec->codec_type==CODEC_TYPE_VIDEO) // avutil 50.33.0
#else
    if(pFormatCtx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) // avutil 51.9.1
#endif
    {
      videoStream = i;
      //std::cout << "rate: " << pFormatCtx->streams[i]->r_frame_rate.num << " " << pFormatCtx->streams[i]->r_frame_rate.den << std::endl;
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(55,12,0)
      framerate_stream =  pFormatCtx->streams[i]->r_frame_rate.num;
      framerate_stream /= pFormatCtx->streams[i]->r_frame_rate.den;
#else
      framerate_stream =  pFormatCtx->streams[i]->avg_frame_rate.num;
      framerate_stream /= pFormatCtx->streams[i]->avg_frame_rate.den;
#endif
      found_codec= true;
      break;
    }
  }

  if (found_codec)
  {
    pCodecCtx = pFormatCtx->streams[videoStream]->codec;
    pCodec = avcodec_find_decoder(pCodecCtx->codec_id);

    if (pCodec == NULL)
    {
      vpTRACE("unsuported codec");
      return false;		// Codec not found
    }
    
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(53,35,0) // libavcodec 53.35.0
    if (avcodec_open (pCodecCtx, pCodec) < 0)
#else
    if (avcodec_open2 (pCodecCtx, pCodec, NULL) < 0)
#endif
    {
      vpTRACE("Could not open codec");
      return false;		// Could not open codec
    }

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,34,0)
    pFrame = avcodec_alloc_frame();
#else
    pFrame = av_frame_alloc(); // libavcodec 55.34.1
#endif

    if (color_type == vpFFMPEG::COLORED)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,34,0)
      pFrameRGB=avcodec_alloc_frame();
#else
      pFrameRGB=av_frame_alloc(); // libavcodec 55.34.1
#endif
    
      if (pFrameRGB == NULL)
        return false;
      
      numBytes = avpicture_get_size (PIX_FMT_RGB24,pCodecCtx->width,pCodecCtx->height);
    }
    
    else if (color_type == vpFFMPEG::GRAY_SCALED)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,34,0)
      pFrameGRAY=avcodec_alloc_frame();
#else
      pFrameGRAY=av_frame_alloc(); // libavcodec 55.34.1
#endif
    
      if (pFrameGRAY == NULL)
        return false;
      
      numBytes = avpicture_get_size (PIX_FMT_GRAY8,pCodecCtx->width,pCodecCtx->height);
    }  

    /*
     * Determine required buffer size and allocate buffer
     */
    width = pCodecCtx->width ;
    height = pCodecCtx->height ;
    buffer = (uint8_t *) malloc ((unsigned int)(sizeof (uint8_t)) * (unsigned int)numBytes);
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

  int ret = av_seek_frame(pFormatCtx, (int)videoStream, 0, AVSEEK_FLAG_ANY) ;
  if (ret < 0 )
  {
    vpTRACE("Error rewinding stream for full indexing") ;
    return false ;
  }
  avcodec_flush_buffers(pCodecCtx) ;

  int frame_no = 0 ;
  int frameFinished ;

  av_init_packet(packet);
  while (av_read_frame (pFormatCtx, packet) >= 0)
  {
    if (packet->stream_index == (int)videoStream)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(52,72,2)
      ret = avcodec_decode_video(pCodecCtx, pFrame,
         &frameFinished, packet->data, packet->size);
#else
      ret = avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, packet); // libavcodec >= 52.72.2 (0.6)
#endif
      if (frameFinished)
      {
        if (ret < 0 )
        {
          vpTRACE("Unable to decode video picture");
        }
        index.push_back(packet->dts);
        frame_no++ ;
      }
    }
  }
  
  frameNumber = index.size();
  av_free_packet(packet);
  
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
    av_seek_frame(pFormatCtx, (int)videoStream,targetPts, AVSEEK_FLAG_ANY);
  }
  else
  {
    vpTRACE("Couldn't get a frame");
    return false;
  }
  
  avcodec_flush_buffers(pCodecCtx) ;

  int frameFinished ;

  av_init_packet(packet);
  while (av_read_frame (pFormatCtx, packet) >= 0)
  {
    if (packet->stream_index == (int)videoStream)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(52,72,2)
      avcodec_decode_video(pCodecCtx, pFrame,
                           &frameFinished, packet->data, packet->size);
#else
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, packet); // libavcodec >= 52.72.2 (0.6)
#endif
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

  av_free_packet(packet);
  return true;
}


/*!
  Gets the Next frame in the video.
  
  \param I : The vpImage used to stored the video's frame.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpFFMPEG::acquire(vpImage<vpRGBa> &I)
{
  int frameFinished ;
  
  if (streamWasInitialized == false)
  {
    vpTRACE("Couldn't get a frame. The parameters have to be initialized before ");
    return false;
  }

  av_init_packet(packet);
  while (av_read_frame (pFormatCtx, packet) >= 0)
  {
    if (packet->stream_index == (int)videoStream)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(52,72,2)
      avcodec_decode_video(pCodecCtx, pFrame,
         &frameFinished, packet->data, packet->size);
#else
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, packet); // libavcodec >= 52.72.2 (0.6)
#endif
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
  av_free_packet(packet);
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
    av_seek_frame(pFormatCtx,(int)videoStream,targetPts, AVSEEK_FLAG_ANY);
  }
  else
  {
    vpTRACE("Couldn't get a frame");
    return false;
  }
  
  avcodec_flush_buffers(pCodecCtx) ;

  int frameFinished ;

  av_init_packet(packet);
  while (av_read_frame (pFormatCtx, packet) >= 0)
  {
    if (packet->stream_index == (int)videoStream)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(52,72,2)
      avcodec_decode_video(pCodecCtx, pFrame,
         &frameFinished, packet->data, packet->size);
#else
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, packet); // libavcodec >= 52.72.2 (0.6)
#endif
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

    av_free_packet(packet);
    return true;
}


/*!
  Gets the Next frame in the video.
  
  \param I : The vpImage used to stored the video's frame.
  
  \return It returns true if the frame could be read. Else it returns false.
*/
bool vpFFMPEG::acquire(vpImage<unsigned char> &I)
{
  int frameFinished ;
  
  if (streamWasInitialized == false)
  {
    vpTRACE("Couldn't get a frame. The parameters have to be initialized before ");
    return false;
  }

  av_init_packet(packet);
  while (av_read_frame (pFormatCtx, packet) >= 0)
  {
    if (packet->stream_index == (int)videoStream)
    {
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(52,72,2)
      avcodec_decode_video(pCodecCtx, pFrame,
         &frameFinished, packet->data, packet->size);
#else
      avcodec_decode_video2(pCodecCtx, pFrame, &frameFinished, packet); // libavcodec >= 52.72.2 (0.6)
#endif
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
  av_free_packet(packet);
  return true;
}


/*!
  This method enable to fill the vpImage bitmap thanks to the selected frame.
  
  \throw vpException::dimensionError if either the height or the width 
  associated to the class is negative. 
  
  \param I : the image to fill. 
*/
void vpFFMPEG::copyBitmap(vpImage<vpRGBa> &I)
{
  if(height < 0 || width < 0){
    throw vpException(vpException::dimensionError, "width or height negative.");
  }
  I.resize((unsigned int)height, (unsigned int)width);
  
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
  
  \throw vpException::dimensionError if either the height or the width 
  associated to the class is negative. 
  
  \param I : the image to fill. 
*/
void vpFFMPEG::copyBitmap(vpImage<unsigned char> &I)
{
  if(height < 0 || width < 0){
    throw vpException(vpException::dimensionError, "width or height negative.");
  }
  I.resize((unsigned int)height, (unsigned int)width);
  
  unsigned char* beginOutput = (unsigned char*)I.bitmap;

  if (color_type == GRAY_SCALED)
  {
    unsigned char* input = (unsigned char*)pFrameGRAY->data[0];
    int widthStep = pFrameGRAY->linesize[0];
    for(int i=0 ; i < height ; i++)
    {
      unsigned char *line = input;
      unsigned char *output = beginOutput + width * i;
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
      vpImageConvert::RGBToGrey(input + i*widthStep, beginOutput + i*width, (unsigned int)width, 1, false);
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
    if (buffer != NULL) {
      free(buffer);
      buffer = NULL;
    }
    
    if (color_type == vpFFMPEG::COLORED)
      av_free(pFrameRGB);
    
    else if (color_type == vpFFMPEG::GRAY_SCALED)
      av_free(pFrameGRAY);

    // Free the YUV frame
    av_free(pFrame);

    // Close the codec
    if (pCodecCtx) avcodec_close(pCodecCtx);

    // Close the video file
#if LIBAVFORMAT_VERSION_INT < AV_VERSION_INT(53,17,0) // libavformat 53.17.0
    av_close_input_file(pFormatCtx);
#else
    avformat_close_input(&pFormatCtx);
#endif
  }
  streamWasOpen = false;
  
  if (encoderWasOpened)
  {
    if(f!=NULL) endWrite();
    
    if(buffer!=NULL) delete[] buffer;
    
    if(outbuf != NULL) delete[] outbuf;
    
    if(picture_buf != NULL) delete[] picture_buf;
    
    av_free(pFrameRGB);
    av_free(pFrame);
    if (pCodecCtx) avcodec_close(pCodecCtx);
  }
  
  encoderWasOpened = false;

  if(streamWasInitialized || encoderWasOpened){
    sws_freeContext (img_convert_ctx);
  }
  streamWasInitialized = false;
}

/*!
  Allocates and initializes the parameters depending on the video to write.
  
  \param filename : Path to the video which has to be writen.
  \param w,h : Width and height of the image which will be saved.
  \param codec : Type of codec used to encode the video.
  
  By default codec is set to AV_CODEC_ID_MPEG1VIDEO. But if installed, you can use one of the
  AVCodecID proposed by ffmpeg such as : AV_CODEC_ID_MPEG2VIDEO, AV_CODEC_ID_MPEG2VIDEO_XVMC,
  AV_CODEC_ID_MPEG4, AV_CODEC_ID_H264, ... (More AVCodecID can be found in the ffmpeg documentation).
  
  Of course to use the codec it must be installed on your computer.
  
  \return It returns true if the paramters could be initialized. Else it returns false.
*/
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,51,110) // libavcodec 54.51.100
bool vpFFMPEG::openEncoder(const char *filename, unsigned int w, unsigned int h, CodecID codec)
#else
bool vpFFMPEG::openEncoder(const char *filename, unsigned int w, unsigned int h, AVCodecID codec)
#endif
{
  av_register_all();

  /* find the mpeg1 video encoder */
  pCodec = avcodec_find_encoder(codec);
  if (pCodec == NULL) {
    fprintf(stderr, "codec not found\n");
    return false;
  }

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(53,5,0) // libavcodec 53.5.0
  pCodecCtx = avcodec_alloc_context();
#else
  pCodecCtx = avcodec_alloc_context3(NULL);
#endif

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55,34,0)
  pFrame = avcodec_alloc_frame();
  pFrameRGB = avcodec_alloc_frame();
#else
  pFrame = av_frame_alloc(); // libavcodec 55.34.1
  pFrameRGB = av_frame_alloc(); // libavcodec 55.34.1
#endif

  /* put sample parameters */
  pCodecCtx->bit_rate = (int)bit_rate;
  /* resolution must be a multiple of two */
  pCodecCtx->width = (int)w;
  pCodecCtx->height = (int)h;
  this->width = (int)w;
  this->height = (int)h;
  /* frames per second */
  pCodecCtx->time_base.num = 1;
  pCodecCtx->time_base.den = framerate_encoder;
  pCodecCtx->gop_size = 10; /* emit one intra frame every ten frames */
  pCodecCtx->max_b_frames=1;
  pCodecCtx->pix_fmt = PIX_FMT_YUV420P;

  /* open it */
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(53,35,0) // libavcodec 53.35.0
  if (avcodec_open (pCodecCtx, pCodec) < 0) {
#else
  if (avcodec_open2 (pCodecCtx, pCodec, NULL) < 0) {
#endif
    fprintf(stderr, "could not open codec\n");
    return false;
  }

  /* the codec gives us the frame size, in samples */

  f = fopen(filename, "wb");
  if (!f) {
    fprintf(stderr, "could not open %s\n", filename);
    return false;
  }

  outbuf_size = 100000;
  outbuf = new uint8_t[outbuf_size];

  numBytes = avpicture_get_size (PIX_FMT_YUV420P,pCodecCtx->width,pCodecCtx->height);
  picture_buf = new uint8_t[numBytes];
  avpicture_fill((AVPicture *)pFrame, picture_buf, PIX_FMT_YUV420P, pCodecCtx->width, pCodecCtx->height);

  numBytes = avpicture_get_size (PIX_FMT_RGB24,pCodecCtx->width,pCodecCtx->height);
  buffer = new uint8_t[numBytes];
  avpicture_fill((AVPicture *)pFrameRGB, buffer, PIX_FMT_RGB24, pCodecCtx->width, pCodecCtx->height);

  img_convert_ctx= sws_getContext(pCodecCtx->width, pCodecCtx->height, PIX_FMT_RGB24, pCodecCtx->width,pCodecCtx->height,PIX_FMT_YUV420P, SWS_BICUBIC, NULL, NULL, NULL);
  
  encoderWasOpened = true;

  return true;
}


/*!
  Saves the image I as frame of the video.
  
  \param I : the image to save.
  
  \return It returns true if the image could be saved.
*/
bool vpFFMPEG::saveFrame(vpImage<vpRGBa> &I)
{
  if (encoderWasOpened == false)
  {
    vpTRACE("Couldn't save a frame. The parameters have to be initialized before ");
    return false;
  }
  
  writeBitmap(I);
  sws_scale(img_convert_ctx, pFrameRGB->data, pFrameRGB->linesize, 0, pCodecCtx->height, pFrame->data, pFrame->linesize);
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,2,100) // libavcodec 54.2.100
  out_size = avcodec_encode_video(pCodecCtx, outbuf, outbuf_size, pFrame);
  fwrite(outbuf, 1, (size_t)out_size, f);
#else
  AVPacket pkt;
  av_init_packet(&pkt);
  pkt.data = NULL;    // packet data will be allocated by the encoder
  pkt.size = 0;

  int got_output;
  int ret = avcodec_encode_video2(pCodecCtx, &pkt, pFrame, &got_output);
  if (ret < 0) {
    std::cerr << "Error encoding frame in " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__ << std::endl;
    return false;
  }
  if (got_output) {
    fwrite(pkt.data, 1, pkt.size, f);
    av_free_packet(&pkt);
  }
#endif
  fflush(stdout);
  return true;
}


/*!
  Saves the image I as frame of the video.
  
  \param I : the image to save.
  
  \return It returns true if the image could be saved.
*/
bool vpFFMPEG::saveFrame(vpImage<unsigned char> &I)
{
  if (encoderWasOpened == false)
  {
    vpTRACE("Couldn't save a frame. The parameters have to be initialized before ");
    return false;
  }
  
  writeBitmap(I);
  sws_scale(img_convert_ctx, pFrameRGB->data, pFrameRGB->linesize, 0, pCodecCtx->height, pFrame->data, pFrame->linesize);  
#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,2,100) // libavcodec 54.2.100
  out_size = avcodec_encode_video(pCodecCtx, outbuf, outbuf_size, pFrame);
  fwrite(outbuf, 1, (size_t)out_size, f);
#else
  AVPacket pkt;
  av_init_packet(&pkt);
  pkt.data = NULL;    // packet data will be allocated by the encoder
  pkt.size = 0;

  int got_output;
  int ret = avcodec_encode_video2(pCodecCtx, &pkt, pFrame, &got_output);
  if (ret < 0) {
    std::cerr << "Error encoding frame in " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__ << std::endl;
    return false;
  }
  if (got_output) {
    fwrite(pkt.data, 1, pkt.size, f);
    av_free_packet(&pkt);
  }
#endif

  fflush(stdout);
  return true;
}

/*!
  Ends the writing of the video and close the file.
  
  \return It returns true if the file was closed without problem
*/
bool vpFFMPEG::endWrite()
{
  if (encoderWasOpened == false)
  {
    vpTRACE("Couldn't save a frame. The parameters have to be initialized before ");
    return false;
  }
  
  int ret = 1;
  while (ret != 0)
  {

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54,2,100) // libavcodec 54.2.100
    ret = avcodec_encode_video(pCodecCtx, outbuf, outbuf_size, NULL);
    fwrite(outbuf, 1, (size_t)out_size, f);
#else
    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = NULL;    // packet data will be allocated by the encoder
    pkt.size = 0;
    int got_output;
    ret = avcodec_encode_video2(pCodecCtx, &pkt, NULL, &got_output);
    if (ret < 0) {
      std::cerr << "Error encoding frame in " << __FILE__ << " " << __LINE__ << " " << __FUNCTION__ << std::endl;
      return false;
    }
    if (got_output) {
      fwrite(pkt.data, 1, pkt.size, f);
      av_free_packet(&pkt);
    }
#endif
  }

  /*The end of a mpeg file*/
  outbuf[0] = 0x00;
  outbuf[1] = 0x00;
  outbuf[2] = 0x01;
  outbuf[3] = 0xb7;
  fwrite(outbuf, 1, 4, f);
  fclose(f);
  f = NULL;
  return true;
}

/*!
  This method enables to fill the frame bitmap thanks to the vpImage bitmap.
*/
void vpFFMPEG::writeBitmap(vpImage<vpRGBa> &I)
{
  unsigned char* beginInput = (unsigned char*)I.bitmap;
  unsigned char* beginOutput = (unsigned char*)pFrameRGB->data[0];
  int widthStep = pFrameRGB->linesize[0];
  
  for(int i=0 ; i < height ; i++)
  {
    unsigned char *input = beginInput + 4 * i * width;
    unsigned char *output = beginOutput + i * widthStep;
    for(int j=0 ; j < width ; j++)
    {
      *(output++) = *(input);
      *(output++) = *(input+1);
      *(output++) = *(input+2);

      input+=4;
    }
  }
}


/*!
  This method enables to fill the frame bitmap thanks to the vpImage bitmap.
*/
void vpFFMPEG::writeBitmap(vpImage<unsigned char> &I)
{
  unsigned char* beginInput = (unsigned char*)I.bitmap;
  unsigned char* beginOutput = (unsigned char*)pFrameRGB->data[0];
  int widthStep = pFrameRGB->linesize[0];
  
  for(int i=0 ; i < height ; i++)
  {
    unsigned char *input = beginInput + i * width;
    unsigned char *output = beginOutput + i * widthStep;
    for(int j=0 ; j < width ; j++)
    {
      *(output++) = *(input);
      *(output++) = *(input);
      *(output++) = *(input);

      input++;
    }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpFFMPEG.cpp.o) has no symbols
void dummy_vpFFMPEG() {};
#endif
