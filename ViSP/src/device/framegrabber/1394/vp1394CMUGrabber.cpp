/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Firewire cameras video capture based on CMU 1394 Digital Camera SDK.
 *
 * Authors:
 * Lucas Lopes Lemos FEMTO-ST, AS2M departement, Besancon
 * Guillaume Laurent FEMTO-ST, AS2M departement, Besancon
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_CMU1394

#include <iostream>

#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vp1394CMUGrabber.h>

/*!
   Basic constructor.
 */
vp1394CMUGrabber::vp1394CMUGrabber()
{	
  // public members
  init = false;

  // protected members
  width = height = -1;

  // private members
  camera = new C1394Camera;
  index = 0;   // If a camera was not selected the first one (index = 0) will be used
  _format = _mode = _fps = -1;
  _modeauto=true;

}

/*!
  Destructor.
  */
vp1394CMUGrabber::~vp1394CMUGrabber( )
{
  close();
  // delete camera instance
  delete camera;

}

/*!
 Select the camera on the bus from its index. The first camera found on the bus has index 0.
 \param cam_id : Camera index.
*/
void 
vp1394CMUGrabber::selectCamera(int cam_id)
{
  int camerror;

  index = cam_id ;

  camerror = camera->SelectCamera(index);
  if ( camerror!= CAM_SUCCESS)
  {
    switch (camerror)
    {
      case CAM_ERROR_PARAM_OUT_OF_RANGE:
        vpERROR_TRACE("vp1394CMUGrabber error: Found no camera number %i",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"The required camera is not present") );
        break;
      case CAM_ERROR_BUSY:
        vpERROR_TRACE("vp1394CMUGrabber error: The camera %i is busy",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"The required camera is in use by other application") );
        break;
      case CAM_ERROR:
        vpERROR_TRACE("vp1394CMUGrabber error: General I/O error when selecting camera number %i",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"Resolve camera can not be used") );
        break;
    }
    close();
  }
} // end camera select

/*!
 Init the selected camera.
 */
void 
vp1394CMUGrabber::initCamera()
{
  int camerror;
  unsigned long width, height;


  if (camera->CheckLink() != CAM_SUCCESS)
  {
    vpERROR_TRACE("C1394Camera error: Found no cameras on the 1394 bus");
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"The is no detected camera") );
  }

  camerror = camera->InitCamera();
  if ( camerror != CAM_SUCCESS )
  {
    switch (camerror)
    {
      case CAM_ERROR_NOT_INITIALIZED:
        vpERROR_TRACE("vp1394CMUGrabber error: No camera selected",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"The is no selected camera") );
        break;
      case CAM_ERROR_BUSY:
        vpERROR_TRACE("vp1394CMUGrabber error: The camera %i is busy",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"The required camera is in use by other application") );
        break;
      case CAM_ERROR:
        vpERROR_TRACE("vp1394CMUGrabber error: General I/O error when selecting camera number %i",index);
        throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"Resolve camera can not be used") );
        break;
    }
    close();
  }

  if (camera->Has1394b())
    camera->Set1394b(TRUE);

  // Set format and mode
  if ((_format != -1) && (_mode != -1))
  {
    if (!camera->HasVideoMode(_format, _mode))
    {
      close();
      vpERROR_TRACE("vp1394CMUGrabber error: The image format is not supported by the IEEE 1394 camera number %i",index);
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,"Video mode not supported") );
    }

    if (camera->SetVideoFormat(_format) != CAM_SUCCESS)
    {
      close();
      vpERROR_TRACE("vp1394CMUGrabber error: Can't set video format of IEEE 1394 camera number %i",index);
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,"Can't set video format") );
    }

    if (camera->SetVideoMode(_mode) != CAM_SUCCESS)
    {
      close();
      vpERROR_TRACE("vp1394CMUGrabber error: Can't set video mode of IEEE 1394 camera number %i",index);
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,"Can't set video mode") );
    }
  }
  else {
    // Get the current format and mode
    _format = camera->GetVideoFormat();
    _mode = camera->GetVideoMode();
  }

  // Update the color coding
  _color = getVideoColorCoding();
  //std::cout << "color coding: " << _color << std::endl;

  // Set fps
  if (_fps!=-1)
  {
    if (!camera->HasVideoFrameRate(_format,_mode,_fps))
    {
      close();
      vpERROR_TRACE("vp1394CMUGrabber error: The frame rate is not supported by the IEEE 1394 camera number %i for the selected image format",index);
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,"The frame rate is not supported") );
    }

    if (camera->SetVideoFrameRate(_fps) != CAM_SUCCESS)
    {
      close();
      vpERROR_TRACE("vp1394CMUGrabber error: Can't set video frame rate of IEEE 1394 camera number %i",index);
      throw (vpFrameGrabberException(vpFrameGrabberException::settingError,"Can't set video frame rate") );
    }
  }

  // Set shutter and gain
  if ( _modeauto == false )
  {
    unsigned short min,max;
    C1394CameraControl *Control;

    Control = camera->GetCameraControl(FEATURE_GAIN);
    Control->Inquire();
    Control->GetRange(&min,&max);

    if (_gain<min)
    {
      _gain = min;
      std::cout << "vp1394CMUGrabber warning: Desired gain register value of IEEE 1394 camera number " << index << " can't be less than " << _gain << std::endl;
    } else
      if (_gain>max)
      {
        _gain = max;
        std::cout << "vp1394CMUGrabber warning: Desired gain register value of IEEE 1394 camera number " << index << " can't be greater than " << _gain << std::endl;
      }

    Control->SetAutoMode(false);
    if(Control->SetValue(_gain) != CAM_SUCCESS)
    {
      std::cout << "vp1394CMUGrabber warning: Can't set gain register value of IEEE 1394 camera number " << index << std::endl;
    }

    Control = camera->GetCameraControl(FEATURE_SHUTTER);
    Control->Inquire();
    Control->GetRange(&min,&max);

    if (_shutter<min)
    {
      _shutter = min;
      std::cout << "vp1394CMUGrabber warning: Desired exposure time register value of IEEE 1394 camera number " << index << " can't be less than " << _shutter << std::endl;
    }
    else if (_shutter>max)
    {
      _shutter = max;
      std::cout << "vp1394CMUGrabber warning: Desired exposure time register value of IEEE 1394 camera number " << index << " can't be greater than " << _shutter << std::endl;
    }
    Control->SetAutoMode(false);
    if(Control->SetValue(_shutter) != CAM_SUCCESS)
    {
      std::cout << "vp1394CMUGrabber warning: Can't set exposure time register value of IEEE 1394 camera number " << index << std::endl;
    }
  }
  else
  {
    camera->GetCameraControl(FEATURE_SHUTTER)->SetAutoMode(true);
    camera->GetCameraControl(FEATURE_GAIN)->SetAutoMode(true);
  }

  // Set trigger off
  camera->GetCameraControlTrigger()->SetOnOff(false);


  camera->GetVideoFrameDimensions(&width,&height);
  this->height = height;
  this->width = width;

  init = true;

} // end camera init


/*!
  Initialization of the grabber using a greyscale image.
  \param I : gray level image.
  */
void 
vp1394CMUGrabber::open(vpImage<unsigned char> &I)
{
  initCamera();
  I.init(height,width);

  // start acquisition
  if (camera->StartImageAcquisition() != CAM_SUCCESS)
  {
    close();
    vpERROR_TRACE("vp1394CMUGrabber error: Can't start image acquisition from IEEE 1394 camera number %i",index);
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
                                   "Error while strating image acquisition") );
  }
}

/*!
  Initialization of the grabber using a color image.
  \param I : color image.
  */
void 
vp1394CMUGrabber::open(vpImage<vpRGBa> &I)
{
  initCamera();
  I.init(this->height,this->width);

  // start acquisition
  if (camera->StartImageAcquisition() != CAM_SUCCESS)
  {
    close();
    vpERROR_TRACE("vp1394CMUGrabber error: Can't start image acquisition from IEEE 1394 camera number %i",index);
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
                                   "Error while strating image acquisition") );
  }
}

/*!
  Grabs a grayscale image from the selected camera. If the camera color
  coding differs from vp1394CMUGrabber::MONO8, the acquired image is
  converted in a gray level image to match the requested format.

  \param I : Acquired gray level image.
  */
void 
vp1394CMUGrabber::acquire(vpImage<unsigned char> &I)
{
  // get image data
  unsigned long length;
  unsigned char *rawdata = NULL ;
  int dropped;
  unsigned int size;

  if(init == false){
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }

  camera->AcquireImageEx(TRUE,&dropped);
  rawdata = camera->GetRawData(&length);

  size = I.getWidth() * I.getHeight();
  switch(_color) {
    case vp1394CMUGrabber::MONO8:
      memcpy(I.bitmap, (unsigned char *) rawdata, size);
      break;
    case vp1394CMUGrabber::MONO16:
      vpImageConvert::MONO16ToGrey(rawdata, I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV411:
      vpImageConvert::YUV411ToGrey(rawdata, I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV422:
      vpImageConvert::YUV422ToGrey(rawdata, I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV444:
      vpImageConvert::YUV444ToGrey(rawdata, I.bitmap, size);
      break;

    case vp1394CMUGrabber::RGB8:
      vpImageConvert::RGBToGrey(rawdata, I.bitmap, size);
      break;

    default:
      close();
      vpERROR_TRACE("Format conversion not implemented. Acquisition failed.");
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
                                     "Format conversion not implemented. "
                                     "Acquisition failed.") );
      break;
  };

  //unsigned short depth = 0;
  //camera->GetVideoDataDepth(&depth);
  //std::cout << "depth: " << depth << " computed: " << (float)(length/(I.getHeight() * I.getWidth())) <<  std::endl;


  //memcpy(I.bitmap,rawdata,length);

}

/*!
  Grabs a color image from the selected camera. Since the cameras
  are not able to grab RGBa color coding format, the acquired image is
  converted in a RGBa to match the requested format. This transformation
  could be time consuming.

  \param I : Acquired color image in RGBa format.
 */
void 
vp1394CMUGrabber::acquire(vpImage<vpRGBa> &I)
{
  // get image data
  unsigned long length;
  unsigned char *rawdata = NULL;
  int dropped;
  unsigned int size;

  if(init == false){
    close();
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,
                                   "Initialization not done") );
  }

  camera->AcquireImageEx(TRUE,&dropped);
  rawdata = camera->GetRawData(&length);
  size = I.getWidth() * I.getHeight();

  switch (_color) {
    case vp1394CMUGrabber::MONO8:
      vpImageConvert::GreyToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    case vp1394CMUGrabber::MONO16:
      vpImageConvert::MONO16ToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV411:
      vpImageConvert::YUV411ToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV422:
      vpImageConvert::YUV422ToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    case vp1394CMUGrabber::YUV444:
      vpImageConvert::YUV444ToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    case vp1394CMUGrabber::RGB8:
      size = length / 3;
      vpImageConvert::RGBToRGBa(rawdata, (unsigned char *)I.bitmap, size);
      break;

    default:
      close();
      vpERROR_TRACE("Format conversion not implemented. Acquisition failed.");
      throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
                                     "Format conversion not implemented. "
                                     "Acquisition failed.") );
      break;
  };
}

/*!
  Stop the acquisition of images and free the camera.
  */
void 
vp1394CMUGrabber::close()
{
  // stop acquisition
  camera->StopImageAcquisition();

  if (camera->StopImageAcquisition() != CAM_SUCCESS)
  {
    throw (vpFrameGrabberException(vpFrameGrabberException::otherError,
                                   "vp1394CMUGrabber error: Can't stop image acquisition from IEEE 1394 camera") );
  }

  init = false;

}

/*!
 Set the gain and the shutter values. This method has to be called before open().
 */
void
vp1394CMUGrabber::setControl(unsigned short gain, unsigned short shutter)
{
  /*
  unsigned short min,max;
  C1394CameraControl *Control;

  if ( modeauto == false )
  {
    Control = camera->GetCameraControl(FEATURE_GAIN);
    Control->Inquire();
    Control->GetRange(&min,&max);

    if (gain<min)
    {
      gain = min;
      std::cout << "vp1394CMUGrabber warning: Desired gain register value of IEEE 1394 camera number " << index << " can't be less than " << gain << std::endl;
    } else
    if (gain>max)
    {
      gain = max;
      std::cout << "vp1394CMUGrabber warning: Desired gain register value of IEEE 1394 camera number " << index << " can't be greater than " << gain << std::endl;
    }

    Control->SetAutoMode(false);
    if(Control->SetValue(gain) != CAM_SUCCESS)
    {
      std::cout << "vp1394CMUGrabber warning: Can't set gain register value of IEEE 1394 camera number " << index << std::endl;
    }

    Control = camera->GetCameraControl(FEATURE_SHUTTER);
    Control->Inquire();
    Control->GetRange(&min,&max);

    if (shutter<min)
    {
      shutter = min;
      std::cout << "vp1394CMUGrabber warning: Desired exposure time register value of IEEE 1394 camera number " << index << " can't be less than " << shutter << std::endl;
    }
    else if (shutter>max)
    {
      shutter = max;
      std::cout << "vp1394CMUGrabber warning: Desired exposure time register value of IEEE 1394 camera number " << index << " can't be greater than " << shutter << std::endl;
    }
    Control->SetAutoMode(false);
    if(Control->SetValue(shutter) != CAM_SUCCESS)
    {
      std::cout << "vp1394CMUGrabber warning: Can't set exposure time register value of IEEE 1394 camera number " << index << std::endl;
    }
  }
  else
  {
    camera->GetCameraControl(FEATURE_SHUTTER)->SetAutoMode(true);
    camera->GetCameraControl(FEATURE_GAIN)->SetAutoMode(true);
  } */
  _shutter=shutter;
  _gain=gain;
}

/*!
   Get the number of connected cameras.
   */
int
vp1394CMUGrabber::getNumberOfConnectedCameras() const
{
  int n_cam = camera->RefreshCameraList();

  /*if( n_cam >= 0 )
  {
    std::cout << "vp1394CMUGrabber: Found " << n_cam << " Camera";

    if( n_cam > 1 ) std::cout << "s."  << std::endl;
    else std::cout << "."  << std::endl;
  }
  else {
    throw (vpFrameGrabberException(vpFrameGrabberException::initializationError,"vp1394CMUGrabber: Found no cameras on the 1394 bus(es)"));
  }*/

  return n_cam;
}

/*!
  Display information about the camera on the standard output.
 */
void
vp1394CMUGrabber::displayCameraDescription(int cam_id)
{
  char buf[512];


  if( camera->GetNumberCameras() > cam_id )
  {
    camera->GetNodeDescription(cam_id,buf,512);
    std::cout << "Camera " << cam_id << ": " << buf << std::endl ;

  }
  else {
    std::cout << "Camera " << cam_id << ": camera not found" << std::endl ;
  }
}


/*!
 Display camera model on the standard output. Call it after open the grabber.
 */
void 
vp1394CMUGrabber::displayCameraModel() 
{
  char vendor[256] , model[256] , buf[256];
  LARGE_INTEGER ID;

  camera->GetCameraName(model,sizeof(model));
  camera->GetCameraVendor(vendor,sizeof(vendor));
  camera->GetCameraUniqueID(&ID);

  std::cout << "Vendor: " << vendor << std::endl;
  std::cout << "Model: " << model << std::endl;

  sprintf(buf,"%08X%08X",ID.HighPart,ID.LowPart);
  std::cout << "UniqueID: " << buf << std::endl;

}


/*!
  Set the camera format and video mode.
  This method has to be called before open().

  \param format : Camera video format.
  \param mode : Camera video mode.

  See the following table for the correspondances between the input
  format and mode and the resulting video color coding.

  <TABLE BORDER="1">
  <TR><TH> Format </TH><TH> Mode </TH><TH> (H) x (W) </TH><TH> Color  </TH></TR>
  <TR><TD>   0    </TD><TD>  0   </TD><TD> 160 x 120 </TD><TD> YUV444 </TD></TR>
  <TR><TD>   0    </TD><TD>  1   </TD><TD> 320 x 240 </TD><TD> YUV422 </TD></TR>
  <TR><TD>   0    </TD><TD>  2   </TD><TD> 640 x 480 </TD><TD> YUV411 </TD></TR>
  <TR><TD>   0    </TD><TD>  3   </TD><TD> 640 x 480 </TD><TD> YUV422 </TD></TR>
  <TR><TD>   0    </TD><TD>  4   </TD><TD> 640 x 480 </TD><TD>  RGB8  </TD></TR>
  <TR><TD>   0    </TD><TD>  5   </TD><TD> 640 x 480 </TD><TD>  MONO8 </TD></TR>
  <TR><TD>   0    </TD><TD>  6   </TD><TD> 640 x 480 </TD><TD> MONO16 </TD></TR>
  <TR><TD>   1    </TD><TD>  0   </TD><TD> 800 x 600 </TD><TD> YUV422 </TD></TR>
  <TR><TD>   1    </TD><TD>  1   </TD><TD> 800 x 600 </TD><TD>  RGB8  </TD></TR>
  <TR><TD>   1    </TD><TD>  2   </TD><TD> 800 x 600 </TD><TD>  MONO8 </TD></TR>
  <TR><TD>   1    </TD><TD>  3   </TD><TD>1024 x 768 </TD><TD> YUV422 </TD></TR>
  <TR><TD>   1    </TD><TD>  4   </TD><TD>1024 x 768 </TD><TD>  RGB8  </TD></TR>
  <TR><TD>   1    </TD><TD>  5   </TD><TD>1024 x 768 </TD><TD>  MONO8 </TD></TR>
  <TR><TD>   1    </TD><TD>  6   </TD><TD> 800 x 600 </TD><TD> MONO16 </TD></TR>
  <TR><TD>   1    </TD><TD>  7   </TD><TD>1024 x 768 </TD><TD> MONO16 </TD></TR>
  <TR><TD>   2    </TD><TD>  0   </TD><TD>1280 x 960 </TD><TD> YUV422 </TD></TR>
  <TR><TD>   2    </TD><TD>  1   </TD><TD>1280 x 960 </TD><TD>  RGB8  </TD></TR>
  <TR><TD>   2    </TD><TD>  2   </TD><TD>1280 x 960 </TD><TD>  MONO8 </TD></TR>
  <TR><TD>   2    </TD><TD>  3   </TD><TD>1600 x 1200</TD><TD> YUV422 </TD></TR>
  <TR><TD>   2    </TD><TD>  4   </TD><TD>1600 x 1200</TD><TD>  RGB8  </TD></TR>
  <TR><TD>   2    </TD><TD>  5   </TD><TD>1600 x 1200</TD><TD>  MONO8 </TD></TR>
  <TR><TD>   2    </TD><TD>  6   </TD><TD>1280 x 960 </TD><TD> MONO16 </TD></TR>
  <TR><TD>   2    </TD><TD>  7   </TD><TD>1600 x 1200</TD><TD> MONO16 </TD></TR>
  </TABLE>

 */
void 
vp1394CMUGrabber::setVideoMode( unsigned long format, unsigned long mode )
{
  _format = format ;
  _mode = mode ;
}

/*!
  Set camera framerate rate. This method has to be called before open().

  \param fps : Value form 0 to 7 used to select a specific camera framerate.
  See the following table for the correspondances between the input
  value and the framerate.

  <TABLE BORDER="1">
  <TR><TH> Value </TH>  <TH> Frame rate </TH></TR>
  <TR><TD>   0   </TD>  <TD>  1.875 fps </TD></TR>
  <TR><TD>   1   </TD>  <TD>   3.75 fps </TD></TR>
  <TR><TD>   2   </TD>  <TD>    7.5 fps </TD></TR>
  <TR><TD>   3   </TD>  <TD>     15 fps </TD></TR>
  <TR><TD>   4   </TD>  <TD>     30 fps </TD></TR>
  <TR><TD>   5   </TD>  <TD>     60 fps </TD></TR>
  <TR><TD>   6   </TD>  <TD>    120 fps </TD></TR>
  <TR><TD>   7   </TD>  <TD>    240 fps </TD></TR>
  </TABLE>

 */
void 
vp1394CMUGrabber::setFramerate(unsigned long fps)
{
  _fps = fps;
}

#endif

