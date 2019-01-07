/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * API for using a Microsoft Kinect device
 * Requires libfreenect as a third party library
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

// Note that libfreenect needs libusb-1.0 and libpthread
#if defined(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES)

#include <limits> // numeric_limits

#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/sensor/vpKinect.h>

/*!
  Default constructor.
*/
vpKinect::vpKinect(freenect_context *ctx, int index)
  : Freenect::FreenectDevice(ctx, index), m_rgb_mutex(), m_depth_mutex(), RGBcam(), IRcam(), rgbMir(), irMrgb(),
    DMres(DMAP_LOW_RES), hd(240), wd(320), dmap(), IRGB(), m_new_rgb_frame(false), m_new_depth_map(false),
    m_new_depth_image(false), height(480), width(640)
{
  dmap.resize(height, width);
  IRGB.resize(height, width);
  vpPoseVector r(-0.0266, -0.0047, -0.0055, 0.0320578, 0.0169041,
                 -0.0076519); //! Those are the parameters found for our
                              //! Kinect device. Note that they can differ from
                              //! one device to another.
  rgbMir.buildFrom(r);
  irMrgb = rgbMir.inverse();
}

/*!
  Destructor.
*/
vpKinect::~vpKinect() {}

void vpKinect::start(vpKinect::vpDMResolution res)
{
  DMres = res;
  height = 480;
  width = 640;
  //! Calibration parameters are the parameters found for our Kinect device.
  //! Note that they can differ from one device to another.
  if (DMres == DMAP_LOW_RES) {
    std::cout << "vpKinect::start LOW depth map resolution 240x320" << std::endl;
    //		IRcam.setparameters(IRcam.get_px()/2, IRcam.get_py()/2,
    // IRcam.get_u0()/2, IRcam.get_v0()/2);
    // IRcam.initPersProjWithoutDistortion(303.06,297.89,160.75,117.9);
    IRcam.initPersProjWithDistortion(303.06, 297.89, 160.75, 117.9, -0.27, 0);
    hd = 240;
    wd = 320;
  } else {
    std::cout << "vpKinect::start MEDIUM depth map resolution 480x640" << std::endl;

    // IRcam.initPersProjWithoutDistortion(606.12,595.78,321.5,235.8);
    IRcam.initPersProjWithDistortion(606.12, 595.78, 321.5, 235.8, -0.27, 0);
    //		Idmap.resize(height, width);
    hd = 480;
    wd = 640;
  }

#if defined(VISP_HAVE_VIPER850_DATA) && defined(VISP_HAVE_XML2)
  vpXmlParserCamera cameraParser;
  std::string cameraXmlFile = std::string(VISP_VIPER850_DATA_PATH) + std::string("/include/const_camera_Viper850.xml");
  cameraParser.parse(RGBcam, cameraXmlFile, "Generic-camera", vpCameraParameters::perspectiveProjWithDistortion, width,
                     height);
#else
  //  RGBcam.initPersProjWithoutDistortion(525.53, 524.94, 309.9, 282.8);//old
  //  RGBcam.initPersProjWithDistortion(536.76, 537.25, 313.45,
  //  273.27,0.04,-0.04);//old
  //  RGBcam.initPersProjWithoutDistortion(512.0559503505,511.9352058050,310.6693938678,267.0673901049);//new
  RGBcam.initPersProjWithDistortion(522.5431816996, 522.7191431808, 311.4001982614, 267.4283562142, 0.0477365207,
                                    -0.0462326418); // new
#endif

  this->startVideo();
  this->startDepth();
}

void vpKinect::stop()
{
  this->stopVideo();
  this->stopDepth();
}

/*!
  Acquire a new RGB image.
*/
void vpKinect::VideoCallback(void *rgb, uint32_t /* timestamp */)
{
  //  	std::cout << "vpKinect Video callback" << std::endl;
  vpMutex::vpScopedLock lock(m_rgb_mutex);
  uint8_t *rgb_ = static_cast<uint8_t *>(rgb);
  for (unsigned i = 0; i < height; i++) {
    for (unsigned j = 0; j < width; j++) {
      IRGB[i][j].R = rgb_[3 * (width * i + j) + 0];
      IRGB[i][j].G = rgb_[3 * (width * i + j) + 1];
      IRGB[i][j].B = rgb_[3 * (width * i + j) + 2];
    }
  }

  m_new_rgb_frame = true;
}

/*!
  Acquire a new depth image.

  Depth value send by the kinect is coded on 11 bits : 10 for the
  value itself (between 0 and 1023) and one for overflow.

  In this function this value is converted into a metric depth map and
  stored in dmap.  (range : 0.3 - 5m).

*/
void vpKinect::DepthCallback(void *depth, uint32_t /* timestamp */)
{
  //	std::cout << "vpKinect Depth callback" << std::endl;
  vpMutex::vpScopedLock lock(m_depth_mutex);
  uint16_t *depth_ = static_cast<uint16_t *>(depth);
  for (unsigned i = 0; i < height; i++) {
    for (unsigned j = 0; j < width; j++) {
      dmap[i][j] =
          0.1236f * tan(depth_[width * i + j] / 2842.5f + 1.1863f); // formula from
                                                                    // http://openkinect.org/wiki/Imaging_Information
      if (depth_[width * i + j] > 1023) {                           // Depth cannot be computed
        dmap[i][j] = -1;
      }
    }
  }
  m_new_depth_map = true;
  m_new_depth_image = true;
}

/*!
  Get metric depth map (float).
*/
bool vpKinect::getDepthMap(vpImage<float> &map)
{
  vpMutex::vpScopedLock lock(m_depth_mutex);
  if (!m_new_depth_map)
    return false;
  map = this->dmap;
  m_new_depth_map = false;
  return true;
}

/*!
 *   Get metric depth map (float) and corresponding image.
 */
bool vpKinect::getDepthMap(vpImage<float> &map, vpImage<unsigned char> &Imap)
{
  //	vpMutex::vpScopedLock lock(m_depth_mutex);
  vpImage<float> tempMap;
  m_depth_mutex.lock();
  if (!m_new_depth_map && !m_new_depth_image) {
    m_depth_mutex.unlock();
    return false;
  }
  tempMap = dmap;

  m_new_depth_map = false;
  m_new_depth_image = false;
  m_depth_mutex.unlock();

  if ((Imap.getHeight() != hd) || (map.getHeight() != hd))
    vpERROR_TRACE(1, "Image size does not match vpKinect DM resolution");
  if (DMres == DMAP_LOW_RES) {
    for (unsigned int i = 0; i < hd; i++)
      for (unsigned int j = 0; j < wd; j++) {
        map[i][j] = tempMap[i << 1][j << 1];
        // if (map[i][j] != -1)
        if (fabs(map[i][j] + 1.f) > std::numeric_limits<float>::epsilon())
          Imap[i][j] = (unsigned char)(255 * map[i][j] / 5);
        else
          Imap[i][j] = 255;
      }
  } else {
    for (unsigned i = 0; i < height; i++)
      for (unsigned j = 0; j < width; j++) {
        map[i][j] = tempMap[i][j];
        // if (map[i][j] != -1)
        if (fabs(map[i][j] + 1.f) > std::numeric_limits<float>::epsilon())
          Imap[i][j] = (unsigned char)(255 * map[i][j] / 5);
        else
          Imap[i][j] = 255;
      }
  }

  return true;
}

/*!
  Get RGB image
*/
bool vpKinect::getRGB(vpImage<vpRGBa> &I_RGB)
{
  vpMutex::vpScopedLock lock(m_rgb_mutex);
  if (!m_new_rgb_frame)
    return false;
  I_RGB = this->IRGB;
  m_new_rgb_frame = false;
  return true;
}

/*!
  Warp the RGB frame to the depth camera frame. The size of the resulting
  IrgbWarped frame is the same as the size of the depth map Idepth
*/
void vpKinect::warpRGBFrame(const vpImage<vpRGBa> &Irgb, const vpImage<float> &Idepth, vpImage<vpRGBa> &IrgbWarped)
{
  if ((Idepth.getHeight() != hd) || (Idepth.getWidth() != wd)) {
    vpERROR_TRACE(1, "Idepth image size does not match vpKinect DM resolution");
  } else {
    if ((IrgbWarped.getHeight() != hd) || (IrgbWarped.getWidth() != wd))
      IrgbWarped.resize(hd, wd);
    IrgbWarped = 0;
    double x1 = 0., y1 = 0., x2 = 0., y2 = 0., Z1, Z2;
    vpImagePoint imgPoint(0, 0);
    double u = 0., v = 0.;
    vpColVector P1(4), P2(4);

    //		std::cout <<"rgbMir : "<<rgbMir<<std::endl;

    for (unsigned int i = 0; i < hd; i++)
      for (unsigned int j = 0; j < wd; j++) {
        //! Compute metric coordinates in the ir camera Frame :
        vpPixelMeterConversion::convertPoint(IRcam, j, i, x1, y1);
        Z1 = Idepth[i][j];
        // if (Z1!=-1){
        if (std::fabs(Z1 + 1) <= std::numeric_limits<double>::epsilon()) {
          P1[0] = x1 * Z1;
          P1[1] = y1 * Z1;
          P1[2] = Z1;
          P1[3] = 1;

          //! Change frame :
          P2 = rgbMir * P1;
          Z2 = P2[2];
          // if (Z2!= 0){
          if (std::fabs(Z2) > std::numeric_limits<double>::epsilon()) {
            x2 = P2[0] / P2[2];
            y2 = P2[1] / P2[2];
          } else
            std::cout << "Z2 = 0 !!" << std::endl;

          //! compute pixel coordinates of the corresponding point in the
          //! depth image
          vpMeterPixelConversion::convertPoint(RGBcam, x2, y2, u, v);

          unsigned int u_ = (unsigned int)u;
          unsigned int v_ = (unsigned int)v;
          //! Fill warped image value
          if ((u_ < width) && (v_ < height)) {
            IrgbWarped[i][j] = Irgb[v_][u_];
          } else
            IrgbWarped[i][j] = 0;
        }
      }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_sensor.a(vpKinect.cpp.o) has no
// symbols
void dummy_vpKinect(){};
#endif // VISP_HAVE_LIBFREENECT
