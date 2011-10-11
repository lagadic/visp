/****************************************************************************
 *
 * $Id$
 *
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
 * API for using a Microsoft Kinect device
 * Requires libfreenect as a third party library
 *
 * Authors:
 * Celine Teuliere
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#ifdef VISP_HAVE_LIBFREENECT
#include <visp/vpKinect.h>

/*!
  Default constructor.
*/
vpKinect::vpKinect(freenect_context *ctx, int index)
		: Freenect::FreenectDevice(ctx, index),
		  m_new_rgb_frame(false), 
		  m_new_depth_map(false),
		  m_new_depth_image(false),
		  height(480), width(640)
{
	dmap.resize(height, width);
	IRGB.resize(height, width);
}

/*!
  Destructor.
*/
vpKinect::~vpKinect()
{
}

void vpKinect::start(vpDMResolution res)
{
	DMres = res;
	RGBcam.initPersProjWithoutDistortion(606, 595, 321, 235);// to calibrate
	if (DMres == LOW){
		std::cout << "vpKinect::start LOW depth map resolution 240x320" << std::endl;
//		IRcam.setparameters(IRcam.get_px()/2, IRcam.get_py()/2, IRcam.get_u0()/2, IRcam.get_v0()/2);
		//IRcam.initPersProjWithoutDistortion(303.06,297.89,160.75,117.9);
		IRcam.initPersProjWithDistortion(303.06, 297.89, 160.75, 117.9, -0.27, -0.27);
	}
	else
	{
		std::cout << "vpKinect::start MEDIUM depth map resolution 480x640" << std::endl;
		//IRcam.initPersProjWithoutDistortion(606.12,595.78,321.5,235.8);
		IRcam.initPersProjWithDistortion(606.12, 595.78, 321.5, 235.8, -0.27, -0.27);
//		Idmap.resize(height, width);

	}
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
void vpKinect::VideoCallback(void* rgb, uint32_t /* timestamp */)
{
//	std::cout << "vpKinect Video callback" << std::endl;
	vpMutex::vpScopedLock lock(m_rgb_mutex);
	uint8_t* rgb_ = static_cast<uint8_t*>(rgb);
	for (unsigned i = 0; i< height;i++){
		for (unsigned j = 0 ; j < width ; j++)
		{
			IRGB[i][j].R = rgb_[3*(width*i +j)+0];
			IRGB[i][j].G = rgb_[3*(width*i +j)+1];
			IRGB[i][j].B = rgb_[3*(width*i +j)+2];
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
void vpKinect::DepthCallback(void* depth, uint32_t /* timestamp */)
{
	vpMutex::vpScopedLock lock(m_depth_mutex);
	uint16_t* depth_ = static_cast<uint16_t*>(depth);
	for (unsigned i = 0; i< height;i++){
		for (unsigned j = 0 ; j < width ; j++)
		{
			dmap[i][j] = 0.1236 * tan(depth_[width*i +j] / 2842.5 + 1.1863);//formula from http://openkinect.org/wiki/Imaging_Information
			if(depth_[width*i +j]>1023){//Depth cannot be computed
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
bool vpKinect::getDepthMap(vpImage<float>& map)
{
  vpMutex::vpScopedLock lock(m_depth_mutex);
  if (!m_new_depth_map)
    return false;
  map = dmap;
  m_new_depth_map = false;
  return true;
}


/*!
 *   Get metric depth map (float) and corresponding image.
 */
bool vpKinect::getDepthMap(vpImage<float>& map,vpImage<unsigned char>& Imap)
{
	vpImage<float> tempMap;
	m_depth_mutex.lock();
	if (!m_new_depth_map && !m_new_depth_image)
	{
		m_depth_mutex.unlock();
		return false;
	}
	tempMap = dmap;

	m_new_depth_map = false;
	m_new_depth_image = false;
	m_depth_mutex.unlock();

	if (DMres == LOW){
	  unsigned int h = height/2;
	  unsigned int w = width/2;
	  if ((Imap.getHeight()!=320 )||(map.getHeight()!=320))
		  vpERROR_TRACE(1, "Image size does not match vpKinect DM resolution");
	  for(unsigned int i = 0; i < h; i++)
	    for(unsigned int j = 0; j < w; j++){
	    	map[i][j] = tempMap[i<<1][j<<1];
	    	if (map[i][j] != -1)
	    		Imap[i][j] = (unsigned char)(255*map[i][j]/5);
	    	else
	    		Imap[i][j] = 255;
	    }
	}
	else
	{
		if ((Imap.getHeight()!=480 )||(map.getHeight()!=480))
				  vpERROR_TRACE(1, "Image size does not match vpKinect DM resolution");
		for (unsigned i = 0; i< height;i++)
			for (unsigned j = 0 ; j < width ; j++){
				map[i][j] = tempMap[i][j];
				if (map[i][j] != -1)
					Imap[i][j] = (unsigned char)(255*map[i][j]/5);
				else
					Imap[i][j] = 255;
			}
	}

	return true;
}


/*!
  Get RGB image
 */
bool vpKinect::getRGB(vpImage<vpRGBa>& IRGB)
{
	vpMutex::vpScopedLock lock(m_rgb_mutex);
	if (!m_new_rgb_frame)
		return false;
	IRGB = this->IRGB;
	m_new_rgb_frame = false;
	return true;
}


#endif // VISP_HAVE_LIBFREENECT

