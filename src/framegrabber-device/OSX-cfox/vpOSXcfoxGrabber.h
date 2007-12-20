/****************************************************************************
 *
 * $Id: vpOSXcfoxGrabber.h,v 1.7 2007-12-20 08:19:38 fspindle Exp $
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
 * This file is part of the ViSP toolkit
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
 * 1394 frame grabbing for OSX.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpOSXcfoxGrabber.h
  \brief class provinding an interface with the cfox library
  (1394 grabber for OSX)


   C+FOX is an Open Source C++ library that provides an easy to use
   interface for acquiring images from FireWire camera on OS X
   platform. It has been developed by Joel FALCOU and is based on the
   JAVA Firewire Camera Acquisition library FWCamAkiz by Adrian Daerr.

   http://cfox.sourceforge.net/

   \ingroup libdevice
*/

#ifndef vpOSXcfoxGrabber_hh
#define vpOSXcfoxGrabber_hh

#include <visp/vpConfig.h>


#if ( defined(APPLE) && defined(VISP_HAVE_CFOX) )

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>


#include <cfox/cfox.h>
#include <cfox/camera/Camera.h>


/*!
  \class vpOSXcfoxGrabber
  \brief class for the Video For Linux 2 video device

  \ingroup libdevice


  \sa vpFrameGrabber
*/
class VISP_EXPORT vpOSXcfoxGrabber : public vpFrameGrabber
{
public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;

  typedef enum 
    {
      framerate_30fps, //!< 30 frames per second
      framerate_15fps  //!< 15 frames per second
    } vpOSXcfoxFramerateType ;

public:
  vpOSXcfoxGrabber();
  virtual ~vpOSXcfoxGrabber() ;

public:
  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  void setFramerate(vpOSXcfoxFramerateType framerate);
  vpOSXcfoxFramerateType getFramerate();
  void close();




  // fct pour changer le port d'entree
  void setInput(unsigned int input = vpOSXcfoxGrabber::DEFAULT_INPUT) ;
  void setScale(unsigned int scale = vpOSXcfoxGrabber::DEFAULT_SCALE) ;


private:
  void open();

  cfox::Camera cam ;
  // Camera::Frame img;
  int input ;
  int scale ;
  vpOSXcfoxFramerateType framerate ;


} ;

#endif
#endif

