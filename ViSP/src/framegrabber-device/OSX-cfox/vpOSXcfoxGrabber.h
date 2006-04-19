/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 2005  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Fabien Spindler
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: fspindle@irisa.fr
#    www  : http://www.irisa.fr/lagadic
#
#----------------------------------------------------------------------------
*/

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

using namespace cfox ;

/*!
  \class vpOSXcfoxGrabber
  \brief class for the Video For Linux 2 video device

  \ingroup libdevice

  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes


  \sa vpFrameGrabber
*/
class vpOSXcfoxGrabber : public vpFrameGrabber
{
public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;

  enum framerateEnum
    {
      framerate_30fps, //!< 30 frames per second
      framerate_15fps  //!< 15 frames per second
    };

public:
  vpOSXcfoxGrabber();
  ~vpOSXcfoxGrabber() ;

public:
  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  void setFramerate(framerateEnum framerate);
  framerateEnum getFramerate();
  void close();




  // fct pour changer le port d'entree
  void setInput(unsigned input = vpOSXcfoxGrabber::DEFAULT_INPUT) ;
  void setScale(unsigned scale = vpOSXcfoxGrabber::DEFAULT_SCALE) ;


private:
  void open();

  Camera cam ;
  // Camera::Frame img;
  int input ;
  int scale ;
  framerateEnum framerate ;


} ;

#endif
#endif

