/*                                                                -*-c++-*-
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/

/*!
  \file vpIcCompGrabber.h
  \brief Class for the IcComp (Imaging Technology)  video device.
  \ingroup libdevice
*/

#ifndef vpIcCompGrabber_hh
#define vpIcCompGrabber_hh

#include <visp/vpConfig.h>

#ifdef HAVE_FG_ICCOMP

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>


#include "ic-comp2x.h"

#define DEFAULT_INPUT_BOARD 2

/*!
  \class vpIcCompGrabber
  \brief class for the ICComp (Imaging Technology) video device

  \ingroup libdevice

  \author Eric Marchand (Eric.Marchand@irisa.fr) and
  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

  \warning suitable for "new" Linux Kernel > 2.4
*/
class vpIcCompGrabber : public vpFrameGrabber
{
public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;

  enum framerateEnum
    {
      framerate_50fps, //!< 50 frames per second
      framerate_25fps  //!< 25 frames per second
    };
  int ncols, nrows ;
  ICcomp2x *framegrabber ; //!< pointeur sur l'objet ICcomp
private:
  unsigned input ; //!< video entry
  unsigned scale ;
  framerateEnum framerate;
  bool field; // The type of the acquired frame (0 if odd, 1 if even).

public:
  vpIcCompGrabber(unsigned input,
		  unsigned scale = vpIcCompGrabber::DEFAULT_SCALE);
  vpIcCompGrabber(vpImage<unsigned char> &I,
		  unsigned input,
		  unsigned scale = vpIcCompGrabber::DEFAULT_SCALE);
  vpIcCompGrabber(vpImage<vpRGBa> &I,
		  unsigned input,
		  unsigned scale = vpIcCompGrabber::DEFAULT_SCALE);
  ~vpIcCompGrabber() ;
public:

  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  bool getField();
  void setFramerate(framerateEnum framerate);
  framerateEnum getFramerate();
  void close()  ;



  // fct pour changer le port d'entree
  void setInput(unsigned input = vpIcCompGrabber::DEFAULT_INPUT) ;
  void setScale(unsigned scale = vpIcCompGrabber::DEFAULT_SCALE) ;
} ;

#endif
#endif

