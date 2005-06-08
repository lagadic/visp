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
  \file CICcomp.h
  \brief class for the ICComp (Imaging Technology)  video device
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

  \date  june, 3 2002

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes
  Fabien Spindler, Irisa / Inria Rennes

  \warning suitable for "new" Linux Kernel > 2.4

  \sa CVideo,  vpICcomp, Acq-iccomp.cpp
*/
class vpIcCompGrabber : public vpFrameGrabber
{
public:
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
  vpIcCompGrabber(unsigned input =DEFAULT_INPUT_BOARD,
		  unsigned scale=2 )  ;
  vpIcCompGrabber(vpImage<unsigned char> &I,
		  unsigned input =DEFAULT_INPUT_BOARD,
		  unsigned scale=2 ) ;
  vpIcCompGrabber(vpImage<vpRGBa> &I,
		  unsigned input =DEFAULT_INPUT_BOARD,
		  unsigned scale=2 ) ;
  ~vpIcCompGrabber() ;
public:

  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  bool getField();
  void setFramerate(framerateEnum framerate);
  framerateEnum getFramerate();
  /*!
    This virtual function is used to de-allocate
    the memory used by a specific frame grabber
  */
  void close()  ;



  // fct pour changer le port d'entree
  void setInput(unsigned input=1) ;
  void setScale(unsigned scale) ;
} ;

#endif
#endif

