/****************************************************************************
 *
 * $Id: vpIcCompGrabber.h,v 1.8 2007-12-19 08:25:25 fspindle Exp $
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
 * IcComp (Imaging Technology) framegrabber.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/



/*!
  \file vpIcCompGrabber.h
  \brief Class for the IcComp (Imaging Technology)  video device.
  \ingroup libdevice
*/

#ifndef vpIcCompGrabber_hh
#define vpIcCompGrabber_hh

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_ICCOMP

#include <visp/vpImage.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>


#include "ic-comp2x.h"


/*!
  \class vpIcCompGrabber
  \brief class for the ICComp (Imaging Technology) video device

  \ingroup libdevice

  \author Eric Marchand (Eric.Marchand@irisa.fr) and
  Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

  \warning suitable for "new" Linux Kernel > 2.4
*/
class VISP_EXPORT vpIcCompGrabber : public vpFrameGrabber
{
public:
  static const int DEFAULT_INPUT;
  static const int DEFAULT_SCALE;

  typedef enum 
    {
      framerate_50fps, //!< 50 frames per second
      framerate_25fps  //!< 25 frames per second
    } vpIcCompFramerateType;
  int ncols, nrows ;
  ICcomp2x *framegrabber ; //!< pointeur sur l'objet ICcomp
private:
  unsigned int input ; //!< video entry
  unsigned int scale ;
  vpIcCompFramerateType framerate;
  bool field; // The type of the acquired frame (0 if odd, 1 if even).

public:
  vpIcCompGrabber();
  vpIcCompGrabber(unsigned int input,
		  unsigned int scale = vpIcCompGrabber::DEFAULT_SCALE);
  vpIcCompGrabber(vpImage<unsigned char> &I,
		  unsigned int input,
		  unsigned int scale = vpIcCompGrabber::DEFAULT_SCALE);
  vpIcCompGrabber(vpImage<vpRGBa> &I,
		  unsigned int input,
		  unsigned int scale = vpIcCompGrabber::DEFAULT_SCALE);
  virtual ~vpIcCompGrabber() ;
public:

  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I) ;
  void acquire(vpImage<vpRGBa> &I)  ;
  bool getField();
  void setFramerate(vpIcCompFramerateType framerate);
  vpIcCompFramerateType getFramerate();
  void close()  ;



  // fct pour changer le port d'entree
  void setInput(unsigned int input = vpIcCompGrabber::DEFAULT_INPUT) ;
  void setScale(unsigned int scale = vpIcCompGrabber::DEFAULT_SCALE) ;
} ;

#endif
#endif

