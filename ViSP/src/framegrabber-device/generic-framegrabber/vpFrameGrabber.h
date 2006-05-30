/****************************************************************************
 *
 * $Id: vpFrameGrabber.h,v 1.2 2006-05-30 08:40:42 fspindle Exp $
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
 * Frame grabbing.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpFrameGrabber_hh
#define vpFrameGrabber_hh


#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

/*!
  \file vpFrameGrabber.h
  \brief Base class for all video devices. It is
         designed to provide a generic front end to video sources.
*/

/*!
  \class vpFrameGrabber
  \brief Base class for all video devices. It is
         designed to provide a generic front end to video sources.

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes

  This class should provide a virtual function that allows the acquisition
  of an image.

*/
class VISP_EXPORT vpFrameGrabber
{
public :
  bool   init ;  //!< bit 1 if the frame grabber has been initialized
protected:
  int ncols ;  //!< number of rows in the image
  int nrows ;  //!< number of columns in the image


public:
  //! return the number of rows in the image
  inline  int getRows() { return nrows ; }
  //! return the number of columns in the image
  inline  int getCols() { return ncols ; }

public:
  virtual ~vpFrameGrabber() { ; }

  virtual void open(vpImage<unsigned char> &I) =0 ;
  virtual void open(vpImage<vpRGBa> &I) =0 ;

  virtual void acquire(vpImage<unsigned char> &I) =0 ;
  virtual void acquire(vpImage<vpRGBa> &I) =0 ;


  /*!
    This virtual function is used to de-allocate
    the memory used by a specific frame grabber
  */
  virtual void close() =0 ;

} ;

#endif
