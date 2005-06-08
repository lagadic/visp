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


#ifndef vpFrameGrabber_hh
#define vpFrameGrabber_hh


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
class vpFrameGrabber
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
