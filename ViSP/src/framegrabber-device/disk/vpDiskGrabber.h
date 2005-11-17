
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
#    www  : http://www.irisa.fr/lagadic
#
#----------------------------------------------------------------------------
*/


/*!
  \file vpDiskGrabber.h
  \brief Class to load image sequence from the disk.
*/
#ifndef vpDiskGrabber_hh
#define vpDiskGrabber_hh

#include <visp/vpImageIo.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>

/*!
  \class vpDiskGrabber

  Defined a virtual video device. "Grab" the images from the disk
  Derived from the vpFrameGrabber class.

  \sa vpFrameGrabber
*/
class vpDiskGrabber  : public vpFrameGrabber
{
private:
  unsigned long image_number ; //!< id of the next image to be read
  int image_step ;    //!< increment between two image id
  int number_of_zero ; //!< number of zero in the image name (image.00000.pgm)

  char directory[FILENAME_MAX] ; //!< image location
  char base_name[FILENAME_MAX] ; //!< image base name

public:
  vpDiskGrabber();
  vpDiskGrabber(const char *dir, const char *basename,
		unsigned long number, int step, int noz) ;
  ~vpDiskGrabber() ;

  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);

  void close();

  void setDirectory(const char *name);
  void setBaseName(const char *name);
  void setImageNumber(unsigned long number) ;
  void setStep(int a);
  void setNumberOfZero(unsigned noz);

  /*!
    Return the current image number.
  */
  unsigned long getImageNumber() { return image_number; };
} ;

#endif

