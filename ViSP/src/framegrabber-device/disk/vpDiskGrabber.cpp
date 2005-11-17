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



#include <vpDiskGrabber.h>


/*!
  Elementary constructor.


*/
vpDiskGrabber::vpDiskGrabber()
{
  setDirectory("/tmp");
  setBaseName("I");
  setImageNumber(0);
  setStep(1);
  setNumberOfZero(0);

  init = false;
}


/*!
  Constructor.

  \param dir  Location of the image sequence.
  \param basename  Base name of each image.
  \param number Initial image number.
  \param step   Increment between two images.
  \param noz Number of zero to code the image number.

*/

vpDiskGrabber::vpDiskGrabber(const char *dir, const char *basename,
			     unsigned long number,
			     int step, int noz)
{
  setDirectory(dir);
  setBaseName(basename);
  setImageNumber(image_number);
  setStep(step);
  setNumberOfZero(noz);

  init = false;
}

void
vpDiskGrabber::open(vpImage<unsigned char> &I)
{
  unsigned long first_number = getImageNumber();

  DEBUG_TRACE(2, "first %ld", first_number);

  acquire(I);

  setImageNumber(first_number);

  init = true;
}

/*!
  Read the fist image of the sequence.
  The image number is not incremented.

*/
void
vpDiskGrabber::open(vpImage<vpRGBa> &I)
{
  // Fisrt we save the image number, so that it can be reaffected after the
  // acquisition. That means that the first image is readed twice
  unsigned long first_number = getImageNumber();
  DEBUG_TRACE(2, "first %ld", first_number);

  acquire(I);

  setImageNumber(first_number);

  init = true;
}


/*!
  Acquire an image: read a pgm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I the read image
 */
void
vpDiskGrabber::acquire(vpImage<unsigned char> &I)
{

  char name[FILENAME_MAX] ;

  switch(number_of_zero)
  {
  case 0:
    sprintf(name,"%s/%s%ld.pgm",directory,base_name,image_number) ;
    break ;
  case 1:
    sprintf(name,"%s/%s%01ld.pgm",directory,base_name,image_number) ;
    break ;
  case 2:
    sprintf(name,"%s/%s%02ld.pgm",directory,base_name,image_number) ;
    break ;
  case 3:
    sprintf(name,"%s/%s%03ld.pgm",directory,base_name,image_number) ;
    break ;
  case 4:
    sprintf(name,"%s/%s%04ld.pgm",directory,base_name,image_number) ;
    break ;
  case 5:
    sprintf(name,"%s/%s%05ld.pgm",directory,base_name,image_number) ;
    break ;
  case 6:
    sprintf(name,"%s/%s%06ld.pgm",directory,base_name,image_number) ;
    break ;
  }
  image_number += image_step ;

  DEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::readPGM(I, name) ;

}

void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I)
{


}
/*!
  Not usefull

  Here for compatibility issue with the vpFrameGrabber class
 */
void
vpDiskGrabber::close()
{
  // Nothing do do here...
}


/*!
  Destructor

  In fact nothing to destroy...
 */
vpDiskGrabber::~vpDiskGrabber()
{
}


/*!
  Set the main directory name (ie location of the image sequence)
*/
void
vpDiskGrabber::setDirectory(const char *dir)
{
  sprintf(directory, dir) ;
}

/*!
  Set the image base name.
*/
void
vpDiskGrabber::setBaseName(const char *name)
{
  sprintf(base_name, name) ;
}

/*!
  Set the number of the image to be read.
*/
void
vpDiskGrabber::setImageNumber(unsigned long number)
{
  image_number = number ;
  DEBUG_TRACE(2, "image number %ld", image_number);

}

/*!
  Set the step between two images.
*/
void
vpDiskGrabber::setStep(int step)
{
  image_step = step;
}
/*!
  Set the step between two images.
*/
void
vpDiskGrabber::setNumberOfZero(unsigned noz)
{
  number_of_zero = noz ;
}
