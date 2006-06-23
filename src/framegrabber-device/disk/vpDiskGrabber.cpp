/****************************************************************************
 *
 * $Id: vpDiskGrabber.cpp,v 1.5 2006-06-23 15:35:44 brenier Exp $
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
 * Disk framegrabber.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpDiskGrabber.h>


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
  setImageNumber(number);
  setStep(step);
  setNumberOfZero(noz);

  init = false;
}

void
vpDiskGrabber::open(vpImage<unsigned char> &I)
{
  unsigned long first_number = getImageNumber();

  vpDEBUG_TRACE(2, "first %ld", first_number);

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
  vpDEBUG_TRACE(2, "first %ld", first_number);

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

  sprintf(name,"%s/%s%0*ld.pgm",directory,base_name,number_of_zero,image_number) ;

  image_number += image_step ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

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
  vpDEBUG_TRACE(2, "image number %ld", image_number);

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
