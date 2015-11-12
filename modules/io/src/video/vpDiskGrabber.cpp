/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Disk framegrabber.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp3/io/vpDiskGrabber.h>


/*!
  Elementary constructor.
*/
vpDiskGrabber::vpDiskGrabber()
  : image_number(0), image_step(1), number_of_zero(0), useGenericName(false)
{
  setDirectory("/tmp");
  setBaseName("I");
  setExtension("pgm");

  init = false;
}


vpDiskGrabber::vpDiskGrabber(const char *generic_name)
  : image_number(0), image_step(1), number_of_zero(0), useGenericName(false)
{
  setDirectory("/tmp");
  setBaseName("I");
  setExtension("pgm");

  init = false;
  if (strlen( generic_name ) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the generic name"));
  }

  strcpy(this->genericName, generic_name);
  useGenericName = true;
}


/*!
  Constructor.

  \param dir : Location of the image sequence.
  \param basename : Base name of each image.
  \param number : Initial image number.
  \param step : Increment between two images.
  \param noz : Number of zero to code the image number.
  \param ext : Extension of the image file.
*/

vpDiskGrabber::vpDiskGrabber(const char *dir, const char *basename,
                             long number,
                             int step, unsigned int noz,
                             const char *ext)
  : image_number(number), image_step(step), number_of_zero(noz), useGenericName(false)
{
  setDirectory(dir);
  setBaseName(basename);
  setExtension(ext);

  init = false;
}

void
vpDiskGrabber::open(vpImage<unsigned char> &I)
{
  long first_number = getImageNumber();

  vpDEBUG_TRACE(2, "first %ld", first_number);

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

/*!
  Read the fist image of the sequence.
  The image number is not incremented.

*/
void
vpDiskGrabber::open(vpImage<vpRGBa> &I)
{
  // First we save the image number, so that it can be reaffected after the
  // acquisition. That means that the first image is readed twice
  long first_number = getImageNumber();
  vpDEBUG_TRACE(2, "first %ld", first_number);

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

/*!
  Read the fist image of the sequence.
  The image number is not incremented.

*/
void
vpDiskGrabber::open(vpImage<float> &I)
{
  // First we save the image number, so that it can be reaffected after the
  // acquisition. That means that the first image is readed twice
  long first_number = getImageNumber();
  vpDEBUG_TRACE(2, "first %ld", first_number);

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

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

  if(useGenericName)
    sprintf(name,genericName,image_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,image_number,extension) ;

  image_number += image_step ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image: read a ppm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I the read image
 */
void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,image_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,image_number,extension) ;

  image_number += image_step ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();

}

/*!
  Acquire an image: read a pfm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I the read image
 */
void
vpDiskGrabber::acquire(vpImage<float> &I)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,image_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,image_number,extension) ;

  image_number += image_step ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::readPFM(I, name) ;

  width = I.getWidth();
  height = I.getHeight();

}

/*!
  Acquire an image: read a pgm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<unsigned char> &I, long img_number)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,img_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,img_number,extension) ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image: read a ppm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I, long img_number)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,img_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,img_number,extension) ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();

}


/*!
  Acquire an image: read a pfm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<float> &I, long img_number)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,img_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,img_number,extension) ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::readPFM(I, name) ;

  width = I.getWidth();
  height = I.getHeight();

}

/*!
  Not useful

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
  sprintf(directory, "%s", dir) ;
}

/*!
  Set the image base name.
*/
void
vpDiskGrabber::setBaseName(const char *name)
{
  sprintf(base_name, "%s", name) ;
}

/*!
  Set the image extension.
 */
void
vpDiskGrabber::setExtension(const char *ext)
{
  sprintf(extension, "%s", ext) ;
}

/*!
  Set the number of the image to be read.
*/
void
vpDiskGrabber::setImageNumber(long number)
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
vpDiskGrabber::setNumberOfZero(unsigned int noz)
{
  number_of_zero = noz ;
}

void
vpDiskGrabber::setGenericName(const char *generic_name)
{
  if (strlen( generic_name ) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the generic name"));
  }

  strcpy(this->genericName, generic_name) ;
  useGenericName = true;
}
