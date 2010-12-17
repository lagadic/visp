/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  setExtension("pgm");

  init = false;
  useGenericName = false;
}


vpDiskGrabber::vpDiskGrabber(const char *genericName)
{
  strcpy(this->genericName, genericName);
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
{
  setDirectory(dir);
  setBaseName(basename);
  setImageNumber(number);
  setStep(step);
  setNumberOfZero(noz);
  setExtension(ext);

  init = false;
  useGenericName = false;
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
  // Fisrt we save the image number, so that it can be reaffected after the
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
  Acquire an image: read a pgm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I the read image
  \param image_number The index of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<unsigned char> &I, long image_number)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,image_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,image_number,extension) ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image: read a ppm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I the read image
  \param image_number The index of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I, long image_number)
{

  char name[FILENAME_MAX] ;

  if(useGenericName)
    sprintf(name,genericName,image_number) ;
  else
    sprintf(name,"%s/%s%0*ld.%s",directory,base_name,number_of_zero,image_number,extension) ;

  vpDEBUG_TRACE(2, "load: %s\n", name);

  vpImageIo::read(I, name) ;

  width = I.getWidth();
  height = I.getHeight();

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
vpDiskGrabber::setGenericName(const char *genericName)
{
  strcpy(this->genericName, genericName) ;
  useGenericName = true;
}
