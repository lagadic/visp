/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
  : m_image_number(0), m_image_number_next(0), m_image_step(1), m_number_of_zero(0),
    m_directory("/tmp"), m_base_name("I"), m_extension("pgm"), m_use_generic_name(false), m_generic_name("empty")
{
  init = false;
}

/*!
  Constructor that takes a generic image sequence as input.
*/
vpDiskGrabber::vpDiskGrabber(const std::string &generic_name)
  : m_image_number(0), m_image_number_next(0), m_image_step(1), m_number_of_zero(0),
    m_directory("/tmp"), m_base_name("I"), m_extension("pgm"), m_use_generic_name(true), m_generic_name(generic_name)
{
  init = false;
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

vpDiskGrabber::vpDiskGrabber(const std::string &dir, const std::string &basename,
                             long number,
                             int step, unsigned int noz,
                             const std::string &ext)
  : m_image_number(number), m_image_number_next(number), m_image_step(step), m_number_of_zero(noz),
    m_directory(dir), m_base_name(basename), m_extension(ext), m_use_generic_name(false), m_generic_name("empty")
{
  init = false;
}

/*!
  Read the first image of the sequence.
  The image number is not incremented.
*/
void
vpDiskGrabber::open(vpImage<unsigned char> &I)
{
  long first_number = getImageNumber();

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

/*!
  Read the first image of the sequence.
  The image number is not incremented.
*/
void
vpDiskGrabber::open(vpImage<vpRGBa> &I)
{
  // First we save the image number, so that it can be reaffected after the
  // acquisition. That means that the first image is readed twice
  long first_number = getImageNumber();

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

/*!
  Read the first image of the sequence.
  The image number is not incremented.
*/
void
vpDiskGrabber::open(vpImage<float> &I)
{
  // First we save the image number, so that it can be reaffected after the
  // acquisition. That means that the first image is readed twice
  long first_number = getImageNumber();

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

/*!
  Acquire an image reading the next image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
 */
void
vpDiskGrabber::acquire(vpImage<unsigned char> &I)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;

  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;
  vpImageIo::read(I, ss.str()) ;

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image reading the next image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
 */
void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;

  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;

  vpImageIo::read(I, ss.str()) ;

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image reading the next pfm image from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
 */
void
vpDiskGrabber::acquire(vpImage<float> &I)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;
  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;

  vpImageIo::readPFM(I, ss.str());

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image reading the image with number \e img_number from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<unsigned char> &I, long img_number)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;
  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << img_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;

  vpImageIo::read(I, ss.str());

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Acquire an image reading the image with number \e img_number from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<vpRGBa> &I, long img_number)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;
  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << img_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;

  vpImageIo::read(I, ss.str());

  width = I.getWidth();
  height = I.getHeight();
}


/*!
  Acquire an image reading the pfm image with number \e img_number from the disk.
  After this call, the image number is incremented considering the step.

  \param I : The image read from a file.
  \param img_number : The number of the desired image.
 */
void
vpDiskGrabber::acquire(vpImage<float> &I, long img_number)
{
  m_image_number = m_image_number_next;
  std::stringstream ss;
  if(m_use_generic_name) {
    char filename[FILENAME_MAX];
    sprintf(filename, m_generic_name.c_str(), m_image_number);
    ss << filename;
  }
  else {
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << img_number << "." << m_extension;
  }

  m_image_number_next += m_image_step;

  vpImageIo::readPFM(I, ss.str());

  width = I.getWidth();
  height = I.getHeight();
}

/*!
  Not useful.

  Here for compatibility issue with the vpFrameGrabber class.
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
vpDiskGrabber::setDirectory(const std::string &dir)
{
  m_directory = dir;
}

/*!
  Set the image base name.
*/
void
vpDiskGrabber::setBaseName(const std::string &name)
{
  m_base_name = name;
}

/*!
  Set the image extension.
 */
void
vpDiskGrabber::setExtension(const std::string &ext)
{
  m_extension = ext;
}

/*!
  Set the number of the image to be read.
*/
void
vpDiskGrabber::setImageNumber(long number)
{
  m_image_number = number;
  m_image_number_next = number;
}

/*!
  Set the step between two images.
*/
void
vpDiskGrabber::setStep(long step)
{
  m_image_step = step;
}
/*!
  Set the step between two images.
*/
void
vpDiskGrabber::setNumberOfZero(unsigned int noz)
{
  m_number_of_zero = noz;
}

void
vpDiskGrabber::setGenericName(const std::string &generic_name)
{
  m_generic_name = generic_name;
  m_use_generic_name = true;
}
