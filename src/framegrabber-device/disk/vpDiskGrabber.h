/****************************************************************************
 *
 * $Id: vpDiskGrabber.h,v 1.7 2008-05-14 14:02:49 asaunier Exp $
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


/*!
  \file vpDiskGrabber.h
  \brief Class to load image sequence from the disk.
*/
#ifndef vpDiskGrabber_hh
#define vpDiskGrabber_hh

#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpFrameGrabber.h>
#include <visp/vpRGBa.h>
#include <visp/vpDebug.h>

/*!
  \class vpDiskGrabber

  \brief class to grab images from the disk.

  Defined a virtual video device. "Grab" the images from the disk
  Derived from the vpFrameGrabber class.

  \sa vpFrameGrabber
*/
class VISP_EXPORT vpDiskGrabber  : public vpFrameGrabber
{
private:
  unsigned long image_number ; //!< id of the next image to be read
  int image_step ;    //!< increment between two image id
  unsigned int number_of_zero ; //!< number of zero in the image name (image.00000.pgm)

  char directory[FILENAME_MAX] ; //!< image location
  char base_name[FILENAME_MAX] ; //!< image base name

public:
  vpDiskGrabber();
  vpDiskGrabber(const char *dir, const char *basename,
		unsigned long number, int step, int noz) ;
  virtual ~vpDiskGrabber() ;

  void open(vpImage<unsigned char> &I) ;
  void open(vpImage<vpRGBa> &I) ;

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);

  void close();

  void setDirectory(const char *name);
  void setBaseName(const char *name);
  void setImageNumber(unsigned long number) ;
  void setStep(int a);
  void setNumberOfZero(unsigned int noz);

  /*!
    Return the current image number.
  */
  unsigned long getImageNumber() { return image_number; };
} ;

#endif

