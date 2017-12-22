/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

/*!
  \file vpDiskGrabber.h
  \brief Class to load image sequence from the disk.
*/
#ifndef vpDiskGrabber_hh
#define vpDiskGrabber_hh

#include <string>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

/*!
  \class vpDiskGrabber

  \ingroup group_io_video

  \brief Class to grab (ie. read) images from the disk.

  Defined a virtual video device. "Grab" the images from the disk.
  Derived from the vpFrameGrabber class.

  \sa vpFrameGrabber

  Here an example of capture from the directory
  "/local/soft/ViSP/ViSP-images/cube". We want to acquire 10 images
  from the first named "image.0001.pgm" by steps of 2.

\code
#include <visp3/core/vpImage.h>
#include <visp3/io/vpDiskGrabber.h>

int main(){
  vpImage<unsigned char> I; // Grey level image

  // Declare a framegrabber able to read a sequence of successive
  // images from the disk
  vpDiskGrabber g;

  // Set the path to the directory containing the sequence
  g.setDirectory("/local/soft/ViSP/ViSP-images/cube");
  // Set the image base name. The directory and the base name constitute
  // the constant part of the full filename
  g.setBaseName("image.");
  // Set the step between two images of the sequence
  g.setStep(2);
  // Set the number of digits to build the image number
  g.setNumberOfZero(4);
  // Set the first frame number of the sequence
  g.setImageNumber(1);
  // Set the image file extension
  g.setExtension("pgm");

  // Open the framegrabber by loading the first image of the sequence
  g.open(I) ;

  unsigned int cpt = 1;
  // this is the loop over the image sequence
  while(cpt ++ < 10)
  {
    // read the image and then increment the image counter so that the next
    // call to acquire(I) will get the next image
    g.acquire(I) ;
  }
}
\endcode
*/
class VISP_EXPORT vpDiskGrabber : public vpFrameGrabber
{
private:
  long m_image_number;           //!< id of the current image to be read
  long m_image_number_next;      //!< id of the next image to be read
  long m_image_step;             //!< increment between two image id
  unsigned int m_number_of_zero; //!< number of zero in the image name
                                 //!< (image.00000.pgm)

  std::string m_directory; //!< image location
  std::string m_base_name; //!< image base name
  std::string m_extension; //!< image extension

  bool m_use_generic_name;
  std::string m_generic_name;

public:
  vpDiskGrabber();
  explicit vpDiskGrabber(const std::string &genericName);
  explicit vpDiskGrabber(const std::string &dir, const std::string &basename, long number, int step, unsigned int noz,
                         const std::string &ext);
  virtual ~vpDiskGrabber();

  void acquire(vpImage<unsigned char> &I);
  void acquire(vpImage<vpRGBa> &I);
  void acquire(vpImage<float> &I);
  void acquire(vpImage<unsigned char> &I, long image_number);
  void acquire(vpImage<vpRGBa> &I, long image_number);
  void acquire(vpImage<float> &I, long image_number);

  void close();

  /*!
    Return the current image number.
  */
  long getImageNumber() { return m_image_number; };

  void open(vpImage<unsigned char> &I);
  void open(vpImage<vpRGBa> &I);
  void open(vpImage<float> &I);

  void setBaseName(const std::string &name);
  void setDirectory(const std::string &dir);
  void setExtension(const std::string &ext);
  void setGenericName(const std::string &genericName);
  void setImageNumber(long number);
  void setNumberOfZero(unsigned int noz);
  void setStep(long step);
};

#endif
