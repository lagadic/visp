/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpDiskGrabber.h
 * \brief Class to load image sequence from the disk.
 */
#ifndef VP_DISK_GRABBER_H
#define VP_DISK_GRABBER_H

#include <string>

#include <visp3/core/vpFrameGrabber.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/io/vpImageIo.h>

BEGIN_VISP_NAMESPACE

/*!
 * \class vpDiskGrabber
 *
 * \ingroup group_io_video
 *
 * \brief Class to grab (ie. read) images from the disk.
 *
 * Defined a virtual video device. "Grab" the images from the disk.
 * Derived from the vpFrameGrabber class.
 *
 * \sa vpFrameGrabber
 *
 * Here an example of capture from the directory
 * "/local/soft/ViSP/ViSP-images/cube". We want to acquire 10 images
 * from the first named "image.0001.pgm" by steps of 2.
 *
 * \code
 * #include <visp3/core/vpImage.h>
 * #include <visp3/io/vpDiskGrabber.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main(){
 *   vpImage<unsigned char> I; // Grey level image
 *
 *   // Declare a framegrabber able to read a sequence of successive
 *   // images from the disk
 *   vpDiskGrabber g;
 *
 *   // Set the path to the directory containing the sequence
 *   g.setDirectory("/local/soft/ViSP/ViSP-images/cube");
 *   // Set the image base name. The directory and the base name constitute
 *   // the constant part of the full filename
 *   g.setBaseName("image.");
 *   // Set the step between two images of the sequence
 *   g.setStep(2);
 *   // Set the number of digits to build the image number
 *   g.setNumberOfZero(4);
 *   // Set the first frame number of the sequence
 *   g.setImageNumber(1);
 *   // Set the image file extension
 *   g.setExtension("pgm");
 *
 *   // Open the framegrabber by loading the first image of the sequence
 *   g.open(I) ;
 *
 *   unsigned int cpt = 1;
 *   // this is the loop over the image sequence
 *   while(cpt ++ < 10)
 *   {
 *     // read the image and then increment the image counter so that the next
 *     // call to acquire(I) will get the next image
 *     g.acquire(I) ;
 *   }
 * }
 * \endcode
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
  std::string m_image_name;

public:
  /*!
   * Default constructor.
   */
  vpDiskGrabber();

  /*!
   * Constructor that takes a generic image sequence as input.
   */
  VP_EXPLICIT vpDiskGrabber(const std::string &genericName);

  /*!
   * Destructor.
   * In fact nothing to destroy...
   */
  virtual ~vpDiskGrabber() { };

  /*!
   * Constructor.
   *
   * \param dir : Location of the image sequence.
   * \param basename : Base name of each image.
   * \param number : Initial image number.
   * \param step : Increment between two images.
   * \param noz : Number of zero to code the image number.
   * \param ext : Extension of the image file.
   */
  VP_EXPLICIT vpDiskGrabber(const std::string &dir, const std::string &basename, long number, int step, unsigned int noz,
                         const std::string &ext);

  /*!
   * Acquire an image reading the next image from the disk.
   * After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   */
  void acquire(vpImage<unsigned char> &I);

  /*!
   * Acquire an image reading the next image from the disk.
   * After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   */
  void acquire(vpImage<vpRGBa> &I);

  /*!
   * Acquire an image reading the next image from the disk.
   * After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   */
  void acquire(vpImage<float> &I);

  /*!
   * Acquire an image reading the image with number \e img_number from the disk.
   * After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   * \param image_number : The number of the desired image.
   */
  void acquire(vpImage<unsigned char> &I, long image_number);

  /*!
   * Acquire an image reading the image with number \e img_number from the disk.
   * After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   * \param image_number : The number of the desired image.
   */
  void acquire(vpImage<vpRGBa> &I, long image_number);

  /*!
   * Acquire an image reading the pfm image with number \e img_number from the
   * disk. After this call, the image number is incremented considering the step.
   *
   * \param I : The image read from a file.
   * \param image_number : The number of the desired image.
   */
  void acquire(vpImage<float> &I, long image_number);

  /*!
   * Not useful.
   *
   * Does nothing. Here for compatibility issue with the vpFrameGrabber class.
   */
  void close() { };

  /*!
   * Return the current image number.
   */
  inline long getImageNumber() const { return m_image_number; }

  /*!
   * Return the name of the file in which the last frame was read.
   */
  inline std::string getImageName() const { return m_image_name; }

  /*!
   * Read the first image of the sequence.
   * The image number is not incremented.
   */
  void open(vpImage<unsigned char> &I);

  /*!
   * Read the first image of the sequence.
   * The image number is not incremented.
   */
  void open(vpImage<vpRGBa> &I);

  /*!
   * Read the first image of the sequence.
   * The image number is not incremented.
   */
  void open(vpImage<float> &I);

  /*!
   * Set the image base name.
   */
  void setBaseName(const std::string &name) { m_base_name = name; }

  /*!
   * Set the main directory name (ie location of the image sequence).
   */
  void setDirectory(const std::string &dir) { m_directory = dir; }

  /*!
   * Set the image extension.
   */
  void setExtension(const std::string &ext) { m_extension = ext; }

  /*!
   * Set the image generic name like `image-%04d.png`.
   */
  void setGenericName(const std::string &genericName);

  /*!
   * Set the number of the image to be read.
   */
  void setImageNumber(long number);

  /*!
   * Set the step between two images.
   */
  void setNumberOfZero(unsigned int noz) { m_number_of_zero = noz; }

  /*!
   * Set the step between two images.
   */
  void setStep(long step) { m_image_step = step; }
};

END_VISP_NAMESPACE

#endif
