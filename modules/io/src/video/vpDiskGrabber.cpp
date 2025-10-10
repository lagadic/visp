/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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

#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpDiskGrabber.h>

BEGIN_VISP_NAMESPACE

vpDiskGrabber::vpDiskGrabber()
  : m_image_number(0), m_image_number_next(0), m_image_step(1), m_number_of_zero(0), m_directory("/tmp"),
  m_base_name("I"), m_extension("pgm"), m_use_generic_name(false), m_generic_name("empty")
{
  init = false;
}

vpDiskGrabber::vpDiskGrabber(const vpDiskGrabber &grabber) : vpFrameGrabber(grabber)
{
  *this = grabber;
}

vpDiskGrabber &vpDiskGrabber::operator=(const vpDiskGrabber &grabber)
{
  m_image_number = grabber.m_image_number;
  m_image_number_next = grabber.m_image_number_next;
  m_image_step = grabber.m_image_step;
  m_number_of_zero = grabber.m_number_of_zero;
  m_directory = grabber.m_directory;
  m_base_name = grabber.m_base_name;
  m_extension = grabber.m_extension;
  m_use_generic_name = grabber.m_use_generic_name;
  m_generic_name = grabber.m_generic_name;

  return *this;
}

vpDiskGrabber::vpDiskGrabber(const std::string &generic_name)
  : m_image_number(0), m_image_number_next(0), m_image_step(1), m_number_of_zero(0), m_directory("/tmp"),
  m_base_name("I"), m_extension("pgm"), m_use_generic_name(true), m_generic_name(generic_name)
{
  init = false;
}

vpDiskGrabber::vpDiskGrabber(const std::string &dir, const std::string &basename, long number, int step,
                             unsigned int noz, const std::string &ext)
  : m_image_number(number), m_image_number_next(number), m_image_step(step), m_number_of_zero(noz), m_directory(dir),
  m_base_name(basename), m_extension(ext), m_use_generic_name(false), m_generic_name("empty"), m_image_name()
{
  init = false;
}

void vpDiskGrabber::open(vpImage<unsigned char> &I)
{
  long first_number = getImageNumber();

  acquire(I);

  setImageNumber(first_number);

  width = I.getWidth();
  height = I.getHeight();

  init = true;
}

void vpDiskGrabber::open(vpImage<vpRGBa> &I)
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

void vpDiskGrabber::open(vpImage<float> &I)
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

void vpDiskGrabber::acquire(vpImage<unsigned char> &I)
{
  m_image_number = m_image_number_next;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next += m_image_step;
  vpImageIo::read(I, m_image_name);

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::acquire(vpImage<vpRGBa> &I)
{
  m_image_number = m_image_number_next;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next += m_image_step;

  vpImageIo::read(I, m_image_name);

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::acquire(vpImage<float> &I)
{
  m_image_number = m_image_number_next;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next += m_image_step;

  std::string extension = vpIoTools::toLowerCase(vpIoTools::getFileExtension(m_image_name, true));

  if (extension.find("npy") != std::string::npos) {
#if defined (VISP_HAVE_MINIZ) && (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98) && defined(VISP_HAVE_WORKING_REGEX)
    visp::cnpy::NpyArray array = visp::cnpy::npy_load(m_image_name);
    float *data = array.data<float>();
    I.init(data, static_cast<unsigned int>(array.shape[0]), static_cast<unsigned int>(array.shape[1]), true);
#else
    throw(vpException(vpException::ioError, "Miniz is not installed, npy files cannot be read"));
#endif
  }
  else {
    vpImageIo::read(I, m_image_name);
  }

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::acquire(vpImage<unsigned char> &I, long image_number)
{
  m_image_number = image_number;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next = m_image_number + m_image_step;

  vpImageIo::read(I, m_image_name);

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::acquire(vpImage<vpRGBa> &I, long image_number)
{
  m_image_number = image_number;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << m_image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next = m_image_number + m_image_step;

  vpImageIo::read(I, m_image_name);

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::acquire(vpImage<float> &I, long image_number)
{
  m_image_number = m_image_number_next;

  if (m_use_generic_name) {
    m_image_name = vpIoTools::formatString(m_generic_name, static_cast<unsigned int>(m_image_number));
  }
  else {
    std::stringstream ss;
    ss << m_directory << "/" << m_base_name << std::setfill('0') << std::setw(m_number_of_zero) << image_number << "."
      << m_extension;
    m_image_name = ss.str();
  }

  m_image_number_next += m_image_step;

  vpImageIo::readPFM(I, m_image_name);

  width = I.getWidth();
  height = I.getHeight();
}

void vpDiskGrabber::setImageNumber(long number)
{
  m_image_number = number;
  m_image_number_next = number;
}

void vpDiskGrabber::setGenericName(const std::string &generic_name)
{
  m_generic_name = generic_name;
  m_use_generic_name = true;
}

END_VISP_NAMESPACE
