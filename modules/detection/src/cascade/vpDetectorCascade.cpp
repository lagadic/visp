/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Object detection using cascade of classifiers.
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

#include <algorithm>

#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorCascade.h>

#include <Simd/SimdDetection.hpp>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace {
typedef Simd::Detection<Simd::Allocator> Detection;
bool vpSortBoundingBox(const Detection::Object& o1, const Detection::Object& o2)
{
  return (o1.rect.Area() > o2.rect.Area());
}
}

class vpDetectorCascade::Impl
{
  typedef Simd::Detection<Simd::Allocator> Detection;

public:
  Impl() : m_detector(), m_image(), m_initialized(false), m_maxSize(INT_MAX, INT_MAX),
    m_minSize(30, 30), m_scale(1.1), m_objects() {}

  void detect(const vpImage<unsigned char> &I)
  {
    m_image.Recreate(I.getWidth(), I.getHeight(), Detection::View::Gray8, I.bitmap);

    if (!m_initialized) {
      m_detector.Init(m_image.Size(), m_scale, m_minSize, m_maxSize);
      m_initialized = true;
    }

    m_detector.Detect(m_image, m_objects);
    std::sort(m_objects.begin(), m_objects.end(), vpSortBoundingBox);
  }

  void setCascadeClassifierFile(const std::string &filename)
  {
    m_initialized = false;
    m_detector.Load(filename);
  }

  void setDetectionMaxSize(unsigned int width, unsigned int height)
  {
    m_maxSize.x = width;
    m_maxSize.y = height;
  }

  void setDetectionMinSize(unsigned int width, unsigned int height)
  {
    m_minSize.x = width;
    m_minSize.y = height;
  }

  void setDetectionScaleFactor(double scale)
  {
    m_scale = scale;
  }

private:
  Detection m_detector;
  Detection::View m_image;
  bool m_initialized;
  Detection::Size m_maxSize;
  Detection::Size m_minSize;
  double m_scale;

public:
  Detection::Objects m_objects;
};
#endif

vpDetectorCascade::vpDetectorCascade() : m_impl(new Impl()) {}

vpDetectorCascade::~vpDetectorCascade()
{
  delete m_impl;
}

/*!
  Set path to the Haar cascade file.
 */
void vpDetectorCascade::setCascadeClassifierFile(const std::string &filename)
{
  m_impl->setCascadeClassifierFile(filename);
}

/*!
   Object detection using the Viola & Jones cascade of classifiers.

   \param I : Input image.
 */
bool vpDetectorCascade::detect(const vpImage<unsigned char> &I)
{
  m_impl->detect(I);

  m_message.clear();
  m_polygon.clear();
  m_nb_objects = m_impl->m_objects.size();

  for (size_t i = 0; i < m_impl->m_objects.size(); i++) {
    const Detection::Object& bb = m_impl->m_objects[i];

    std::ostringstream message;
    message << "Object " << i;
    m_message.push_back(message.str());

    std::vector<vpImagePoint> polygon;
    double x = static_cast<double>(bb.rect.Left());
    double y = static_cast<double>(bb.rect.Top());
    double w = static_cast<double>(bb.rect.Width());
    double h = static_cast<double>(bb.rect.Height());

    polygon.push_back(vpImagePoint(y, x));
    polygon.push_back(vpImagePoint(y + h, x));
    polygon.push_back(vpImagePoint(y + h, x + w));
    polygon.push_back(vpImagePoint(y, x + w));

    m_polygon.push_back(polygon);
  }

  return !m_impl->m_objects.empty();
}

void vpDetectorCascade::setDetectionMaxSize(unsigned int width, unsigned int height)
{
  m_impl->setDetectionMaxSize(width, height);
}

void vpDetectorCascade::setDetectionMinSize(unsigned int width, unsigned int height)
{
  m_impl->setDetectionMinSize(width, height);
}

void vpDetectorCascade::setDetectionScaleFactor(double scale)
{
  m_impl->setDetectionScaleFactor(scale);
}
