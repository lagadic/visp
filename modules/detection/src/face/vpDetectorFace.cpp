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
 * Detect faces.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020200)

#include <algorithm>

#include <visp3/core/vpImageConvert.h>
#include <visp3/detection/vpDetectorFace.h>

bool vpSortLargestFace(cv::Rect rect1, cv::Rect rect2) { return (rect1.area() > rect2.area()); }

/*!
  Default constructor.
 */
vpDetectorFace::vpDetectorFace() : m_faces(), m_face_cascade(), m_frame_gray() {}

/*!
  Set the name of the OpenCV cascade classifier file used for face detection.
  \param filename : Full path to access to the file. Such a file can be found
  in OpenCV. Within the last versions it was name
  "haarcascade_frontalface_alt.xml".
 */
void vpDetectorFace::setCascadeClassifierFile(const std::string &filename)
{
  if (!m_face_cascade.load(filename)) {
    throw vpException(vpException::ioError, "Cannot read haar file: %s", filename.c_str());
  }
}

/*!
   Allows to detect a face in the image. When more than one face is detected,
   faces are sorted from largest to smallest.

   \param I : Input image to process. This ViSP image is converted internally
   in an OpenCV cv::Mat image. If you original image is an gray level OpenCV
   image, we suggest rather the use of detect(const cv::Mat &). \return true
   if one or more faces are found, false otherwise.

   The number of detected faces is returned using getNbObjects().
   If a face is found the functions getBBox(), getCog() return some
   information about the location of the face.

   The largest face is always available using getBBox(0) or getCog(0).

   \sa detect(const cv::Mat &)
 */
bool vpDetectorFace::detect(const vpImage<unsigned char> &I)
{
  vpImageConvert::convert(I, m_frame_gray);

  return detect(m_frame_gray);
}
/*!
   Allows to detect a face in the image. When more than one face is detected,
   faces are sorted from largest to smallest.

   \param frame_gray : Input gray level image to process.
   \return true if one or more faces are found, false otherwise.

   The number of detected faces is returned using getNbObjects().
   If a face is found the functions getBBox(), getCog() return some
   information about the location of the face.

   The largest face is always available using getBBox(0) or getCog(0).
 */
bool vpDetectorFace::detect(const cv::Mat &frame_gray)
{
  bool detected = false;
  m_message.clear();
  m_polygon.clear();
  m_nb_objects = 0;

  m_faces.clear();
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  m_face_cascade.detectMultiScale(frame_gray, m_faces, 1.1, 2, 0, cv::Size(30, 30));
#else
  m_face_cascade.detectMultiScale(frame_gray, m_faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
#endif

  m_nb_objects = m_faces.size();

  std::sort(m_faces.begin(), m_faces.end(), vpSortLargestFace);

  if (m_faces.size())
    for (size_t i = 0; i < m_faces.size(); i++) {
      std::ostringstream message;
      message << "Face " << i;
      m_message.push_back(message.str());

      detected = true;

      std::vector<vpImagePoint> polygon;
      double x = m_faces[i].tl().x;
      double y = m_faces[i].tl().y;
      double w = m_faces[i].size().width;
      double h = m_faces[i].size().height;

      polygon.push_back(vpImagePoint(y, x));
      polygon.push_back(vpImagePoint(y + h, x));
      polygon.push_back(vpImagePoint(y + h, x + w));
      polygon.push_back(vpImagePoint(y, x + w));

      m_polygon.push_back(polygon);
    }

  return detected;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpDetectorFace.cpp.o) has no
// symbols
void dummy_vpDetectorFace(){};
#endif
