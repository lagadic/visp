/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Description:
 * Detect faces.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpDetectorFace_h__
#define __vpDetectorFace_h__

#include <visp/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020200)

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <visp/vpDetectorBase.h>

/*!
  \class vpDetectorFace

  The vpDetectorFace class is a wrapper over OpenCV Haar cascade face detection capabilities.
  To use this class ViSP should be build against OpenCV 2.2.0 or a more recent version.

  The following sample code shows how to use this class to detect the largest face in the image.
  The cascade classifier file "haarcascade_frontalface_alt.xml" can be found in ViSP source code or in OpenCV.
  \code
#include <visp/vpDetectorFace.h>

int main()
{
  vpImage<unsigned char> I;
  vpDetectorFace face_detector;
  face_detector.setCascadeClassifierFile("haarcascade_frontalface_alt.xml");

  while(1) {
    // acquire a new image in I
    bool face_found = face_detector.track(I);
    if (face_found) {
      vpRect face_bbox = face_detector.getBoundingBox(0); // largest face has index 0
    }
  }
}
  \endcode

  A more complete example that works with images acquired from a camera is provided in tutorial-face-detector-live.cpp.
 */
class VISP_EXPORT vpDetectorFace : public vpDetectorBase
{
protected:
  std::vector<cv::Rect> m_faces; //!< Bounding box of each detected face.
  cv::CascadeClassifier m_face_cascade; //!< Haar cascade classifier file name.
  cv::Mat m_frame_gray; //!< OpenCV image used as input for the face detection.

public:
  vpDetectorFace();
  /*!
    Default destructor.
   */
  virtual ~vpDetectorFace() {};
  void setCascadeClassifierFile(const std::string &filename);

  bool detect(const vpImage<unsigned char> &I);
};

#endif
#endif
