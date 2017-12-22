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
 * Detect faces.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpDetectorFace_h__
#define __vpDetectorFace_h__

#include <visp3/core/vpConfig.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020200)

#include <algorithm> // needed by (std::min) in opencv2/objdetect/objdetect.hpp

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <visp3/detection/vpDetectorBase.h>

/*!
  \class vpDetectorFace
  \ingroup group_detection_face
  The vpDetectorFace class is a wrapper over OpenCV Haar cascade face
detection capabilities. To use this class ViSP should be build against
OpenCV 2.2.0 or a more recent version. Installation instructions are provided
here https://visp.inria.fr/3rd_opencv.

  The following sample code shows how to use this class to detect the largest
face in the image. The cascade classifier file
"haarcascade_frontalface_alt.xml" can be found in ViSP source code or in
OpenCV.
\code
#include <visp3/detection/vpDetectorFace.h>

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

  A more complete example that works with images acquired from a camera is
provided in tutorial-face-detector-live.cpp.
 */
class VISP_EXPORT vpDetectorFace : public vpDetectorBase
{
protected:
  std::vector<cv::Rect> m_faces;        //!< Bounding box of each detected face.
  cv::CascadeClassifier m_face_cascade; //!< Haar cascade classifier file name.
  cv::Mat m_frame_gray;                 //!< OpenCV image used as input for the face detection.

public:
  vpDetectorFace();
  /*!
    Default destructor.
   */
  virtual ~vpDetectorFace(){};

  bool detect(const vpImage<unsigned char> &I);
  bool detect(const cv::Mat &frame_gray);
  void setCascadeClassifierFile(const std::string &filename);
};

#endif
#endif
