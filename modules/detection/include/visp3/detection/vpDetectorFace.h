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
 * Detect faces.
 */

#ifndef VP_DETECTOR_FACE_H
#define VP_DETECTOR_FACE_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OPENCV) && \
    (((VISP_HAVE_OPENCV_VERSION < 0x050000) && defined(HAVE_OPENCV_OBJDETECT)) || \
     ((VISP_HAVE_OPENCV_VERSION >= 0x050000) && defined(HAVE_OPENCV_XOBJDETECT)))

#include <algorithm> // needed by (std::min) in opencv2/objdetect/objdetect.hpp

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#if (VISP_HAVE_OPENCV_VERSION < 0x050000)
#include <opencv2/objdetect/objdetect.hpp>
#elif (VISP_HAVE_OPENCV_VERSION >= 0x050000)
#include <opencv2/xobjdetect.hpp>
#endif

#include <visp3/detection/vpDetectorBase.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpDetectorFace
 * \ingroup group_detection_face
 * The vpDetectorFace class is a wrapper over OpenCV Haar cascade face
 * detection capabilities. To use this class ViSP should be build against
 * OpenCV 2.2.0 or a more recent version. Installation instructions are provided
 * here https://visp.inria.fr/3rd_opencv.
 *
 * The following sample code shows how to use this class to detect the largest
 * face in the image. The cascade classifier file
 * "haarcascade_frontalface_alt.xml" can be found in ViSP source code or in
 * OpenCV.
 * \code
 * #include <visp3/detection/vpDetectorFace.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_OBJDETECT)
 *   vpImage<unsigned char> I;
 *   vpDetectorFace face_detector;
 *   face_detector.setCascadeClassifierFile("haarcascade_frontalface_alt.xml");
 *
 *   while(1) {
 *     // Acquire a new image in I
 *     bool face_found = face_detector.detect(I);
 *     if (face_found) {
 *       vpRect face_bbox = face_detector.getBBox(0); // largest face has index 0
 *     }
 *   }
 * #endif
 * }
 * \endcode
 *
 * <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>
 *
 * <b>Tutorials</b><br>
 * <span style="margin-left:2em"> If you want a more complete example that works with images acquired from a camera, you may have a look at:</span><br>
 *
 * - \ref tutorial-detection-face
*/
class VISP_EXPORT vpDetectorFace : public vpDetectorBase
{
protected:
  std::vector<cv::Rect> m_faces;        //!< Bounding box of each detected face.
  cv::CascadeClassifier m_face_cascade; //!< Haar cascade classifier file name.
  cv::Mat m_frame_gray;                 //!< OpenCV image used as input for the face detection.

public:
  vpDetectorFace();

  bool detect(const vpImage<unsigned char> &I) VP_OVERRIDE;
  bool detect(const cv::Mat &frame_gray);
  void setCascadeClassifierFile(const std::string &filename);
};
END_VISP_NAMESPACE
#endif
#endif
