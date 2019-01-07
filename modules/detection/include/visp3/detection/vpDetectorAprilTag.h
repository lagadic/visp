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
 * Base class for April Tag detection.
 *
 *****************************************************************************/
#ifndef _vpDetectorAprilTag_h_
#define _vpDetectorAprilTag_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_APRILTAG
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpColor.h>
#include <visp3/detection/vpDetectorBase.h>

/*!
  \class vpDetectorAprilTag
  \ingroup group_detection_tag
  Base class for AprilTag detector. This class is a wrapper over <a
  href="https://april.eecs.umich.edu/software/apriltag.html">AprilTag</a>. There
  is no need to download and install AprilTag from source code or existing
  pre-built packages since the source code is embedded in ViSP. Reference papers
  are <I> AprilTag: A robust and flexible visual fiducial system </I>
  (\cite olson2011tags) and <I> AprilTag 2: Efficient and robust fiducial
  detection</I> (\cite wang2016iros).

  The detect() function allows to detect multiple tags in an image. Once
  detected, for each tag it is possible to retrieve the location of the corners
  using getPolygon(), the encoded message using getMessage(), the bounding box
  using getBBox() and the center of gravity using getCog().

  If camera parameters and the size of the tag are provided, you can also estimate
  the 3D pose of the tag in terms of position and orientation wrt the camera considering 2 cases:
  - 1. If all the tags have the same size use
  detect(const vpImage<unsigned char> &, const double, const vpCameraParameters &, std::vector<vpHomogeneousMatrix> &)
  - 2. If tag sizes differ, use rather getPose()

  The following sample code shows how to use this class to detect the location
of 36h11 AprilTag patterns in an image.
\code
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_APRILTAG
  vpImage<unsigned char> I;
  vpImageIo::read(I, "image-tag36h11.pgm");

  vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);

  bool status = detector.detect(I);
  if (status) {
    for(size_t i=0; i < detector.getNbObjects(); i++) {
      std::cout << "Tag code " << i << ":" << std::endl;
      std::vector<vpImagePoint> p = detector.getPolygon(i);
      for(size_t j=0; j < p.size(); j++)
        std::cout << "  Point " << j << ": " << p[j] << std::endl;
      std::cout << "  Message: \"" << detector.getMessage(i) << "\"" << std::endl;
    }
  }
#endif
}
  \endcode

  The previous example may produce results like:
  \code
Tag code 0:
  Point 0: 124.008, 442.226
  Point 1: 194.614, 441.237
  Point 2: 184.833, 540.386
  Point 3: 111.948, 533.634
  Message: "36h11 id: 0"
Tag code 1:
  Point 0: 245.327, 438.801
  Point 1: 338.116, 437.221
  Point 2: 339.341, 553.539
  Point 3: 238.954, 543.855
  Message: "36h11 id: 1"
  \endcode

  This other example shows how to estimate the 3D pose of 36h11 AprilTag
patterns considering that all the tags have the same size (in our example 0.053 m).
\code
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_APRILTAG
  vpImage<unsigned char> I;
  vpImageIo::read(I, "image-tag36h11.pgm");

  vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
  std::vector<vpHomogeneousMatrix> cMo;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
  double tagSize = 0.053;

  bool status = detector.detect(I, tagSize, cam, cMo);
  if (status) {
    for(size_t i=0; i < detector.getNbObjects(); i++) {
      std::cout << "Tag number " << i << ":" << std::endl;
      std::cout << "  Message: \"" << detector.getMessage(i) << "\"" << std::endl;
      std::cout << "  Pose: " << vpPoseVector(cMo[i]).t() << std::endl;
      std::size_t tag_id_pos = detector.getMessage(i).find("id: ");
      if (tag_id_pos != std::string::npos) {
        std::string tag_id = detector.getMessage(i).substr(tag_id_pos + 4);
        std::cout << "  Tag Id: " << tag_id << std::endl;
      }
    }
  }
#endif
}
  \endcode
  The previous example may produce results like:
  \code
Tag number 0:
  Message: "36h11 id: 0"
  Pose: 0.1015061088  -0.05239057228  0.3549037285  1.991474322  2.04143538 -0.9412360063
  Tag Id: 0
Tag number 1:
  Message: "36h11 id: 1"
  Pose: 0.08951250829 0.02243780207  0.306540622  1.998073197  2.061488008  -0.8699567948
  Tag Id: 1
\endcode

  In this other example we estimate the 3D pose of 36h11 AprilTag
  patterns considering that tag 36h11 with id 0 (in that case the tag message is "36h11 id: 0")
  has a size of 0.040 m, while all the others have a size of 0.053m.
\code
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_APRILTAG
  vpImage<unsigned char> I;
  vpImageIo::read(I, "image-tag36h11.pgm");

  vpDetectorAprilTag detector(vpDetectorAprilTag::TAG_36h11);
  vpHomogeneousMatrix cMo;
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
  double tagSize_id_0 = 0.04;
  double tagSize_id_others = 0.053;

  bool status = detector.detect(I);
  if (status) {
    for(size_t i=0; i < detector.getNbObjects(); i++) {
      std::cout << "Tag code " << i << ":" << std::endl;
      std::cout << "  Message: \"" << detector.getMessage(i) << "\"" << std::endl;
      if (detector.getMessage(i) == std::string("36h11 id: 0")) {
        if (! detector.getPose(i, tagSize_id_0, cam, cMo)) {
          std::cout << "Unable to get tag index " << i << " pose!" << std::endl;
        }
      }
      else {
        if (! detector.getPose(i, tagSize_id_others, cam, cMo)) {
          std::cout << "Unable to get tag index " << i << " pose!" << std::endl;
        }
      }
      std::cout << "  Pose: " << vpPoseVector(cMo).t() << std::endl;
    }
  }
#endif
}
\endcode
  With respect to the previous example, this example may now produce a different pose for tag with id 0:
  \code
Tag code 0:
  Message: "36h11 id: 0"
  Pose: 0.07660838403  -0.03954005455  0.2678518706  1.991474322  2.04143538  -0.9412360063
Tag code 1:
  Message: "36h11 id: 1"
  Pose: 0.08951250829  0.02243780207  0.306540622  1.998073197  2.061488008  -0.8699567948
\endcode

  Other examples are also provided in tutorial-apriltag-detector.cpp and
  tutorial-apriltag-detector-live.cpp
*/
class VISP_EXPORT vpDetectorAprilTag : public vpDetectorBase
{

public:
  enum vpAprilTagFamily {
    TAG_36h11,       /*!< AprilTag <a
                        href="https://april.eecs.umich.edu/software/apriltag.html">36h11</a>
                        pattern (recommended) */
    TAG_36h10,       /*!< AprilTag <a
                        href="https://april.eecs.umich.edu/software/apriltag.html">36h10</a>
                        pattern */
    TAG_36ARTOOLKIT, /*!< <a href="https://artoolkit.org/">ARToolKit</a>
                        pattern. */
    TAG_25h9,        /*!< AprilTag <a
                        href="https://april.eecs.umich.edu/software/apriltag.html">25h9</a>
                        pattern */
    TAG_25h7,        /*!< AprilTag <a
                        href="https://april.eecs.umich.edu/software/apriltag.html">25h7</a>
                        pattern */
    TAG_16h5         /*!< AprilTag <a
                        href="https://april.eecs.umich.edu/software/apriltag.html">16h5</a>
                        pattern */
  };

  enum vpPoseEstimationMethod {
    HOMOGRAPHY,              /*!< Pose from homography */
    HOMOGRAPHY_VIRTUAL_VS,   /*!< Non linear virtual visual servoing approach
                                initialized by the homography approach */
    DEMENTHON_VIRTUAL_VS,    /*!< Non linear virtual visual servoing approach
                                initialized by the Dementhon approach */
    LAGRANGE_VIRTUAL_VS,     /*!< Non linear virtual visual servoing approach
                                initialized by the Lagrange approach */
    BEST_RESIDUAL_VIRTUAL_VS /*!< Non linear virtual visual servoing approach
                                initialized by the approach that gives the
                                lowest residual */
  };

  vpDetectorAprilTag(const vpAprilTagFamily &tagFamily = TAG_36h11,
                     const vpPoseEstimationMethod &poseEstimationMethod = HOMOGRAPHY_VIRTUAL_VS);
  virtual ~vpDetectorAprilTag();

  bool detect(const vpImage<unsigned char> &I);
  bool detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam,
              std::vector<vpHomogeneousMatrix> &cMo_vec);

  bool getPose(size_t tagIndex, const double tagSize, const vpCameraParameters &cam, vpHomogeneousMatrix &cMo);

  /*!
    Return the pose estimation method.
  */
  inline vpPoseEstimationMethod getPoseEstimationMethod() const { return m_poseEstimationMethod; }

  void setAprilTagNbThreads(const int nThreads);
  void setAprilTagPoseEstimationMethod(const vpPoseEstimationMethod &poseEstimationMethod);
  void setAprilTagQuadDecimate(const float quadDecimate);
  void setAprilTagQuadSigma(const float quadSigma);
  void setAprilTagRefineDecode(const bool refineDecode);
  void setAprilTagRefineEdges(const bool refineEdges);
  void setAprilTagRefinePose(const bool refinePose);

  /*! Allow to enable the display of overlay tag information in the windows
   * (vpDisplay) associated to the input image. */
  inline void setDisplayTag(const bool display, const vpColor &color=vpColor::none,
                            const unsigned int thickness=2) {
    m_displayTag = display;
    m_displayTagColor = color;
    m_displayTagThickness = thickness;
  }

  void setZAlignedWithCameraAxis(bool zAlignedWithCameraFrame);

protected:
  bool m_displayTag;
  vpColor m_displayTagColor;
  unsigned int m_displayTagThickness;
  vpPoseEstimationMethod m_poseEstimationMethod;
  vpAprilTagFamily m_tagFamily;
  bool m_zAlignedWithCameraFrame;

private:
  vpDetectorAprilTag(const vpDetectorAprilTag &);            // noncopyable
  vpDetectorAprilTag &operator=(const vpDetectorAprilTag &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};

inline std::ostream &operator<<(std::ostream &os, const vpDetectorAprilTag::vpPoseEstimationMethod &method)
{
  switch (method) {
  case vpDetectorAprilTag::HOMOGRAPHY:
    os << "HOMOGRAPHY";
    break;

  case vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS:
    os << "HOMOGRAPHY_VIRTUAL_VS";
    break;

  case vpDetectorAprilTag::DEMENTHON_VIRTUAL_VS:
    os << "DEMENTHON_VIRTUAL_VS";
    break;

  case vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS:
    os << "LAGRANGE_VIRTUAL_VS";
    break;

  case vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS:
    os << "BEST_RESIDUAL_VIRTUAL_VS";
    break;

  default:
      os << "ERROR_UNKNOWN_POSE_METHOD!";
    break;
  }

  return os;
}

inline std::ostream &operator<<(std::ostream &os, const vpDetectorAprilTag::vpAprilTagFamily &tagFamily)
{
  switch (tagFamily) {
  case vpDetectorAprilTag::TAG_36h11:
    os << "36h11";
    break;

  case vpDetectorAprilTag::TAG_36h10:
    os << "36h10";
    break;

  case vpDetectorAprilTag::TAG_36ARTOOLKIT:
    os << "36artoolkit";
    break;

  case vpDetectorAprilTag::TAG_25h9:
    os << "25h9";
    break;

  case vpDetectorAprilTag::TAG_25h7:
    os << "25h7";
    break;

  case vpDetectorAprilTag::TAG_16h5:
    os << "16h5";
    break;

  default:
    break;
  }

  return os;
}

#endif
#endif
