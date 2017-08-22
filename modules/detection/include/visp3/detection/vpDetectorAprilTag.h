/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
#ifndef __vpDetectorAprilTag_h__
#define __vpDetectorAprilTag_h__

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_APRILTAG
#include <visp3/detection/vpDetectorBase.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHomogeneousMatrix.h>

class VISP_EXPORT vpDetectorAprilTag : public vpDetectorBase {

public:
  enum vpAprilTagFamily {
    TAG_36h11,
    TAG_36h10,
    TAG_36ARTOOLKIT,
    TAG_25h9,
    TAG_25h7
  };

  enum vpPoseEstimationMethod {
    HOMOGRAPHY_VIRTUAL_VS,    /*!< Non linear virtual visual servoing approach initialized by the homography approach */
    DEMENTHON_VIRTUAL_VS,     /*!< Non linear virtual visual servoing approach initialized by the Dementhon approach */
    LAGRANGE_VIRTUAL_VS,      /*!< Non linear virtual visual servoing approach initialized by the Lagrange approach */
    BEST_RESIDUAL_VIRTUAL_VS  /*!< Non linear virtual visual servoing approach initialized by the approach that gives the lowest residual */
  };

  vpDetectorAprilTag(const vpAprilTagFamily &tagFamily=TAG_36h11, const vpPoseEstimationMethod &poseEstimationMethod=HOMOGRAPHY_VIRTUAL_VS);
  virtual ~vpDetectorAprilTag();

  bool detect(const vpImage<unsigned char> &I);
  void detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam, std::vector<vpHomogeneousMatrix> &cMo_vec);

  /*!
    Return the pose estimation method.
  */
  inline vpPoseEstimationMethod getPoseEstimationMethod() const {
    return m_poseEstimationMethod;
  }

  void setAprilTagNbThreads(const int nThreads);
  void setAprilTagPoseEstimationMethod(const vpPoseEstimationMethod &poseEstimationMethod);
  void setAprilTagQuadDecimate(const float quadDecimate);
  void setAprilTagQuadSigma(const float quadSigma);
  void setAprilTagRefineDecode(const bool refineDecode);
  void setAprilTagRefineEdges(const bool refineEdges);
  void setAprilTagRefinePose(const bool refinePose);

  inline void setDisplayTag(const bool display) {
    m_displayTag = display;
  }

protected:
  bool m_displayTag;
  vpPoseEstimationMethod m_poseEstimationMethod;
  vpAprilTagFamily m_tagFamily;

private:
  vpDetectorAprilTag(const vpDetectorAprilTag&);              // noncopyable
  vpDetectorAprilTag& operator=(const vpDetectorAprilTag&);   //

  //PIMPL idiom
  class Impl;
  Impl *m_impl;
};

inline std::ostream & operator <<(std::ostream &os, const vpDetectorAprilTag::vpPoseEstimationMethod &method) {
  switch (method) {
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
      break;
  }

  return os;
}

inline std::ostream & operator <<(std::ostream &os, const vpDetectorAprilTag::vpAprilTagFamily &tagFamily) {
  switch (tagFamily) {
    case vpDetectorAprilTag::TAG_36h11:
      os << "TAG_36h11";
      break;

    case vpDetectorAprilTag::TAG_36h10:
      os << "TAG_36h10";
      break;

    case vpDetectorAprilTag::TAG_36ARTOOLKIT:
      os << "TAG_36ARTOOLKIT";
      break;

    case vpDetectorAprilTag::TAG_25h9:
      os << "TAG_25h9";
      break;

    case vpDetectorAprilTag::TAG_25h7:
      os << "TAG_25h7";
      break;

    default:
      break;
  }

  return os;
}

#endif
#endif
