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

  vpDetectorAprilTag(const vpAprilTagFamily &tagFamily=TAG_36h11);
  virtual ~vpDetectorAprilTag();

  bool detect(const vpImage<unsigned char> &I);
  void detect(const vpImage<unsigned char> &I, const double tagSize, const vpCameraParameters &cam, std::vector<vpHomogeneousMatrix> &cMo_vec);

  inline bool getPoseFromHomography() const {
    return m_poseFromHomography;
  }

  void setAprilTagNbThreads(const int nThreads);
  void setAprilTagPoseFromHomography(const bool use);
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
  bool m_poseFromHomography;
  vpAprilTagFamily m_tagFamily;

private:
  vpDetectorAprilTag(const vpDetectorAprilTag&);              // noncopyable
  vpDetectorAprilTag& operator=(const vpDetectorAprilTag&);   //

  //PIMPL idiom
  class Impl;
  Impl *m_impl;
};

#endif
#endif
