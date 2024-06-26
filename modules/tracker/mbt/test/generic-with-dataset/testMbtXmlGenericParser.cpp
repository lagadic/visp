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
 * Test vpMbtXmlGenericParser parse / save.
 */

/*!
  \file testMbtXmlGenericParser.cpp

  Test vpMbtXmlGenericParser parse / save.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbtXmlGenericParser.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if defined(VISP_HAVE_PUGIXML) && (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  std::string visp_images_dir = vpIoTools::getViSPImagesDataPath();
  if (vpIoTools::checkDirectory(visp_images_dir + "/xml")) {
    double eps = std::numeric_limits<double>::epsilon();
    {
      std::string filename = visp_images_dir + "/xml/chateau.xml";
      vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::EDGE_PARSER | vpMbtXmlGenericParser::KLT_PARSER);
      std::cout << "Parse config: " << filename << std::endl;
      xml.parse(filename);

      vpMe me_ref;
      me_ref.setMaskSize(5);
      me_ref.setMaskNumber(180);
      me_ref.setRange(8);
      me_ref.setMu1(0.5);
      me_ref.setMu2(0.5);
      me_ref.setSampleStep(5);

      vpMe me;
      xml.getEdgeMe(me);
      // Due to changes in visp-images/xml/chateau.xml where it can be now possible to have a normalized me threshold,
      // two cases have to be considered depending on visp-images version
      if (me.getLikelihoodThresholdType() == vpMe::NORMALIZED_THRESHOLD) {
        me_ref.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
        me_ref.setThreshold(5);
      }
      else {
        me_ref.setLikelihoodThresholdType(vpMe::OLD_THRESHOLD);
        me_ref.setThreshold(10000);
      }

      if (me.getMaskSize() != me_ref.getMaskSize() || me.getMaskNumber() != me_ref.getMaskNumber() ||
        me.getRange() != me_ref.getRange() || !vpMath::equal(me.getThreshold(), me_ref.getThreshold(), eps) ||
        !vpMath::equal(me.getMu1(), me_ref.getMu1(), eps) || !vpMath::equal(me.getMu2(), me_ref.getMu2(), eps) ||
        !vpMath::equal(me.getSampleStep(), me_ref.getSampleStep(), eps)) {
        std::cerr << "Issue when parsing xml: " << filename << " (ME)" << std::endl;
        return EXIT_FAILURE;
      }

      if (xml.getKltMaskBorder() != 5 || xml.getKltMaxFeatures() != 10000 || xml.getKltWindowSize() != 5 ||
        !vpMath::equal(xml.getKltQuality(), 0.01, eps) || !vpMath::equal(xml.getKltMinDistance(), 5.0, eps) ||
        !vpMath::equal(xml.getKltHarrisParam(), 0.02, eps) || xml.getKltBlockSize() != 3 ||
        xml.getKltPyramidLevels() != 3) {
        std::cerr << "Issue when parsing xml: " << filename << " (KLT)" << std::endl;
        return EXIT_FAILURE;
      }

      vpCameraParameters cam_ref;
      cam_ref.initPersProjWithoutDistortion(615.1674804688, 615.1675415039, 312.1889953613, 243.4373779297);
      vpCameraParameters cam;
      xml.getCameraParameters(cam);
      if (cam != cam_ref) {
        std::cerr << "Issue when parsing xml: " << filename << " (cam)" << std::endl;
        return EXIT_FAILURE;
      }

      if (!vpMath::equal(xml.getAngleAppear(), 70.0, eps) || !vpMath::equal(xml.getAngleDisappear(), 80.0, eps) ||
        !vpMath::equal(xml.getNearClippingDistance(), 0.01, eps) ||
        !vpMath::equal(xml.getFarClippingDistance(), 2, eps) || !xml.getFovClipping()) {
        std::cerr << "Issue when parsing xml: " << filename << " (visibility)" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::string filename = visp_images_dir + "/xml/chateau.xml";
      vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::PROJECTION_ERROR_PARSER);
      std::cout << "Parse config: " << filename << std::endl;
      xml.parse(filename);
      vpMe me_proj;
      xml.getProjectionErrorMe(me_proj);
      if (!vpMath::equal(me_proj.getSampleStep(), 12.0, eps) || xml.getProjectionErrorKernelSize() != 3) {
        std::cerr << "Issue when parsing xml: " << filename << " (projection error)" << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::string filename = visp_images_dir + "/xml/chateau_depth.xml";
      vpMbtXmlGenericParser xml(vpMbtXmlGenericParser::DEPTH_NORMAL_PARSER | vpMbtXmlGenericParser::DEPTH_DENSE_PARSER);
      std::cout << "Parse config: " << filename << std::endl;
      xml.parse(filename);

      if (xml.getDepthNormalFeatureEstimationMethod() != 0 || xml.getDepthNormalPclPlaneEstimationMethod() != 2 ||
        xml.getDepthNormalPclPlaneEstimationRansacMaxIter() != 200 ||
        !vpMath::equal(xml.getDepthNormalPclPlaneEstimationRansacThreshold(), 0.001, eps) ||
        xml.getDepthNormalSamplingStepX() != 2 || xml.getDepthNormalSamplingStepY() != 2) {
        std::cerr << "Issue when parsing xml: " << filename << " (depth normal)" << std::endl;
        return EXIT_FAILURE;
      }

      if (xml.getDepthDenseSamplingStepX() != 4 || xml.getDepthDenseSamplingStepY() != 4) {
        std::cerr << "Issue when parsing xml: " << filename << " (depth dense)" << std::endl;
        return EXIT_FAILURE;
      }

      vpCameraParameters cam_ref;
      cam_ref.initPersProjWithoutDistortion(476.0536193848, 476.0534973145, 311.4845581055, 246.2832336426);
      vpCameraParameters cam;
      xml.getCameraParameters(cam);
      if (cam != cam_ref) {
        std::cerr << "Issue when parsing xml: " << filename << " (cam)" << std::endl;
        return EXIT_FAILURE;
      }

      if (!vpMath::equal(xml.getAngleAppear(), 70.0, eps) || !vpMath::equal(xml.getAngleDisappear(), 80.0, eps) ||
        !vpMath::equal(xml.getNearClippingDistance(), 0.01, eps) ||
        !vpMath::equal(xml.getFarClippingDistance(), 2, eps) || !xml.getFovClipping()) {
        std::cerr << "Issue when parsing xml: " << filename << " (visibility)" << std::endl;
        return EXIT_FAILURE;
      }
    }
  }
#elif !(defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
#elif !(defined(VISP_HAVE_PUGIXML))
  std::cout << "Cannot run this example: enable pugixml built-in" << std::endl;
#endif

  std::cout << "Test succeed" << std::endl;
  return EXIT_SUCCESS;
}
