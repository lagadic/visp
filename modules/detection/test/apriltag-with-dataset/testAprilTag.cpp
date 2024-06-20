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
 * Test AprilTag detection.
 */
/*!
  \example testAprilTag.cpp

  \brief Test AprilTag detection.
*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_CATCH2) && defined(VISP_HAVE_APRILTAG) && (VISP_HAVE_DATASET_VERSION >= 0x030300)
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <iostream>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
namespace
{
struct TagGroundTruth
{
  std::string m_message;
  std::vector<vpImagePoint> m_corners;

  TagGroundTruth(const std::string &msg, const std::vector<vpImagePoint> &c) : m_message(msg), m_corners(c) { }

  bool operator==(const TagGroundTruth &b) const
  {
    if (m_message != b.m_message || m_corners.size() != b.m_corners.size()) {
      return false;
    }

    for (size_t i = 0; i < m_corners.size(); i++) {
      // Allow 0.5 pixel of difference
      if (!vpMath::equal(m_corners[i].get_u(), b.m_corners[i].get_u(), 0.5) ||
          !vpMath::equal(m_corners[i].get_v(), b.m_corners[i].get_v(), 0.5)) {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const TagGroundTruth &b) const { return !(*this == b); }

  double rmse(const std::vector<vpImagePoint> &c)
  {
    double error = 0;

    if (m_corners.size() == c.size()) {
      for (size_t i = 0; i < m_corners.size(); i++) {
        const vpImagePoint &a = m_corners[i];
        const vpImagePoint &b = c[i];
        error += (a.get_i() - b.get_i()) * (a.get_i() - b.get_i()) + (a.get_j() - b.get_j()) * (a.get_j() - b.get_j());
      }
    }
    else {
      return -1;
    }

    return sqrt(error / (2 * m_corners.size()));
  }
};

#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_EIGEN3)
std::ostream &operator<<(std::ostream &os, TagGroundTruth &t)
{
  os << t.m_message << std::endl;
  for (size_t i = 0; i < t.m_corners.size(); i++) {
    os << t.m_corners[i] << std::endl;
  }

  return os;
}
#endif

struct FailedTestCase
{
  vpDetectorAprilTag::vpAprilTagFamily m_family;
  vpDetectorAprilTag::vpPoseEstimationMethod m_method;
  int m_tagId;

  FailedTestCase(const vpDetectorAprilTag::vpAprilTagFamily &family,
                 const vpDetectorAprilTag::vpPoseEstimationMethod &method, int tagId)
    : m_family(family), m_method(method), m_tagId(tagId)
  { }

  bool operator==(const FailedTestCase &b) const
  {
    return m_family == b.m_family && m_method == b.m_method && m_tagId == b.m_tagId;
  }

  bool operator!=(const FailedTestCase &b) const { return !(*this == b); }
};
} // namespace

TEST_CASE("Apriltag pose estimation test", "[apriltag_pose_estimation_test]")
{
  std::map<vpDetectorAprilTag::vpAprilTagFamily, std::string> apriltagMap = {
      {vpDetectorAprilTag::TAG_16h5, "tag16_05"},
      {vpDetectorAprilTag::TAG_25h9, "tag25_09"},
      {vpDetectorAprilTag::TAG_36h11, "tag36_11"},
      {vpDetectorAprilTag::TAG_CIRCLE21h7, "tag21_07"}
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      ,
      {vpDetectorAprilTag::TAG_CIRCLE49h12, "tag49_12"},
      {vpDetectorAprilTag::TAG_CUSTOM48h12, "tag48_12"},
      {vpDetectorAprilTag::TAG_STANDARD41h12, "tag41_12"},
      {vpDetectorAprilTag::TAG_STANDARD52h13, "tag52_13"},
#endif
  };

  std::map<vpDetectorAprilTag::vpAprilTagFamily, double> tagSizeScales = {
      {vpDetectorAprilTag::TAG_16h5, 6.0 / 8},
      {vpDetectorAprilTag::TAG_25h9, 7.0 / 9},
      {vpDetectorAprilTag::TAG_36h11, 8.0 / 10},
      {vpDetectorAprilTag::TAG_CIRCLE21h7, 5.0 / 9}
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      ,
      {vpDetectorAprilTag::TAG_CIRCLE49h12, 5.0 / 11},
      {vpDetectorAprilTag::TAG_CUSTOM48h12, 6.0 / 10},
      {vpDetectorAprilTag::TAG_STANDARD41h12, 5.0 / 9},
      {vpDetectorAprilTag::TAG_STANDARD52h13, 6.0 / 10},
#endif
  };

  std::vector<vpDetectorAprilTag::vpPoseEstimationMethod> poseMethods = {
    vpDetectorAprilTag::HOMOGRAPHY,
    vpDetectorAprilTag::HOMOGRAPHY_ORTHOGONAL_ITERATION,
#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_EIGEN3)
    vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS,
    vpDetectorAprilTag::DEMENTHON_VIRTUAL_VS,
    vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS,
    vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS
#endif
  };
  std::map<vpDetectorAprilTag::vpPoseEstimationMethod, std::string> methodNames = {
    {vpDetectorAprilTag::HOMOGRAPHY, "HOMOGRAPHY"},
    {vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS, "HOMOGRAPHY_VIRTUAL_VS"},
    {vpDetectorAprilTag::HOMOGRAPHY_ORTHOGONAL_ITERATION, "HOMOGRAPHY_ORTHOGONAL_ITERATION"},
#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_EIGEN3)
    {vpDetectorAprilTag::DEMENTHON_VIRTUAL_VS, "DEMENTHON_VIRTUAL_VS"},
    {vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS, "LAGRANGE_VIRTUAL_VS"},
    {vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS, "BEST_RESIDUAL_VIRTUAL_VS"}
#endif
  };

  const size_t nbTags = 5;
  const double tagSize_ = 0.25;
  std::map<int, double> tagsSize = {
      {0, tagSize_}, {1, tagSize_}, {2, tagSize_}, {3, tagSize_ / 2}, {4, tagSize_ / 2},
  };

  std::map<int, double> errorTranslationThresh = {
      {0, 0.025}, {1, 0.09}, {2, 0.05}, {3, 0.13}, {4, 0.09},
  };
  std::map<int, double> errorRotationThresh = {
      {0, 0.04}, {1, 0.075}, {2, 0.07}, {3, 0.18}, {4, 0.13},
  };
  std::vector<FailedTestCase> ignoreTests = {
      FailedTestCase(vpDetectorAprilTag::TAG_STANDARD41h12, vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS, 3),
      FailedTestCase(vpDetectorAprilTag::TAG_STANDARD41h12, vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS, 4),
      FailedTestCase(vpDetectorAprilTag::TAG_CIRCLE21h7, vpDetectorAprilTag::LAGRANGE_VIRTUAL_VS, 3) };

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(700.0, 700.0, 320.0, 240.0);

  std::map<int, vpHomogeneousMatrix> groundTruthPoses;
  for (size_t i = 0; i < nbTags; i++) {
    std::string filename =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), std::string("AprilTag/benchmark/640x480/cMo_") +
                                                                        std::to_string(i) + std::string(".txt"));
    std::ifstream file(filename);
    groundTruthPoses[static_cast<int>(i)].load(file);
  }

  for (const auto &kv : apriltagMap) {
    auto family = kv.first;
    std::cout << "\nApriltag family: " << family << std::endl;
    std::string filename =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                std::string("AprilTag/benchmark/640x480/") + kv.second + std::string("_640x480.png"));
    const double tagSize = tagSize_ * tagSizeScales[family];
    REQUIRE(vpIoTools::checkFilename(filename));

    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);
    REQUIRE(I.getSize() == 640 * 480);

    vpDetectorAprilTag apriltag_detector(family);

    for (auto method : poseMethods) {
      std::cout << "\tPose estimation method: " << method << std::endl;
      apriltag_detector.setAprilTagPoseEstimationMethod(method);

      // Same tags size
      {
        std::vector<vpHomogeneousMatrix> cMo_vec;
        apriltag_detector.detect(I, tagSize, cam, cMo_vec);
        CHECK(cMo_vec.size() == nbTags);

        std::vector<std::vector<vpImagePoint> > tagsCorners = apriltag_detector.getPolygon();
        CHECK(tagsCorners.size() == nbTags);

        std::vector<std::string> messages = apriltag_detector.getMessage();
        CHECK(messages.size() == nbTags);

        std::vector<int> tagsId = apriltag_detector.getTagsId();
        CHECK(tagsId.size() == nbTags);
        std::map<int, int> idsCount;
        for (size_t i = 0; i < tagsId.size(); i++) {
          idsCount[tagsId[i]]++;
          CHECK((tagsId[i] >= 0 && tagsId[i] < 5));
        }
        CHECK(idsCount.size() == nbTags);

        for (size_t i = 0; i < cMo_vec.size(); i++) {
          const vpHomogeneousMatrix &cMo = cMo_vec[i];
          const vpPoseVector pose(cMo);
          int id = tagsId[i];
          if (id >= 3) {
            continue;
          }
          std::cout << "\t\tSame size, Tag: " << i << std::endl;
          const vpHomogeneousMatrix &cMo_truth = groundTruthPoses[id];
          const vpPoseVector pose_truth(cMo_truth);

          vpColVector error_translation = vpColVector(pose.getTranslationVector() - pose_truth.getTranslationVector());
          vpColVector error_thetau = vpColVector(pose.getThetaUVector()) - vpColVector(pose_truth.getThetaUVector());
          double error_trans = sqrt(error_translation.sumSquare() / 3);
          double error_orientation = sqrt(error_thetau.sumSquare() / 3);
          std::cout << "\t\t\tTranslation error: " << error_trans << " / Rotation error: " << error_orientation
            << std::endl;
          CHECK((error_trans < errorTranslationThresh[id] && error_orientation < errorRotationThresh[id]));
        }
      }

      // Custom tags size
      {
        apriltag_detector.detect(I);

        std::vector<std::vector<vpImagePoint> > tagsCorners = apriltag_detector.getPolygon();
        CHECK(tagsCorners.size() == nbTags);

        std::vector<std::string> messages = apriltag_detector.getMessage();
        CHECK(messages.size() == nbTags);

        std::vector<int> tagsId = apriltag_detector.getTagsId();
        CHECK(tagsId.size() == nbTags);
        std::map<int, int> idsCount;
        for (size_t i = 0; i < tagsId.size(); i++) {
          idsCount[tagsId[i]]++;
          CHECK((tagsId[i] >= 0 && tagsId[i] < 5));
        }
        CHECK(idsCount.size() == nbTags);

        for (size_t idx = 0; idx < tagsId.size(); idx++) {
          std::cout << "\t\tCustom size, Tag: " << idx << std::endl;
          const int id = tagsId[idx];
          vpHomogeneousMatrix cMo;
          apriltag_detector.getPose(idx, tagsSize[id] * tagSizeScales[family], cam, cMo);

          const vpPoseVector pose(cMo);
          const vpHomogeneousMatrix &cMo_truth = groundTruthPoses[id];
          const vpPoseVector pose_truth(cMo_truth);

          vpColVector error_translation = vpColVector(pose.getTranslationVector() - pose_truth.getTranslationVector());
          vpColVector error_thetau = vpColVector(pose.getThetaUVector()) - vpColVector(pose_truth.getThetaUVector());
          double error_trans = sqrt(error_translation.sumSquare() / 3);
          double error_orientation = sqrt(error_thetau.sumSquare() / 3);
          std::cout << "\t\t\tTranslation error: " << error_trans << " / Rotation error: " << error_orientation
            << std::endl;
          if (std::find(ignoreTests.begin(), ignoreTests.end(),
                        FailedTestCase(family, method, static_cast<int>(idx))) == ignoreTests.end()) {
            CHECK((error_trans < errorTranslationThresh[id] && error_orientation < errorRotationThresh[id]));
          }
        }
      }

      // Custom tags size + aligned Z-axis
      {
        apriltag_detector.detect(I);

        std::vector<std::vector<vpImagePoint> > tagsCorners = apriltag_detector.getPolygon();
        CHECK(tagsCorners.size() == nbTags);

        std::vector<std::string> messages = apriltag_detector.getMessage();
        CHECK(messages.size() == nbTags);

        std::vector<int> tagsId = apriltag_detector.getTagsId();
        CHECK(tagsId.size() == nbTags);
        std::map<int, int> idsCount;
        for (size_t i = 0; i < tagsId.size(); i++) {
          idsCount[tagsId[i]]++;
          CHECK((tagsId[i] >= 0 && tagsId[i] < 5));
        }
        CHECK(idsCount.size() == nbTags);

        for (size_t idx = 0; idx < tagsId.size(); idx++) {
          std::cout << "\t\tCustom size + aligned Z-axis, Tag: " << idx << std::endl;
          const int id = tagsId[idx];
          vpHomogeneousMatrix cMo;
          apriltag_detector.setZAlignedWithCameraAxis(true);
          apriltag_detector.getPose(idx, tagsSize[id] * tagSizeScales[family], cam, cMo);
          apriltag_detector.setZAlignedWithCameraAxis(false);

          const vpPoseVector pose(cMo);
          const vpHomogeneousMatrix oMo2(vpTranslationVector(), vpThetaUVector(M_PI, 0, 0));
          const vpHomogeneousMatrix cMo_truth = groundTruthPoses[id] * oMo2;
          const vpPoseVector pose_truth(cMo_truth);

          vpColVector error_translation = vpColVector(pose.getTranslationVector() - pose_truth.getTranslationVector());
          vpColVector error_thetau = vpColVector(pose.getThetaUVector()) - vpColVector(pose_truth.getThetaUVector());
          double error_trans = sqrt(error_translation.sumSquare() / 3);
          double error_orientation = sqrt(error_thetau.sumSquare() / 3);
          std::cout << "\t\t\tTranslation error: " << error_trans << " / Rotation error: " << error_orientation
            << std::endl;
          if (std::find(ignoreTests.begin(), ignoreTests.end(),
                        FailedTestCase(family, method, static_cast<int>(idx))) == ignoreTests.end()) {
            CHECK((error_trans < errorTranslationThresh[id] && error_orientation < errorRotationThresh[id]));
          }
        }
      }
    }
  }
}

TEST_CASE("Apriltag corners accuracy test", "[apriltag_corners_accuracy_test]")
{
  std::map<vpDetectorAprilTag::vpAprilTagFamily, std::string> apriltagMap = {
      {vpDetectorAprilTag::TAG_16h5, "tag16_05"},
      {vpDetectorAprilTag::TAG_25h9, "tag25_09"},
      {vpDetectorAprilTag::TAG_36h11, "tag36_11"},
      {vpDetectorAprilTag::TAG_CIRCLE21h7, "tag21_07"}
#if defined(VISP_HAVE_APRILTAG_BIG_FAMILY)
      ,
      {vpDetectorAprilTag::TAG_CIRCLE49h12, "tag49_12"},
      {vpDetectorAprilTag::TAG_CUSTOM48h12, "tag48_12"},
      {vpDetectorAprilTag::TAG_STANDARD41h12, "tag41_12"},
      {vpDetectorAprilTag::TAG_STANDARD52h13, "tag52_13"},
#endif
  };

  const size_t nbTags = 5;
  std::map<vpDetectorAprilTag::vpAprilTagFamily, std::map<int, std::vector<vpImagePoint> > > groundTruthCorners;
  for (const auto &kv : apriltagMap) {
    std::string filename =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                std::string("AprilTag/benchmark/640x480/corners_") + kv.second + std::string(".txt"));
    std::ifstream file(filename);
    REQUIRE(file.is_open());

    int id = 0;
    double p0x = 0, p0y = 0;
    double p1x = 0, p1y = 0;
    double p2x = 0, p2y = 0;
    double p3x = 0, p3y = 0;
    while (file >> id >> p0x >> p0y >> p1x >> p1y >> p2x >> p2y >> p3x >> p3y) {
      groundTruthCorners[kv.first][id].push_back(vpImagePoint(p0y, p0x));
      groundTruthCorners[kv.first][id].push_back(vpImagePoint(p1y, p1x));
      groundTruthCorners[kv.first][id].push_back(vpImagePoint(p2y, p2x));
      groundTruthCorners[kv.first][id].push_back(vpImagePoint(p3y, p3x));
      REQUIRE(groundTruthCorners[kv.first][id].size() == 4);
    }
  }

  for (const auto &kv : apriltagMap) {
    auto family = kv.first;
    std::cout << "\nApriltag family: " << family << std::endl;
    std::string filename =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(),
                                std::string("AprilTag/benchmark/640x480/") + kv.second + std::string("_640x480.png"));
    REQUIRE(vpIoTools::checkFilename(filename));

    vpImage<unsigned char> I;
    vpImageIo::read(I, filename);
    REQUIRE(I.getSize() == 640 * 480);

    vpDetectorAprilTag apriltag_detector(family);
    apriltag_detector.detect(I);
    std::vector<int> tagsId = apriltag_detector.getTagsId();
    std::vector<std::vector<vpImagePoint> > tagsCorners = apriltag_detector.getTagsCorners();

    REQUIRE(tagsCorners.size() == nbTags);
    REQUIRE(tagsId.size() == nbTags);
    for (size_t i = 0; i < tagsCorners.size(); i++) {
      const int tagId = tagsId[i];
      REQUIRE(tagsCorners[i].size() == 4);

      TagGroundTruth corners_ref("", groundTruthCorners[family][tagId]);
      TagGroundTruth corners_cur("", tagsCorners[i]);
      CHECK((corners_ref == corners_cur));

      std::cout << "\tid: " << tagId << " - RMSE: " << corners_ref.rmse(corners_cur.m_corners) << std::endl;
    }
  }
}

#if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_EIGEN3)
TEST_CASE("Apriltag regression test", "[apriltag_regression_test]")
{
#if (VISP_HAVE_DATASET_VERSION >= 0x030600)
  const std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/AprilTag.png");
#else
  const std::string filename = vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/AprilTag.pgm");
#endif
  REQUIRE(vpIoTools::checkFilename(filename));

  vpImage<unsigned char> I;
  vpImageIo::read(I, filename);
  REQUIRE(I.getSize() == 640 * 480);

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  const double tagSize = 0.053;
  const float quad_decimate = 1.0;
  vpDetectorBase *detector = new vpDetectorAprilTag(tagFamily);
  dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagQuadDecimate(quad_decimate);
  dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagPoseEstimationMethod(poseEstimationMethod);

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);

  std::vector<vpHomogeneousMatrix> cMo_vec;
  dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);

  // Ground truth
  std::map<std::string, TagGroundTruth> mapOfTagsGroundTruth;
  {
    std::string filename_ground_truth =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/ground_truth_detection.txt");
    std::ifstream file_ground_truth(filename_ground_truth.c_str());
    REQUIRE(file_ground_truth.is_open());
    std::string message = "";
    double v1 = 0.0, v2 = 0.0, v3 = 0.0, v4 = 0.0;
    double u1 = 0.0, u2 = 0.0, u3 = 0.0, u4 = 0.0;
    while (file_ground_truth >> message >> v1 >> u1 >> v2 >> u2 >> v3 >> u3 >> v4 >> u4) {
      std::vector<vpImagePoint> tagCorners(4);
      tagCorners[0].set_ij(v1, u1);
      tagCorners[1].set_ij(v2, u2);
      tagCorners[2].set_ij(v3, u3);
      tagCorners[3].set_ij(v4, u4);
      mapOfTagsGroundTruth.insert(std::make_pair(message, TagGroundTruth(message, tagCorners)));
    }
  }

  std::map<std::string, vpPoseVector> mapOfPosesGroundTruth;
  {
    std::string filename_ground_truth =
      vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/ground_truth_pose.txt");
    std::ifstream file_ground_truth(filename_ground_truth.c_str());
    REQUIRE(file_ground_truth.is_open());
    std::string message = "";
    double tx = 0.0, ty = 0.0, tz = 0.0;
    double tux = 0.0, tuy = 0.0, tuz = 0.0;
    while (file_ground_truth >> message >> tx >> ty >> tz >> tux >> tuy >> tuz) {
      mapOfPosesGroundTruth.insert(std::make_pair(message, vpPoseVector(tx, ty, tz, tux, tuy, tuz)));
    }
  }

  for (size_t i = 0; i < detector->getNbObjects(); i++) {
    std::vector<vpImagePoint> p = detector->getPolygon(i);

    std::string message = detector->getMessage(i);
    std::replace(message.begin(), message.end(), ' ', '_');
    std::map<std::string, TagGroundTruth>::iterator it = mapOfTagsGroundTruth.find(message);
    TagGroundTruth current(message, p);
    if (it == mapOfTagsGroundTruth.end()) {
      std::cerr << "Problem with tag decoding (tag_family or id): " << message << std::endl;
    }
    else if (it->second != current) {
      std::cerr << "Problem, current detection:\n" << current << "\nReference:\n" << it->second << std::endl;
    }
    REQUIRE(it != mapOfTagsGroundTruth.end());
    CHECK(it->second == current);
  }

  for (size_t i = 0; i < cMo_vec.size(); i++) {
    vpPoseVector pose_vec(cMo_vec[i]);

    std::string message = detector->getMessage(i);
    std::replace(message.begin(), message.end(), ' ', '_');
    std::map<std::string, vpPoseVector>::iterator it = mapOfPosesGroundTruth.find(message);
    if (it == mapOfPosesGroundTruth.end()) {
      std::cerr << "Problem with tag decoding (tag_family or id): " << message << std::endl;
    }
    REQUIRE(it != mapOfPosesGroundTruth.end());
    std::cout << "Tag: " << message << std::endl;
    std::cout << "\tEstimated pose: " << pose_vec.t() << std::endl;
    std::cout << "\tReference pose: " << it->second.t() << std::endl;
    for (unsigned int cpt = 0; cpt < 3; cpt++) {
      if (!vpMath::equal(it->second[cpt], pose_vec[cpt], 0.005) ||
          !vpMath::equal(it->second[cpt + 3], pose_vec[cpt + 3], 0.005)) {
        std::cerr << "Problem, current pose: " << pose_vec.t() << "\nReference pose: " << it->second.t() << std::endl;
      }
      CHECK((vpMath::equal(it->second[cpt], pose_vec[cpt], 0.005) &&
             vpMath::equal(it->second[cpt + 3], pose_vec[cpt + 3], 0.005)));
    }
  }

  delete detector;
}

TEST_CASE("Apriltag copy constructor test", "[apriltag_copy_constructor_test]")
{
  const std::string filename =
    vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/benchmark/640x480/tag21_07_640x480.png");
  REQUIRE(vpIoTools::checkFilename(filename));

  vpImage<unsigned char> I;
  vpImageIo::read(I, filename);
  REQUIRE(I.getSize() == 640 * 480);

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_CIRCLE21h7;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
  const double tagSize = 0.25 * 5 / 9;
  const float quad_decimate = 1.0;
  vpDetectorAprilTag *detector = new vpDetectorAprilTag(tagFamily, poseEstimationMethod);
  detector->setAprilTagQuadDecimate(quad_decimate);

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(700, 700, 320, 240);

  std::vector<vpHomogeneousMatrix> cMo_vec;
  detector->detect(I, tagSize, cam, cMo_vec);
  std::vector<std::vector<vpImagePoint> > tagsCorners = detector->getTagsCorners();
  std::vector<int> tagsId = detector->getTagsId();

  // Copy
  vpDetectorAprilTag detector_copy(*detector);
  // Delete old detector
  delete detector;

  std::vector<std::vector<vpImagePoint> > tagsCorners_copy = detector_copy.getTagsCorners();
  std::vector<int> tagsId_copy = detector_copy.getTagsId();
  REQUIRE(tagsCorners_copy.size() == tagsCorners.size());
  REQUIRE(tagsId_copy.size() == tagsId.size());
  REQUIRE(tagsCorners_copy.size() == tagsId_copy.size());

  for (size_t i = 0; i < tagsCorners.size(); i++) {
    const std::vector<vpImagePoint> &corners_ref = tagsCorners[i];
    const std::vector<vpImagePoint> &corners_copy = tagsCorners_copy[i];
    REQUIRE(corners_ref.size() == corners_copy.size());

    for (size_t j = 0; j < corners_ref.size(); j++) {
      const vpImagePoint &corner_ref = corners_ref[j];
      const vpImagePoint &corner_copy = corners_copy[j];
      CHECK(corner_ref == corner_copy);
    }

    int id_ref = tagsId[i];
    int id_copy = tagsId_copy[i];
    CHECK(id_ref == id_copy);
  }

  std::vector<vpHomogeneousMatrix> cMo_vec_copy;
  detector_copy.detect(I, tagSize, cam, cMo_vec_copy);
  REQUIRE(cMo_vec.size() == cMo_vec_copy.size());
  for (size_t idx = 0; idx < cMo_vec_copy.size(); idx++) {
    const vpHomogeneousMatrix &cMo = cMo_vec[idx];
    const vpHomogeneousMatrix &cMo_copy = cMo_vec_copy[idx];
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        CHECK(vpMath::equal(cMo[i][j], cMo_copy[i][j], std::numeric_limits<double>::epsilon()));
      }
    }
  }
}

TEST_CASE("Apriltag assignment operator test", "[apriltag_assignment_operator_test]")
{
  const std::string filename =
    vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/benchmark/640x480/tag21_07_640x480.png");
  REQUIRE(vpIoTools::checkFilename(filename));

  vpImage<unsigned char> I;
  vpImageIo::read(I, filename);
  REQUIRE(I.getSize() == 640 * 480);

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_CIRCLE21h7;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::BEST_RESIDUAL_VIRTUAL_VS;
  const double tagSize = 0.25 * 5 / 9;
  const float quad_decimate = 1.0;
  vpDetectorAprilTag *detector = new vpDetectorAprilTag(tagFamily, poseEstimationMethod);
  detector->setAprilTagQuadDecimate(quad_decimate);

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(700, 700, 320, 240);

  std::vector<vpHomogeneousMatrix> cMo_vec;
  detector->detect(I, tagSize, cam, cMo_vec);
  std::vector<std::vector<vpImagePoint> > tagsCorners = detector->getTagsCorners();
  std::vector<int> tagsId = detector->getTagsId();

  // Copy
  vpDetectorAprilTag detector_copy = *detector;
  // Delete old detector
  delete detector;

  std::vector<std::vector<vpImagePoint> > tagsCorners_copy = detector_copy.getTagsCorners();
  std::vector<int> tagsId_copy = detector_copy.getTagsId();
  REQUIRE(tagsCorners_copy.size() == tagsCorners.size());
  REQUIRE(tagsId_copy.size() == tagsId.size());
  REQUIRE(tagsCorners_copy.size() == tagsId_copy.size());

  for (size_t i = 0; i < tagsCorners.size(); i++) {
    const std::vector<vpImagePoint> &corners_ref = tagsCorners[i];
    const std::vector<vpImagePoint> &corners_copy = tagsCorners_copy[i];
    REQUIRE(corners_ref.size() == corners_copy.size());

    for (size_t j = 0; j < corners_ref.size(); j++) {
      const vpImagePoint &corner_ref = corners_ref[j];
      const vpImagePoint &corner_copy = corners_copy[j];
      CHECK(corner_ref == corner_copy);
    }

    int id_ref = tagsId[i];
    int id_copy = tagsId_copy[i];
    CHECK(id_ref == id_copy);
  }

  std::vector<vpHomogeneousMatrix> cMo_vec_copy;
  detector_copy.detect(I, tagSize, cam, cMo_vec_copy);
  REQUIRE(cMo_vec.size() == cMo_vec_copy.size());
  for (size_t idx = 0; idx < cMo_vec_copy.size(); idx++) {
    const vpHomogeneousMatrix &cMo = cMo_vec[idx];
    const vpHomogeneousMatrix &cMo_copy = cMo_vec_copy[idx];
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        CHECK(vpMath::equal(cMo[i][j], cMo_copy[i][j], std::numeric_limits<double>::epsilon()));
      }
    }
  }
}

TEST_CASE("Apriltag getTagsPoints3D test", "[apriltag_get_tags_points3D_test]")
{
  const std::string filename =
    vpIoTools::createFilePath(vpIoTools::getViSPImagesDataPath(), "AprilTag/benchmark/640x480/tag21_07_640x480.png");
  REQUIRE(vpIoTools::checkFilename(filename));

  vpImage<unsigned char> I;
  vpImageIo::read(I, filename);
  REQUIRE(I.getSize() == 640 * 480);

  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_CIRCLE21h7;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::DEMENTHON_VIRTUAL_VS;
  const double familyScale = 5.0 / 9;
  const double tagSize = 0.25;
  std::map<int, double> tagsSize = {
      {-1, tagSize * familyScale}, {3, tagSize / 2 * familyScale}, {4, tagSize / 2 * familyScale} };

  vpDetectorAprilTag detector(tagFamily, poseEstimationMethod);

  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(700, 700, 320, 240);

  std::vector<vpHomogeneousMatrix> cMo_vec;
  REQUIRE(detector.detect(I));

  // Compute pose with getPose
  std::vector<int> tagsId = detector.getTagsId();
  for (size_t i = 0; i < tagsId.size(); i++) {
    int id = tagsId[i];
    double size = tagsSize[-1];
    if (tagsSize.find(id) != tagsSize.end()) {
      size = tagsSize[id];
    }

    vpHomogeneousMatrix cMo;
    detector.getPose(i, size, cam, cMo);
    cMo_vec.push_back(cMo);
  }

  // Compute pose manually
  std::vector<std::vector<vpPoint> > tagsPoints = detector.getTagsPoints3D(tagsId, tagsSize);
  std::vector<std::vector<vpImagePoint> > tagsCorners = detector.getTagsCorners();
  REQUIRE(tagsPoints.size() == tagsCorners.size());

  for (size_t i = 0; i < tagsPoints.size(); i++) {
    REQUIRE(tagsPoints[i].size() == tagsCorners[i].size());

    for (size_t j = 0; j < tagsPoints[i].size(); j++) {
      vpPoint &pt = tagsPoints[i][j];
      const vpImagePoint &imPt = tagsCorners[i][j];
      double x = 0, y = 0;
      vpPixelMeterConversion::convertPoint(cam, imPt, x, y);
      pt.set_x(x);
      pt.set_y(y);
    }

    vpPose pose(tagsPoints[i]);
    vpHomogeneousMatrix cMo_manual;
    pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo_manual);

    const vpHomogeneousMatrix &cMo = cMo_vec[i];
    // Note that using epsilon = std::numeric_limits<double>::epsilon() makes this test
    // failing on Ubuntu 18.04 when none of the Lapack 3rd party libraries, nor the built-in are used.
    // Admissible espilon value is 1e-14. Using 1e-15 makes the test failing.
    // Again on Debian i386 where Lapack is enable, using std::numeric_limits<double>::epsilon()
    // makes this test failing.
    double epsilon = 1e-12;

    for (unsigned int row = 0; row < cMo.getRows(); row++) {
      for (unsigned int col = 0; col < cMo.getCols(); col++) {
        CHECK(vpMath::equal(cMo[row][col], cMo_manual[row][col], epsilon));
      }
    }
  }
}
#endif // #if defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_EIGEN3)

int main(int argc, const char *argv[])
{
  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
}
#else
int main() { return EXIT_SUCCESS; }
#endif
