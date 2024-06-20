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
 * Test RANSAC 3D pose estimation method.
 */

/*!
  \example testPoseRansac2.cpp
 */

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_CATCH2
#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#include <algorithm>
#include <iomanip>
#include <map>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>
#include <visp3/vision/vpPose.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
#if (VISP_HAVE_DATASET_VERSION >= 0x030300)
bool samePoints(const vpPoint &pt1, const vpPoint &pt2)
{
  return vpMath::equal(pt1.get_oX(), pt2.get_oX(), std::numeric_limits<double>::epsilon()) &&
    vpMath::equal(pt1.get_oY(), pt2.get_oY(), std::numeric_limits<double>::epsilon()) &&
    vpMath::equal(pt1.get_oZ(), pt2.get_oZ(), std::numeric_limits<double>::epsilon()) &&
    vpMath::equal(pt1.get_x(), pt2.get_x(), std::numeric_limits<double>::epsilon()) &&
    vpMath::equal(pt1.get_y(), pt2.get_y(), std::numeric_limits<double>::epsilon());
}

int checkInlierIndex(const std::vector<unsigned int> &vectorOfFoundInlierIndex,
                     const std::vector<bool> &vectorOfOutlierFlags)
{
  int nbInlierIndexOk = 0;

  for (std::vector<unsigned int>::const_iterator it = vectorOfFoundInlierIndex.begin();
       it != vectorOfFoundInlierIndex.end(); ++it) {
    if (!vectorOfOutlierFlags[*it]) {
      nbInlierIndexOk++;
    }
  }

  return nbInlierIndexOk;
}

bool checkInlierPoints(const std::vector<vpPoint> &vectorOfFoundInlierPoints,
                       const std::vector<unsigned int> &vectorOfFoundInlierIndex,
                       const std::vector<vpPoint> &bunnyModelPoints_noisy)
{
  for (size_t i = 0; i < vectorOfFoundInlierPoints.size(); i++) {
    if (!samePoints(vectorOfFoundInlierPoints[i], bunnyModelPoints_noisy[vectorOfFoundInlierIndex[i]])) {
      std::cerr << "Problem with the inlier index and the corresponding "
        "inlier point!"
        << std::endl;
      std::cerr << "Returned inliers: oX=" << std::setprecision(std::numeric_limits<double>::max_digits10)
        << vectorOfFoundInlierPoints[i].get_oX() << ", oY=" << vectorOfFoundInlierPoints[i].get_oY()
        << ", oZ=" << vectorOfFoundInlierPoints[i].get_oZ() << " ; x=" << vectorOfFoundInlierPoints[i].get_x()
        << ", y=" << vectorOfFoundInlierPoints[i].get_y() << std::endl;
      const vpPoint &pt = bunnyModelPoints_noisy[vectorOfFoundInlierIndex[i]];
      std::cerr << "Object points: oX=" << std::setprecision(std::numeric_limits<double>::max_digits10) << pt.get_oX()
        << ", oY=" << pt.get_oY() << ", oZ=" << pt.get_oZ() << " ; x=" << pt.get_x() << ", y=" << pt.get_y()
        << std::endl;
      return false;
    }
  }

  return true;
}

void readBunnyModelPoints(const std::string &filename, std::vector<vpPoint> &bunnyModelPoints,
                          std::vector<vpPoint> &bunnyModelPoints_noisy)
{
  // Read the model
  std::ifstream file(filename);
  if (!file.is_open()) {
    return;
  }

  // ground truth cMo
  const vpTranslationVector translation(-0.14568, 0.154567, 1.4462);
  const vpRzyxVector zyxVector(vpMath::rad(12.4146f), vpMath::rad(-75.5478f), vpMath::rad(138.5607f));
  vpHomogeneousMatrix cMo_groundTruth(translation, vpThetaUVector(zyxVector));

  vpGaussRand gaussian_noise(0.0002, 0.0);
  double oX = 0, oY = 0, oZ = 0;

  while (file >> oX >> oY >> oZ) {
    vpPoint pt(oX, oY, oZ);
    pt.project(cMo_groundTruth);
    bunnyModelPoints.push_back(pt);

    // Add a small gaussian noise to the data
    pt.set_x(pt.get_x() + gaussian_noise());
    pt.set_y(pt.get_y() + gaussian_noise());
    bunnyModelPoints_noisy.push_back(pt);
  }

  // Print the number of model points
  std::cout << "The raw model contains " << bunnyModelPoints.size() << " points." << std::endl;
  std::cout << "cMo_groundTruth=\n" << cMo_groundTruth << std::endl << std::endl;
}

bool testRansac(const std::vector<vpPoint> &bunnyModelPoints_original,
                const std::vector<vpPoint> &bunnyModelPoints_noisy_original, size_t nb_model_points,
                bool test_duplicate, bool test_degenerate)
{
  std::vector<vpPoint> bunnyModelPoints = bunnyModelPoints_original;
  std::vector<vpPoint> bunnyModelPoints_noisy = bunnyModelPoints_noisy_original;
  // Resize
  if (nb_model_points > 0) {
    bunnyModelPoints.resize(nb_model_points);
    bunnyModelPoints_noisy.resize(nb_model_points);
  }

  vpPose ground_truth_pose, real_pose;
  vpHomogeneousMatrix cMo_estimated;
  ground_truth_pose.addPoints(bunnyModelPoints);
  real_pose.addPoints(bunnyModelPoints_noisy);
  real_pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo_estimated);
  double r_vvs = ground_truth_pose.computeResidual(cMo_estimated);

  std::cout << "\ncMo estimated using VVS on data with small gaussian noise:\n" << cMo_estimated << std::endl;
  std::cout << "Corresponding residual: " << r_vvs << std::endl;

  size_t nbOutliers = (size_t)(0.35 * bunnyModelPoints_noisy.size());
  vpGaussRand noise(0.01, 0.008);
  // Vector that indicates if the point is an outlier or not
  std::vector<bool> vectorOfOutlierFlags(bunnyModelPoints_noisy.size(), false);
  // Generate outliers points
  for (size_t i = 0; i < nbOutliers; i++) {
    bunnyModelPoints_noisy[i].set_x(bunnyModelPoints_noisy[i].get_x() + noise());
    bunnyModelPoints_noisy[i].set_y(bunnyModelPoints_noisy[i].get_y() + noise());
    vectorOfOutlierFlags[i] = true;
  }

  if (test_duplicate) {
    // Add some duplicate points
    size_t nbDuplicatePoints = 100;
    for (size_t i = 0; i < nbDuplicatePoints; i++) {
      size_t index = (size_t)rand() % bunnyModelPoints_noisy.size();
      vpPoint duplicatePoint = bunnyModelPoints_noisy[index];
      bunnyModelPoints_noisy.push_back(duplicatePoint);
      vectorOfOutlierFlags.push_back(true);
    }
  }

  if (test_degenerate) {
    // Add some degenerate points
    size_t nbDegeneratePoints = 100;
    double degenerate_tolerence = 9.999e-7; // 1e-6 is used in the code to
                                            // detect if a point is degenerate
                                            // or not
    std::vector<vpPoint> listOfDegeneratePoints;
    for (size_t i = 0; i < nbDegeneratePoints; i++) {
      size_t index = (size_t)rand() % bunnyModelPoints_noisy.size();
      vpPoint degeneratePoint = bunnyModelPoints_noisy[index];

      // Object point is degenerate
      degeneratePoint.set_oX(degeneratePoint.get_oX() + degenerate_tolerence);
      degeneratePoint.set_oY(degeneratePoint.get_oY() + degenerate_tolerence);
      degeneratePoint.set_oZ(degeneratePoint.get_oZ() - degenerate_tolerence);

      // Add duplicate 3D points
      listOfDegeneratePoints.push_back(degeneratePoint);

      // Image point is degenerate
      index = (size_t)rand() % bunnyModelPoints_noisy.size();
      degeneratePoint = bunnyModelPoints_noisy[index];

      degeneratePoint.set_x(degeneratePoint.get_x() + degenerate_tolerence);
      degeneratePoint.set_y(degeneratePoint.get_y() - degenerate_tolerence);

      // Add duplicate 2D points
      listOfDegeneratePoints.push_back(degeneratePoint);
    }

    for (std::vector<vpPoint>::const_iterator it_degenerate = listOfDegeneratePoints.begin();
         it_degenerate != listOfDegeneratePoints.end(); ++it_degenerate) {
      bunnyModelPoints_noisy.push_back(*it_degenerate);
      vectorOfOutlierFlags.push_back(true);
    }
  }

  // Shuffle the data vector
  std::vector<size_t> vectorOfIndex(bunnyModelPoints_noisy.size());
  for (size_t i = 0; i < vectorOfIndex.size(); i++) {
    vectorOfIndex[i] = i;
  }

  // std::random_shuffle(vectorOfIndex.begin(), vectorOfIndex.end()); // std::random_shuffle is deprecated in C++14
  std::random_device rng;
  std::mt19937 urng(rng());
  std::shuffle(vectorOfIndex.begin(), vectorOfIndex.end(), urng);

  std::vector<vpPoint> bunnyModelPoints_noisy_tmp = bunnyModelPoints_noisy;
  bunnyModelPoints_noisy.clear();
  std::vector<bool> vectorOfOutlierFlags_tmp = vectorOfOutlierFlags;
  vectorOfOutlierFlags.clear();
  for (std::vector<size_t>::const_iterator it = vectorOfIndex.begin(); it != vectorOfIndex.end(); ++it) {
    bunnyModelPoints_noisy.push_back(bunnyModelPoints_noisy_tmp[*it]);
    vectorOfOutlierFlags.push_back(vectorOfOutlierFlags_tmp[*it]);
  }

  // Add data to vpPose
  vpPose pose;
  vpPose pose_ransac, pose_ransac2;

  vpPose pose_ransac_parallel, pose_ransac_parallel2;
  pose_ransac_parallel.setUseParallelRansac(true);
  pose_ransac_parallel2.setUseParallelRansac(true);

  pose_ransac_parallel.setRansacFilterFlag(vpPose::PREFILTER_DEGENERATE_POINTS);
  pose_ransac_parallel2.setRansacFilterFlag(vpPose::PREFILTER_DEGENERATE_POINTS);
  pose_ransac.setRansacFilterFlag(vpPose::PREFILTER_DEGENERATE_POINTS);
  pose_ransac2.setRansacFilterFlag(vpPose::PREFILTER_DEGENERATE_POINTS);
  for (std::vector<vpPoint>::const_iterator it = bunnyModelPoints_noisy.begin(); it != bunnyModelPoints_noisy.end();
       ++it) {
    pose.addPoint(*it);
  }
  // Test addPoints
  pose_ransac.addPoints(bunnyModelPoints_noisy);
  pose_ransac2.addPoints(bunnyModelPoints_noisy);
  pose_ransac_parallel.addPoints(bunnyModelPoints_noisy);
  pose_ransac_parallel2.addPoints(bunnyModelPoints_noisy);

  // Print the number of points in the final data vector
  std::cout << "\nNumber of model points in the noisy data vector: " << bunnyModelPoints_noisy.size() << " points."
    << std::endl
    << std::endl;

  unsigned int nbInlierToReachConsensus = (unsigned int)(60.0 * (double)(bunnyModelPoints_noisy.size()) / 100.0);
  double threshold = 0.001;

  // RANSAC with 1000 iterations
  pose_ransac.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  pose_ransac.setRansacThreshold(threshold);
  pose_ransac.setRansacMaxTrials(1000);
  pose_ransac_parallel.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  pose_ransac_parallel.setRansacThreshold(threshold);
  pose_ransac_parallel.setRansacMaxTrials(1000);

  pose_ransac_parallel2.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  pose_ransac_parallel2.setRansacThreshold(threshold);
  pose_ransac_parallel2.setRansacMaxTrials(vpPose::computeRansacIterations(0.99, 0.4, 4, -1));

  // RANSAC with p=0.99, epsilon=0.4
  pose_ransac2.setRansacNbInliersToReachConsensus(nbInlierToReachConsensus);
  pose_ransac2.setRansacThreshold(threshold);
  int ransac_iterations = vpPose::computeRansacIterations(0.99, 0.4, 4, -1);
  pose_ransac2.setRansacMaxTrials(ransac_iterations);
  std::cout << "Number of RANSAC iterations to ensure p=0.99 and epsilon=0.4: " << ransac_iterations << std::endl;

  vpHomogeneousMatrix cMo_estimated_RANSAC;
  vpChrono chrono_RANSAC;
  chrono_RANSAC.start();
  pose_ransac.computePose(vpPose::RANSAC, cMo_estimated_RANSAC);
  chrono_RANSAC.stop();

  std::cout << "\ncMo estimated with RANSAC (1000 iterations) on noisy data:\n" << cMo_estimated_RANSAC << std::endl;
  std::cout << "Computation time: " << chrono_RANSAC.getDurationMs() << " ms" << std::endl;

  double r_RANSAC_estimated = ground_truth_pose.computeResidual(cMo_estimated_RANSAC);
  std::cout << "Corresponding residual (1000 iterations): " << r_RANSAC_estimated << std::endl;

  vpHomogeneousMatrix cMo_estimated_RANSAC_2;
  chrono_RANSAC.start();
  pose_ransac2.computePose(vpPose::RANSAC, cMo_estimated_RANSAC_2);
  chrono_RANSAC.stop();

  std::cout << "\ncMo estimated with RANSAC (" << ransac_iterations << " iterations) on noisy data:\n"
    << cMo_estimated_RANSAC_2 << std::endl;
  std::cout << "Computation time: " << chrono_RANSAC.getDurationMs() << " ms" << std::endl;

  double r_RANSAC_estimated_2 = ground_truth_pose.computeResidual(cMo_estimated_RANSAC_2);
  std::cout << "Corresponding residual (" << ransac_iterations << " iterations): " << r_RANSAC_estimated_2 << std::endl;

  pose.computePose(vpPose::DEMENTHON_LAGRANGE_VIRTUAL_VS, cMo_estimated);
  std::cout << "\ncMo estimated with only VVS on noisy data:\n" << cMo_estimated << std::endl;

  double r_estimated = ground_truth_pose.computeResidual(cMo_estimated);
  std::cout << "Corresponding residual: " << r_estimated << std::endl;

  vpHomogeneousMatrix cMo_estimated_RANSAC_parallel;
  vpChrono chrono_RANSAC_parallel;
  chrono_RANSAC_parallel.start();
  pose_ransac_parallel.computePose(vpPose::RANSAC, cMo_estimated_RANSAC_parallel);
  chrono_RANSAC_parallel.stop();

  std::cout << "\ncMo estimated with parallel RANSAC (1000 iterations) on "
    "noisy data:\n"
    << cMo_estimated_RANSAC_parallel << std::endl;
  std::cout << "Computation time: " << chrono_RANSAC_parallel.getDurationMs() << " ms" << std::endl;

  double r_RANSAC_estimated_parallel = ground_truth_pose.computeResidual(cMo_estimated_RANSAC_parallel);
  std::cout << "Corresponding residual (1000 iterations): " << r_RANSAC_estimated_parallel << std::endl;

  vpHomogeneousMatrix cMo_estimated_RANSAC_parallel2;
  vpChrono chrono_RANSAC_parallel2;
  chrono_RANSAC_parallel2.start();
  pose_ransac_parallel2.computePose(vpPose::RANSAC, cMo_estimated_RANSAC_parallel2);
  chrono_RANSAC_parallel2.stop();

  std::cout << "\ncMo estimated with parallel RANSAC (" << ransac_iterations << " iterations) on noisy data:\n"
    << cMo_estimated_RANSAC_parallel2 << std::endl;
  std::cout << "Computation time: " << chrono_RANSAC_parallel2.getDurationMs() << " ms" << std::endl;

  double r_RANSAC_estimated_parallel2 = ground_truth_pose.computeResidual(cMo_estimated_RANSAC_parallel2);
  std::cout << "Corresponding residual (" << ransac_iterations << " iterations): " << r_RANSAC_estimated_parallel2
    << std::endl;

// Check inlier index
  std::vector<unsigned int> vectorOfFoundInlierIndex = pose_ransac.getRansacInlierIndex();
  int nbInlierIndexOk = checkInlierIndex(vectorOfFoundInlierIndex, vectorOfOutlierFlags);

  int nbTrueInlierIndex = (int)std::count(vectorOfOutlierFlags.begin(), vectorOfOutlierFlags.end(), false);
  std::cout << "\nThere are " << nbInlierIndexOk << " true inliers found, " << vectorOfFoundInlierIndex.size()
    << " inliers returned and " << nbTrueInlierIndex << " true inliers." << std::endl;

// Check inlier points returned
  std::vector<vpPoint> vectorOfFoundInlierPoints = pose_ransac.getRansacInliers();

  if (vectorOfFoundInlierPoints.size() != vectorOfFoundInlierIndex.size()) {
    std::cerr << "The number of inlier index is different from the number of "
      "inlier points!"
      << std::endl;
    return false;
  }
  if (!checkInlierPoints(vectorOfFoundInlierPoints, vectorOfFoundInlierIndex, bunnyModelPoints_noisy)) {
    return false;
  }

  // Check for RANSAC with p=0.99, epsilon=0.4
  // Check inlier index
  std::cout << "\nCheck for RANSAC iterations: " << ransac_iterations << std::endl;
  std::vector<unsigned int> vectorOfFoundInlierIndex_2 = pose_ransac2.getRansacInlierIndex();
  nbInlierIndexOk = checkInlierIndex(vectorOfFoundInlierIndex_2, vectorOfOutlierFlags);

  std::cout << "There are " << nbInlierIndexOk << " true inliers found, " << vectorOfFoundInlierIndex_2.size()
    << " inliers returned and " << nbTrueInlierIndex << " true inliers." << std::endl;

// Check inlier points returned
  std::vector<vpPoint> vectorOfFoundInlierPoints_2 = pose_ransac2.getRansacInliers();
  if (vectorOfFoundInlierPoints_2.size() != vectorOfFoundInlierIndex_2.size()) {
    std::cerr << "The number of inlier index is different from the number of "
      "inlier points!"
      << std::endl;
    return false;
  }
  if (!checkInlierPoints(vectorOfFoundInlierPoints_2, vectorOfFoundInlierIndex_2, bunnyModelPoints_noisy)) {
    return false;
  }

  // Check for parallel RANSAC
  // Check inlier index
  std::cout << "\nCheck for parallel RANSAC (1000 iterations)" << std::endl;
  std::vector<unsigned int> vectorOfFoundInlierIndex_parallel = pose_ransac_parallel.getRansacInlierIndex();
  nbInlierIndexOk = checkInlierIndex(vectorOfFoundInlierIndex_parallel, vectorOfOutlierFlags);

  std::cout << "There are " << nbInlierIndexOk << " true inliers found, " << vectorOfFoundInlierIndex_parallel.size()
    << " inliers returned and " << nbTrueInlierIndex << " true inliers." << std::endl;

// Check inlier points returned
  std::vector<vpPoint> vectorOfFoundInlierPoints_parallel = pose_ransac_parallel.getRansacInliers();
  if (vectorOfFoundInlierPoints_parallel.size() != vectorOfFoundInlierIndex_parallel.size()) {
    std::cerr << "The number of inlier index is different from the number "
      "of inlier points!"
      << std::endl;
    return false;
  }
  if (!checkInlierPoints(vectorOfFoundInlierPoints_parallel, vectorOfFoundInlierIndex_parallel,
                         bunnyModelPoints_noisy)) {
    return false;
  }

  // Check for parallel RANSAC 2
  // Check inlier index
  std::cout << "\nCheck for parallel RANSAC (" << ransac_iterations << " iterations)" << std::endl;
  std::vector<unsigned int> vectorOfFoundInlierIndex_parallel2 = pose_ransac_parallel2.getRansacInlierIndex();
  nbInlierIndexOk = checkInlierIndex(vectorOfFoundInlierIndex_parallel2, vectorOfOutlierFlags);

  std::cout << "There are " << nbInlierIndexOk << " true inliers found, " << vectorOfFoundInlierIndex_parallel2.size()
    << " inliers returned and " << nbTrueInlierIndex << " true inliers." << std::endl;

// Check inlier points returned
  std::vector<vpPoint> vectorOfFoundInlierPoints_parallel2 = pose_ransac_parallel2.getRansacInliers();
  if (vectorOfFoundInlierPoints_parallel2.size() != vectorOfFoundInlierIndex_parallel2.size()) {
    std::cerr << "The number of inlier index is different from the number "
      "of inlier points!"
      << std::endl;
    return false;
  }
  if (!checkInlierPoints(vectorOfFoundInlierPoints_parallel2, vectorOfFoundInlierIndex_parallel2,
                         bunnyModelPoints_noisy)) {
    return false;
  }

  if (r_RANSAC_estimated > threshold /*|| r_RANSAC_estimated_2 > threshold*/) {
    std::cerr << "The pose estimated with the RANSAC method is badly estimated!" << std::endl;
    std::cerr << "r_RANSAC_estimated=" << r_RANSAC_estimated << std::endl;
    std::cerr << "threshold=" << threshold << std::endl;
    return false;
  }
  else {
    if (r_RANSAC_estimated_parallel > threshold) {
      std::cerr << "The pose estimated with the parallel RANSAC method is "
        "badly estimated!"
        << std::endl;
      std::cerr << "r_RANSAC_estimated_parallel=" << r_RANSAC_estimated_parallel << std::endl;
      std::cerr << "threshold=" << threshold << std::endl;
      return false;
    }
    std::cout << "The pose estimated with the RANSAC method is well estimated!" << std::endl;
  }

  return true;
}
#endif
} // namespace

TEST_CASE("Print RANSAC number of iterations", "[ransac_pose]")
{
  const int sample_sizes[] = { 2, 3, 4, 5, 6, 7, 8 };
  const double epsilon[] = { 0.05, 0.1, 0.2, 0.25, 0.3, 0.4, 0.5 };

  // Format output
  const std::string spacing = "       ";

  std::cout << spacing << " outliers percentage\n"
    << "nb pts\\";
  for (int cpt2 = 0; cpt2 < 7; cpt2++) {
    std::cout << std::setfill(' ') << std::setw(5) << epsilon[cpt2] << " ";
  }
  std::cout << std::endl;

  std::cout << std::setfill(' ') << std::setw(7) << "+";
  for (int cpt2 = 0; cpt2 < 6; cpt2++) {
    std::cout << std::setw(7) << "-------";
  }
  std::cout << std::endl;

  for (int cpt1 = 0; cpt1 < 7; cpt1++) {
    std::cout << std::setfill(' ') << std::setw(6) << sample_sizes[cpt1] << "|";

    for (int cpt2 = 0; cpt2 < 7; cpt2++) {
      int ransac_iters = vpPose::computeRansacIterations(0.99, epsilon[cpt2], sample_sizes[cpt1], -1);
      std::cout << std::setfill(' ') << std::setw(6) << ransac_iters;
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

#if (VISP_HAVE_DATASET_VERSION >= 0x030300)
TEST_CASE("RANSAC pose estimation tests", "[ransac_pose]")
{
  const std::vector<size_t> model_sizes = { 10, 20, 50, 100, 200, 500, 1000, 0, 0 };
  const std::vector<bool> duplicates = { false, false, false, false, false, false, false, false, true };
  const std::vector<bool> degenerates = { false, false, false, false, false, false, true, true, true };

  std::string visp_input_images = vpIoTools::getViSPImagesDataPath();
  std::string model_filename = vpIoTools::createFilePath(visp_input_images, "3dmodel/bunny/bunny.xyz");
  CHECK(vpIoTools::checkFilename(model_filename));

  std::vector<vpPoint> bunnyModelPoints, bunnyModelPoints_noisy_original;
  readBunnyModelPoints(model_filename, bunnyModelPoints, bunnyModelPoints_noisy_original);
  CHECK(bunnyModelPoints.size() == bunnyModelPoints_noisy_original.size());

  for (size_t i = 0; i < model_sizes.size(); i++) {
    std::cout << "\n\n===============================================================================" << std::endl;
    if (model_sizes[i] == 0) {
      std::cout << "Test on " << bunnyModelPoints_noisy_original.size() << " model points." << std::endl;
    }
    else {
      std::cout << "Test on " << model_sizes[i] << " model points." << std::endl;
    }
    std::cout << "Test duplicate: " << duplicates[i] << " ; Test degenerate: " << degenerates[i] << std::endl;

    CHECK(testRansac(bunnyModelPoints, bunnyModelPoints_noisy_original, model_sizes[i], duplicates[i], degenerates[i]));
  }
}
#endif

int main(int argc, char *argv[])
{
#if defined(__mips__) || defined(__mips) || defined(mips) || defined(__MIPS__)
  // To avoid Debian test timeout
  return EXIT_SUCCESS;
#endif

#if (defined(VISP_HAVE_LAPACK) || defined(VISP_HAVE_EIGEN3) || defined(VISP_HAVE_OPENCV))

  Catch::Session session; // There must be exactly one instance

  // Let Catch (using Clara) parse the command line
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();

  // numFailed is clamped to 255 as some unices only use the lower 8 bits.
  // This clamping has already been applied, so just return it here
  // You can also do any post run clean-up here
  return numFailed;
#else
  std::cout << "Cannot run this example: install Lapack, Eigen3 or OpenCV" << std::endl;
  return EXIT_SUCCESS;
#endif
}
#else
int main() { return EXIT_SUCCESS; }
#endif
