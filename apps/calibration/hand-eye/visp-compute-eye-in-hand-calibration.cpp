/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Compute eye-in-hand calibration from chessboard poses and robot end-effector poses.
 */

//! \example visp-compute-eye-in-hand-calibration.cpp
#include <map>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpHandEyeCalibration.h>

void usage(const char *argv[], int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--data-path <path>]"
    << " [--rPe <generic name>]"
    << " [--cPo <generic name>]"
    << " [--output-ePc <filename>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute eye-in-hand calibration." << std::endl
    << std::endl
    << "  --data-path <path>" << std::endl
    << "    Path to the folder containing pose_rPe_%d.yaml and pose_cPo_%d.yaml data files." << std::endl
    << "    Default: \"./\"" << std::endl
    << std::endl
    << "  --rPe <generic name>" << std::endl
    << "    Generic name of the yaml files containing the pose of the end-effector expressed in the robot" << std::endl
    << "    base frame and located in the data path folder." << std::endl
    << "    Default: pose_rPe_%d.yaml" << std::endl
    << std::endl
    << "  --cPo <generic name>" << std::endl
    << "    Generic name of the yaml files" << std::endl
    << "    containing the pose of the calibration grid expressed in the camera frame and located in the" << std::endl
    << "    data path folder." << std::endl
    << "    Default: pose_cPo_%d.yaml" << std::endl
    << std::endl
    << "  --output-ePc <filename>" << std::endl
    << "    File in yaml format containing the pose of the camera" << std::endl
    << "    in the end-effector frame. Data are saved as a pose vector with first the 3 translations" << std::endl
    << "    along X,Y,Z in [m] and then the 3 rotations in axis-angle representation (thetaU) in [rad]." << std::endl
    << "    Default: ePc.yaml" << std::endl
    << std::endl
    << "  --output-rPo <filename>" << std::endl
    << "    File in yaml format containing the pose of the object" << std::endl
    << "    in the robot reference frame. Data are saved as a pose vector with first the 3 translations" << std::endl
    << "    along X,Y,Z in [m] and then the 3 rotations in axis-angle representation (thetaU) in [rad]." << std::endl
    << "    Default: rPo.yaml" << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char *argv[])
{
#if defined(ENABLE_VISP_NAMESPACE)
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_data_path = "./";
  std::string opt_rPe_files = "pose_rPe_%d.yaml";
  std::string opt_cPo_files = "pose_cPo_%d.yaml";
  std::string opt_ePc_file = "ePc.yaml";
  std::string opt_rPo_file = "rPo.yaml";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--data-path" && i + 1 < argc) {
      opt_data_path = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--rPe" && i + 1 < argc) {
      opt_rPe_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--cPo" && i + 1 < argc) {
      opt_cPo_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--output-ePc" && i + 1 < argc) {
      opt_ePc_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--output-rPo" && i + 1 < argc) {
      opt_rPo_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i);
      return EXIT_FAILURE;
    }
  }

  // Create output folder if necessary
  std::string output_parent = vpIoTools::getParent(opt_ePc_file);
  if (!vpIoTools::checkDirectory(output_parent)) {
    std::cout << "Create output directory: " << output_parent << std::endl;
    vpIoTools::makeDirectory(output_parent);
  }

  std::vector<vpHomogeneousMatrix> cMo;
  std::vector<vpHomogeneousMatrix> rMe;
  vpHomogeneousMatrix eMc;
  vpHomogeneousMatrix rMo;

  std::map<long, std::string> map_rPe_files;
  std::map<long, std::string> map_cPo_files;
  std::vector<std::string> files = vpIoTools::getDirFiles(opt_data_path);
  for (unsigned int i = 0; i < files.size(); i++) {
    long index_rPe = vpIoTools::getIndex(files[i], opt_rPe_files);
    long index_cPo = vpIoTools::getIndex(files[i], opt_cPo_files);
    if (index_rPe != -1) {
      map_rPe_files[index_rPe] = files[i];
    }
    if (index_cPo != -1) {
      map_cPo_files[index_cPo] = files[i];
    }
  }

  if (map_rPe_files.size() == 0) {
    std::cout << "No " << opt_rPe_files
      << " files found. Use --data-path <path> or --rPe <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }
  if (map_cPo_files.size() == 0) {
    std::cout << "No " << opt_cPo_files
      << " files found. Use --data-path <path> or --cPo <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }

  for (std::map<long, std::string>::const_iterator it_rPe = map_rPe_files.begin(); it_rPe != map_rPe_files.end();
    ++it_rPe) {
    std::string file_rPe = vpIoTools::createFilePath(opt_data_path, it_rPe->second);
    std::map<long, std::string>::const_iterator it_cPo = map_cPo_files.find(it_rPe->first);
    if (it_cPo != map_cPo_files.end()) {
      vpPoseVector rPe;
      if (rPe.loadYAML(file_rPe, rPe) == false) {
        std::cout << "Unable to read data from " << file_rPe << ". Skip data" << std::endl;
        continue;
      }

      vpPoseVector cPo;
      std::string file_cPo = vpIoTools::createFilePath(opt_data_path, it_cPo->second);
      if (cPo.loadYAML(file_cPo, cPo) == false) {
        std::cout << "Unable to read data from " << file_cPo << ". Skip data" << std::endl;
        continue;
      }
      std::cout << "Use data from " << opt_data_path << "/" << file_rPe << " and from " << file_cPo << std::endl;
      rMe.push_back(vpHomogeneousMatrix(rPe));
      cMo.push_back(vpHomogeneousMatrix(cPo));
    }
  }

  if (rMe.size() < 3) {
    std::cout << "Not enough data pairs found." << std::endl;
    return EXIT_FAILURE;
  }

  int ret = vpHandEyeCalibration::calibrate(cMo, rMe, eMc, rMo);

  if (ret == 0) {
    std::cout << std::endl << "Eye-in-hand calibration succeed" << std::endl;
    std::cout << std::endl << "Estimated eMc transformation:" << std::endl;
    std::cout << "-----------------------------" << std::endl;
    std::cout << eMc << std::endl << std::endl;
    std::cout << "- Corresponding pose vector [tx ty tz tux tuy tuz] in [m] and [rad]: " << vpPoseVector(eMc).t() << std::endl;

    vpThetaUVector erc(eMc.getRotationMatrix());
    std::cout << std::endl << "** Translation [m]: " << eMc[0][3] << " " << eMc[1][3] << " " << eMc[2][3] << std::endl;
    std::cout << "- Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "- Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
      << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "- Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;
    vpRxyzVector rxyz(eMc.getRotationMatrix());
    std::cout << "- Rotation (r-x-y-z representation) [rad]: " << rxyz.t() << std::endl;
    std::cout << "- Rotation (r-x-y-z representation) [deg]: " << vpMath::deg(rxyz).t() << std::endl;

    std::cout << std::endl << "Estimated rMo transformation:" << std::endl;
    std::cout << "-----------------------------" << std::endl;
    std::cout << rMo << std::endl;
    std::cout << "- Corresponding pose vector [tx ty tz tux tuy tuz] in [m] and [rad]: " << vpPoseVector(rMo).t() << std::endl;

    vpThetaUVector wrc(rMo.getRotationMatrix());
    std::cout << std::endl << "- Translation [m]: " << rMo[0][3] << " " << rMo[1][3] << " " << rMo[2][3] << std::endl;
    std::cout << "- Rotation (theta-u representation) [rad]: " << wrc.t() << std::endl;
    std::cout << "- Rotation (theta-u representation) [deg]: " << vpMath::deg(wrc[0]) << " " << vpMath::deg(wrc[1])
      << " " << vpMath::deg(wrc[2]) << std::endl;
    vpQuaternionVector quaternion2(rMo.getRotationMatrix());
    std::cout << "- Rotation (quaternion representation) [rad]: " << quaternion2.t() << std::endl;
    vpRxyzVector rxyz2(rMo.getRotationMatrix());
    std::cout << "- Rotation (r-x-y-z representation) [rad]: " << rxyz2.t() << std::endl;
    std::cout << "- Rotation (r-x-y-z representation) [deg]: " << vpMath::deg(rxyz).t() << std::endl;

    {
      // save eMc
      std::string name_we = vpIoTools::createFilePath(vpIoTools::getParent(opt_ePc_file), vpIoTools::getNameWE(opt_ePc_file)) + ".txt";
      std::cout << std::endl << "Save transformation matrix eMc as an homogeneous matrix in: " << name_we << std::endl;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      std::ofstream file_eMc(name_we);
#else
      std::ofstream file_eMc(name_we.c_str());
#endif

      eMc.save(file_eMc);

      vpPoseVector pose_vec(eMc);
      std::string output_filename = vpIoTools::createFilePath(vpIoTools::getParent(opt_ePc_file), vpIoTools::getName(opt_ePc_file));
      std::cout << "Save transformation matrix eMc as a vpPoseVector in       : " << output_filename << std::endl;
      pose_vec.saveYAML(output_filename, pose_vec);
    }

    {
      // save rMo
      std::string name_we = vpIoTools::createFilePath(vpIoTools::getParent(opt_rPo_file), vpIoTools::getNameWE(opt_rPo_file)) + ".txt";
      std::cout << std::endl << "Save transformation matrix rMo as an homogeneous matrix in: " << name_we << std::endl;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      std::ofstream file_rMo(name_we);
#else
      std::ofstream file_rMo(name_we.c_str());
#endif

      rMo.save(file_rMo);

      vpPoseVector pose_vec(rMo);
      std::string output_filename = vpIoTools::createFilePath(vpIoTools::getParent(opt_rPo_file), vpIoTools::getName(opt_rPo_file));
      std::cout << "Save transformation matrix rMo as a vpPoseVector in       : " << output_filename << std::endl;
      pose_vec.saveYAML(output_filename, pose_vec, "Robot reference to object frames transformation (rMo)");
    }
  }
  else {
    std::cout << std::endl << "** Eye-in-hand calibration failed" << std::endl;
    std::cout << std::endl << "Check your input data and ensure they are covering the half sphere over the chessboard." << std::endl;
    std::cout << std::endl << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic-eye-in-hand.html" << std::endl;
  }

  return EXIT_SUCCESS;
}
