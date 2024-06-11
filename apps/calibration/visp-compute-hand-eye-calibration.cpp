/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Compute hand-eye calibration from chessboard poses and robot end-effector poses.
 */

//! \example visp-compute-hand-eye-calibration.cpp
#include <map>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpHandEyeCalibration.h>

void usage(const char *argv[], int error)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0] << " [--data-path <path>] [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  --data-path <path>  Path to the folder containing" << std::endl
    << "    pose_fPe_%d.yaml and pose_cPo_%d.yaml data files." << std::endl
    << "    Default: \"./\"" << std::endl
    << std::endl
    << "  --fPe <generic name>  Generic name of the yaml files" << std::endl
    << "    containing the pose of the end-effector expressed in the robot base" << std::endl
    << "    frame and located in the data path folder." << std::endl
    << "    Default: pose_fPe_%d.yaml" << std::endl
    << std::endl
    << "  --cPo <generic name>  Generic name of the yaml files" << std::endl
    << "    containing the pose of the calibration grid expressed in the camera" << std::endl
    << "    frame and located in the data path folder." << std::endl
    << "    Default: pose_cPo_%d.yaml" << std::endl
    << std::endl
    << "  --output <filename>  File in yaml format containing the pose of the camera" << std::endl
    << "    in the end-effector frame. Data are saved as a pose vector" << std::endl
    << "    with first the 3 translations along X,Y,Z in [m]" << std::endl
    << "    and then the 3 rotations in axis-angle representation (thetaU) in [rad]." << std::endl
    << "    Default: eMc.yaml" << std::endl
    << std::endl
    << "  --help, -h  Print this helper message." << std::endl
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
  std::string opt_fPe_files = "pose_fPe_%d.yaml";
  std::string opt_cPo_files = "pose_cPo_%d.yaml";
  std::string opt_eMc_file = "eMc.yaml";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--data-path" && i + 1 < argc) {
      opt_data_path = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--fPe" && i + 1 < argc) {
      opt_fPe_files = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--cPo" && i + 1 < argc) {
      opt_cPo_files = std::string(argv[i + 1]);
      i++;
    }
    else if (std::string(argv[i]) == "--output" && i + 1 < argc) {
      opt_eMc_file = std::string(argv[i + 1]);
      i++;
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

  std::vector<vpHomogeneousMatrix> cMo;
  std::vector<vpHomogeneousMatrix> wMe;
  vpHomogeneousMatrix eMc;

  std::map<long, std::string> map_fPe_files;
  std::map<long, std::string> map_cPo_files;
  std::vector<std::string> files = vpIoTools::getDirFiles(opt_data_path);
  for (unsigned int i = 0; i < files.size(); i++) {
    long index_fPe = vpIoTools::getIndex(files[i], opt_fPe_files);
    long index_cPo = vpIoTools::getIndex(files[i], opt_cPo_files);
    if (index_fPe != -1) {
      map_fPe_files[index_fPe] = files[i];
    }
    if (index_cPo != -1) {
      map_cPo_files[index_cPo] = files[i];
    }
  }

  if (map_fPe_files.size() == 0) {
    std::cout << "No " << opt_fPe_files
      << " files found. Use --data-path <path> or --fPe <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }
  if (map_cPo_files.size() == 0) {
    std::cout << "No " << opt_cPo_files
      << " files found. Use --data-path <path> or --cPo <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }

  for (std::map<long, std::string>::const_iterator it_fPe = map_fPe_files.begin(); it_fPe != map_fPe_files.end();
    ++it_fPe) {
    std::string file_fPe = vpIoTools::createFilePath(opt_data_path, it_fPe->second);
    std::map<long, std::string>::const_iterator it_cPo = map_cPo_files.find(it_fPe->first);
    if (it_cPo != map_cPo_files.end()) {
      vpPoseVector wPe;
      if (wPe.loadYAML(file_fPe, wPe) == false) {
        std::cout << "Unable to read data from " << file_fPe << ". Skip data" << std::endl;
        continue;
      }

      vpPoseVector cPo;
      std::string file_cPo = vpIoTools::createFilePath(opt_data_path, it_cPo->second);
      if (cPo.loadYAML(file_cPo, cPo) == false) {
        std::cout << "Unable to read data from " << file_cPo << ". Skip data" << std::endl;
        continue;
      }
      std::cout << "Use data from " << opt_data_path << "/" << file_fPe << " and from " << file_cPo << std::endl;
      wMe.push_back(vpHomogeneousMatrix(wPe));
      cMo.push_back(vpHomogeneousMatrix(cPo));
    }
  }

  if (wMe.size() < 3) {
    std::cout << "Not enough data pairs found." << std::endl;
    return EXIT_FAILURE;
  }

  int ret = vpHandEyeCalibration::calibrate(cMo, wMe, eMc);

  if (ret == 0) {
    std::cout << std::endl << "** Hand-eye calibration succeed" << std::endl;
    std::cout << std::endl << "** Hand-eye (eMc) transformation estimated:" << std::endl;
    std::cout << eMc << std::endl;
    std::cout << "** Corresponding pose vector [tx ty tz tux tuy tuz] in [m] and [rad]: " << vpPoseVector(eMc).t() << std::endl;

    vpThetaUVector erc(eMc.getRotationMatrix());
    std::cout << std::endl << "** Translation [m]: " << eMc[0][3] << " " << eMc[1][3] << " " << eMc[2][3] << std::endl;
    std::cout << "** Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "** Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
      << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(eMc.getRotationMatrix());
    std::cout << "** Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;
    vpRxyzVector rxyz(eMc.getRotationMatrix());
    std::cout << "** Rotation (r-x-y-z representation) [rad]: " << rxyz.t() << std::endl;
    std::cout << "** Rotation (r-x-y-z representation) [deg]: " << vpMath::deg(rxyz).t() << std::endl;

    // save eMc
    std::string name_we = vpIoTools::createFilePath(vpIoTools::getParent(opt_eMc_file), vpIoTools::getNameWE(opt_eMc_file)) + ".txt";
    std::cout << std::endl << "Save transformation matrix eMc as an homogeneous matrix in: " << name_we << std::endl;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
    std::ofstream file_eMc(name_we);
#else
    std::ofstream file_eMc(name_we.c_str());
#endif

    eMc.save(file_eMc);

    vpPoseVector pose_vec(eMc);
    std::string output_filename = vpIoTools::createFilePath(vpIoTools::getParent(opt_eMc_file), vpIoTools::getName(opt_eMc_file));
    std::cout << "Save transformation matrix eMc as a vpPoseVector in       : " << output_filename << std::endl;
    pose_vec.saveYAML(output_filename, pose_vec);
  }
  else {
    std::cout << std::endl << "** Hand-eye calibration failed" << std::endl;
    std::cout << std::endl << "Check your input data and ensure they are covering the half sphere over the chessboard." << std::endl;
    std::cout << std::endl << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic.html" << std::endl;
  }

  return EXIT_SUCCESS;
}
