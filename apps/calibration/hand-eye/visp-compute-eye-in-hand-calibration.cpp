/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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

/*!
 * \example visp-compute-eye-in-hand-calibration.cpp
 * App that allows to perform eye-in-hand calibration.
 */
#include <map>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/vision/vpHandEyeCalibration.h>

void usage(const char *argv[], int error, const std::string &data_path, const std::string &w_P_ee_files,
           const std::string &c_P_o_files, const std::string &ee_P_c_file, const std::string &w_P_o_file)
{
  std::cout << "Synopsis" << std::endl
    << "  " << argv[0]
    << " [--data-path <path>]"
    << " [--w_P_ee <generic name>]"
    << " [--c_P_o <generic name>]"
    << " [--output-ee_P_c <filename>]"
    << " [--output-w_P_o <filename>]"
    << " [--help, -h]" << std::endl
    << std::endl;
  std::cout << "Description" << std::endl
    << "  Compute eye-in-hand calibration." << std::endl
    << std::endl
    << "  --data-path <path>" << std::endl
    << "    Path to the folder containing data (poses, camera parameters)." << std::endl
    << "    Default: \"" << data_path << "\"" << std::endl
    << std::endl
    << "  --w_P_ee <generic name>" << std::endl
    << "    Generic name of the yaml files containing the pose of the end-effector expressed in the robot" << std::endl
    << "    world frame and located in the data path folder." << std::endl
    << "    Default: \"" << w_P_ee_files << "\"" << std::endl
    << std::endl
    << "  --c_P_o <generic name>" << std::endl
    << "    Generic name of the yaml files containing the pose of the chessboard grid " << std::endl
    << "    expressed in the camera frame and located in the data path folder." << std::endl
    << "    Default: \"" << c_P_o_files << "\"" << std::endl
    << std::endl
    << "  --output-ee_P_c <filename>" << std::endl
    << "    File in yaml format containing the pose of the camera" << std::endl
    << "    in the end-effector frame. Data are saved as a pose vector with first the 3 translations" << std::endl
    << "    along X,Y,Z in [m] and then the 3 rotations in axis-angle representation (thetaU) in [rad]." << std::endl
    << "    Default: \"" << ee_P_c_file << "\"" << std::endl
    << std::endl
    << "  --output-w_P_o <filename>" << std::endl
    << "    File in yaml format containing the pose of the object" << std::endl
    << "    in the robot reference frame. Data are saved as a pose vector with first the 3 translations" << std::endl
    << "    along X,Y,Z in [m] and then the 3 rotations in axis-angle representation (thetaU) in [rad]." << std::endl
    << "    Default: \"" << w_P_o_file << "\"" << std::endl
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

  std::string opt_data_path    = "./data";
  std::string opt_w_P_ee_files = "pose_w_P_ee-%d.yaml";
  std::string opt_c_P_o_files  = "pose_c_P_o-%d.yaml";
  std::string opt_ee_P_c_file  = "ee_P_c.yaml";
  std::string opt_w_P_o_file   = "w_P_o.yaml";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--data-path" && i + 1 < argc) {
      opt_data_path = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--w_P_ee" && i + 1 < argc) {
      opt_w_P_ee_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--c_P_o" && i + 1 < argc) {
      opt_c_P_o_files = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--output-ee_P_c" && i + 1 < argc) {
      opt_ee_P_c_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--output-w_P_o" && i + 1 < argc) {
      opt_w_P_o_file = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      usage(argv, 0, opt_data_path, opt_w_P_ee_files, opt_c_P_o_files, opt_ee_P_c_file, opt_w_P_o_file);
      return EXIT_SUCCESS;
    }
    else {
      usage(argv, i, opt_data_path, opt_w_P_ee_files, opt_c_P_o_files, opt_ee_P_c_file, opt_w_P_o_file);
      return EXIT_FAILURE;
    }
  }

  // Create output folder if necessary
  std::string output_parent = vpIoTools::getParent(opt_ee_P_c_file);
  if (!vpIoTools::checkDirectory(output_parent)) {
    std::cout << "Create output directory: " << output_parent << std::endl;
    vpIoTools::makeDirectory(output_parent);
  }

  std::vector<vpHomogeneousMatrix> c_M_o;
  std::vector<vpHomogeneousMatrix> w_M_ee;
  vpHomogeneousMatrix ee_M_c;
  vpHomogeneousMatrix w_M_o;

  std::map<long, std::string> map_w_P_ee_files;
  std::map<long, std::string> map_c_P_o_files;
  std::vector<std::string> files = vpIoTools::getDirFiles(opt_data_path);
  for (unsigned int i = 0; i < files.size(); i++) {
    long index_w_P_ee = vpIoTools::getIndex(files[i], opt_w_P_ee_files);
    long index_c_P_o = vpIoTools::getIndex(files[i], opt_c_P_o_files);
    if (index_w_P_ee != -1) {
      map_w_P_ee_files[index_w_P_ee] = files[i];
    }
    if (index_c_P_o != -1) {
      map_c_P_o_files[index_c_P_o] = files[i];
    }
  }

  if (map_w_P_ee_files.size() == 0) {
    std::cout << "No " << opt_w_P_ee_files
      << " files found. Use --data-path <path> or --w_P_ee <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }
  if (map_c_P_o_files.size() == 0) {
    std::cout << "No " << opt_c_P_o_files
      << " files found. Use --data-path <path> or --c_P_o <generic name> to be able to read your data." << std::endl;
    std::cout << "Use --help option to see full usage..." << std::endl;
    return EXIT_FAILURE;
  }

  for (std::map<long, std::string>::const_iterator it_w_P_ee = map_w_P_ee_files.begin(); it_w_P_ee != map_w_P_ee_files.end();
    ++it_w_P_ee) {
    std::string file_w_P_ee = vpIoTools::createFilePath(opt_data_path, it_w_P_ee->second);
    std::map<long, std::string>::const_iterator it_c_P_o = map_c_P_o_files.find(it_w_P_ee->first);
    if (it_c_P_o != map_c_P_o_files.end()) {
      vpPoseVector w_P_ee;
      if (w_P_ee.loadYAML(file_w_P_ee, w_P_ee) == false) {
        std::cout << "Unable to read data from " << file_w_P_ee << ". Skip data" << std::endl;
        continue;
      }

      vpPoseVector c_P_o;
      std::string file_c_P_o = vpIoTools::createFilePath(opt_data_path, it_c_P_o->second);
      if (c_P_o.loadYAML(file_c_P_o, c_P_o) == false) {
        std::cout << "Unable to read data from " << file_c_P_o << ". Skip data" << std::endl;
        continue;
      }
      std::cout << "Use data from " << file_w_P_ee << " and from " << file_c_P_o << std::endl;
      w_M_ee.push_back(vpHomogeneousMatrix(w_P_ee));
      c_M_o.push_back(vpHomogeneousMatrix(c_P_o));
    }
  }

  if (w_M_ee.size() < 3) {
    std::cout << "Not enough data pairs found." << std::endl;
    return EXIT_FAILURE;
  }

  int ret = vpHandEyeCalibration::calibrate(c_M_o, w_M_ee, ee_M_c, w_M_o);

  if (ret == 0) {
    std::cout << std::endl << "Eye-in-hand calibration succeed" << std::endl;
    std::cout << std::endl << "Estimated ee_M_c transformation:" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    vpMatrix(ee_M_c).print(std::cout, 15, "ee_M_c");
    std::cout << "- Corresponding pose vector [tx ty tz tux tuy tuz] in [m] and [rad]: " << vpPoseVector(ee_M_c).t() << std::endl;

    vpThetaUVector erc(ee_M_c.getRotationMatrix());
    std::cout << std::endl << "- Translation [m]: " << ee_M_c[0][3] << " " << ee_M_c[1][3] << " " << ee_M_c[2][3] << std::endl;
    std::cout << "- Rotation (theta-u representation) [rad]: " << erc.t() << std::endl;
    std::cout << "- Rotation (theta-u representation) [deg]: " << vpMath::deg(erc[0]) << " " << vpMath::deg(erc[1])
      << " " << vpMath::deg(erc[2]) << std::endl;
    vpQuaternionVector quaternion(ee_M_c.getRotationMatrix());
    std::cout << "- Rotation (quaternion representation) [rad]: " << quaternion.t() << std::endl;
    vpRxyzVector rxyz(ee_M_c.getRotationMatrix());
    std::cout << "- Rotation (r-x-y-z representation) [rad]: " << rxyz.t() << std::endl;
    std::cout << "- Rotation (r-x-y-z representation) [deg]: " << vpMath::deg(rxyz).t() << std::endl;

    std::cout << std::endl << "Estimated w_M_o transformation:" << std::endl;
    std::cout << "-------------------------------" << std::endl;
    //std::cout << w_M_o << std::endl;
    vpMatrix(w_M_o).print(std::cout, 15, "w_M_o");
    std::cout << "- Corresponding pose vector [tx ty tz tux tuy tuz] in [m] and [rad]: " << vpPoseVector(w_M_o).t() << std::endl;

    vpThetaUVector wrc(w_M_o.getRotationMatrix());
    std::cout << std::endl << "- Translation [m]: " << w_M_o[0][3] << " " << w_M_o[1][3] << " " << w_M_o[2][3] << std::endl;
    std::cout << "- Rotation (theta-u representation) [rad]: " << wrc.t() << std::endl;
    std::cout << "- Rotation (theta-u representation) [deg]: " << vpMath::deg(wrc[0]) << " " << vpMath::deg(wrc[1])
      << " " << vpMath::deg(wrc[2]) << std::endl;
    vpQuaternionVector quaternion2(w_M_o.getRotationMatrix());
    std::cout << "- Rotation (quaternion representation) [rad]: " << quaternion2.t() << std::endl;
    vpRxyzVector rxyz2(w_M_o.getRotationMatrix());
    std::cout << "- Rotation (r-x-y-z representation) [rad]: " << rxyz2.t() << std::endl;
    std::cout << "- Rotation (r-x-y-z representation) [deg]: " << vpMath::deg(rxyz).t() << std::endl;

    {
      // save ee_M_c
      std::string name_we = vpIoTools::createFilePath(vpIoTools::getParent(opt_ee_P_c_file), vpIoTools::getNameWE(opt_ee_P_c_file)) + ".txt";
      std::cout << std::endl << "Save transformation matrix ee_M_c as an homogeneous matrix in: " << name_we << std::endl;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      std::ofstream file_ee_M_c(name_we);
#else
      std::ofstream file_ee_M_c(name_we.c_str());
#endif

      ee_M_c.save(file_ee_M_c);

      vpPoseVector pose_vec(ee_M_c);
      std::string output_filename = vpIoTools::createFilePath(vpIoTools::getParent(opt_ee_P_c_file), vpIoTools::getName(opt_ee_P_c_file));
      std::cout << "Save transformation matrix ee_M_c as a vpPoseVector in       : " << output_filename << std::endl;
      pose_vec.saveYAML(output_filename, pose_vec);
    }

    {
      // save w_M_o
      std::string name_we = vpIoTools::createFilePath(vpIoTools::getParent(opt_w_P_o_file), vpIoTools::getNameWE(opt_w_P_o_file)) + ".txt";
      std::cout << std::endl << "Save transformation matrix w_M_o as an homogeneous matrix in: " << name_we << std::endl;

#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      std::ofstream file_w_M_o(name_we);
#else
      std::ofstream file_w_M_o(name_we.c_str());
#endif

      w_M_o.save(file_w_M_o);

      vpPoseVector pose_vec(w_M_o);
      std::string output_filename = vpIoTools::createFilePath(vpIoTools::getParent(opt_w_P_o_file), vpIoTools::getName(opt_w_P_o_file));
      std::cout << "Save transformation matrix w_M_o as a vpPoseVector in       : " << output_filename << std::endl;
      pose_vec.saveYAML(output_filename, pose_vec, "Robot reference to object frames transformation (w_M_o)");
    }
  }
  else {
    std::cout << std::endl << "** Eye-in-hand calibration failed" << std::endl;
    std::cout << std::endl << "Check your input data and ensure they are covering the half sphere over the object." << std::endl;
    std::cout << std::endl << "See https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-calibration-extrinsic-eye-in-hand.html" << std::endl;
  }

  return EXIT_SUCCESS;
}
