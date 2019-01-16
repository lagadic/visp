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
 * Test AprilTag detection.
 *
 *****************************************************************************/
/*!
  \example testAprilTag.cpp

  \brief Test AprilTag detection.
*/

#include <iostream>
#include <map>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

#if defined(VISP_HAVE_APRILTAG)

// List of allowed command line options
#define GETOPTARGS "cdi:p:C:T:h"

namespace
{
/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
 */
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
  Test AprilTag detection.\n\
  \n\
  SYNOPSIS\n\
    %s [-c] [-d] [-i <input image path>] [-p <personal image path>] \
       [-C <tag color>] [-T <tag thickness>]\n\
       [-h]\n            \
  ", name);

  fprintf(stdout, "\n\
  OPTIONS:                                               Default\n\
    -i <input image path>                                %s\n\
       Set image input path.\n\
       From this path read \"AprilTag/AprilTag.pgm image.\n\
       Setting the VISP_INPUT_IMAGE_PATH environment\n\
       variable produces the same behaviour than using\n\
       this option.\n\
  \n\
    -p <personal image path>                               \n\
       Path to an image used to test image reading function.\n\
       Example: -p /my_path_to/image.png\n\
  \n\
    -c \n\
       Disable the mouse click. Useful to automate the \n\
       execution of this program without human intervention.\n\
  \n\
    -d \n\
       Turn off the display.\n\
  \n\
    -C <color (0, 1, ...)> \n\
       Color for tag detection display.\n\
  \n\
    -T <thickness> \n\
       Thickness for tag detection display.\n\
  \n\
    -h\n\
       Print the help.\n\n", ipath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param ppath : Personal image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \param color_id : Color id for tag detection display.
  \param thickness : Thickness for tag detection display.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath, bool &click_allowed, bool &display,
                int &color_id, unsigned int &thickness)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'p':
      ppath = optarg_;
      break;
    case 'h':
      usage(argv[0], NULL, ipath);
      return false;
      break;
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'C':
      color_id = atoi(optarg_);
      break;
    case 'T':
      thickness = (unsigned int) atoi(optarg_);
      break;

    default:
      usage(argv[0], optarg_, ipath);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

struct TagGroundTruth {
  std::string message;
  std::vector<vpImagePoint> corners;

  TagGroundTruth(const std::string &msg, const std::vector<vpImagePoint> &c) : message(msg), corners(c) {}

  bool operator==(const TagGroundTruth &b) const
  {
    if (message != b.message || corners.size() != b.corners.size())
      return false;

    for (size_t i = 0; i < corners.size(); i++) {
      // Allow 0.5 pixel of difference
      if (!vpMath::equal(corners[i].get_u(), b.corners[i].get_u(), 0.5) ||
          !vpMath::equal(corners[i].get_v(), b.corners[i].get_v(), 0.5)) {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const TagGroundTruth &b) const { return !(*this == b); }
};

std::ostream &operator<<(std::ostream &os, TagGroundTruth &t)
{
  os << t.message << std::endl;
  for (size_t i = 0; i < t.corners.size(); i++)
    os << t.corners[i] << std::endl;

  return os;
}
}

int main(int argc, const char *argv[])
{
  try {
    std::string env_ipath;
    std::string opt_ipath = "";
    std::string opt_ppath = "";
    std::string ipath = "";
    std::string filename = "";
    bool opt_click_allowed = true;
    bool opt_display = true;
    int opt_color_id = -1;
    unsigned int opt_thickness = 2;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_click_allowed, opt_display,
                   opt_color_id, opt_thickness) == false) {
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    //
    // Here starts really the test
    //

    vpImage<unsigned char> I;
    if (opt_ppath.empty()) {
      filename = vpIoTools::createFilePath(ipath, "AprilTag/AprilTag.pgm");
    } else {
      filename = opt_ppath;
    }

    if (!vpIoTools::checkFilename(filename)) {
      std::cerr << "Filename: " << filename << " does not exist." << std::endl;
      return EXIT_SUCCESS;
    }
    vpImageIo::read(I, filename);

#ifdef VISP_HAVE_X11
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#else
    opt_display = false;
#endif

    vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.053;
    float quad_decimate = 1.0;
    int nThreads = 1;
    bool display_tag = true;

    vpDetectorBase *detector = new vpDetectorAprilTag(tagFamily);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagQuadDecimate(quad_decimate);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagPoseEstimationMethod(poseEstimationMethod);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setAprilTagNbThreads(nThreads);
    dynamic_cast<vpDetectorAprilTag *>(detector)->setDisplayTag(display_tag,
                                                                opt_color_id < 0 ? vpColor::none : vpColor::getColor(opt_color_id),
                                                                opt_thickness);

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);

    if (opt_display) {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
      d.init(I, 0, 0, "AprilTag detection");
#endif
      vpDisplay::display(I);
    }

    std::vector<vpHomogeneousMatrix> cMo_vec;
    dynamic_cast<vpDetectorAprilTag *>(detector)->detect(I, tagSize, cam, cMo_vec);

    // Ground truth
    std::map<std::string, TagGroundTruth> mapOfTagsGroundTruth;
    bool use_detection_ground_truth = false;
    {
      std::string filename_ground_truth = vpIoTools::createFilePath(ipath, "AprilTag/ground_truth_detection.txt");
      std::ifstream file_ground_truth(filename_ground_truth.c_str());
      if (file_ground_truth.is_open() && opt_ppath.empty()) {
        use_detection_ground_truth = true;

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
    }

    std::map<std::string, vpPoseVector> mapOfPosesGroundTruth;
    bool use_pose_ground_truth = false;
    {
      std::string filename_ground_truth = vpIoTools::createFilePath(ipath, "AprilTag/ground_truth_pose.txt");
      std::ifstream file_ground_truth(filename_ground_truth.c_str());
      if (file_ground_truth.is_open() && opt_ppath.empty()) {
        use_pose_ground_truth = true;

        std::string message = "";
        double tx = 0.0, ty = 0.0, tz = 0.0;
        double tux = 0.0, tuy = 0.0, tuz = 0.0;
        while (file_ground_truth >> message >> tx >> ty >> tz >> tux >> tuy >> tuz) {
          mapOfPosesGroundTruth.insert(std::make_pair(message, vpPoseVector(tx, ty, tz, tux, tuy, tuz)));
        }
      }
    }

    std::cout << "use_pose_ground_truth: " << use_pose_ground_truth << std::endl;

    for (size_t i = 0; i < detector->getNbObjects(); i++) {
      std::vector<vpImagePoint> p = detector->getPolygon(i);

      if (use_detection_ground_truth) {
        std::string message = detector->getMessage(i);
        std::replace(message.begin(), message.end(), ' ', '_');
        std::map<std::string, TagGroundTruth>::iterator it = mapOfTagsGroundTruth.find(message);
        TagGroundTruth current(message, p);
        if (it == mapOfTagsGroundTruth.end()) {
          std::cerr << "Problem with tag decoding (tag_family or id): " << message << std::endl;
          return EXIT_FAILURE;
        } else if (it->second != current) {
          std::cerr << "Problem, current detection:\n" << current << "\nGround truth:\n" << it->second << std::endl;
          return EXIT_FAILURE;
        }
      }

      if (opt_display) {
        vpRect bbox = detector->getBBox(i);
        vpDisplay::displayRectangle(I, bbox, vpColor::green);
        vpDisplay::displayText(I, (int)(bbox.getTop() - 10), (int)bbox.getLeft(), detector->getMessage(i),
                               vpColor::red);
      }
    }

    if (opt_display) {
      vpDisplay::displayText(I, 20, 20, "Click to display tag poses", vpColor::red);
      vpDisplay::flush(I);
      if (opt_click_allowed)
        vpDisplay::getClick(I);

      vpDisplay::display(I);
    }

    for (size_t i = 0; i < cMo_vec.size(); i++) {
      if (opt_display)
        vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);

      if (use_pose_ground_truth) {
        vpPoseVector pose_vec(cMo_vec[i]);

        std::string message = detector->getMessage(i);
        std::replace(message.begin(), message.end(), ' ', '_');
        std::map<std::string, vpPoseVector>::iterator it = mapOfPosesGroundTruth.find(message);
        if (it == mapOfPosesGroundTruth.end()) {
          std::cerr << "Problem with tag decoding (tag_family or id): " << message << std::endl;
          return EXIT_FAILURE;
        } else {
          for (unsigned int cpt = 0; cpt < 6; cpt++) {
            if (!vpMath::equal(it->second[cpt], pose_vec[cpt], 0.005)) {
              std::cerr << "Problem, current pose: " << pose_vec.t() << "\nGround truth pose: " << it->second.t()
                        << std::endl;
              return EXIT_FAILURE;
            }
          }
        }
      }
    }

    if (opt_display) {
      vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I);
      if (opt_click_allowed)
        vpDisplay::getClick(I);
    }

    delete detector;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\ntestAprilTag is ok." << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "Need ViSP AprilTag." << std::endl;
  return 0;
}
#endif
