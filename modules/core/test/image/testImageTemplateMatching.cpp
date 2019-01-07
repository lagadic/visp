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
 * Test vpImageTools::templateMatching().
 *
 *****************************************************************************/
/*!
  \example testImageTemplateMatching.cpp

  \brief Test vpImageTools::templateMatching().
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>

#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x040000)
#  include <opencv2/imgproc.hpp>
#endif

// List of allowed command line options
#define GETOPTARGS "cdi:th"

namespace
{
  void usage(const char *name, const char *badparam, std::string ipath)
  {
    fprintf(stdout, "\n\
  Test vpImageTools::templateMatching().\n\
  \n\
  SYNOPSIS\n\
    %s [-i <VISP_IMAGES directory>]                           \n\
       [-c] [-t]                                              \n\
       [-h]\n            \
  ", name);

    fprintf(stdout, "\n\
  OPTIONS:                                               Default\n\
    -i <VISP_IMAGES directory>                                %s\n\
       Set VISP_IMAGES input path.\n\
       Setting the VISP_INPUT_IMAGE_PATH environment\n\
       variable produces the same behaviour than using\n\
       this option.\n\
  \n\
    -c \n\
       Mouse click.\n\
    -t \n\
       Perform template matching on cube sequence.\n\
    -h\n\
       Print the help.\n\n", ipath.c_str());

    if (badparam)
      fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
  }

  bool getOptions(int argc, const char **argv, std::string &ipath, bool &click,
                  bool &doTemplateMatching)
  {
    const char *optarg_;
    int c;
    while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

      switch (c) {
      case 'i':
        ipath = optarg_;
        break;
      case 'h':
        usage(argv[0], NULL, ipath);
        return false;
        break;
      case 't':
        doTemplateMatching = true;
        break;

      case 'c':
          click = true;
          break;
      case 'd':
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
}

int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  {
    const int h = 5, w = 5;
    vpImage<unsigned char> I(h, w);
    I[0][0] = 1;  I[0][1] = 2;  I[0][2] = 2;  I[0][3] = 4;  I[0][4] = 1;
    I[1][0] = 3;  I[1][1] = 4;  I[1][2] = 1;  I[1][3] = 5;  I[1][4] = 2;
    I[2][0] = 2;  I[2][1] = 3;  I[2][2] = 3;  I[2][3] = 2;  I[2][4] = 4;
    I[3][0] = 4;  I[3][1] = 1;  I[3][2] = 5;  I[3][3] = 4;  I[3][4] = 6;
    I[4][0] = 6;  I[4][1] = 3;  I[4][2] = 2;  I[4][3] = 1;  I[4][4] = 3;

    vpImage<double> II, IIsq;
    vpImageTools::integralImage(I, II, IIsq);
    std::cout << "I:\n" << I << std::endl;
    std::cout << "II:\n" << II << std::endl;
    std::cout << "IIsq:\n" << IIsq << std::endl;

    cv::Mat mat(h, w, CV_64F);
    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
        mat.at<double>(i,j) = I[i][j];
      }
    }

    cv::Mat sum, sqsum;
    cv::integral(mat, sum, sqsum);
    std::cout << "mat:\n" << mat << std::endl;
    std::cout << "sum:\n" << sum << std::endl;
    std::cout << "sqsum:\n" << sqsum << std::endl;

    for (int i = 0; i < h; i++) {
      for (int j = 0; j < w; j++) {
        if ( !vpMath::equal(II[i][j], sum.at<double>(i,j), std::numeric_limits<double>::epsilon()) ) {
          std::cerr << "Error vpImageTools::integralImage(II), reference: " << std::setprecision(17)
                    << sum.at<double>(i,j) << " ; compute: " << II[i][j] << std::endl;
          return EXIT_FAILURE;
        }

        if ( !vpMath::equal(IIsq[i][j], sqsum.at<double>(i,j), std::numeric_limits<double>::epsilon()) ) {
          std::cerr << "Error vpImageTools::integralImage(IIsq), reference: " << std::setprecision(17)
                    << sqsum.at<double>(i,j) << " ; compute: " << IIsq[i][j] << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
  }
#endif

  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;
    bool click = false;
    bool doTemplateMatching = false;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty()) {
      ipath = env_ipath;
    }

    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath, click, doTemplateMatching)) {
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
    }

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

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(EXIT_FAILURE);
    }

    //
    // Here starts really the test
    //

    // Load cube sequence
    filename = vpIoTools::createFilePath(ipath, "mbt/cube/image%04d.pgm");

    vpVideoReader reader;
    reader.setFileName(filename);
    vpImage<unsigned char> I, I_template;
    reader.open(I);
    vpRect template_roi( vpImagePoint(201, 310), vpImagePoint(201+152-1, 310+138-1) );
    vpImageTools::crop(I, template_roi, I_template);

    if (doTemplateMatching) {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)

#if defined(VISP_HAVE_X11)
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#endif

    d.init(I, 0, 0, "Image");

    vpImage<double> I_score;
    std::vector<double> benchmark_vec;
    bool quit = false;
    while (!reader.end() && !quit) {
      reader.acquire(I);

      vpDisplay::display(I);

      std::stringstream ss;
      ss << "Frame: " << reader.getFrameIndex();
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);

      //Basic template matching
      double t_proc = vpTime::measureTimeMs();
      const unsigned int step_u = 5, step_v = 5;
      vpImageTools::templateMatching(I, I_template, I_score, step_u, step_v);

      vpImagePoint max_loc;
      double max_correlation = -1.0;
      I_score.getMinMaxLoc(NULL, &max_loc, NULL, &max_correlation);
      t_proc = vpTime::measureTimeMs() - t_proc;
      benchmark_vec.push_back(t_proc);

      ss.str("");
      ss << "Template matching: " << t_proc << " ms";
      vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);

      ss.str("");
      ss << "Max correlation: " << max_correlation;
      vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::red);

      vpDisplay::displayRectangle(I, max_loc, I_template.getWidth(), I_template.getHeight(), vpColor::red, false, 1);

      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, click)) {
        switch (button) {
          case vpMouseButton::button1:
            quit = !click;
            break;

          case vpMouseButton::button3:
            click = !click;
            break;

          default:
            break;
        }
      }
    }

    if (!benchmark_vec.empty()) {
      std::cout << "Processing time, Mean: " << vpMath::getMean(benchmark_vec) << " ms ; Median: "
                << vpMath::getMedian(benchmark_vec) << " ms ; Std: "
                << vpMath::getStdev(benchmark_vec) << " ms" << std::endl;
    }
#endif
    } else {
      //ctest case
      //Basic template matching
      const unsigned int step_u = 5, step_v = 5;
      vpImage<double> I_score, I_score_gold;

      double t = vpTime::measureTimeMs();
      vpImageTools::templateMatching(I, I_template, I_score, step_u, step_v, true);
      t = vpTime::measureTimeMs() - t;

      double t_gold = vpTime::measureTimeMs();
      vpImageTools::templateMatching(I, I_template, I_score_gold, step_u, step_v, false);
      t_gold = vpTime::measureTimeMs() - t_gold;

      std::cout << "Template matching: " << t << " ms" << std::endl;
      std::cout << "Template matching (gold): " << t_gold << " ms" << std::endl;

      for (unsigned int i = 0; i < I_score.getHeight(); i++) {
        for (unsigned int j = 0; j < I_score.getWidth(); j++) {
          if ( !vpMath::equal(I_score[i][j], I_score_gold[i][j], 1e-9) ) {
            std::cerr << "Issue with template matching, gold: " << std::setprecision(17) << I_score_gold[i][j]
                      << " ; compute: " << I_score[i][j] << std::endl;
            return EXIT_FAILURE;
          }
        }
      }
    }

  } catch (const vpException &e) {
    std::cerr << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
