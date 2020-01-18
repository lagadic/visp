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
 * Test vpImageTools::normalizedCorrelation().
 *
 *****************************************************************************/
/*!
  \example testImageNormalizedCorrelation.cpp

  \brief Test vpImageTools::normalizedCorrelation().
*/

#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>

// List of allowed command line options
#define GETOPTARGS "cdi:h"

namespace
{
  void usage(const char *name, const char *badparam, std::string ipath)
  {
    fprintf(stdout, "\n\
  Test vpImageTools::normalizedCorrelation().\n\
  \n\
  SYNOPSIS\n\
    %s [-i <VISP_IMAGES directory>]                           \n\
       [-h]\n            \
  ", name);

    fprintf(stdout, "\n\
  OPTIONS:                                               Default\n\
    -i <VISP_IMAGES directory>                                %s\n\
       Set VISP_IMAGES input path.\n\
       Setting the VISP_INPUT_IMAGE_PATH environment\n\
       variable produces the same behaviour than using\n\
       this option.\n\
    -h\n\
       Print the help.\n\n", ipath.c_str());

    if (badparam)
      fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
  }

  bool getOptions(int argc, const char **argv, std::string &ipath)
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

      case 'c':
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

  void templateMatching(const vpImage<unsigned char> &I, const vpImage<unsigned char> &I_tpl,
                        vpImage<double> &I_score, const unsigned int step_u,
                        const unsigned int step_v, const bool useOptimized)
  {
    unsigned int height_tpl = I_tpl.getHeight(), width_tpl = I_tpl.getWidth();
    vpImage<double> I_double, I_tpl_double, I_cur;
    vpImageConvert::convert(I, I_double);
    vpImageConvert::convert(I_tpl, I_tpl_double);
    I_score.resize(I.getHeight() - height_tpl, I.getWidth() - width_tpl, 0.0);

    for (unsigned int i = 0; i < I.getHeight()-height_tpl; i += step_v) {
      for (unsigned int j = 0; j < I.getWidth()-width_tpl; j += step_u) {
        vpRect roi( vpImagePoint(i, j), vpImagePoint(i+height_tpl-1, j+width_tpl-1) );
        vpImageTools::crop(I_double, roi, I_cur);

        I_score[i][j] = vpImageTools::normalizedCorrelation(I_cur, I_tpl_double, useOptimized);
      }
    }
  }
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty()) {
      ipath = env_ipath;
    }

    // Read the command line options
    if (!getOptions(argc, argv, opt_ipath)) {
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

    vpImage<double> I_score, I_score_gold;
    const unsigned int step_i = 5, step_j = 5;
    double t = vpTime::measureTimeMs();
    templateMatching(I, I_template, I_score, step_i, step_j, true);
    t = vpTime::measureTimeMs() - t;

    double t_gold = vpTime::measureTimeMs();
    templateMatching(I, I_template, I_score_gold, step_i, step_j, false);
    t_gold = vpTime::measureTimeMs() - t_gold;

    for (unsigned int  i = 0; i < I_score.getHeight(); i++) {
      for (unsigned int  j = 0; j < I_score.getWidth(); j++) {
        if ( !vpMath::equal(I_score[i][j], I_score_gold[i][j], 1e-9) ) {
          std::cerr << "Issue with normalizedCorrelation, gold: " << std::setprecision(17)
                    << I_score_gold[i][j] << " ; compute: " << I_score[i][j] << std::endl;
          return EXIT_FAILURE;
        }
      }
    }

    vpImagePoint max_loc, max_loc_gold;
    double max_correlation = -1.0, max_correlation_gold = -1.0;
    I_score.getMinMaxLoc(NULL, &max_loc, NULL, &max_correlation);
    I_score_gold.getMinMaxLoc(NULL, &max_loc_gold, NULL, &max_correlation_gold);

    std::cout << "Compare regular and SSE version of vpImageTools::normalizedCorrelation()" << std::endl;
    std::cout << "vpImageTools::normalizedCorrelation(): " << max_correlation << " ; " << t << " ms" << std::endl;
    std::cout << "Gold normalizedCorrelation(): " << max_correlation_gold << " ; " << t_gold << " ms" << std::endl;

    std::cerr << "\nTrue template position: " << template_roi.getTopLeft() << std::endl;
    std::cerr << "Found template position: " << max_loc << std::endl;
    if ( vpImagePoint::distance(max_loc, template_roi.getTopLeft()) > step_i ) {
      std::cerr << "Issue with vpImageTools::normalizedCorrelation:" << std::endl;
      return EXIT_FAILURE;
    }

  } catch (const vpException &e) {
    std::cerr << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
