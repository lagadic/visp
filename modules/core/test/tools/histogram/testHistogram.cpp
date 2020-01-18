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
 * Test histogram computation.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <visp3/core/vpHistogram.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

/*!
  \example testHistogram.cpp

  \brief Test histogram computation.

*/

// List of allowed command line options
#define GETOPTARGS "cdi:t:h"

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.

 */
void usage(const char *name, const char *badparam, std::string ipath)
{
  fprintf(stdout, "\n\
Test histogram.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-t <nb threads>]\n\
     [-h]\n                 \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.ppm\"\n\
     image.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -t <nb threads>\n\
     Set the number of threads to use for the computation.\n\
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
  \param nbThreads: Number of threads to use.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, unsigned int &nbThreads)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 't':
      nbThreads = (unsigned int)atoi(optarg_);
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

/*!
  Compute the histogram of an image.

  \param I : Input color image.
  \param nbBins : Number of histogram bins.
  \param nbThreads : Number of computation threads.
*/
unsigned int histogramSum(const vpImage<unsigned char> &I, const unsigned int nbBins, const unsigned int nbThreads)
{
  unsigned int sum = 0;

  vpHistogram histogram;
  histogram.calculate(I, nbBins, nbThreads);

  for (unsigned int cpt = 0; cpt < histogram.getSize(); cpt++) {
    sum += histogram[cpt];
  }

  return sum;
}

/*!
  Compare two histograms.

  \param I : Input color image.
  \param nbBins : Number of histogram bins.
*/
bool compareHistogram(const vpImage<unsigned char> &I, const unsigned int nbBins)
{
  vpHistogram histogram_single_threaded;
  histogram_single_threaded.calculate(I, nbBins, 1);

  vpHistogram histogram_multi_threaded;
  histogram_multi_threaded.calculate(I, nbBins, 4);

  unsigned int sum = 0;
  for (unsigned int cpt = 0; cpt < nbBins; cpt++) {
    if (histogram_single_threaded[cpt] != histogram_multi_threaded[cpt]) {
      std::cerr << "histogram_single_threaded[" << cpt << "]=" << histogram_single_threaded[cpt]
                << " ; histogram_multi_threaded[" << cpt << "]=" << histogram_multi_threaded[cpt] << std::endl;

      return false;
    }

    sum += histogram_single_threaded[cpt];
  }

  if (sum != I.getSize()) {
    std::cerr << "Sum of histogram is different with the image size!" << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char **argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;
    unsigned int nbThreads = 4;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, nbThreads) == false) {
      exit(-1);
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

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(-1);
    }

    //
    // Here starts really the test
    //

    // Create a grey level image
    vpImage<unsigned char> I;

    // Load a grey image from the disk
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    std::cout << "Read image: " << filename << std::endl;
    vpImageIo::read(I, filename);

    std::cout << "I=" << I.getWidth() << "x" << I.getHeight() << std::endl;

    int nbIterations = 100;
    unsigned int nbBins = 256;
    unsigned int sum_single_thread = 0;
    unsigned int sum_single_multithread = 0;

    double t_single_thread = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      sum_single_thread = histogramSum(I, nbBins, 1);
    }
    t_single_thread = vpTime::measureTimeMs() - t_single_thread;

    double t_multithread = vpTime::measureTimeMs();
    for (int iteration = 0; iteration < nbIterations; iteration++) {
      sum_single_multithread = histogramSum(I, nbBins, nbThreads);
    }
    t_multithread = vpTime::measureTimeMs() - t_multithread;

    std::cout << "sum_single_thread=" << sum_single_thread << " ; t_single_thread=" << t_single_thread
              << " ms ; mean=" << t_single_thread / (double)nbIterations << " ms" << std::endl;
    std::cout << "sum_single_multithread=" << sum_single_multithread << " ; t_multithread=" << t_multithread
              << " ms ; mean=" << t_multithread / (double)nbIterations << " ms" << std::endl;
    std::cout << "Speed-up=" << t_single_thread / (double)t_multithread << "X" << std::endl;

    if (sum_single_thread != I.getSize() || sum_single_multithread != I.getSize()) {
      std::cerr << "Problem with histogram!" << std::endl;
      return -1;
    }

    nbBins = 101;
    if (!compareHistogram(I, nbBins)) {
      std::cerr << "Histogram are different!" << std::endl;
      return -1;
    }

    // Test histogram computation on empty image
    vpHistogram histogram;
    vpImage<unsigned char> I_test(0, 0);
    histogram.calculate(I_test, 256, 4);
    if (histogram.getSize() == 256) {
      for (unsigned int cpt = 0; cpt < 256; cpt++) {
        if (histogram[cpt] != 0) {
          std::cerr << "Problem with histogram computation: histogram[" << cpt << "]=" << histogram[cpt]
                    << " but should be zero!" << std::endl;
        }
      }
    } else {
      std::cerr << "Bad histogram size!" << std::endl;
      return -1;
    }

    // Test histogram computation on image size < nbThreads
    I_test.init(3, 1);
    I_test = 100;
    histogram.calculate(I_test, 256, 4);
    if (histogram.getSize() == 256) {
      for (unsigned int cpt = 0; cpt < 256; cpt++) {
        if (cpt == 100) {
          if (histogram[cpt] != I_test.getSize()) {
            std::cerr << "Problem with histogram computation: histogram[" << cpt << "]=" << histogram[cpt]
                      << " but should be: " << I_test.getSize() << std::endl;
            return -1;
          }
        } else {
          if (histogram[cpt] != 0) {
            std::cerr << "Problem with histogram computation: histogram[" << cpt << "]=" << histogram[cpt]
                      << " but should be zero!" << std::endl;
          }
        }
      }
    } else {
      std::cerr << "Bad histogram size!" << std::endl;
      return -1;
    }

    // Test histogram computation on small image size
    I_test.init(7, 1);
    I_test = 50;
    histogram.calculate(I_test, 256, 4);
    if (histogram.getSize() == 256) {
      for (unsigned int cpt = 0; cpt < 256; cpt++) {
        if (cpt == 50) {
          if (histogram[cpt] != I_test.getSize()) {
            std::cerr << "Problem with histogram computation: histogram[" << cpt << "]=" << histogram[cpt]
                      << " but should be: " << I_test.getSize() << std::endl;
            return -1;
          }
        } else {
          if (histogram[cpt] != 0) {
            std::cerr << "Problem with histogram computation: histogram[" << cpt << "]=" << histogram[cpt]
                      << " but should be zero!" << std::endl;
          }
        }
      }
    } else {
      std::cerr << "Bad histogram size!" << std::endl;
      return -1;
    }

    std::cout << "testHistogram is OK!" << std::endl;
    return 0;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return 1;
  }
}
