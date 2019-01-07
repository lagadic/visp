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
 * Test for vpImageMorphology functions.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/
/*!
  \example testImageMorphology.cpp

  \brief Test vpImageMorphology functions.
*/

#include <visp3/core/vpImageMorphology.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/io/vpParseArgv.h>

// List of allowed command line options
#define GETOPTARGS "cdi:n:h"

/*
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param nbiter : Iteration number.
 */
void usage(const char *name, const char *badparam, std::string ipath, int nbiter)
{
  fprintf(stdout, "\n\
Test vpImageMorphology functions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-n <nb benchmark iterations>]\n\
     [-h]\n            \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"Klimt/Klimt.pgm\"\n\
     and \"Klimt/Klimt.ppm\" images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
  -n <nb benchmark iterations>                               %d\n\
     Set the number of benchmark iterations.\n\
\n\
  -h\n\
     Print the help.\n\n", ipath.c_str(), nbiter);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param nbIterations : Number of benchmark iterations.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, std::string &ipath, int &nbIterations)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'i':
      ipath = optarg_;
      break;
    case 'n':
      nbIterations = atoi(optarg_);
      break;
    case 'h':
      usage(argv[0], NULL, ipath, nbIterations);
      return false;
      break;

    case 'c':
    case 'd':
      break;

    default:
      usage(argv[0], optarg_, ipath, nbIterations);
      return false;
      break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, nbIterations);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

void printMatrix(const vpImage<unsigned char> &I, const std::string &name)
{
  std::cout << "\n" << name << ":" << std::endl;
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      std::cout << static_cast<unsigned>(I[i][j]) << " ";
    }
    std::cout << std::endl;
  }
}

// Erosion in the general case on grayscale images
void generalErosion(vpImage<unsigned char> &I,
                    vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 255;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    } else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    unsigned int offset[5] = {1, J.getWidth(), J.getWidth() + 1, J.getWidth() + 2, J.getWidth() * 2 + 1};

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char min_value = null_value;
        for (int k = 0; k < 5; k++) {
          min_value = (std::min)(min_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = min_value;
      }
    }
  } else {
    // CONNEXITY_8
    unsigned int offset[9] = {0,
                              1,
                              2,
                              J.getWidth(),
                              J.getWidth() + 1,
                              J.getWidth() + 2,
                              J.getWidth() * 2,
                              J.getWidth() * 2 + 1,
                              J.getWidth() * 2 + 2};

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char min_value = null_value;
        for (int k = 0; k < 9; k++) {
          min_value = (std::min)(min_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = min_value;
      }
    }
  }
}

// Dilatation in the general case on grayscale images
void generalDilatation(vpImage<unsigned char> &I,
                       vpImageMorphology::vpConnexityType connexity = vpImageMorphology::CONNEXITY_4)
{
  if (I.getSize() == 0) {
    std::cerr << "Input image is empty!" << std::endl;
    return;
  }

  const unsigned char null_value = 0;

  vpImage<unsigned char> J(I.getHeight() + 2, I.getWidth() + 2);
  // Copy I to J and add border
  for (unsigned int i = 0; i < J.getHeight(); i++) {
    if (i == 0 || i == J.getHeight() - 1) {
      for (unsigned int j = 0; j < J.getWidth(); j++) {
        J[i][j] = null_value;
      }
    } else {
      J[i][0] = null_value;
      memcpy(J[i] + 1, I[i - 1], sizeof(unsigned char) * I.getWidth());
      J[i][J.getWidth() - 1] = null_value;
    }
  }

  if (connexity == vpImageMorphology::CONNEXITY_4) {
    unsigned int offset[5] = {1, J.getWidth(), J.getWidth() + 1, J.getWidth() + 2, J.getWidth() * 2 + 1};

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char max_value = null_value;
        for (int k = 0; k < 5; k++) {
          max_value = (std::max)(max_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = max_value;
      }
    }
  } else {
    // CONNEXITY_8
    unsigned int offset[9] = {0,
                              1,
                              2,
                              J.getWidth(),
                              J.getWidth() + 1,
                              J.getWidth() + 2,
                              J.getWidth() * 2,
                              J.getWidth() * 2 + 1,
                              J.getWidth() * 2 + 2};

    for (unsigned int i = 0; i < I.getHeight(); i++) {
      unsigned char *ptr_curr_J = J.bitmap + i * J.getWidth();
      unsigned char *ptr_curr_I = I.bitmap + i * I.getWidth();

      for (unsigned int j = 0; j < I.getWidth(); j++) {
        unsigned char max_value = null_value;
        for (int k = 0; k < 9; k++) {
          max_value = (std::max)(max_value, *(ptr_curr_J + j + offset[k]));
        }

        *(ptr_curr_I + j) = max_value;
      }
    }
  }
}

// Generate a magic square matrix to get a consistent grayscale image
void magicSquare(vpImage<unsigned char> &magic_square, const int N)
{
  magic_square.resize((unsigned int)N, (unsigned int)N, 0);

  int n = 1;
  int i = 0, j = N / 2;

  while (n <= N * N) {
    magic_square[i][j] = vpMath::saturate<unsigned char>(n);
    n++;

    int newi = vpMath::modulo((i - 1), N), newj = vpMath::modulo((j + 1), N);

    if (magic_square[newi][newj]) {
      i++;
    } else {
      i = newi;
      j = newj;
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
    int nbIterations = 100;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
    // environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (!env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, nbIterations) == false) {
      exit(EXIT_FAILURE);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (opt_ipath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty()) {
      usage(argv[0], NULL, ipath, nbIterations);
      std::cerr << std::endl << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH " << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  image path where test images are located." << std::endl
                << std::endl;
      exit(EXIT_FAILURE);
    }

    // Create a binary image
    unsigned char image_data[8 * 16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0,
                                        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1,
                                        1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0,
                                        0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1,
                                        0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};

    vpImage<unsigned char> I(image_data, 8, 16, true);
    printMatrix(I, "I");

    // Regular code
    // Dilatation
    vpImage<unsigned char> I_dilatation1 = I;
    vpImage<unsigned char> I_dilatation2 = I;

    vpImageMorphology::dilatation(I_dilatation1, (unsigned char)1, (unsigned char)0, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::dilatation(I_dilatation2, (unsigned char)1, (unsigned char)0, vpImageMorphology::CONNEXITY_8);

    printMatrix(I_dilatation1, "I_dilatation1");
    printMatrix(I_dilatation2, "I_dilatation2");

    // Erosion
    vpImage<unsigned char> I_erosion1 = I_dilatation1;
    vpImage<unsigned char> I_erosion2 = I_dilatation2;

    vpImageMorphology::erosion(I_erosion1, (unsigned char)1, (unsigned char)0, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::erosion(I_erosion2, (unsigned char)1, (unsigned char)0, vpImageMorphology::CONNEXITY_8);

    printMatrix(I_erosion1, "I_erosion1");
    printMatrix(I_erosion2, "I_erosion2");

    // SSE code
    // Dilatation
    vpImage<unsigned char> I_dilatation1_sse = I;
    vpImage<unsigned char> I_dilatation2_sse = I;

    vpImageMorphology::dilatation(I_dilatation1_sse, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::dilatation(I_dilatation2_sse, vpImageMorphology::CONNEXITY_8);

    printMatrix(I_dilatation1_sse, "I_dilatation1_sse");
    printMatrix(I_dilatation2_sse, "I_dilatation2_sse");

    std::cout << "\n(I_dilatation1 == I_dilatation1_sse)? " << (I_dilatation1 == I_dilatation1_sse) << std::endl;
    std::cout << "(I_dilatation2 == I_dilatation2_sse)? " << (I_dilatation2 == I_dilatation2_sse) << std::endl;

    if ((I_dilatation1 != I_dilatation1_sse)) {
      throw vpException(vpException::fatalError, "(I_dilatation1 != I_dilatation1_sse)");
    }
    if ((I_dilatation2 != I_dilatation2_sse)) {
      throw vpException(vpException::fatalError, "(I_dilatation2 != I_dilatation2_sse)");
    }

    // Erosion
    vpImage<unsigned char> I_erosion1_sse = I_dilatation1_sse;
    vpImage<unsigned char> I_erosion2_sse = I_dilatation2_sse;

    vpImageMorphology::erosion(I_erosion1_sse, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::erosion(I_erosion2_sse, vpImageMorphology::CONNEXITY_8);

    printMatrix(I_erosion1_sse, "I_erosion1_sse");
    printMatrix(I_erosion2_sse, "I_erosion2_sse");

    std::cout << "\n(I_erosion1 == I_erosion1_sse)? " << (I_erosion1 == I_erosion1_sse) << std::endl;
    std::cout << "(I_erosion2 == I_erosion2_sse)? " << (I_erosion2 == I_erosion2_sse) << std::endl;

    if ((I_erosion1 != I_erosion1_sse)) {
      throw vpException(vpException::fatalError, "(I_erosion1 != I_erosion1_sse)");
    }
    if ((I_erosion2 != I_erosion2_sse)) {
      throw vpException(vpException::fatalError, "(I_erosion2 != I_erosion2_sse)");
    }

    // Check results against Matlab
    unsigned char image_data_dilated1[8 * 16] = {
        0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0,
        0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1,
        1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1,
        1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1};

    vpImage<unsigned char> I_check_dilated1(image_data_dilated1, 8, 16, true);

    if (I_check_dilated1 != I_dilatation1_sse) {
      throw vpException(vpException::fatalError, "(I_check_dilated1 != I_dilatation1_sse)");
    }

    unsigned char image_data_dilated2[8 * 16] = {
        0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1,
        0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1,
        1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1};

    vpImage<unsigned char> I_check_dilated2(image_data_dilated2, 8, 16, true);

    if (I_check_dilated2 != I_dilatation2_sse) {
      throw vpException(vpException::fatalError, "(I_check_dilated2 != I_dilatation2_sse)");
    }

    unsigned char image_data_eroded1[8 * 16] = {
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};

    vpImage<unsigned char> I_check_eroded1(image_data_eroded1, 8, 16, true);

    if (I_check_eroded1 != I_erosion1_sse) {
      throw vpException(vpException::fatalError, "(I_check_eroded1 != I_erosion1_sse)");
    }

    unsigned char image_data_eroded2[8 * 16] = {
        0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1,
        1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1};

    vpImage<unsigned char> I_check_eroded2(image_data_eroded2, 8, 16, true);

    if (I_check_eroded2 != I_erosion2_sse) {
      throw vpException(vpException::fatalError, "(I_check_eroded2 != I_erosion2_sse)");
    }

    // Morphology on grayscale images
    vpImage<unsigned char> I_magic_square;
    magicSquare(I_magic_square, 17);
    printMatrix(I_magic_square, "I_magic_square");

    // Dilatation CONNEXITY_4 grayscale
    vpImage<unsigned char> I_magic_square_dilatation1 = I_magic_square;
    vpImage<unsigned char> I_magic_square_dilatation1_sse = I_magic_square;

    generalDilatation(I_magic_square_dilatation1, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::dilatation(I_magic_square_dilatation1_sse, vpImageMorphology::CONNEXITY_4);

    if ((I_magic_square_dilatation1 != I_magic_square_dilatation1_sse)) {
      throw vpException(vpException::fatalError, "(I_magic_square_dilatation1 != I_magic_square_dilatation1_sse)");
    }

    // Dilatation CONNEXITY_8 grayscale
    vpImage<unsigned char> I_magic_square_dilatation2 = I_magic_square;
    vpImage<unsigned char> I_magic_square_dilatation2_sse = I_magic_square;

    generalDilatation(I_magic_square_dilatation2, vpImageMorphology::CONNEXITY_8);
    vpImageMorphology::dilatation(I_magic_square_dilatation2_sse, vpImageMorphology::CONNEXITY_8);

    if ((I_magic_square_dilatation2 != I_magic_square_dilatation2_sse)) {
      throw vpException(vpException::fatalError, "(I_magic_square_dilatation2 != I_magic_square_dilatation2_sse)");
    }

    // Erosion CONNEXITY_4 grayscale
    vpImage<unsigned char> I_magic_square_erosion1 = I_magic_square_dilatation1;
    vpImage<unsigned char> I_magic_square_erosion1_sse = I_magic_square_dilatation1_sse;

    generalErosion(I_magic_square_erosion1, vpImageMorphology::CONNEXITY_4);
    vpImageMorphology::erosion(I_magic_square_erosion1_sse, vpImageMorphology::CONNEXITY_4);

    if ((I_magic_square_erosion1 != I_magic_square_erosion1_sse)) {
      throw vpException(vpException::fatalError, "(I_magic_square_erosion1 != I_magic_square_erosion1_sse)");
    }

    // Dilatation CONNEXITY_8 grayscale
    vpImage<unsigned char> I_magic_square_erosion2 = I_magic_square_dilatation2;
    vpImage<unsigned char> I_magic_square_erosion2_sse = I_magic_square_dilatation2_sse;

    generalErosion(I_magic_square_erosion2, vpImageMorphology::CONNEXITY_8);
    vpImageMorphology::erosion(I_magic_square_erosion2_sse, vpImageMorphology::CONNEXITY_8);

    if ((I_magic_square_erosion2 != I_magic_square_erosion2_sse)) {
      throw vpException(vpException::fatalError, "(I_magic_square_erosion2 != I_magic_square_erosion2_sse)");
    }

    // Check results against Matlab (grayscale)
    unsigned char image_data2_dilated1[17 * 17] = {
        174, 193, 212, 231, 250, 255, 255, 255, 255, 39,  58,  77,  96,  115, 134, 153, 154, 192, 211, 230, 249,
        255, 255, 255, 255, 38,  57,  76,  95,  114, 133, 152, 170, 172, 210, 229, 248, 255, 255, 255, 255, 37,
        56,  75,  94,  113, 132, 151, 170, 172, 190, 228, 247, 255, 255, 255, 255, 36,  55,  74,  93,  112, 131,
        150, 169, 187, 190, 208, 246, 255, 255, 255, 255, 51,  54,  73,  92,  111, 130, 149, 168, 187, 189, 208,
        226, 255, 255, 255, 255, 51,  53,  72,  91,  110, 129, 148, 167, 186, 204, 207, 226, 244, 255, 255, 255,
        50,  68,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 255, 255, 49,  68,  70,  89,  108,
        127, 146, 165, 184, 203, 221, 224, 243, 255, 255, 255, 48,  67,  85,  88,  107, 126, 145, 164, 183, 202,
        221, 223, 242, 255, 255, 255, 47,  66,  85,  87,  106, 125, 144, 163, 182, 201, 220, 238, 241, 255, 255,
        255, 255, 65,  84,  102, 105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 255, 255, 45,  83,  102,
        104, 123, 142, 161, 180, 199, 218, 237, 255, 255, 255, 255, 255, 45,  63,  101, 119, 122, 141, 160, 179,
        198, 217, 236, 255, 255, 255, 255, 255, 44,  63,  81,  119, 121, 140, 159, 178, 197, 216, 235, 254, 255,
        255, 255, 255, 43,  62,  81,  99,  136, 139, 158, 177, 196, 215, 234, 253, 255, 255, 255, 255, 42,  61,
        80,  99,  117, 138, 157, 176, 195, 214, 233, 252, 255, 255, 255, 255, 41,  60,  79,  98,  117, 135, 156,
        175, 194, 213, 232, 251, 255, 255, 255, 255, 40,  59,  78,  97,  116, 135, 135};

    vpImage<unsigned char> I2_check_dilated1(image_data2_dilated1, 17, 17, true);

    if (I2_check_dilated1 != I_magic_square_dilatation1_sse) {
      throw vpException(vpException::fatalError, "(I2_check_dilated1 != I_magic_square_dilatation1_sse)");
    }

    unsigned char image_data2_dilated2[17 * 17] = {
        192, 211, 230, 249, 255, 255, 255, 255, 255, 57,  76,  95,  114, 133, 152, 154, 154, 210, 229, 248, 255,
        255, 255, 255, 255, 255, 75,  94,  113, 132, 151, 170, 172, 172, 228, 247, 255, 255, 255, 255, 255, 255,
        74,  93,  112, 131, 150, 169, 171, 190, 190, 246, 255, 255, 255, 255, 255, 255, 73,  92,  111, 130, 149,
        168, 187, 189, 208, 208, 255, 255, 255, 255, 255, 255, 72,  91,  110, 129, 148, 167, 186, 188, 207, 226,
        226, 255, 255, 255, 255, 255, 71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 244, 255, 255, 255,
        255, 70,  89,  108, 127, 146, 165, 184, 203, 205, 224, 243, 255, 255, 255, 255, 255, 69,  88,  107, 126,
        145, 164, 183, 202, 221, 223, 242, 255, 255, 255, 255, 255, 85,  87,  106, 125, 144, 163, 182, 201, 220,
        222, 241, 255, 255, 255, 255, 65,  84,  86,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 255,
        255, 255, 83,  102, 104, 123, 142, 161, 180, 199, 218, 237, 239, 255, 255, 255, 255, 255, 255, 101, 103,
        122, 141, 160, 179, 198, 217, 236, 255, 255, 255, 255, 255, 255, 255, 63,  119, 121, 140, 159, 178, 197,
        216, 235, 254, 255, 255, 255, 255, 255, 255, 81,  81,  120, 139, 158, 177, 196, 215, 234, 253, 255, 255,
        255, 255, 255, 255, 80,  99,  99,  138, 157, 176, 195, 214, 233, 252, 255, 255, 255, 255, 255, 255, 79,
        98,  117, 117, 156, 175, 194, 213, 232, 251, 255, 255, 255, 255, 255, 255, 78,  97,  116, 135, 135, 156,
        175, 194, 213, 232, 251, 255, 255, 255, 255, 255, 59,  78,  97,  116, 135, 135};

    vpImage<unsigned char> I2_check_dilated2(image_data2_dilated2, 17, 17, true);

    if (I2_check_dilated2 != I_magic_square_dilatation2_sse) {
      throw vpException(vpException::fatalError, "(I2_check_dilated2 != I_magic_square_dilatation2_sse)");
    }

    unsigned char image_data2_eroded1[17 * 17] = {
        174, 174, 193, 212, 231, 250, 255, 255, 38,  39,  39,  58,  77,  96,  115, 134, 153, 174, 192, 211, 230,
        249, 255, 255, 37,  38,  38,  57,  76,  95,  114, 133, 152, 154, 192, 210, 229, 248, 255, 255, 36,  37,
        37,  56,  75,  94,  113, 132, 151, 170, 172, 210, 228, 247, 255, 255, 36,  36,  36,  55,  74,  93,  112,
        131, 150, 169, 172, 190, 228, 246, 255, 255, 51,  51,  36,  54,  73,  92,  111, 130, 149, 168, 187, 189,
        208, 246, 255, 255, 50,  51,  51,  53,  72,  91,  110, 129, 148, 167, 186, 189, 207, 226, 255, 255, 49,
        50,  50,  53,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 48,  49,  49,  68,  70,  89,
        108, 127, 146, 165, 184, 203, 206, 224, 243, 255, 47,  48,  48,  67,  70,  88,  107, 126, 145, 164, 183,
        202, 221, 223, 242, 255, 255, 47,  47,  66,  85,  87,  106, 125, 144, 163, 182, 201, 220, 223, 241, 255,
        255, 45,  47,  65,  84,  87,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 45,  45,  65,  83,
        102, 104, 123, 142, 161, 180, 199, 218, 237, 240, 255, 255, 44,  45,  45,  83,  101, 104, 122, 141, 160,
        179, 198, 217, 236, 255, 255, 255, 43,  44,  44,  63,  101, 119, 121, 140, 159, 178, 197, 216, 235, 254,
        255, 255, 42,  43,  43,  62,  81,  119, 121, 139, 158, 177, 196, 215, 234, 253, 255, 255, 41,  42,  42,
        61,  80,  99,  136, 138, 157, 176, 195, 214, 233, 252, 255, 255, 40,  41,  41,  60,  79,  98,  117, 138,
        156, 175, 194, 213, 232, 251, 255, 255, 40,  40,  40,  59,  78,  97,  116, 135};

    vpImage<unsigned char> I2_check_eroded1(image_data2_eroded1, 17, 17, true);

    if (I2_check_eroded1 != I_magic_square_erosion1_sse) {
      printMatrix(I_magic_square_erosion1_sse, "I_magic_square_erosion1_sse");

      throw vpException(vpException::fatalError, "(I2_check_eroded1 != I_magic_square_erosion1_sse)");
    }

    unsigned char image_data2_eroded2[17 * 17] = {
        192, 192, 211, 230, 249, 255, 255, 255, 57,  57,  57,  76,  95,  114, 133, 152, 154, 192, 192, 211, 230,
        249, 255, 255, 74,  57,  57,  57,  76,  95,  114, 133, 152, 154, 210, 210, 229, 248, 255, 255, 73,  73,
        73,  74,  75,  94,  113, 132, 151, 170, 172, 228, 228, 247, 255, 255, 72,  72,  72,  73,  74,  93,  112,
        131, 150, 169, 171, 190, 246, 246, 255, 255, 71,  71,  71,  72,  73,  92,  111, 130, 149, 168, 187, 189,
        208, 255, 255, 255, 70,  70,  70,  71,  72,  91,  110, 129, 148, 167, 186, 188, 207, 226, 255, 255, 69,
        69,  69,  70,  71,  90,  109, 128, 147, 166, 185, 204, 206, 225, 244, 255, 85,  69,  69,  69,  70,  89,
        108, 127, 146, 165, 184, 203, 205, 224, 243, 255, 65,  65,  69,  69,  69,  88,  107, 126, 145, 164, 183,
        202, 221, 223, 242, 255, 255, 65,  65,  84,  85,  87,  106, 125, 144, 163, 182, 201, 220, 222, 241, 255,
        255, 255, 65,  65,  84,  86,  105, 124, 143, 162, 181, 200, 219, 238, 240, 255, 255, 63,  63,  83,  83,
        102, 104, 123, 142, 161, 180, 199, 218, 237, 239, 255, 255, 81,  63,  63,  101, 101, 103, 122, 141, 160,
        179, 198, 217, 236, 255, 255, 255, 80,  80,  63,  63,  119, 119, 121, 140, 159, 178, 197, 216, 235, 254,
        255, 255, 79,  79,  79,  80,  81,  120, 120, 139, 158, 177, 196, 215, 234, 253, 255, 255, 78,  78,  78,
        79,  80,  99,  138, 138, 157, 176, 195, 214, 233, 252, 255, 255, 59,  59,  59,  78,  79,  98,  117, 156,
        156, 175, 194, 213, 232, 251, 255, 255, 255, 59,  59,  59,  78,  97,  116, 135};

    vpImage<unsigned char> I2_check_eroded2(image_data2_eroded2, 17, 17, true);

    if (I2_check_eroded2 != I_magic_square_erosion2_sse) {
      throw vpException(vpException::fatalError, "(I2_check_eroded2 != I_magic_square_erosion2_sse)");
    }

    std::cout << std::endl;
    vpImage<unsigned char> I_Klimt;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImageIo::read(I_Klimt, filename);

    // Benchmark on binarized image (compare regular code with SSE
    // implementation)

    vpImage<unsigned char> I_Klimt_binarized = I_Klimt;
    vpImageTools::binarise(I_Klimt_binarized, (unsigned char)127, (unsigned char)127, (unsigned char)0,
                           (unsigned char)1, (unsigned char)1, true);

    // Dilatation CONNEXITY_4
    vpImage<unsigned char> I_Klimt_binarized_dilatation1 = I_Klimt_binarized;
    vpImage<unsigned char> I_Klimt_binarized_dilatation1_sse = I_Klimt_binarized;

    double t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_binarized_dilatation1, (unsigned char)1, (unsigned char)0,
                                    vpImageMorphology::CONNEXITY_4);
    }
    t = vpTime::measureTimeMs() - t;

    double t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_binarized_dilatation1_sse, vpImageMorphology::CONNEXITY_4);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_binarized_dilatation1 == "
                 "I_Klimt_binarized_dilatation1_sse)? "
              << (I_Klimt_binarized_dilatation1 == I_Klimt_binarized_dilatation1_sse) << " ; t=" << t
              << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Dilatation CONNEXITY_8
    vpImage<unsigned char> I_Klimt_binarized_dilatation2 = I_Klimt_binarized;
    vpImage<unsigned char> I_Klimt_binarized_dilatation2_sse = I_Klimt_binarized;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_binarized_dilatation2, (unsigned char)1, (unsigned char)0,
                                    vpImageMorphology::CONNEXITY_8);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_binarized_dilatation2_sse, vpImageMorphology::CONNEXITY_8);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_binarized_dilatation2 == "
                 "I_Klimt_binarized_dilatation2_sse)? "
              << (I_Klimt_binarized_dilatation2 == I_Klimt_binarized_dilatation2_sse) << " ; t=" << t
              << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Erosion CONNEXITY_4
    vpImage<unsigned char> I_Klimt_binarized_erosion1 = I_Klimt_binarized;
    vpImage<unsigned char> I_Klimt_binarized_erosion1_sse = I_Klimt_binarized;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_binarized_erosion1, (unsigned char)1, (unsigned char)0,
                                 vpImageMorphology::CONNEXITY_4);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_binarized_erosion1_sse, vpImageMorphology::CONNEXITY_4);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_binarized_erosion1 == I_Klimt_binarized_erosion1_sse)? "
              << (I_Klimt_binarized_erosion1 == I_Klimt_binarized_erosion1_sse) << " ; t=" << t
              << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Erosion CONNEXITY_8
    vpImage<unsigned char> I_Klimt_binarized_erosion2 = I_Klimt_binarized;
    vpImage<unsigned char> I_Klimt_binarized_erosion2_sse = I_Klimt_binarized;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_binarized_erosion2, (unsigned char)1, (unsigned char)0,
                                 vpImageMorphology::CONNEXITY_8);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_binarized_erosion2_sse, vpImageMorphology::CONNEXITY_8);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_binarized_erosion2 == I_Klimt_binarized_erosion2_sse)? "
              << (I_Klimt_binarized_erosion2 == I_Klimt_binarized_erosion2_sse) << " ; t=" << t
              << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Benchmark on grayscale images (compare regular code with SSE
    // implementation)

    // Dilatation CONNEXITY_4 grayscale
    vpImage<unsigned char> I_Klimt_dilatation1 = I_Klimt;
    vpImage<unsigned char> I_Klimt_dilatation1_sse = I_Klimt;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      generalDilatation(I_Klimt_dilatation1, vpImageMorphology::CONNEXITY_4);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_dilatation1_sse, vpImageMorphology::CONNEXITY_4);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_dilatation1 == I_Klimt_dilatation1_sse)? "
              << (I_Klimt_dilatation1 == I_Klimt_dilatation1_sse) << " ; t=" << t << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Dilatation CONNEXITY_8 grayscale
    vpImage<unsigned char> I_Klimt_dilatation2 = I_Klimt;
    vpImage<unsigned char> I_Klimt_dilatation2_sse = I_Klimt;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      generalDilatation(I_Klimt_dilatation2, vpImageMorphology::CONNEXITY_8);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::dilatation(I_Klimt_dilatation2_sse, vpImageMorphology::CONNEXITY_8);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_dilatation2 == I_Klimt_dilatation2_sse)? "
              << (I_Klimt_dilatation2 == I_Klimt_dilatation2_sse) << " ; t=" << t << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Erosion CONNEXITY_4 grayscale
    vpImage<unsigned char> I_Klimt_erosion1 = I_Klimt;
    vpImage<unsigned char> I_Klimt_erosion1_sse = I_Klimt;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      generalErosion(I_Klimt_erosion1, vpImageMorphology::CONNEXITY_4);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_erosion1_sse, vpImageMorphology::CONNEXITY_4);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_erosion1 == I_Klimt_erosion1_sse)? " << (I_Klimt_erosion1 == I_Klimt_erosion1_sse)
              << " ; t=" << t << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

    // Erosion CONNEXITY_8 grayscale
    vpImage<unsigned char> I_Klimt_erosion2 = I_Klimt;
    vpImage<unsigned char> I_Klimt_erosion2_sse = I_Klimt;

    t = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      generalErosion(I_Klimt_erosion2, vpImageMorphology::CONNEXITY_8);
    }
    t = vpTime::measureTimeMs() - t;

    t_sse = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      vpImageMorphology::erosion(I_Klimt_erosion2_sse, vpImageMorphology::CONNEXITY_8);
    }
    t_sse = vpTime::measureTimeMs() - t_sse;

    std::cout << "(I_Klimt_erosion2 == I_Klimt_erosion2_sse)? " << (I_Klimt_erosion2 == I_Klimt_erosion2_sse)
              << " ; t=" << t << " ms ; t_sse=" << t_sse << " ms"
              << " ; speed-up=" << (t / t_sse) << "X" << std::endl;

// Compare with OpenCV
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    std::cout << std::endl;

    cv::Mat cross_SE = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    cv::Mat rect_SE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    // Dilatation CONNEXITY_4 grayscale
    cv::Mat matImg_dilatation1;
    vpImageConvert::convert(I_Klimt, matImg_dilatation1);

    double t_opencv = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      cv::morphologyEx(matImg_dilatation1, matImg_dilatation1, cv::MORPH_DILATE, cross_SE);
    }
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    vpImage<unsigned char> I_matImg_dilatation1;
    vpImageConvert::convert(matImg_dilatation1, I_matImg_dilatation1);
    std::cout << "(I_matImg_dilatation1 == I_Klimt_dilatation1_sse)? "
              << (I_matImg_dilatation1 == I_Klimt_dilatation1_sse) << " ; t_opencv=" << t_opencv << " ms" << std::endl;

    // Dilatation CONNEXITY_8 grayscale
    cv::Mat matImg_dilatation2;
    vpImageConvert::convert(I_Klimt, matImg_dilatation2);

    t_opencv = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      cv::morphologyEx(matImg_dilatation2, matImg_dilatation2, cv::MORPH_DILATE, rect_SE);
    }
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    vpImage<unsigned char> I_matImg_dilatation2;
    vpImageConvert::convert(matImg_dilatation2, I_matImg_dilatation2);
    std::cout << "(I_matImg_dilatation2 == I_Klimt_dilatation2_sse)? "
              << (I_matImg_dilatation2 == I_Klimt_dilatation2_sse) << " ; t_opencv=" << t_opencv << " ms" << std::endl;

    // Erosion CONNEXITY_4 grayscale
    cv::Mat matImg_erosion1;
    vpImageConvert::convert(I_Klimt, matImg_erosion1);

    t_opencv = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      cv::morphologyEx(matImg_erosion1, matImg_erosion1, cv::MORPH_ERODE, cross_SE);
    }
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    vpImage<unsigned char> I_matImg_erosion1;
    vpImageConvert::convert(matImg_erosion1, I_matImg_erosion1);
    std::cout << "(I_matImg_erosion1 == I_Klimt_erosion1_sse)? " << (I_matImg_erosion1 == I_Klimt_erosion1_sse)
              << " ; t_opencv=" << t_opencv << " ms" << std::endl;

    // Erosion CONNEXITY_8 grayscale
    cv::Mat matImg_erosion2;
    vpImageConvert::convert(I_Klimt, matImg_erosion2);

    t_opencv = vpTime::measureTimeMs();
    for (int cpt = 0; cpt < nbIterations; cpt++) {
      cv::morphologyEx(matImg_erosion2, matImg_erosion2, cv::MORPH_ERODE, rect_SE);
    }
    t_opencv = vpTime::measureTimeMs() - t_opencv;

    vpImage<unsigned char> I_matImg_erosion2;
    vpImageConvert::convert(matImg_erosion2, I_matImg_erosion2);
    std::cout << "(I_matImg_erosion2 == I_Klimt_erosion2_sse)? " << (I_matImg_erosion2 == I_Klimt_erosion2_sse)
              << " ; t_opencv=" << t_opencv << " ms" << std::endl;

#endif

  } catch (const vpException &e) {
    std::cout << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nTest of morphology erosion / dilatation functions are OK!" << std::endl;
  return EXIT_SUCCESS;
}
