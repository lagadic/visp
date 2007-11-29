/****************************************************************************
 *
 * $Id: testUndistortImage.cpp,v 1.1 2007-11-29 15:09:45 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Test for image undistortion.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/

#include <stdlib.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <visp/vpDebug.h>

/*!
  \example testCreateSubImage.cpp

  \brief Create a sub-image from an image by cropping a rectangular area.

  Read an image from the disk, undistort it and save the
  undistorted image on the disk.

 */

// List of allowed command line options
#define GETOPTARGS  "i:o:h"

/*

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.

 */
void usage(char *name, char *badparam, std::string ipath, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
          Read an image from the disk, undistort it \n\
          and save the undistorted image on the disk.\n\
          (Klimt_undistorted.pgm).\n\
          \n\
          SYNOPSIS\n\
          %s [-i <input image path>] [-o <output image path>]\n\
          [-h]\n                 \
          ", name);

  fprintf(stdout, "\n\
      OPTIONS:                                               Default\n\
          -i <input image path>                                %s\n\
          Set image input path.\n\
          From this path read \"ViSP-images/Klimt/Klimt.pgm\"\n\
          image.\n\
          Setting the VISP_INPUT_IMAGE_PATH environment\n\
          variable produces the same behaviour than using\n\
          this option.\n\
          \n\
          -o <output image path>                               %s\n\
          Set image output path.\n\
          From this directory, creates the \"%s\"\n\
          subdirectory depending on the username, where \n\
          Klimt_undistorted.pgm output image is written.\n\
          \n\
          -h\n\
          Print the help.\n\n",
          ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

 */
bool getOptions(int argc, char **argv,
                std::string &ipath, std::string &opath, std::string user)
{
  char *optarg;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
      case 'i': ipath = optarg; break;
      case 'o': opath = optarg; break;
      case 'h': usage(argv[0], NULL, ipath, opath, user); return false; break;

      default:
        usage(argv[0], optarg, ipath, opath, user); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, opath, user);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}

int
    main(int argc, char ** argv)
{
  std::string env_ipath;
  std::string opt_ipath;
  std::string opt_opath;
  std::string ipath;
  std::string opath;
  std::string filename;
  std::string username;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Set the default output path
#ifdef UNIX
  opt_opath = "/tmp";
#elif WIN32
  opt_opath = "C:\\temp";
#endif

  // Get the user login name
  vpIoTools::getUserName(username);

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_opath, username) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;
  if (!opt_opath.empty())
    opath = opt_opath;

  // Append to the output path string, the login name of the user
  opath += vpIoTools::path("/") + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(opath) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(opath);
    }
    catch (...) {
      usage(argv[0], NULL, ipath, opt_opath, username);
      std::cerr << std::endl
          << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << opath << std::endl;
      std::cerr << "  Check your -o " << opt_opath << " option " << std::endl;
      exit(-1);
    }
  }

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (opt_ipath.empty()) {
    if (ipath != env_ipath) {
      std::cout << std::endl
          << "WARNING: " << std::endl;
      std::cout << "  Since -i <visp image path=" << ipath << "> "
          << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
          << "  we skip the environment variable." << std::endl;
    }
  }

  // Test if an input path is set
  if (opt_ipath.empty() && env_ipath.empty()){
    usage(argv[0], NULL, ipath, opt_opath, username);
    std::cerr << std::endl
        << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
        << std::endl
        << "  environment variable to specify the location of the " << std::endl
        << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }

  //
  // Here starts really the test
  //
  vpImage<unsigned char> I; // Input image
  vpImage<unsigned char> U; // undistorted output image
  vpCameraParameters cam;
  cam.init_pm(600,600,192,144,0.17);  
  cam.init_mp(600,600,192,144,-0.17);
  // Read the input grey image from the disk
  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
  std::cout << "Read image: " << filename << std::endl;
  vpImageIo::readPGM(I, filename) ;

  // Create the undistorted image
  vpImageTools::undistort(I, cam, U);

  // Write the undistorted image on the disk
  filename = opath +  vpIoTools::path("/Klimt_undistorted.pgm");
  std::cout << "Write undistorted image: " << filename << std::endl;
  vpImageIo::writePGM(U, filename) ;

}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
