/****************************************************************************
 *
 * $Id: imageDiskRW.cpp,v 1.9 2008-11-18 12:41:37 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Reading and writting images on the disk.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file imageDiskRW.cpp
  \brief   reading and writting of PPM image using vpImageIo class.

  Example of exception handling the exception :
  read an image that does not exist
  write in a directory that does no exist
 */

/*!
  \example imageDiskRW.cpp
  Reading and writting of PPM image using vpImageIo class.

  Example of exception handling the exception :
  read an image that does not exist,
  write in a directory that does no exist
 */

#include <stdlib.h>
#include <stdio.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>

// List of allowed command line options
#define GETOPTARGS	"i:o:h"

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param ipath : Input image path.
\param opath : Output image path.
\param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string opath, std::string user)
{
  fprintf(stdout, "\n\
Read and write PGM images on the disk. Also test exceptions.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-o <output image path>]\n\
     [-h]\n						      \
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
     Klimt_grey.pgm output image is written.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), opath.c_str(), user.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param opath : Output image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv,
		std::string &ipath, std::string &opath, std::string user)
{
  const char *optarg;
  int	c;
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
main(int argc, const char ** argv)
{

  std::string env_ipath;
  std::string opt_ipath;
  std::string opt_opath;
  std::string ipath;
  std::string opath;
  std::string filename;
  std::string username;

  std::cout <<  "-------------------------------------------------------" << std::endl ;
  std::cout <<  "  imageDiskRW.cpp" <<std::endl << std::endl ;

  std::cout <<  "  reading and writting of PPM image" << std::endl ;
  std::cout <<  "  read an image that does not exist" << std::endl ;
  std::cout <<  "  write in a directory that does no exist" << std::endl ;
  std::cout <<  "-------------------------------------------------------" << std::endl ;
  std::cout << std::endl ;


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
  std::string dirname = opath + vpIoTools::path("/") + username;

  // Test if the output path exist. If no try to create it
  if (vpIoTools::checkDirectory(dirname) == false) {
    try {
      // Create the dirname
      vpIoTools::makeDirectory(dirname);
    }
    catch (...) {
      usage(argv[0], NULL, ipath, opath, username);
      std::cerr << std::endl
	   << "ERROR:" << std::endl;
      std::cerr << "  Cannot create " << dirname << std::endl;
      std::cerr << "  Check your -o " << opath << " option " << std::endl;
      exit(-1);
    }
  }

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (!opt_ipath.empty() && !env_ipath.empty()) {
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
    usage(argv[0], NULL, ipath, opath, username);
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << std::endl
	 << "  environment variable to specify the location of the " << std::endl
	 << "  image path where test images are located." << std::endl << std::endl;
    exit(-1);
  }


/////////////////////////////////////////////////////////////////////



  // First we wanted to have gray level image (8bits)
  // vpImage is a template class you can declare vpImage of ... everything...
  vpImage<unsigned char> I ;

  // Although I is a gray level image you can read and write
  // color image. Obviously the color will be translated as a gray level

  filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");
  vpImageIo::readPPM(I, filename);

  filename = dirname +  vpIoTools::path("/IoPPM.Klimt_char.ppm");
  vpImageIo::writePPM(I, filename) ;


  // test io error
  // if the image you want to read on the disk does not exist
  // an exception is thrown
  try
  {
    //Try to load a non existing image
    filename = ipath + vpIoTools::path("/ViSP-images/image-that-does-not-exist.ppm");

    vpImageIo::readPPM(I,filename) ;
  }
  catch(vpImageException e)
  {
    vpERROR_TRACE("at main level");
    std::cout << e << std::endl ;
  }

  // same thing if you to write in a directory that does not exist
  // or where you are not allowd to write.
  try
  {
    filename = opath + vpIoTools::path("/directory-that-does-not-exist/Klimt.ppm");
    vpImageIo::writePPM(I,filename) ;
  }
  catch(vpImageException e)
  {
    vpERROR_TRACE("at main level");
    std::cout << e << std::endl ;
  }

  std::cout << "----------------------------------------------------" << std::endl ;

 // Let's consider that the image is now a color image (32 bits RGBa)
  vpImage<vpRGBa> Irgba ;

  // read write unsigned char ppm image.

 // Load a color image from the disk
  filename = ipath + vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");
  vpImageIo::readPPM(Irgba, filename);

  // Write the content of the color image on the disk
  filename = opath + vpIoTools::path("/IoPGM.Klimt_rgba.ppm");
  vpImageIo::writePPM(Irgba, filename) ;

  // test io error
  try
  {
    filename = ipath + vpIoTools::path("/ViSP-images/image-that-does-not-exist.ppm");
    vpImageIo::readPPM(Irgba,filename) ;
  }
  catch(vpImageException e)
  {
    vpERROR_TRACE("at main level");
    std::cout << e << std::endl ;
  }

 // test io error
  try
  {
    filename = opath + vpIoTools::path("/directory-that-does-not-exist/Klimt.ppm");
    vpImageIo::writePPM(Irgba,filename) ;
  }
  catch(vpImageException e)
  {
    vpERROR_TRACE("at main level");
    std::cout << e << std::endl ;
  }



}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
