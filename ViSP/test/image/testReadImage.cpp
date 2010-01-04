/****************************************************************************
 *
 * $Id: testIoPPM.cpp,v 1.14 2008-06-17 08:08:29 asaunier Exp $
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
 * Read images on the disk.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpParseArgv.h>
#include <visp/vpDebug.h>

/*!
  \example testReadImage.cpp

  \brief Read images on the disk.

*/

// List of allowed command line options
#define GETOPTARGS	"i:p:h"


/*

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath: Input image path.
  \param opath : Personal image path.
  \param user : Username.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath)
{
  fprintf(stdout, "\n\
Read images on the disk.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-p <personal image path>]\n\
     [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read \"ViSP-images/Klimt/Klimt.pgm,\n\
     .ppm, .jpeg and .png images.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -p <personal image path>                               %s\n\
     Set personal image path.\n\
     From this path read the image \"%s\".\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), ppath.c_str(), ppath.c_str());

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath: Input image path.
  \param ppath : Personal image path.
  \param user : Username.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv,
		std::string &ipath, std::string &ppath)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'i': ipath = optarg; break;
    case 'p': ppath = optarg; break;
    case 'h': usage(argv[0], NULL, ipath, ppath); return false; break;

    default:
      usage(argv[0], optarg, ipath, ppath); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath);
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
  std::string opt_ppath;
  std::string ipath;
  std::string ppath;
  std::string filename;

  // Get the VISP_IMAGE_PATH environment variable value
  char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
  if (ptenv != NULL)
    env_ipath = ptenv;

  // Set the default input path
  if (! env_ipath.empty())
    ipath = env_ipath;

  // Read the command line options
  if (getOptions(argc, argv, opt_ipath, opt_ppath) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;
  if (!opt_ppath.empty())
    ppath = opt_ppath;

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


  // 
  // Here starts really the test
  // 

  /////////////////////////////////////////////////////////////////////
  // Create a grey level image
  vpImage<vpRGBa> I;
  
  if (opt_ppath.empty())
  {
    filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.ppm");
    vpImageIo::readPPM(I,filename);
    vpTRACE("Read ppm ok");
    filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.pgm");
    vpImageIo::readPGM(I,filename);
    vpTRACE("Read pgm ok");
    #if defined(VISP_HAVE_LIBJPEG)
    filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.jpeg");
    vpImageIo::readJPEG(I,filename);
    vpTRACE("Read jpeg ok");
    #else
    vpTRACE("To read jpeg you must have the libjpeg library");
    #endif
    
    #if defined(VISP_HAVE_LIBPNG)
    filename = ipath +  vpIoTools::path("/ViSP-images/Klimt/Klimt.png");
    vpImageIo::readPNG(I,filename);
    vpTRACE("Read png ok");
    #else
    vpTRACE("To read jpeg you must have the libpng library");
    #endif
  }
  
  else
  {
    filename = opt_ppath;
    vpImageIo::read(I,filename);
    vpTRACE("image read without problem");
  }

  return 0;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
