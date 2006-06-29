/****************************************************************************
 *
 * $Id: testConversion.cpp,v 1.2 2006-06-29 13:38:24 fspindle Exp $
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
 * Test for image conversions.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>
#include <visp/vpParseArgv.h>

/*!
  \example testConversion.cpp

  \brief Test image conversion.

*/

int
main(int argc, char ** argv)
{
  char *_ipath = NULL;
  char *ipath = new char [FILENAME_MAX];
  char *_opath = NULL;
  char *opath = new char [FILENAME_MAX];
  char *filename = new char [FILENAME_MAX];

  sprintf(ipath, "images");
  sprintf(opath, "images-res");
  _ipath = ipath;
  _opath = opath;

  vpArgvInfo argTable[] =
    {
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL," Test image conversion "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {"-i", ARGV_STRING, (char *) 1, (char *) &_ipath,
      "Set image input path. Default: "},
      {"-o", ARGV_STRING, (char *) 1, (char *) &_opath,
      "Set image output path. Default: "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_HELP, NULL, NULL,"     "},
      {NULL, ARGV_END, NULL,NULL,NULL}
    } ;
  //Parsing of the table
  if (vpParseArgv::parse(&argc,argv,argTable,0))
  {
    cout << endl << "Usage : " << argv[0]
	 << " [-i <image input path>] [-o <image output path>]" << endl
	 << " [-help] " << endl << endl;
    delete [] ipath;
    delete [] opath;
    delete [] filename;
    exit(1) ;
  }

  if (_ipath != NULL)
    sprintf(ipath, "%s", _ipath);
  if (_opath != NULL)
    sprintf(opath, "%s", _opath);

  vpImage<unsigned char> Ig ; // Grey image
  vpImage<vpRGBa> Ic ; // Color image

  //-------------------- .pgm -> .ppm
  vpTRACE("Convert a grey image (.pgm) to a color image (.ppm)");
  // Load a grey image from the disk
  sprintf(filename, "%s/Klimt.pgm", ipath);
  vpTRACE("Load %s", filename);
  vpImageIo::readPGM(Ig, filename) ;
  // Create a color image from the grey
  vpImageConvert::convert(Ig, Ic);
  sprintf(filename, "%s/Klimt_color.pgm", opath);
  vpTRACE("Write %s", filename);
  vpImageIo::writePPM(Ic, filename) ;

  //-------------------- .ppm -> .pgm
  vpTRACE("Convert a color image (.ppm) to a grey image (.pgm)");
  // Load a color image from the disk
  sprintf(filename, "%s/Klimt.ppm", ipath);
  vpTRACE("Load %s", filename);
  vpImageIo::readPPM(Ic, filename) ;
  // Create a grey image from the color
  vpImageConvert::convert(Ic, Ig);
  sprintf(filename, "%s/Klimt_grey.ppm", opath);
  vpTRACE("Write %s", filename);
  vpImageIo::writePPM(Ig, filename) ;

  //-------------------- YUV -> RGB
  unsigned char y=187, u=10, v=30;
  unsigned char r, g, b;

  vpImageConvert::YUVToRGB(y, u, v, r, g, b);
  vpTRACE("y(%d) u(%d) v(%d) = r(%d) g(%d) b(%d)", y, u, v, r, g, b);


  //--------------------
  delete [] ipath;
  delete [] opath;
  delete [] filename;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
