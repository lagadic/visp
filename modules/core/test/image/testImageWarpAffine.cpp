/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test for vpImageTools::warpAffine() function.
 *
 * Author:
 * Vikas Thamizharasan
 *
 *****************************************************************************/

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpImageTools.h>



int main()
{
  std::cout << "Testing vpImageTools::warpAffine()." << std::endl;

  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string filename;

    // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH environment variable value
    env_ipath = vpIoTools::getViSPImagesDataPath();

    // Set the default input path
    if (! env_ipath.empty()) {
      ipath = env_ipath;
    }

    // Get the option values
    if (!opt_ipath.empty()) {
      ipath = opt_ipath;
    }


    //Initialize transformation matrix
    vpMatrix transform_matrix(3,3);

    // Test Vertical Sheer
    transform_matrix[0][0] = 1;   transform_matrix[0][1] = 0;  transform_matrix[0][2] = 0;
    transform_matrix[1][0] = 0.5; transform_matrix[1][1] = 1;  transform_matrix[1][2] = 0;
    transform_matrix[2][0] = 0;   transform_matrix[2][1] = 0;  transform_matrix[2][2] = 1;

    // // Test Rotation
    // transform_matrix[0][0]= 0.50; transform_matrix[0][1] =  0.86; transform_matrix[0][2] =  0.2;
    // transform_matrix[1][0]= -0.86; transform_matrix[1][1] =  0.50; transform_matrix[1][2] =  0;
    // transform_matrix[2][0]= 1; transform_matrix[2][1] =  1; transform_matrix[2][2] =  1;
    
    // // Test Translation
    // transform_matrix[0][0]= 0; transform_matrix[0][1] =  0; transform_matrix[0][2] =  0;
    // transform_matrix[1][0]= 0; transform_matrix[1][1] =  0; transform_matrix[1][2] =  0;
    // transform_matrix[2][0]= 0.5; transform_matrix[2][1] =  0.5; transform_matrix[2][2] =  1;

    //Load grayscale Klimt
    filename = vpIoTools::createFilePath(ipath, "ViSP-images/Klimt/Klimt.pgm");

    vpImage<unsigned char> I_Klimt, I_warped_Klimt, I;
    vpImageIo::read(I_Klimt, filename);
    I_warped_Klimt = I_Klimt;

    vpImageTools::warpAffine(I_warped_Klimt, transform_matrix);

    std::cout << "Final Image size() : " <<  I_warped_Klimt.getRows() << " " << I_warped_Klimt.getCols() << std::endl;    

    I = I_warped_Klimt;
    #if defined(VISP_HAVE_X11)
        vpDisplayX d(I);
    #elif defined(VISP_HAVE_OPENCV)
        vpDisplayOpenCV d(I);
    #elif defined(VISP_HAVE_GTK)
        vpDisplayGTK d(I);
    #elif defined(VISP_HAVE_GDI)
        vpDisplayGDI d(I);
    #elif defined(VISP_HAVE_D3D9)
        vpDisplayD3d d(I);
    #else
        std::cout << "No image viewer is available..." << std::endl;
    #endif
        vpDisplay::setTitle(I, "My image");
        vpDisplay::display(I);
        vpDisplay::flush(I);
        std::cout << "A click to quit..." << std::endl;
        vpDisplay::getClick(I);

  } catch(vpException &e) {
    std::cerr << "\nCatch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "\nTest Successful" << std::endl;
  return EXIT_SUCCESS;
}