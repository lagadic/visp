/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Use to display an image behind the internal view of the simulator
 * used for augmented reality application
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpAR.h

  \brief Class used to display an image behind the internal view of
  the simulator. Used for augmented reality applications.

  \warning The content of this file is only available if Coin3D and
  one of the GUI (SoWin, SoXT, SoQt) are installed.

*/

#ifndef vpAR_HH
#define vpAR_HH

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI

// visp
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#include <visp3/ar/vpSimulator.h>

#include <visp3/ar/vpViewer.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpTime.h>

/*!
  \class vpAR

  \ingroup group_ar_renderer

  \brief Implementation of an augmented reality viewer using Coin3D 3rd party.

  Thus to be enabled this class requires Coin3D 3rd party installation.
  Installation instructions are provided here https://visp.inria.fr/3rd_coin.

  This class can be used to display an image behind the internal view
  of the simulator used for augmented reality application.

  \warning This class is only available if Coin3D and one of the GUI
  (SoWin, SoXT, SoQt) are installed.

  The code below shows how to use the class.

  \code
#include <visp3/ar/vpAR.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI
static void *mainloopfunction(void *_simu)
{
  vpAR *simu = (vpAR *)_simu ;
  simu->initMainApplication() ;

  vpImage<unsigned char> I;
  vpHomogeneousMatrix cMo;

  //Your code to compute the pose cMo.

  //Set the image to use as background.
  simu->setImage(I) ;
  //Set the camera position thanks to the pose cMo computed before.
  simu->setCameraPosition(cMo) ;

  simu->closeMainApplication();
}
#endif

int main()
{
#ifdef VISP_HAVE_COIN3D_AND_GUI
  vpAR simu;
  //Camera parameters.
  vpCameraParameters cam(600,600,160,120);

  //Initialize the internal view of the simulator.
  simu.initInternalViewer(640,480, vpSimulator::grayImage);

  vpTime::wait(300);

  // Load the cad model. 4points.iv can be downloaded on the website
  // with the image package
  simu.load("./4points.iv");

  //Initialize the internal camera parameters.
  simu.setInternalCameraParameters(cam);

  simu.initApplication(&mainloopfunction);

  simu.mainLoop();
#endif
  return 0;
}
  \endcode

*/
class VISP_EXPORT vpAR : public vpSimulator
{

private:
  bool background;

public:
  vpAR() : background(false){};

  virtual ~vpAR();
  void initInternalViewer(const unsigned int width, const unsigned int height, vpImageType type = grayImage);
  void setImage(vpImage<unsigned char> &I);
  void setImage(vpImage<vpRGBa> &I);
};

#endif
#endif
