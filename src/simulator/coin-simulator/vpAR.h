/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_COIN_AND_GUI

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

#include <visp/vpSimulator.h>

#include <visp/vpViewer.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpTime.h>

/*!
  \class vpAR

  \ingroup VisuRobotSimu
  
  \brief Implementation of an augmented reality viewer.

  This class can be used to display an image behind the internal view
  of the simulator used for augmented reality application.
  
  \warning This class is only available if Coin3D and one of the GUI
  (SoWin, SoXT, SoQt) are installed.

  The code below shows how to use the class.

  \code
#include <visp/vpConfig.h>
#include <visp/vpAR.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>

#ifdef VISP_HAVE_COIN_AND_GUI
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
#ifdef VISP_HAVE_COIN_AND_GUI
  vpAR simu;
  //Camera parameters.
  vpCameraParameters cam(600,600,160,120);

  //Initialize the internal view of the simulator.
  simu.initInternalViewer(640,480, vpSimulator::grayImage);

  vpTime::wait(300);

  //Load the cad model.
  simu.load("./4points.iv"); //4points.iv can be downloaded on the website with the image package

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


  virtual ~vpAR() ;
  void initInternalViewer(const unsigned int width, const unsigned int height, vpImageType type = grayImage) ;
  void setImage(vpImage<unsigned char> &I) ;
  void setImage(vpImage<vpRGBa> &I) ;

} ;


#endif
#endif
