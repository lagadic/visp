/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
  \brief class used to display an image behind the internal view of the simulator. Used for augmented reality applications.
*/


#ifndef vpAR_HH
#define vpAR_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_COIN

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

  The code below shows how to use the class.

  \code
#include <visp/vpConfig.h>
#include <visp/vpAR.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>

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

int main()
{
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
  void initInternalViewer(int width, int height, vpImageType type = grayImage) ;
  void setImage(vpImage<unsigned char> &I) ;
  void setImage(vpImage<vpRGBa> &I) ;

} ;


#endif
#endif
