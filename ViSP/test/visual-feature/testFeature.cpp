/****************************************************************************
 *
 * $Id: testFeature.cpp,v 1.2 2007-06-20 15:55:11 fspindle Exp $
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
 * Visual feature manipulation.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <iostream>


#include <visp/vpColVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpServo.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureThetaU.h>
#include <visp/vpDot2.h>
#include <visp/vpCameraParameters.h>

int main()
{
  vpServo task ;
  vpColVector v ; // computed robot velocity

  vpFeaturePoint p, pd;
  vpDot2 dot(100.0, 200.0);
  vpCameraParameters cam;
  vpFeatureBuilder::create(p, cam, dot)  ;
  p.set_Z(1.0);
  vpRotationMatrix R;
  R.setIdentity();
  vpTranslationVector td(1, 0, 0);
  vpTranslationVector t (0.5, 0, 0);
  vpHomogeneousMatrix cdMo(R, td);
  vpHomogeneousMatrix cMo(R, t);
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;
  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;

  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::CURRENT) ;
  //  task.addFeature(p, pd);
  //task.addFeature(logZ);
  task.addFeature(tu);
  task.print() ;
  //  double task_error_square = task.error.sumSquare() ;
  double gain = 1.0;
  task.setLambda(gain) ;

  v = task.computeControlLaw() ;

}
