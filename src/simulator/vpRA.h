/****************************************************************************
 *
 * $Id: vpRA.h,v 1.3 2007-03-21 13:48:40 asaunier Exp $
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
 * Use to display an image behind the internal view of the simulator
 * used for augmented reality application
 *
 * Authors:
 * Eric Marchand (march 2007)
 *
 *****************************************************************************/

#ifndef vpRA_HH
#define vpRA_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT

/*   KNOWN ISSUE DEALING WITH X11 and QT
     If you get a strange compiler error on the line with None,
     it's probably because you're also including X11 headers,
     which #define the symbol None. Put the X11 includes after
     the Qt includes to solve this problem.
 */

// Qt and Coin stuff
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/fields/SoSFTime.h>

// open GL
//#include <GL/gl.h>
//#include <QtOpenGL/qgl.h>
#include <qgl.h>

// thread
#include <pthread.h>

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

#include <visp/vpSimulator.h>

#include <visp/vpViewer.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpTime.h>

class VISP_EXPORT vpRA : public vpSimulator
{

private:

  bool background;

 public:


  ~vpRA() ;
  void initInternalViewer(int width, int height, vpImageType type = grayImage) ;
  void setImage(vpImage<unsigned char> &I) ;
  void setImage(vpImage<vpRGBa> &I) ;

} ;


#endif
#endif
