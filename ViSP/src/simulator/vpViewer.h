/****************************************************************************
 *
 * $Id: vpViewer.h,v 1.5 2007-03-21 13:48:40 asaunier Exp $
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
 * Simulator based on SoQt.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpViewer_HH
#define vpViewer_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT


// Qt and Coin stuff
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/sensors/SoTimerSensor.h>

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

class vpSimulator;

class VISP_EXPORT vpViewer : public SoQtExaminerViewer {

  friend class vpSimulator ;

public:
  vpViewer(QWidget * parent,  vpSimulator *simu);
  ~vpViewer();
  void  resize(int x, int y) ;


public:
  virtual void actualRedraw(void);

private:
  vpSimulator *simu ;
  SbBool processSoEvent(const SoEvent * const event) ;


};


#endif
#endif
