/****************************************************************************
 *
 * $Id: vpViewer.h,v 1.12 2008-04-14 12:56:52 asaunier Exp $
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
 * Simulator based on Coin3d.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpViewer_HH
#define vpViewer_HH
/*!
  \file vpViewer.h
  Viewer used by the simulator. Under Windows, the viewer is
  based either on SoWin or SoQt. Under Unix, the viewer is based on SoQt or SoXt.
*/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_COIN

#if defined(VISP_HAVE_SOWIN)

  #include <Inventor/Win/SoWin.h>
  #include <Inventor/Win/viewers/SoWinExaminerViewer.h>

#elif defined(VISP_HAVE_SOQT)

  #include <Inventor/Qt/SoQt.h>
  #include <Inventor/Qt/viewers/SoQtExaminerViewer.h>

#elif defined(VISP_HAVE_SOXT)
  
  #include <Inventor/Xt/SoXt.h>
  #include <Inventor/Xt/viewers/SoXtExaminerViewer.h>

#endif

// Coin stuff

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

#if defined(VISP_HAVE_SOWIN)
#include <GL/gl.h>
#elif defined(VISP_HAVE_SOQT)
#include <qgl.h>
#elif defined(VISP_HAVE_SOXT)
#include <GL/gl.h>
#endif
// thread
//#include <pthread.h>

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

/*!
  \class vpViewer

  \brief Viewer used by the simulator.

  Under Windows, the viewer is based either on SoWin or SoQt. Under
  Unix, the viewer is based on SoQt or SoXt .
*/

class vpSimulator;

#if defined(VISP_HAVE_SOWIN)
class VISP_EXPORT vpViewer : public SoWinExaminerViewer
#elif defined(VISP_HAVE_SOQT)
class VISP_EXPORT vpViewer : public SoQtExaminerViewer
#elif defined(VISP_HAVE_SOXT)
class VISP_EXPORT vpViewer : public SoXtExaminerViewer
#endif
{

  friend class vpSimulator ;

public:
#if defined(VISP_HAVE_SOWIN)
  vpViewer(HWND parent,  vpSimulator *simu);
#elif defined(VISP_HAVE_SOQT)
  vpViewer(QWidget * parent,  vpSimulator *simu);
#elif defined(VISP_HAVE_SOXT)
  vpViewer(Widget parent,  vpSimulator *simu);
#endif

  virtual ~vpViewer();
  void  resize(int x, int y, bool fixed = false) ;
  virtual void actualRedraw(void);

private:

  vpSimulator *simu ;
  SbBool processSoEvent(const SoEvent * const event) ;
#if defined(VISP_HAVE_SOWIN)
  static HWND init(const char * appname) {return SoWin::init(appname);};
  static void mainLoop() {SoWin::mainLoop();};
  static void exitMainLoop() {SoWin::exitMainLoop();};
#elif defined(VISP_HAVE_SOQT)
  static QWidget * init(const char * appname) {return SoQt::init(appname);};
  static void mainLoop() { SoQt::mainLoop();};
  static void exitMainLoop() {SoQt::exitMainLoop();};
#elif defined(VISP_HAVE_SOXT)
  static Widget init(const char * appname) {return SoXt::init(appname);};
  static void mainLoop() { SoXt::mainLoop();};
  static void exitMainLoop() {SoXt::exitMainLoop();};
#endif
};

#endif //VISP_HAVE_COIN

#endif
