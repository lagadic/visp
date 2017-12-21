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

  Viewer used by the simulator. Under Windows, the viewer is based
  either on SoWin or SoQt. Under Unix, the viewer is based on SoQt or
  SoXt.

  \warning The content of this file is only available if Coin3D and
  one of the GUI (SoWin, SoXT, SoQt) are installed.

*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI

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

#if defined(VISP_HAVE_SOWIN)
#include <GL/gl.h>
#elif defined(VISP_HAVE_SOQT)
#include <qgl.h>
#elif defined(VISP_HAVE_SOXT)
#include <GL/gl.h>
#endif

// visp
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>

/*!
  \class vpViewer
  \ingroup group_ar_simulator

  \brief Viewer used by the simulator.

  Under Windows, the viewer is based either on SoWin or SoQt. Under
  Unix, the viewer is based on SoQt or SoXt.

  \warning This class is only available if Coin3D and one of the GUI
  (SoWin, SoXT, SoQt) are installed.

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

  friend class vpSimulator;

public:
  typedef enum { internalView, externalView } vpViewerType;
#if defined(VISP_HAVE_SOWIN)
  vpViewer(HWND parent, vpSimulator *simu, vpViewerType type);
#elif defined(VISP_HAVE_SOQT)
  vpViewer(QWidget *parent, vpSimulator *simu, vpViewerType type);
#elif defined(VISP_HAVE_SOXT)
  vpViewer(Widget parent, vpSimulator *simu, vpViewerType type);
#endif

  virtual ~vpViewer();
  void resize(int x, int y, bool fixed = false);
  virtual void actualRedraw(void);

private:
  vpViewerType viewerType;
  vpSimulator *simu;
  SbBool processSoEvent(const SoEvent *const event);
#if defined(VISP_HAVE_SOWIN)
  static HWND init(const char *appname) { return SoWin::init(appname); };
  static void mainLoop() { SoWin::mainLoop(); };
  static void exitMainLoop() { SoWin::exitMainLoop(); };
#elif defined(VISP_HAVE_SOQT)
  static QWidget *init(const char *appname) { return SoQt::init(appname); };
  static void mainLoop() { SoQt::mainLoop(); };
  static void exitMainLoop() { SoQt::exitMainLoop(); };
#elif defined(VISP_HAVE_SOXT)
  static Widget init(const char *appname) { return SoXt::init(appname); };
  static void mainLoop() { SoXt::mainLoop(); };
  static void exitMainLoop() { SoXt::exitMainLoop(); };
#endif
};

#endif // VISP_HAVE_COIN3D_AND_GUI

#endif
