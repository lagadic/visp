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
 * Anthony Saunier
 *
 *****************************************************************************/

#ifndef vpSimulator_HH
#define vpSimulator_HH
/*!
  \file vpSimulator.h
  \brief Implementation of a simulator based on Coin3d (www.coin3d.org).
  The simulator uses the vpViewer class.

  \warning The content of this file is only available if Coin3D and
  one of the GUI (SoWin, SoXT, SoQt) are installed.
*/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI

#include <visp3/ar/vpViewer.h>

/*   KNOWN ISSUE DEALING WITH X11 and QT
     If you get a strange compiler error on the line with None,
     it's probably because you're also including X11 headers,
     which #define the symbol None. Put the X11 includes after
     the Qt includes to solve this problem.
 */

#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/fields/SoSFTime.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/threads/SbThread.h>

// visp
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpSimulator

  \ingroup group_ar_simulator

  \brief Implementation of a simulator based on Coin3d (www.coin3d.org).

  The simulator uses the vpViewer class.

  \warning This class is only available if Coin3D and one of the GUI
  (SoWin, SoXT, SoQt) are installed.
*/

class VISP_EXPORT vpSimulator
{
protected:
  //! perform some initialization
  void init();
  //! perform some destruction
  void kill();

public:
  //! constructor
  vpSimulator();
  virtual ~vpSimulator();

protected:
//! main Widget
#if defined(VISP_HAVE_SOWIN)
  HWND mainWindow;
#elif defined(VISP_HAVE_SOQT)
  QWidget *mainWindow;
#elif defined(VISP_HAVE_SOXT)
  Widget mainWindow;
#endif

  bool mainWindowInitialized;

  //! open the SoGui application
  void initSoApplication();

public:
  typedef enum { grayImage, colorImage } vpImageType;
  vpImageType typeImage;

  GLubyte *image_background;
  //! activate the mainloop
  virtual void mainLoop();

protected:
  //! view from the camera
  vpViewer *internalView;
  //! view from an external camera
  vpViewer *externalView;

public:
  //! initialize the camera view
  virtual void initInternalViewer(const unsigned int nlig, const unsigned int ncol);
  //! initialize the external view
  void initExternalViewer(const unsigned int nlig, const unsigned int ncol);

protected:
  //! thread with the main program
  SbThread *mainThread;
  // pthread_t mainThread;

public:
  //! begin the main program
  void initApplication(void *(*start_routine)(void *));
  void initApplication(void *(*start_routine)(void *), void *data);
  //! perform some initialization in the main program thread
  void initMainApplication();
  void closeMainApplication();

  //----------------------------------------------------
  // scene description
protected:
  unsigned int internal_width;
  unsigned int internal_height;
  unsigned int external_width;
  unsigned int external_height;

public:
  /*!
    Return the width of the internal view.

    \return The width of the internal view.
  */
  unsigned int getInternalWidth() const { return internal_width; }
  /*!
    Return the height of the internal view.

    \return The height of the internal view.
  */
  unsigned int getInternalHeight() const { return internal_height; }

protected:
  //! root node of the scene : contains everything except stuff specific to
  //! each view
  SoSeparator *scene;
  //! root node of the internal view
  SoSeparator *internalRoot;
  //! root node of the external view
  SoSeparator *externalRoot;

  //! internal camera
  SoPerspectiveCamera *internalCamera;
  //! external camera
  SoPerspectiveCamera *externalCamera;

  //! internal camera position
  SoTransform *internalCameraPosition;

  //! external camera position
  SoTransform *extrenalCameraPosition;

  //! representation of the camera in the external view
  SoSeparator *internalCameraObject;

  //! initialize the scene graph
  void initSceneGraph();

  //! Add a new object in the scene graph ad a given location
  void addObject(SoSeparator *object, const vpHomogeneousMatrix &fMo, SoSeparator *root);

public:
  //! Add a new object in the scene graph ad a given location
  void addObject(SoSeparator *newObject, const vpHomogeneousMatrix &fMo);

public:
  //! display the scene (handle with care)
  void redraw();
  //! load an iv file
  void load(const char *file_name);
  //! load an iv file, set the location of this scene
  void load(const char *iv_filename, const vpHomogeneousMatrix &fMo);
  //! save the scene in an iv file
  void save(const char *name, bool binary = false);

  //!   Add the representation of the absolute frame
  void addAbsoluteFrame(float zoom = 1);
  //!   Add the representation of a frame
  void addFrame(const vpHomogeneousMatrix &fMo, float zoom = 1);
  //! set the size of the camera/frame
  void setZoomFactor(const float zoom);

protected:
  float zoomFactor;
  //---------------------------------------------------
  //  camera description
protected:
  bool cameraPositionInitialized;
  //! internal camera position
  vpHomogeneousMatrix cMf;
  //! internal camera parameters
  vpCameraParameters internalCameraParameters;
  //! internal camera parameters
  vpCameraParameters externalCameraParameters;

public:
  //! set the camera position (from an homogeneous matrix)
  void setCameraPosition(vpHomogeneousMatrix &cMf);
  //! get the camera position (from an homogeneous matrix)
  void getCameraPosition(vpHomogeneousMatrix &_cMf) { _cMf = cMf; }
  //! modify the position of the camera in the scene graph
  void moveInternalCamera(vpHomogeneousMatrix &cMf);
  //! set internal camera parameters
  void setInternalCameraParameters(vpCameraParameters &cam);
  //! set external camera parameters
  void setExternalCameraParameters(vpCameraParameters &cam);
  //! get the external camera position
  void getExternalCameraPosition(vpHomogeneousMatrix &cMf);

  //! get an Image of the internal view
  void getInternalImage(vpImage<unsigned char> &I);
  //! get an Image of the internal view
  void getInternalImage(vpImage<vpRGBa> &I);
  /* --- Off screen rendering  --- */

  void changeZoomFactor(const float zoom, const int index);

public:
  typedef enum { INTERNAL, EXTERNAL } vpSimulatorViewType;
#ifdef VISP_HAVE_MODULE_IO
  void write(const char *fileName);
#endif
protected:
  SbTime *realtime;
  SoOffscreenRenderer *offScreenRenderer;
  void offScreenRendering(vpSimulatorViewType view = vpSimulator::EXTERNAL, int *width = NULL, int *height = NULL);

public:
  //! image of the internal view
  unsigned char *bufferView;

  //! Flag to protect the read and write of the framebuffer (between the
  //! simulator and the viewer).
  int get;

public:
  //! get the image corresponding to the internal view
  unsigned char *getBufferedOffScreenRenderer() { return bufferView; }

  //! get the size of the internal view
  void getSizeInternalView(int &width, int &height);

  //! get the intrinsic parameters of the camera
  void getCameraParameters(vpCameraParameters &cam) { cam = internalCameraParameters; }
};

#endif
#endif
