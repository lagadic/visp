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
 * Augmented Reality viewer using Ogre3D.
 *
 * Authors:
 * Bertrand Delabarre
 *
 *****************************************************************************/

/*!
  \file vpAROgre.h

  \brief Class that implements augmented reality viewer based on Ogre3D.

  \warning The content of this file is only available if Ogre3D and
  one of the renderer (OpenGL or DirectX) are installed.

*/

#ifndef __VP_AROGRE__
#define __VP_AROGRE__

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_OGRE
#include <list>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>

#include <Ogre.h>
#include <OgreFrameListener.h>

#ifdef VISP_HAVE_OIS
#include <OIS.h>
#endif

/*!
  \class vpAROgre

  \ingroup group_ar_renderer

  \brief Implementation of an augmented reality viewer using Ogre3D 3rd party.

  Thus to be enabled this class requires Ogre3D 3rd party installation.
  Installation instructions are provided here https://visp.inria.fr/3rd_ogre.

  This class provides methods to show a 3D scene in a real world. To do that
  you will need to initialise it with the parameters of your camera, then each
  frame you will need to compute a pose for your camera and give it to the
  application.  With that information and the image to be shown in background
  it will set up the 3D scene correspondingly.

*/
class VISP_EXPORT vpAROgre : public Ogre::FrameListener,
                             public Ogre::WindowEventListener
#ifdef VISP_HAVE_OIS
  ,
                             public OIS::KeyListener
#endif
{
public:
  vpAROgre(const vpCameraParameters &cam = vpCameraParameters(), unsigned int width = 0, unsigned int height = 0,
           const char *resourcePath =
#ifdef VISP_HAVE_OGRE_RESOURCES_PATH
               VISP_HAVE_OGRE_RESOURCES_PATH,
#else
               ".",
#endif
           const char *pluginsPath =
#ifdef VISP_HAVE_OGRE_PLUGINS_PATH
               VISP_HAVE_OGRE_PLUGINS_PATH
#else
               "."
#endif
  );

  virtual ~vpAROgre(void);

  /*!
    Add optional resource location. Since a resource file cannot be always
    sufficient to manage multiple location media (depending on the computer
    and the executable path), this method may be used to add such paths.

    \warning To be effective, this method must be called before the init()
    one.

    \param resourceLocation : The resource location (it may be a folder or a
    zip file).
  */
  inline void addResource(const std::string &resourceLocation)
  {
    mOptionnalResourceLocation.push_back(resourceLocation);
  }

  void addRotation(const std::string &sceneName, const vpRotationMatrix &wRo);

  bool continueRendering(void);

  virtual bool customframeStarted(const Ogre::FrameEvent &evt);

  virtual bool customframeEnded(const Ogre::FrameEvent &evt);

  virtual void display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMw);

  virtual void display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMw);

  inline Ogre::Camera *getCamera() { return mCamera; }

  /*!
    Get the far distance for clipping.

    \return Far clipping value.
  */
  inline double getFarClippingDistance() const { return mFarClipping; }

  /*!
    Get the near distance for clipping.

    \return Near clipping value.
  */
  inline double getNearClippingDistance() const { return mNearClipping; }

  vpTranslationVector getPosition(const std::string &sceneName) const;

  void getRenderingOutput(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo);

  inline Ogre::SceneManager *getSceneManager() { return mSceneMgr; }

  virtual void init(vpImage<unsigned char> &I, bool bufferedKeys = false, bool hidden = false);
  virtual void init(vpImage<vpRGBa> &I, bool bufferedKeys = false, bool hidden = false);

  /*!
    Test if the window is hidden or not.

    \warning True if the window is hidden, false otherwise.
  */
  bool isWindowHidden() { return windowHidden; }

#ifdef VISP_HAVE_OIS
  /**
   * Default event handler
   */
  virtual bool keyPressed(const OIS::KeyEvent & /*e*/) { return true; }
  /**
   * Default event handler
   */
  virtual bool keyReleased(const OIS::KeyEvent & /*e*/) { return true; }
#endif

  void load(const std::string &entityName, const std::string &model);

  bool renderOneFrame(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMw);

  bool renderOneFrame(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMw);

  void setCameraParameters(const vpCameraParameters &cameraP);

  /*!
    Set the far distance for clipping.

    \param dist : Far clipping value.
  */
  void setFarClippingDistance(const double &dist)
  {
    mFarClipping = dist;
    updateCameraProjection();
  }

  /*!
    Set the near distance for clipping.

    \param dist : Near clipping value.
  */
  void setNearClippingDistance(const double &dist)
  {
    mNearClipping = dist;
    updateCameraProjection();
  }

  /*!
    Set the plugins path used to locate the plugins.cfg file.

    By default, this path is set to VISP_HAVE_OGRE_PLUGINS_PATH defined in
    vpConfig.h

    \warning To be effective, this method must be called before the init()
    one.

    \param pluginsPath : The new plugins path (must not have a terminate /).
  */
  inline void setPluginsPath(const char *pluginsPath) { mPluginsPath = pluginsPath; }

  void setPosition(const std::string &sceneName, const vpTranslationVector &wTo);
  void setPosition(const std::string &sceneName, const vpHomogeneousMatrix &wMo);

  /*!
    Set the resource path used to locate the resource.cfg file.

    By default, this path is set to VISP_HAVE_OGRE_RESOURCES_PATH defined in
    vpConfig.h

    \warning To be effective, this method must be called before the init()
    one.

    \param resourcePath : The new resource path (must not have a terminate /).
  */
  inline void setResourcePath(const char *resourcePath) { mResourcePath = resourcePath; }

  void setRotation(const std::string &sceneName, const vpRotationMatrix &wRo);

  void setScale(const std::string &sceneName, const float factorx, const float factory, const float factorz);

  /*!
    Enable/Disable the appearance of the config dialog on startup.

    \warning To be effective, this method must be called before the init()
    one.

    \param showConfigDialog : if true, shows the dialog window (used to set
    the display options)
  */
  inline void setShowConfigDialog(const bool showConfigDialog) { mshowConfigDialog = showConfigDialog; }

  void setVisibility(const std::string &sceneName, bool isVisible);

  /*!
    Set the name of the window.

    \warning Has to be called before initialisation.

    \param n : Name of the window.
  */
  inline void setWindowName(const Ogre::String &n) { name = n; }

  /*!
    Set the window position in the screen.

    \throw vpException::notInitialised if the window has not been created
    (using the init() method).

    \param win_x : x coordinate of the new top left corner of the window in
    the screen. \param win_y : y coordinate of the new top left corner of the
    window in the screen.
  */
  inline void setWindowPosition(const unsigned int win_x, const unsigned int win_y)
  {
    if (mWindow == NULL) {
      throw vpException(vpException::notInitialized, "Window not initialised, cannot set its position");
    }
    mWindow->reposition(static_cast<int>(win_x), static_cast<int>(win_y));
  }

  virtual void windowClosed(Ogre::RenderWindow *rw);

protected:
  virtual void init(bool bufferedKeys = false, bool hidden = false);
  virtual void createCamera(void);

  /**
   * Build the 3D scene
   * Override this to show what you want
   */
  virtual void createScene(void){};

  virtual void closeOIS(void);

  /*!
    Update the 3D scene

    \return Always true.
   */
  virtual bool updateScene(const Ogre::FrameEvent & /*evt*/) { return true; };

  /*!
    Check for keyboard, mouse and joystick inputs.

    \return Always true.
  */
  virtual bool processInputEvent(const Ogre::FrameEvent & /*evt*/) { return true; };

  /*!
    Clean up the 3D scene.

    \return Always true.
  */
  virtual bool destroyScene(void)
  {
    if (!mSceneMgr)
      return false;

    mSceneMgr->destroyAllCameras();
    mSceneMgr->clearScene();
    mRoot->destroySceneManager(mSceneMgr);
    return true;
  }

  virtual void updateCameraParameters(const vpHomogeneousMatrix &cMo);

  virtual void updateCameraProjection(void);

  virtual void updateBackgroundTexture(const vpImage<unsigned char> &I);

  virtual void updateBackgroundTexture(const vpImage<vpRGBa> &I);

private:
  void createBackground(vpImage<unsigned char> &I);
  void createBackground(vpImage<vpRGBa> &I);

  bool frameStarted(const Ogre::FrameEvent &evt);

  bool frameEnded(const Ogre::FrameEvent &evt);

  bool stopTest(const Ogre::FrameEvent &evt);

protected:
  // Attributes
  Ogre::String name; /**Name of th Window*/

  // OGRE 3D System
  Ogre::Root *mRoot;             /** Application's root */
  Ogre::Camera *mCamera;         /** Camera */
  Ogre::SceneManager *mSceneMgr; /** Scene manager */
  Ogre::RenderWindow *mWindow;   /** Display window */
  Ogre::String mResourcePath;    /** Path to resources.cfg */
  Ogre::String mPluginsPath;     /** Path to plugins.cfg */

#ifdef VISP_HAVE_OIS
  // OIS Input manager and devices
  OIS::InputManager *mInputManager;
  OIS::Keyboard *mKeyboard;
#endif

  // ViSP AR System
  bool keepOn;                                     /** Has the application received a signal to stop(false) or not
                                                      (true) */
  vpImage<vpRGBa> mImageRGBA;                      /** vpImage to store grabbed image */
  vpImage<unsigned char> mImage;                   /** vpImage to store grabbed image */
  Ogre::HardwarePixelBufferSharedPtr mPixelBuffer; /** Pointer to the pixel buffer */
  Ogre::Rectangle2D *mBackground;                  /** Background image */
  unsigned int mBackgroundHeight;                  /** Height of the acquired image */
  unsigned int mBackgroundWidth;                   /** Width of the acquired image */
  unsigned int mWindowHeight;                      /** Height of the window */
  unsigned int mWindowWidth;                       /** Width of the window */
  bool windowHidden;                               /** Is window hidden */

  // Camera calculations
  double mNearClipping;    /** Near Clipping Distance **/
  double mFarClipping;     /** Far Clipping Distance **/
  vpCameraParameters mcam; /** The intrinsic camera parameters */

  bool mshowConfigDialog; /** if true, shows the dialog window (used to set
                             the display options) */

  std::list<std::string> mOptionnalResourceLocation; /** Optional resource location (used to
                                                        load mesh and material) */
};

#endif // VISP_HAVE_OGRE

#endif
