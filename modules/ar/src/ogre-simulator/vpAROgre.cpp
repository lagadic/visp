/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
  \file vpAROgre.cpp

  \brief Class that implements augmented reality viewer based on Ogre3D.

  \warning The content of this file is only available if Ogre3D and
  one of the renderer (OpenGL or Direct3Dt) are installed.

*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_OGRE

#include <visp3/ar/vpAROgre.h>
#include <visp3/core/vpIoTools.h>

#include <OgreRectangle2D.h>

/*!
  Constructor.

  \param cam : Camera parameters.

  \param width : Width of the window.
  \param height : Height of the window.

  \param resourcePath : Path to the resources.cfg file telling Ogre where to
  look for resources.

  \param pluginsPath : Path to the plugins.cfg of plugins_d.cfg files
  telling Ogre where to look for renderer plugins.

*/
vpAROgre::vpAROgre(const vpCameraParameters &cam, unsigned int width, unsigned int height, const char *resourcePath,
                   const char *pluginsPath)
  : name("ViSP - Augmented Reality"), mRoot(0), mCamera(0), mSceneMgr(0), mWindow(0), mResourcePath(resourcePath),
    mPluginsPath(pluginsPath),
#ifdef VISP_HAVE_OIS
    mInputManager(0), mKeyboard(0),
#endif
    keepOn(true), // When created no reason to stop displaying
    mImageRGBA(), mImage(), mPixelBuffer(), mBackground(NULL), mBackgroundHeight(0), mBackgroundWidth(0),
    mWindowHeight(height), mWindowWidth(width), windowHidden(false), mNearClipping(0.001), mFarClipping(200), mcam(cam),
    mshowConfigDialog(true), mOptionnalResourceLocation()
{
}

/*!
  Initialisation of Ogre with a grey level background.

  Load the plugins that are specified in the plugins.cfg or
  plugins_d.cfg files. These files are located in
  VISP_HAVE_OGRE_PLUGINS_PATH folder that is defined in vpConfig.h.
  Note that plugins.cfg file is always considered under Unix
  platforms. The file plugins_d.cfg is only considered under Windows
  when the build type is Debug.

  Load also the resources that are defined in the resources.cfg
  file. This file is located in VISP_HAVE_OGRE_RESOURCES_PATH folder
  that is defined in vpConfig.h.

  Create also the grey level background used to display the image.

  \param I : Image that is displayed in the background.

  \param bufferedKeys : If true, use of buffered input for the keyboard (see
  Ogre documentation). Note that this parameter is only useful if OIS is used.

  \param hidden : If true, the created window will be hidden.
  Note that this functionnality requires Ogre3D 1.8.1 at least.

  \exception vpException::ioError : If the required plugins.cfg /
  plugins_d.cfg or resources.cfg files are not accessible.

*/
void vpAROgre::init(vpImage<unsigned char> &I,
                    bool
#ifdef VISP_HAVE_OIS
                        bufferedKeys
#endif
                    ,
                    bool hidden)
{
  mBackgroundWidth = I.getWidth();
  mBackgroundHeight = I.getHeight();

  init(
#ifdef VISP_HAVE_OIS
      bufferedKeys,
#else
      false,
#endif
      hidden);
  // Create the background image which will come from the grabber
  createBackground(I);
}

/*!
  Initialisation of Ogre with a color background.

  Load the plugins that are specified in the plugins.cfg or
  plugins_d.cfg files. These files are located in
  VISP_HAVE_OGRE_PLUGINS_PATH folder that is defined in vpConfig.h.
  Note that plugins.cfg file is always considered under Unix
  platforms. The file plugins_d.cfg is only considered under Windows
  when the build type is Debug.

  Load also the resources that are defined in the resources.cfg
  file. This file is located in VISP_HAVE_OGRE_RESOURCES_PATH folder
  that is defined in vpConfig.h.

  Create also a color background used to display the image.

  \param I : Image that is displayed in the background.

  \param bufferedKeys : If true, use of buffered input for the keyboard (see
  Ogre documentation). Note that this parameter is only useful if OIS is used.

  \param hidden : If true, the created window will be hidden.
  Note that this functionnality requires Ogre3D 1.8.1 at least.

  \exception vpException::ioError : If the required plugins.cfg /
  plugins_d.cfg or resources.cfg files are not accessible.

*/
void vpAROgre::init(vpImage<vpRGBa> &I,
                    bool
#ifdef VISP_HAVE_OIS
                        bufferedKeys
#endif
                    ,
                    bool hidden)
{
  mBackgroundWidth = I.getWidth();
  mBackgroundHeight = I.getHeight();

  init(
#ifdef VISP_HAVE_OIS
      bufferedKeys,
#else
      false,
#endif
      hidden);
  // Create the background image which will come from the grabber
  createBackground(I);
}

/*!
  Initialisation of Ogre.

  Load the plugins that are specified in the plugins.cfg or
  plugins_d.cfg files. These files are located in
  VISP_HAVE_OGRE_PLUGINS_PATH folder that is defined in vpConfig.h.
  Note that plugins.cfg file is always considered under Unix
  platforms. The file plugins_d.cfg is only considered under Windows
  when the build type is Debug.

  Load also the resources that are defined in the resources.cfg
  file. This file is located in VISP_HAVE_OGRE_RESOURCES_PATH folder
  that is defined in vpConfig.h.

  \param bufferedKeys : If true, use of buffered input for the keyboard (see
  Ogre documentation). Note that this parameter is only useful if OIS is used.

  \param hidden : If true, the created window will be hidden.
  Note that this functionnality requires Ogre3D 1.8.1 at least.

  \exception vpException::ioError : If the required plugins.cfg /
  plugins_d.cfg or resources.cfg files are not accessible.

*/
void vpAROgre::init(bool
#ifdef VISP_HAVE_OIS
                        bufferedKeys
#endif
                    ,
                    bool hidden)
{
  // Create the root
  // mPluginsPath may contain more than one folder location separated by ";"
  bool pluginsFileExists = false;
  std::string pluginFile;
  std::vector<std::string> plugingsPaths = vpIoTools::splitChain(std::string(mPluginsPath), std::string(";"));
  for (size_t i = 0; i < plugingsPaths.size(); i++) {
#if defined(NDEBUG) || !defined(_WIN32)
    pluginFile = plugingsPaths[i] + "/plugins.cfg";
#else
    pluginFile = plugingsPaths[i] + "/plugins_d.cfg";
#endif

    if (vpIoTools::checkFilename(pluginFile)) {
      pluginsFileExists = true;
      break;
    }
  }
  if (!pluginsFileExists) {
    std::string errorMsg = std::string("Error: the requested plugins file \"")
#if defined(NDEBUG) || !defined(_WIN32)
                           + std::string("plugins.cfg")
#else
                           + std::string("plugins_d.cfg")
#endif
                           + std::string("\" doesn't exist in ") + std::string(mPluginsPath);
    std::cout << errorMsg << std::endl;

    throw(vpException(vpException::ioError, errorMsg));
  }
  std::cout << "######################### Load plugin file: " << pluginFile << std::endl;

  if (Ogre::Root::getSingletonPtr() == NULL)
    mRoot = new Ogre::Root(pluginFile, "ogre.cfg", "Ogre.log");
  else
    mRoot = Ogre::Root::getSingletonPtr();

  // Load resource paths from config file

  // File format is:
  //  [ResourceGroupName]
  //  ArchiveType=Path
  //  .. repeat
  // For example:
  //  [General]
  //  FileSystem=media/
  //  Zip=packages/level1.zip

  // mResourcePath may contain more than one folder location separated by ";"
  bool resourcesFileExists = false;
  std::string resourceFile;
  std::vector<std::string> resourcesPaths = vpIoTools::splitChain(std::string(mResourcePath), std::string(";"));
  for (size_t i = 0; i < resourcesPaths.size(); i++) {
    resourceFile = resourcesPaths[i] + "/resources.cfg";
    if (vpIoTools::checkFilename(resourceFile)) {
      resourcesFileExists = true;
      break;
    }
  }
  if (!resourcesFileExists) {
    std::string errorMsg = std::string("Error: the requested resource file \"resources.cfg\"") +
                           std::string("doesn't exist in ") + std::string(mResourcePath);

    std::cout << errorMsg << std::endl;

    throw(vpException(vpException::ioError, errorMsg));
  }
  std::cout << "######################### Load resource file: " << resourceFile << std::endl;
  Ogre::ConfigFile cf;
  cf.load(resourceFile);

  // Go through all sections & settings in the file
  Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

  Ogre::String secName, typeName, archName;
  while (seci.hasMoreElements()) {
    secName = seci.peekNextKey();
    Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
    Ogre::ConfigFile::SettingsMultiMap::iterator i;
    for (i = settings->begin(); i != settings->end(); ++i) {
      typeName = i->first;
      archName = i->second;
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
    }
  }
  std::cout << "##################### add resources" << std::endl;
  // Add optionnal resources (given by the user).
  for (std::list<std::string>::const_iterator iter = mOptionnalResourceLocation.begin();
       iter != mOptionnalResourceLocation.end(); ++iter) {
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        *iter, "FileSystem", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  // Create the window
  bool canInit = true;
  if (mshowConfigDialog) {
    mRoot->restoreConfig();
    if (!mRoot->showConfigDialog())
      canInit = false;
  } else {
    if (!mRoot->restoreConfig())
      canInit = false;
  }

  if (!mRoot->isInitialised()) {
    if (!canInit) { // We set the default renderer system
      const Ogre::RenderSystemList &lRenderSystemList = mRoot->getAvailableRenderers();
      if (lRenderSystemList.size() == 0)
        throw "ConfigDialog aborted"; // Exit the application on cancel

      Ogre::RenderSystem *lRenderSystem = lRenderSystemList.at(0);
      std::cout << "Using " << lRenderSystem->getName() << " as renderer." << std::endl;
      mRoot->setRenderSystem(lRenderSystem);
    }

    mRoot->initialise(false);
  }

  bool fullscreen = false;
  Ogre::NameValuePairList misc;
  Ogre::ConfigOptionMap config = mRoot->getRenderSystem()->getConfigOptions();
  Ogre::ConfigOptionMap::const_iterator it = config.begin();

  while (it != config.end()) {
    Ogre::String leftconf = (*it).first;
    Ogre::String rightconf = (*it).second.currentValue;

    if (leftconf == "Video Mode") {
      if (canInit) {
        std::stringstream ss(rightconf.c_str());
        std::string dummy;
        ss >> mWindowWidth >> dummy >> mWindowHeight;
        if (ss.fail())
          std::cout << "Cannot read Ogre video mode" << std::endl;
      } else {
        if (mWindowWidth == 0 && mWindowHeight == 0) {
          mWindowWidth = mBackgroundWidth;
          mWindowHeight = mBackgroundHeight;
        }
      }
    } else if (leftconf == "Full Screen") {
      if (canInit) {
        if (rightconf == "Yes")
          fullscreen = true;
      }
    } else
      misc[leftconf] = rightconf;

    ++it;
  }

  // With Ogre version >= 1.8.1 we hide the window
  if (hidden) {
#if (OGRE_VERSION >= (1 << 16 | 8 << 8 | 1))
    misc["hidden"] = "true";
    windowHidden = true;
#endif
  }
  mWindow = mRoot->createRenderWindow(name, mWindowWidth, mWindowHeight, fullscreen, &misc);

  // Initialise resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
  //-----------------------------------------------------
  // 4 Create the SceneManager
  //
  //		ST_GENERIC = octree
  //		ST_EXTERIOR_CLOSE = simple terrain
  //		ST_EXTERIOR_FAR = nature terrain (depreciated)
  //		ST_EXTERIOR_REAL_FAR = paging landscape
  //		ST_INTERIOR = Quake3 BSP
  //-----------------------------------------------------

  mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

  // Create the camera
  createCamera();

  // Create a viewport
  Ogre::Viewport *viewPort = mWindow->addViewport(mCamera);
  //   Ogre::Viewport* viewPort = mCamera->getViewport();
  viewPort->setClearEveryFrame(true);
  // Set the projection parameters to match the camera intrinsic parameters
  updateCameraProjection();

  // Create the 3D scene
  createScene();

  // Initialise and register event handlers
  mRoot->addFrameListener(this);

  // Register as a Window listener
  Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

#ifdef VISP_HAVE_OIS
  // Initialise OIS
  Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
  OIS::ParamList pl;

  size_t windowHnd = 0;
  std::ostringstream windowHndStr;
  // Initialise window
  mWindow->getCustomAttribute("WINDOW", &windowHnd);
  windowHndStr << windowHnd;
  pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
// Let the user use the keyboard elsewhere
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
#endif

  mInputManager = OIS::InputManager::createInputSystem(pl);

  // Create all devices
  // Here we only consider the keyboard input
  mKeyboard = static_cast<OIS::Keyboard *>(mInputManager->createInputObject(OIS::OISKeyboard, bufferedKeys));
  if (!bufferedKeys)
    mKeyboard->setEventCallback(this);
#endif

  // Initialise a render to texture to be able to retrieve a screenshot
  Ogre::TexturePtr Texture = Ogre::TextureManager::getSingleton().createManual(
      "rtf", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, mWindow->getWidth(),
      mWindow->getHeight(), 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);

  //   Ogre::TexturePtr Texture =
  //   Ogre::TextureManager::getSingleton().createManual("rtf",
  //   Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,Ogre::TEX_TYPE_2D,
  //                                                                                640,480, 0, Ogre::PF_R8G8B8A8,
  //                                                                                Ogre::TU_RENDERTARGET);
  Ogre::RenderTexture *RTarget = Texture->getBuffer()->getRenderTarget();
  /*Ogre::Viewport* Viewport =*/RTarget->addViewport(mCamera);
  RTarget->getViewport(0)->setClearEveryFrame(true);
  RTarget->getViewport(0)->setOverlaysEnabled(false);
}

/*!
  Destructor.
*/
vpAROgre::~vpAROgre(void)
{
  // Destroy 3D scene
  destroyScene();
  // Close OIS
  closeOIS();

  if (mWindow) {
    Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
    windowClosed(mWindow);
  }

  // Delete root
  if (Ogre::Root::getSingletonPtr() && !Ogre::Root::getSingletonPtr()->getSceneManagerIterator().hasMoreElements()) {
    if (mRoot) {
      delete mRoot;
    }
  }
  mRoot = 0;
}

/*!
  Function testing if the program must stop rendering or not.
  \param evt : Frame event to process.
  \return False if the program must be stopped.
*/
bool vpAROgre::stopTest(const Ogre::FrameEvent &evt)
{
  // Always keep this part
  if (keepOn) {
    return updateScene(evt);
  } else
    return keepOn;
}

/*!
  Function telling what to do before each frame.
  Calls customframeStarted() and see if the program has to be stopped.

  This function is called before every frame is rendered by Ogre.

  \param evt : Frame event to process.
  \return True if everything went well.
*/
bool vpAROgre::frameStarted(const Ogre::FrameEvent &evt)
{
  // custom method telling what to do at the beginning of each frame
  bool result = customframeStarted(evt);

  // Listen to the window
  Ogre::WindowEventUtilities::messagePump();
  processInputEvent(evt);

  // See if we have to stop rendering
  if (result)
    return stopTest(evt);
  else
    return result;
}

/*!
  Function telling what to do after each frame.
  Calls customframeEnded() and see if the program is to be stopped.
  \param evt : Frame event to process.
  \return True if everything went well.
*/
bool vpAROgre::frameEnded(const Ogre::FrameEvent &evt)
{
  // custom method telling what to do at the end of each frame
  bool result = customframeEnded(evt);

  // See if we have to stop rendering
  if (result)
    return stopTest(evt);
  else
    return result;
}

/*!

  Function telling what to do before each frame. This method is called
  before every frame rendered by Ogre.

  \param evt : Frame event to process
  \return True if everything went well
*/
bool vpAROgre::customframeStarted(const Ogre::FrameEvent & /*evt*/)
{
  // See if window was closed
  if (mWindow->isClosed())
    return false;

#ifdef VISP_HAVE_OIS
  // Get keyboard input
  mKeyboard->capture();
  if (mKeyboard->isKeyDown(OIS::KC_ESCAPE))
    return false;
#endif
  return true;
}

/*!
  Function telling what to do after each frame.
  \param evt : Frame event to process.
  \return True if everything went well.
*/
bool vpAROgre::customframeEnded(const Ogre::FrameEvent & /*evt*/) { return true; }

/*!

  Unattach OIS (Object-oriented Input System library) before window shutdown
  (very important under Linux).

  \warning This method is only available if OIS is installed and detected
  during ViSP configuration stage.

  \param rw : RenderWindow to close.
*/
void vpAROgre::windowClosed(Ogre::RenderWindow *rw)
{
  // Only close for window that created OIS (the main window in these demos)
  if (rw == mWindow)
    closeOIS();
}

/*!
  Render one frame.
  \param I : Grey level image to show in background.
  \param cMw : Camera pose as an homogeneous matrix.
*/
bool vpAROgre::renderOneFrame(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMw)
{
  // Update the background to match the situation
  updateBackgroundTexture(I);

  // Update the camera parameters to match the grabbed image
  updateCameraParameters(cMw);

  // Display on Ogre Window
  return mRoot->renderOneFrame();
}

/*!
  Render one frame.
  \param I : RGBa image to show in background.
  \param cMw : Camera pose as an homogeneous matrix.
*/
bool vpAROgre::renderOneFrame(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMw)
{
  // Update the background to match the situation
  updateBackgroundTexture(I);

  // Update the camera parameters to match the grabbed image
  updateCameraParameters(cMw);

  // Display on Ogre Window
  return mRoot->renderOneFrame();
}

/*!
  Display a frame.
  \param I : Grey level image to show in background.
  \param cMw : Camera pose as an homogeneous matrix.
*/
void vpAROgre::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMw)
{
  // Display on Ogre Window
  if (renderOneFrame(I, cMw)) {
    mWindow->update();
    keepOn = true;
  } else
    keepOn = false;
}

/*!
  Display a frame.
  \param I : RGBa image to show in background.
  \param cMw : Camera pose as an homogeneous matrix.
*/
void vpAROgre::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMw)
{
  // Display on Ogre Window
  if (renderOneFrame(I, cMw)) {
    mWindow->update();
    keepOn = true;
  } else
    keepOn = false;
}

/*!
  Ask the program if we can continue to render.
  \return True if nothing stopped the rendering loop.
*/
bool vpAROgre::continueRendering(void) { return keepOn; }

/*!
  Set the camera intrinsic parameters
*/
void vpAROgre::setCameraParameters(const vpCameraParameters &cameraP) { mcam = cameraP; }

/*!
  Load a mesh in the 3D world.
  \param entityName : Name of the Entity and SceneNode to create.
  \param model : 3D model to load.
*/
void vpAROgre::load(const std::string &entityName, const std::string &model)
{
  Ogre::Entity *newEntity = mSceneMgr->createEntity(entityName, model);
  Ogre::SceneNode *newNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(entityName);
  newNode->attachObject(newEntity);
}

/*!
  Change position of a ScneneNode.
  \param sceneName : Name of the SceneNode to move.
  \param wTo : New position of the node (translation between object frame and
    world frame).
*/
void vpAROgre::setPosition(const std::string &sceneName, const vpTranslationVector &wTo)
{
  // Reset the position
  Ogre::SceneNode *node = mSceneMgr->getSceneNode(sceneName);
  node->setPosition((Ogre::Real)wTo[0], (Ogre::Real)wTo[1], (Ogre::Real)wTo[2]);
}

/*!
  Get position of a SceneNode.
  \param sceneName : Name of the SceneNode in the scene graph.
  \return The position of the node.
*/
vpTranslationVector vpAROgre::getPosition(const std::string &sceneName) const
{
  Ogre::Vector3 translation = mSceneMgr->getSceneNode(sceneName)->getPosition();
  return vpTranslationVector((Ogre::Real)translation[0], (Ogre::Real)translation[1], (Ogre::Real)translation[2]);
}

/*!
  Set the orientation of a SceneNode.
  \param sceneName : Name of the SceneNode to rotate.
  \param wRo : The rotation matrix representing the rotation to apply.
*/
void vpAROgre::setRotation(const std::string &sceneName, const vpRotationMatrix &wRo)
{
  // Get the node in its original position
  mSceneMgr->getSceneNode(sceneName)->resetOrientation();
  // Apply the new rotation
  Ogre::Matrix3 rotationOgre = Ogre::Matrix3((Ogre::Real)wRo[0][0], (Ogre::Real)wRo[0][1], (Ogre::Real)wRo[0][2],
                                             (Ogre::Real)wRo[1][0], (Ogre::Real)wRo[1][1], (Ogre::Real)wRo[1][2],
                                             (Ogre::Real)wRo[2][0], (Ogre::Real)wRo[2][1], (Ogre::Real)wRo[2][2]);
  Ogre::Quaternion q(rotationOgre);
  mSceneMgr->getSceneNode(sceneName)->rotate(q);
}

/*!
  Add a rotation to a SceneNode.
  \param sceneName : Name of the SceneNode to rotate.
  \param wRo : The rotation matrix representing the rotation to apply.
*/
void vpAROgre::addRotation(const std::string &sceneName, const vpRotationMatrix &wRo)
{
  // Apply the new rotation
  Ogre::Matrix3 rotationOgre = Ogre::Matrix3((Ogre::Real)wRo[0][0], (Ogre::Real)wRo[0][1], (Ogre::Real)wRo[0][2],
                                             (Ogre::Real)wRo[1][0], (Ogre::Real)wRo[1][1], (Ogre::Real)wRo[1][2],
                                             (Ogre::Real)wRo[2][0], (Ogre::Real)wRo[2][1], (Ogre::Real)wRo[2][2]);
  Ogre::Quaternion q(rotationOgre);
  mSceneMgr->getSceneNode(sceneName)->rotate(q);
}

/*!
  Set the position and the orientation of a SceneNode.
  \param sceneName : Name of the SceneNode to rotate.

  \param wMo : The homogeneous matrix representing the rotation and
  translation to apply.

*/
void vpAROgre::setPosition(const std::string &sceneName, const vpHomogeneousMatrix &wMo)
{
  // Extract the position and orientation data
  vpRotationMatrix rotations;
  vpTranslationVector translation;
  wMo.extract(rotations);
  wMo.extract(translation);
  // Apply them to the node
  setPosition(sceneName, translation);
  setRotation(sceneName, rotations);
}

/*!
  Tell if a SceneNode is shown on the screen or not.
  \param sceneName : Name of the SceneNode.
  \param isVisible : If true we show the node, if false we don't.
*/
void vpAROgre::setVisibility(const std::string &sceneName, bool isVisible)
{
  mSceneMgr->getSceneNode(sceneName)->setVisible(isVisible);
}

/*!
  Scale a SceneNode.
  \param sceneName : Name of the SceneNode.
  \param factorx : Scale factor along the x-axis.
  \param factory : Scale factor along the x-axis.
  \param factorz : Scale factor along the x-axis.
*/
void vpAROgre::setScale(const std::string &sceneName, const float factorx, const float factory, const float factorz)
{
  // Reset the scale to its original value
  mSceneMgr->getSceneNode(sceneName)->scale(Ogre::Vector3(1, 1, 1) / mSceneMgr->getSceneNode(sceneName)->getScale());
  // Apply the new scale
  mSceneMgr->getSceneNode(sceneName)->scale(Ogre::Vector3(factorx, factory, factorz));
}

/*!
  Create the Ogre camera.
*/
void vpAROgre::createCamera(void) { mCamera = mSceneMgr->createCamera("Camera"); }

/*!
  Create a greylevel background to show the real scene.

  \param I : This parameter is here only used to initialize a grey level
  background.
*/
void vpAROgre::createBackground(vpImage<unsigned char> & /* I */)
{
  // Create a rectangle to show the incoming images from the camera
  mBackground = new Ogre::Rectangle2D(true);     // true = textured
  mBackground->setCorners(-1.0, 1.0, 1.0, -1.0); // Spread all over the window
  mBackground->setBoundingBox(Ogre::AxisAlignedBox(-100000.0 * Ogre::Vector3::UNIT_SCALE,
                                                   100000.0 * Ogre::Vector3::UNIT_SCALE)); // To be shown everywhere

  // Texture options
  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_NONE);
  Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(1);

  // Dynamic texture
  // If we are using opengl we can boost a little bit performances with a
  // dynamic texture
  if (mRoot->getRenderSystem()->getName() == "OpenGL Rendering Subsystem") {
    Ogre::TextureManager::getSingleton().createManual(
        "BackgroundTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        mBackgroundWidth,  // width
        mBackgroundHeight, // height
        0,                 // num of mip maps
        Ogre::PF_BYTE_L, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  } else {
    Ogre::TextureManager::getSingleton().createManual(
        "BackgroundTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        mBackgroundWidth,  // width
        mBackgroundHeight, // height
        0,                 // num of mip maps
        Ogre::PF_BYTE_L, Ogre::TU_DEFAULT);
  }

  // Pointer to the dynamic texture
  Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("BackgroundTexture");
  //#if ( OGRE_VERSION >= (1 << 16 | 9 << 8 | 0) )
  //    .dynamicCast<Ogre::Texture>();// Get the pixel buffer
  //#else
  //      ;
  //#endif
  mPixelBuffer = dynTexPtr->getBuffer();

  // Material to apply the texture to the background
  Ogre::MaterialPtr Backgroundmaterial = Ogre::MaterialManager::getSingleton().create(
      "BackgroundMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  //#if ( OGRE_VERSION >= (1 << 16 | 9 << 8 | 0) )
  //      .dynamicCast<Ogre::Material>();
  //#else
  //      ;
  //#endif
  Ogre::Technique *Backgroundtechnique = Backgroundmaterial->createTechnique();
  Backgroundtechnique->createPass();
  Backgroundmaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false); // Background
  Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false); // Background
  Backgroundmaterial->getTechnique(0)->getPass(0)->createTextureUnitState("BackgroundTexture");
  mBackground->setMaterial("BackgroundMaterial");                  // Attach the material to the rectangle
  mBackground->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND); // To be rendered in Background

  // Add the background to the Scene Graph so it will be rendered
  Ogre::SceneNode *BackgroundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("BackgoundNode");
  BackgroundNode->attachObject(mBackground);
}

/*!
  Create a color background to show the real scene.

  \param I : This parameter is here only used to initialize a color
  background.
*/
void vpAROgre::createBackground(vpImage<vpRGBa> & /* I */)
{
  // Create a rectangle to show the incoming images from the camera
  mBackground = new Ogre::Rectangle2D(true);     // true = textured
  mBackground->setCorners(-1.0, 1.0, 1.0, -1.0); // Spread all over the window
  mBackground->setBoundingBox(Ogre::AxisAlignedBox(-100000.0 * Ogre::Vector3::UNIT_SCALE,
                                                   100000.0 * Ogre::Vector3::UNIT_SCALE)); // To be shown everywhere

  // Texture options
  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(Ogre::TFO_NONE);
  Ogre::MaterialManager::getSingleton().setDefaultAnisotropy(1);

  // Dynamic texture
  // If we are using opengl we can boost a little bit performances with a
  // dynamic texture
  if (mRoot->getRenderSystem()->getName() == "OpenGL Rendering Subsystem") {
    Ogre::TextureManager::getSingleton().createManual(
        "BackgroundTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        mBackgroundWidth,  // width
        mBackgroundHeight, // height
        0,                 // num of mip maps
        // Ogre::PF_BYTE_RGBA,
        Ogre::PF_BYTE_BGRA, Ogre::TU_DYNAMIC_WRITE_ONLY_DISCARDABLE);
  } else { // As that texture does not seem to work properly with direct3D we
           // use a default texture
    Ogre::TextureManager::getSingleton().createManual(
        "BackgroundTexture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D,
        mBackgroundWidth,  // width
        mBackgroundHeight, // height
        0,                 // num of mip maps
        // Ogre::PF_BYTE_RGBA,
        Ogre::PF_BYTE_BGRA, Ogre::TU_DEFAULT);
  }

  // Pointer to the dynamic texture
  Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("BackgroundTexture");
  //#if ( OGRE_VERSION >= (1 << 16 | 9 << 8 | 0) )
  //      .dynamicCast<Ogre::Texture>();// Get the pixel buffer
  //#else
  //      ;
  //#endif

  // Get the pixel buffer
  mPixelBuffer = dynTexPtr->getBuffer();

  // Material to apply the texture to the background
  Ogre::MaterialPtr Backgroundmaterial = Ogre::MaterialManager::getSingleton().create(
      "BackgroundMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  //#if ( OGRE_VERSION >= (1 << 16 | 9 << 8 | 0) )
  //      .dynamicCast<Ogre::Material>();
  //#else
  //      ;
  //#endif
  Ogre::Technique *Backgroundtechnique = Backgroundmaterial->createTechnique();
  Backgroundtechnique->createPass();
  Backgroundmaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false); // Background
  Backgroundmaterial->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false); // Background
  Backgroundmaterial->getTechnique(0)->getPass(0)->createTextureUnitState("BackgroundTexture");
  mBackground->setMaterial("BackgroundMaterial");                  // Attach the material to the rectangle
  mBackground->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND); // To be rendered in Background

  // Add the background to the Scene Graph so it will be rendered
  Ogre::SceneNode *BackgroundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("BackgoundNode");
  BackgroundNode->attachObject(mBackground);
}

/*!
  Close the OIS based input manager.

  \warning This method is only to call if OIS is installed and detected during
  ViSP configuration stage.

*/
void vpAROgre::closeOIS(void)
{
#ifdef VISP_HAVE_OIS
  if (mInputManager) {
    mInputManager->destroyInputObject(mKeyboard);

    OIS::InputManager::destroyInputSystem(mInputManager);
    mInputManager = 0;
  }
#endif
}

/*!
  Update the projection parameters of the camera.
*/
// Note: equation taken from:
// http://strawlab.org/2011/11/05/augmented-reality-with-OpenGL/
void vpAROgre::updateCameraProjection(void)
{
  if (mCamera != 0) {
    Ogre::Real f, n, f_m_n, f_p_n, px, py, u0, v0;
    f = (Ogre::Real)(mFarClipping);  // Far clip distance
    n = (Ogre::Real)(mNearClipping); // Near clip distance
    f_m_n = (Ogre::Real)(f - n);
    f_p_n = (Ogre::Real)(f + n);
    px = (Ogre::Real)mcam.get_px();
    py = (Ogre::Real)mcam.get_py();
    u0 = (Ogre::Real)mcam.get_u0();
    v0 = (Ogre::Real)mcam.get_v0();
    Ogre::Matrix4 Projection = Ogre::Matrix4(
        (Ogre::Real)(2.0 * px / mBackgroundWidth), 0, (Ogre::Real)(1.0 - 2.0 * (u0 / mBackgroundWidth)), 0, 0,
        (Ogre::Real)(2.0 * py / mBackgroundHeight), (Ogre::Real)(-1.0 + 2.0 * (v0 / mBackgroundHeight)), 0, 0, 0,
        (Ogre::Real)(-1.0 * f_p_n / f_m_n), (Ogre::Real)(-2.0 * f * n / f_m_n), 0, 0, -1.0, 0);
    mCamera->setCustomProjectionMatrix(true, Projection);
  }
}

/*!
  Update the texture we see in background with a grey level image.
*/
void vpAROgre::updateBackgroundTexture(const vpImage<unsigned char> &I)
{
  // Inspired from Ogre wiki :
  // http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures Lock the pixel
  // buffer and get a pixel box. HBL_DISCARD is to use for best performance
  // than HBL_NORMAL
  mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
  const Ogre::PixelBox &pixelBox = mPixelBuffer->getCurrentLock();
  // Buffer data
  Ogre::uint8 *pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
  // Fill in the data in the grey level texture
  memcpy(pDest, I.bitmap, mBackgroundHeight * mBackgroundWidth);

  // Unlock the pixel buffer
  mPixelBuffer->unlock();
}

/*!
  Update the texture we see in background with a RGBa image.
*/
void vpAROgre::updateBackgroundTexture(const vpImage<vpRGBa> &I)
{
  // Inspired from Ogre wiki :
  // http://www.ogre3d.org/tikiwiki/Creating+dynamic+textures Lock the pixel
  // buffer and get a pixel box. HBL_DISCARD is to use for best performance
  // than HBL_NORMAL
  mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD); // Lock the buffer
  const Ogre::PixelBox &pixelBox = mPixelBuffer->getCurrentLock();
  // Buffer data
  Ogre::uint8 *pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
// Fill in the data in the grey level texture
#if 1 // if texture in BGRa format
  for (unsigned int i = 0; i < mBackgroundHeight; i++) {
    for (unsigned int j = 0; j < mBackgroundWidth; j++) {
      // Color Image
      //      *pDest++=I[i][mBackgroundWidth-j].B; // Blue component
      //      *pDest++=I[i][mBackgroundWidth-j].G; // Green component
      //      *pDest++=I[i][mBackgroundWidth-j].R; // Red component

      *pDest++ = I[i][j].B; // Blue component
      *pDest++ = I[i][j].G; // Green component
      *pDest++ = I[i][j].R; // Red component

      *pDest++ = 255; // Alpha component
    }
  }
#else // if texture in RGBa format which is the format of the input image
  memcpy(pDest, I.bitmap, mBackgroundHeight * mBackgroundWidth * sizeof(vpRGBa));
#endif

  // Unlock the pixel buffer
  mPixelBuffer->unlock();
}

/*!
  Update Camera parameters from a pose calculation.
*/
void vpAROgre::updateCameraParameters(const vpHomogeneousMatrix &cMw)
{
  // The matrix is given to Ogre with some changes to fit with the world
  // projection
  Ogre::Matrix4 ModelView
      //    = Ogre::Matrix4( (Ogre::Real)-cMo[0][0],  (Ogre::Real)-cMo[0][1],
      //    (Ogre::Real)-cMo[0][2],  (Ogre::Real)-cMo[0][3],
      = Ogre::Matrix4((Ogre::Real)cMw[0][0], (Ogre::Real)cMw[0][1], (Ogre::Real)cMw[0][2], (Ogre::Real)cMw[0][3],
                      (Ogre::Real)-cMw[1][0], (Ogre::Real)-cMw[1][1], (Ogre::Real)-cMw[1][2], (Ogre::Real)-cMw[1][3],
                      (Ogre::Real)-cMw[2][0], (Ogre::Real)-cMw[2][1], (Ogre::Real)-cMw[2][2], (Ogre::Real)-cMw[2][3],
                      (Ogre::Real)0, (Ogre::Real)0, (Ogre::Real)0, (Ogre::Real)1);
  mCamera->setCustomViewMatrix(true, ModelView);
}

/*!
  Get the result of the rendering loop.

  \param I : The image on which to copy the result of the rendering loop.
  \param cMo : The desired camera pose.
*/
void vpAROgre::getRenderingOutput(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo)
{
  updateCameraParameters(cMo);
  Ogre::TexturePtr dynTexPtr = Ogre::TextureManager::getSingleton().getByName("rtf");
  //#if ( OGRE_VERSION >= (1 << 16 | 9 << 8 | 0) )
  //        .dynamicCast<Ogre::Texture>();
  //#else
  //        ;
  //#endif
  Ogre::RenderTexture *RTarget = dynTexPtr->getBuffer()->getRenderTarget();
  mWindow->update();
  RTarget->update();
  if (I.getHeight() != mWindow->getHeight() || I.getWidth() != mWindow->getWidth()) {
    I.resize(mWindow->getHeight(), mWindow->getWidth());
  }
  Ogre::HardwarePixelBufferSharedPtr mPixelBuffer = dynTexPtr->getBuffer();
  mPixelBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD);
  const Ogre::PixelBox &pixelBox = mPixelBuffer->getCurrentLock();
  dynTexPtr->getBuffer()->blitToMemory(pixelBox);
  Ogre::uint8 *pDest = static_cast<Ogre::uint8 *>(pixelBox.data);
#if 1 // if texture in BGRa format
  for (unsigned int i = 0; i < I.getHeight(); i++) {
    for (unsigned int j = 0; j < I.getWidth(); j++) {
      // Color Image
      I[i][j].B = *pDest++; // Blue component
      I[i][j].G = *pDest++; // Green component
      I[i][j].R = *pDest++; // Red component
      I[i][j].A = *pDest++; // Alpha component
    }
  }
#else // if texture in RGBa format which is the format of the input image
  memcpy(I.bitmap, pDest, I.getHeight() * I.getWidth() * sizeof(vpRGBa));
#endif

  // Unlock the pixel buffer
  mPixelBuffer->unlock();
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_ar.a(vpAROgre.cpp.o) has no symbols
void dummy_vpAROgre(){};
#endif
