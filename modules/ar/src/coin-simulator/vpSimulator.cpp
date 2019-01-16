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
 * Simulator based on Coin3d.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/
/*!
  \file vpSimulator.cpp
  \brief Implementation of a simulator based on Coin3d (www.coin3d.org).
  The simulator uses the vpViewer class.
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_COIN3D_AND_GUI

#include <visp3/ar/vpSimulator.h>
#include <visp3/core/vpTime.h>

#include <visp3/core/vpImage.h>

#ifdef VISP_HAVE_MODULE_IO
#include <visp3/io/vpImageIo.h>
#endif

/* Objets OIV. */
#include <Inventor/nodes/SoCone.h>           /* Objet cone.                            */
#include <Inventor/nodes/SoCoordinate3.h>    /* Liste de points.                */
#include <Inventor/nodes/SoCylinder.h>       /* Objet cylindre.                    */
#include <Inventor/nodes/SoIndexedFaceSet.h> /* Liste de face.               */
#include <Inventor/nodes/SoPointLight.h>     /* Objet lumiere ponctuelle.        */
#include <Inventor/nodes/SoRotationXYZ.h>    /* Transfo rotation simple.       */
#include <Inventor/nodes/SoScale.h>          /* Trasnfo mise a l'echelle.             */
#include <Inventor/nodes/SoTranslation.h>    /* Trasnfo translation.            */

#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoDirectionalLight.h> /* Objet lumiere directionnelle*/
#include <Inventor/nodes/SoDrawStyle.h>        /* Style de rendu.                  */
#include <Inventor/nodes/SoEnvironment.h>      /* Eclairage ambiant.              */
#include <Inventor/nodes/SoGroup.h>            /* Groupement de noeuds (sans separation)*/
#include <Inventor/nodes/SoMaterial.h>         /* Matiere (couleur) des objets.     */

// Positions of all of the vertices:
//
static float pyramidVertexes[5][3] = {{0.33f, 0.33f, 0.f},
                                      {-0.33f, 0.33f, 0.f},
                                      {-0.33f, -0.33f, 0.f},
                                      {0.33f, -0.33f, 0.f},

                                      {0.f, 0.f, -1.0f}};

static int32_t pyramidFaces[] = {
    0,
    1,
    2,
    3,
    SO_END_FACE_INDEX, // top face

    0,
    1,
    4,
    SO_END_FACE_INDEX, // 4 faces about top
    1,
    2,
    4,
    SO_END_FACE_INDEX,
    2,
    3,
    4,
    SO_END_FACE_INDEX,
    3,
    0,
    4,
    SO_END_FACE_INDEX,
};

// Routine to create a scene graph representing a dodecahedron
SoSeparator *makePyramide()
{
  SoSeparator *result = new SoSeparator;
  result->ref();

  // Define coordinates for vertices
  SoCoordinate3 *myCoords = new SoCoordinate3;
  myCoords->point.setValues(0, 5, pyramidVertexes);
  result->addChild(myCoords);

  // Define the IndexedFaceSet, with indices into the vertices:
  SoIndexedFaceSet *myFaceSet = new SoIndexedFaceSet;
  myFaceSet->coordIndex.setValues(0, 21, (const int32_t *)pyramidFaces);
  result->addChild(myFaceSet);

  result->unrefNoDelete();
  return result;
}

/* Cree une fleche composee d'un cylindre et d'un cone.
 * La fleche a une hauteur total de <longueur>, dont
 * <proportionFleche>% pour la fleche. Le rayon du cylindre
 * est <radius>, et celui de la fleche <radius> * 5.
 * La fleche est oriente selon l'axe Y.
 */
static SoSeparator *createArrow(float longueur, float proportionFleche, float radius)
{
  SoSeparator *fleche = new SoSeparator;
  fleche->ref();

  SoTranslation *poseCylindre = new SoTranslation;
  SoCylinder *line = new SoCylinder;
  SoTranslation *posePointe = new SoTranslation;
  SoCone *pointe = new SoCone;

  float l_cylindre = longueur * (1 - proportionFleche);
  float l_cone = longueur * proportionFleche;
  float radius_cylindre = radius;
  float radius_cone = radius * 5;

  line->radius.setValue(radius_cylindre);
  line->height.setValue(l_cylindre);

  poseCylindre->translation.setValue(0, l_cylindre / 2, 0);
  posePointe->translation.setValue(0.0, l_cylindre / 2 + l_cone / 2, 0);

  pointe->bottomRadius.setValue(radius_cone);
  pointe->height.setValue(l_cone);

  fleche->addChild(poseCylindre);
  fleche->addChild(line);
  fleche->addChild(posePointe);
  fleche->addChild(pointe);

  return fleche;
}

/*
  Cree un objet repere dans un noeud separator, et le renvoie.
  \return          : code d'erreur, SIMU_CODE_OK si tout s'est bien passe.
*/
#define LONGUEUR_FLECHE 1.0f
#define RAYON_FLECHE 0.002f
#define PROPORTION_FLECHE 0.1f

SoSeparator *createFrame(float longueurFleche = LONGUEUR_FLECHE, float proportionFleche = PROPORTION_FLECHE,
                         float radiusFleche = RAYON_FLECHE)
{
  vpDEBUG_TRACE(15, "# Entree.");

  SoSeparator *frame = new SoSeparator;
  frame->ref();

  SoRotationXYZ *rotationY_X = new SoRotationXYZ;
  rotationY_X->axis = SoRotationXYZ::Z;
  rotationY_X->angle.setValue((float)(-M_PI / 2));

  SoRotationXYZ *rotationX_Y = new SoRotationXYZ;
  rotationX_Y->axis = SoRotationXYZ::Z;
  rotationX_Y->angle.setValue((float)(M_PI / 2));

  SoRotationXYZ *rotationY_Z = new SoRotationXYZ;
  rotationY_Z->axis = SoRotationXYZ::X;
  rotationY_Z->angle.setValue((float)(M_PI / 2));

  SoMaterial *rouge = new SoMaterial;
  rouge->diffuseColor.setValue(1.0, 0.0, 0.0);
  rouge->emissiveColor.setValue(0.5, 0.0, 0.0);

  SoMaterial *vert = new SoMaterial;
  vert->diffuseColor.setValue(0.0, 1.0, 0.0);
  vert->emissiveColor.setValue(0.0, 0.5, 0.0);

  SoMaterial *bleu = new SoMaterial;
  bleu->diffuseColor.setValue(0.0, 0.0, 1.0);
  bleu->emissiveColor.setValue(0.0, 0.0, 0.5);

  SoSeparator *fleche = createArrow(longueurFleche, proportionFleche, radiusFleche);

  frame->addChild(rouge);
  frame->addChild(rotationY_X);
  frame->addChild(fleche);
  frame->addChild(vert);
  frame->addChild(rotationX_Y);
  frame->addChild(fleche);
  frame->addChild(bleu);
  frame->addChild(rotationY_Z);
  frame->addChild(fleche);

  frame->unrefNoDelete();

  vpDEBUG_TRACE(15, "# Sortie.");
  return frame;
}

SoSeparator *createCameraObject(const float zoomFactor = 1.0)
{
  vpDEBUG_TRACE(15, "# Entree.");

  SoSeparator *cam = new SoSeparator;
  cam->ref();

  SoMaterial *myMaterial = new SoMaterial;
  myMaterial->diffuseColor.setValue(1.0, 0.0, 0.0);
  myMaterial->emissiveColor.setValue(0.5, 0.0, 0.0);

  SoScale *taille = new SoScale;
  {
    float zoom = 0.1f * zoomFactor;
    taille->scaleFactor.setValue(zoom, zoom, zoom);
  }

  SoMaterial *couleurBlanc = new SoMaterial;
  couleurBlanc->diffuseColor.setValue(1.0, 1.0, 1.0);
  couleurBlanc->emissiveColor.setValue(1.0, 1.0, 1.0);
  SoDrawStyle *filDeFer = new SoDrawStyle;
  filDeFer->style.setValue(SoDrawStyle::LINES);
  filDeFer->lineWidth.setValue(1);

  SoSeparator *cone = new SoSeparator;
  cone->ref();
  cone->addChild(makePyramide());
  cone->addChild(couleurBlanc);
  cone->addChild(filDeFer);
  cone->addChild(makePyramide());
  cone->unrefNoDelete();

  cam->addChild(myMaterial);
  cam->addChild(taille);
  cam->addChild(cone);
  cam->addChild(createFrame(2.0f, 0.1f, 0.01f));

  //  cam->unref() ;
  vpDEBUG_TRACE(15, "# Sortie.");
  return cam;
}

//--------------------------------------------------------------
void vpSimulator::init()
{
  internal_width = 200;
  internal_height = 200;
  external_width = 200;
  external_height = 200;

  mainWindowInitialized = false;
  internalView = NULL;
  externalView = NULL;
  image_background = NULL;

  zoomFactor = 1;
  cameraPositionInitialized = false;

  // write image process
  realtime = NULL;
  offScreenRenderer = NULL;
  bufferView = NULL;
  get = 1;
  typeImage = grayImage;
  mainThread = NULL;
  scene = NULL;
  internalRoot = NULL;
  externalRoot = NULL;
  internalCamera = NULL;
  externalCamera = NULL;
  internalCameraPosition = NULL;
  extrenalCameraPosition = NULL;
  internalCameraObject = NULL;
#if defined(VISP_HAVE_SOWIN)
// mainWindow = ?;
#elif defined(VISP_HAVE_SOQT)
  mainWindow = NULL;
#elif defined(VISP_HAVE_SOXT)
// mainWindow = ?;
#endif
}
void vpSimulator::kill()
{
  if (internalView != NULL) {
    delete internalView;
    internalView = NULL;
  }
  if (externalView != NULL) {
    delete externalView;
    externalView = NULL;
  }
  if (bufferView != NULL) {
    delete[] bufferView;
    bufferView = NULL;
  }
  if (image_background != NULL) {
    free(image_background);
    image_background = NULL;
  }
}

vpSimulator::vpSimulator()
  :
#if defined(VISP_HAVE_SOWIN)
    mainWindow(),
#elif defined(VISP_HAVE_SOQT)
    mainWindow(NULL),
#elif defined(VISP_HAVE_SOXT)
    mainWindow(),
#endif
    mainWindowInitialized(false), typeImage(vpSimulator::grayImage), image_background(NULL), internalView(NULL),
    externalView(NULL), mainThread(NULL), internal_width(0), internal_height(0), external_width(0), external_height(0),
    scene(NULL), internalRoot(NULL), externalRoot(NULL), internalCamera(NULL), externalCamera(NULL),
    internalCameraPosition(NULL), extrenalCameraPosition(NULL), internalCameraObject(NULL), zoomFactor(0.),
    cameraPositionInitialized(false), cMf(), internalCameraParameters(), externalCameraParameters(), realtime(NULL),
    offScreenRenderer(NULL), bufferView(NULL), get(0)
{
  vpSimulator::init();
}

vpSimulator::~vpSimulator() { vpSimulator::kill(); }

void vpSimulator::initSoApplication()
{
  mainWindow = vpViewer::init("");
  mainWindowInitialized = true;
}

void vpSimulator::initSceneGraph()
{
  this->scene = new SoSeparator;
  this->internalRoot = new SoSeparator;
  this->externalRoot = new SoSeparator;

  this->scene->ref();
  this->internalRoot->ref();
  this->externalRoot->ref();

  // define the camera SoPerspectiveCamera
  this->internalCamera = new SoPerspectiveCamera;
  this->externalCamera = new SoPerspectiveCamera;

  this->internalCameraPosition = new SoTransform;
  this->internalCameraObject = createCameraObject(zoomFactor);

  internalCamera->farDistance.setValue(100);
  internalCamera->nearDistance.setValue(0.0001f);

  // link between camera and internal root
  this->internalRoot->addChild(this->internalCamera);
  this->internalRoot->addChild(this->scene);

  this->externalRoot->addChild(this->externalCamera);
  this->externalRoot->addChild(this->scene);

  SoSeparator *camera = new SoSeparator;
  camera->ref();
  camera->addChild(this->internalCameraPosition);
  camera->addChild(this->internalCameraObject);
  this->externalRoot->addChild(camera);

  // this->externalRoot->addChild (internalCameraPosition);
  //  this->externalRoot->addChild (internalCameraObject);
  SoCube *cube = new SoCube;
  cube->width = 0.01f;
  cube->depth = 0.01f;
  cube->height = 0.01f;

  this->externalRoot->addChild(cube);

  if (realtime == NULL) {

    SoDB::enableRealTimeSensor(FALSE);
    SoSceneManager::enableRealTimeUpdate(FALSE);
    realtime = (SbTime *)SoDB::getGlobalField("realTime");
    realtime->setValue(0.0);
  }
}

/*!
  \brief Define the zoom factor used to define the size of the objects (frame,
  camera, ...)

  \param zoom: zoom factor of the objects. By default, 1.
*/
void vpSimulator::setZoomFactor(const float zoom)
{
  zoomFactor = zoom;
  static bool firstTime = true;
  if (firstTime) {
    SoScale *taille = new SoScale;
    taille->scaleFactor.setValue(zoomFactor, zoomFactor, zoomFactor);
    this->scene->addChild(taille);
    firstTime = false;
  } else {
    SoScale *taille = (SoScale *)this->scene->getChild(0);
    taille->scaleFactor.setValue(zoomFactor, zoomFactor, zoomFactor);
  }
}

/*!
  \brief Change the zoom factor associated to the child given by index.
  In order to create multiple zoom factor for multiple object to display,
  objects loaded the load() function, you have to know the index of the scale
  object associated to.

  Usually, if you define the main zoom factor (for example for the frames) and
  then load two differents objects, You can change the zoom factor of all the
  objects using: changeZoomFactor(newZoom, 0)

  If you want to change the zoom factor of the first object, use
  changeZoomFactor(newZoom, 1)

  And for the second object, use changeZoomFactor(newZoom, 3)

  \param zoomFactor : the new zoom use to specify the apparent size of the
  object \param index : the index of the Scale object to modify (see comments)
*/
void vpSimulator::changeZoomFactor(const float zoomFactor, const int index)
{
  SoScale *taille = (SoScale *)this->scene->getChild(index);
  taille->scaleFactor.setValue(zoomFactor, zoomFactor, zoomFactor);
  //  this->setZoomFactor(zoomFactor);
}

void vpSimulator::initInternalViewer(const unsigned int width, const unsigned int height)
{
  internal_width = width;
  internal_height = height;

  if (mainWindowInitialized == false) {
    initSoApplication();
    initSceneGraph();
  }

  internalView = new vpViewer(mainWindow, this, vpViewer::internalView);

  // set the scene to render from this view
  internalView->setSceneGraph(internalRoot);

  // set the title
  internalView->setTitle("Internal camera view");

  // If the view mode is on, user events will be caught and used to influence
  // the camera position / orientation. in this viewer we do not want that,
  // we set it to false
  internalView->setViewing(false);

  // Turn the viewer decorations
  internalView->setDecoration(false);

  internalView->resize((int)width, (int)height, true);

  // open the window
  internalView->show();

  bufferView = new unsigned char[3 * width * height];
}

void vpSimulator::initExternalViewer(const unsigned int width, const unsigned int height)
{

  external_width = width;
  external_height = height;

  if (mainWindowInitialized == false) {
    initSoApplication();
    initSceneGraph();
  }

  externalView = new vpViewer(mainWindow, this, vpViewer::externalView);

  // set the scene to render this view
  externalView->setSceneGraph(externalRoot);

  // set the title
  externalView->setTitle("External View");
  externalView->resize((int)width, (int)height, false);
  // the goal here is to see all the scene and not to determine
  // a manual viewpoint
  externalView->viewAll();

  // open the window
  externalView->show();
}

void vpSimulator::setInternalCameraParameters(vpCameraParameters &_cam)
{
  internalCameraParameters = _cam;

  float px = (float)_cam.get_px();
  float py = (float)_cam.get_py();
  float v = internal_height / (2.f * py);

  internalCamera->ref();
  internalCamera->heightAngle = 2 * atan(v);
  internalCamera->aspectRatio = (internal_width / internal_height) * (px / py);
  internalCamera->nearDistance = 0.001f;

  internalCamera->farDistance = 1000;
  internalCamera->unrefNoDelete();
}

void vpSimulator::setExternalCameraParameters(vpCameraParameters &_cam)
{
  //   SoPerspectiveCamera *camera ;
  //   camera  = (SoPerspectiveCamera *)this->externalView->getCamera() ;
  externalCameraParameters = _cam;

  float px = (float)_cam.get_px();
  float py = (float)_cam.get_py();
  float v = external_height / (2 * py);

  externalCamera->ref();
  externalCamera->heightAngle = 2 * atan(v);
  externalCamera->aspectRatio = (external_width / external_height) * (px / py);
  externalCamera->nearDistance = 0.001f;
  externalCamera->farDistance = 1000;
  externalCamera->unrefNoDelete();
}

void vpSimulator::getExternalCameraPosition(vpHomogeneousMatrix &cMf)
{
  /*  SoCamera *camera ;
    camera  = this->externalView->getCamera() ;*/
  SoSFVec3f position = externalCamera->position;

  // get the rotation
  SoSFRotation orientation = externalCamera->orientation;
  SbVec3f axis;
  float angle;
  orientation.getValue(axis, angle);
  SbRotation rotation(axis, angle);

  // get the translation
  SbVec3f t;
  t = position.getValue();

  SbMatrix matrix;
  matrix.setRotate(rotation);

  vpHomogeneousMatrix fMc;
  SbMatrix rotX;
  rotX.setRotate(SbRotation(SbVec3f(1.0f, 0.0f, 0.0f), (float)M_PI));
  matrix.multLeft(rotX);
  for (unsigned int i = 0; i < 4; i++)
    for (unsigned int j = 0; j < 4; j++)
      fMc[j][i] = matrix[(int)i][(int)j];
  fMc[0][3] = t[0];
  fMc[1][3] = t[1];
  fMc[2][3] = t[2];

  cMf = fMc.inverse();
}

void vpSimulator::setCameraPosition(vpHomogeneousMatrix &_cMf)
{
  cameraPositionInitialized = true;
  cMf = _cMf;
}
void vpSimulator::moveInternalCamera(vpHomogeneousMatrix &cMf)
{

  SbMatrix matrix;
  SbRotation rotCam;
  SbMatrix rotX;
  rotX.setRotate(SbRotation(SbVec3f(1.0f, 0.0f, 0.0f), (float)M_PI));
  for (unsigned int i = 0; i < 4; i++)
    for (unsigned int j = 0; j < 4; j++)
      matrix[(int)j][(int)i] = (float)cMf[i][j];

  matrix = matrix.inverse();
  matrix.multLeft(rotX);
  rotCam.setValue(matrix);

  internalCamera->ref();
  internalCamera->orientation.setValue(rotCam);
  internalCamera->position.setValue(matrix[3][0], matrix[3][1], matrix[3][2]);
  internalCamera->unref();

  rotX.setRotate(SbRotation(SbVec3f(-1.0f, 0.0f, 0.0f), (float)M_PI));
  matrix.multLeft(rotX);
  rotCam.setValue(matrix);
  internalCameraPosition->ref();
  internalCameraPosition->rotation.setValue(rotCam);
  internalCameraPosition->translation.setValue(matrix[3][0], matrix[3][1], matrix[3][2]);
  internalCameraPosition->unref();
}

/*!  this function MUST NOT be called from a thread where the vpSimulator and
  its mainloop are not
*/
void vpSimulator::redraw()
{

  //  if (this->cameraPositionInitialized==true)
  {
    if (this->externalView != NULL) {
      this->externalView->render(); // call actualRedraw()
      //      vpHomogeneousMatrix c ;
      //      getExternalCameraPosition(c) ;
    }
    if (this->internalView != NULL) {
      this->moveInternalCamera(this->cMf);
      this->internalView->render(); // call actualRedraw()
    }
  }
}

// This function is called 20 times each second.
static void timerSensorCallback(void *data, SoSensor *)
{
  vpSimulator *simulator = (vpSimulator *)data;

  simulator->redraw();
}

void vpSimulator::mainLoop()
{
  if (mainWindowInitialized == false) {
    vpERROR_TRACE("main window is not opened ");
  }

  vpTime::wait(1000);

  // Timer sensor
  SoTimerSensor *timer = new SoTimerSensor(timerSensorCallback, (void *)this);
  timer->setInterval(0.01);
  timer->schedule();
  vpViewer::mainLoop();
}

//-----------------------------------------------------------------
// scene stuff
//-----------------------------------------------------------------

//! loading the virtual scene
void vpSimulator::load(const char *file_name)
{

  SoInput input;
  if (!input.openFile(file_name)) {
    vpERROR_TRACE("Erreur cannot open file %s", file_name);
  }

  SoSeparator *newscene = SoDB::readAll(&input);
  newscene->ref();
  if (newscene == NULL) {
    vpERROR_TRACE("Error while reading %s", file_name);
  }

  SoScale *taille = new SoScale;
  taille->scaleFactor.setValue(zoomFactor, zoomFactor, zoomFactor);

  //  newscene->addChild(taille);

  //  std::cout << "this->scene->getNumChildren() = " <<
  //  this->scene->getNumChildren() << std::endl;

  this->scene->addChild(taille);
  this->scene->addChild(newscene);
  newscene->unref();
}

void vpSimulator::save(const char *name, bool binary)
{
  // get a pointer to the object "name"
  SoOutput output;
  output.openFile(name);

  if (binary == true)
    output.setBinary(TRUE);

  SoWriteAction writeAction(&output);
  writeAction.apply(scene);
  output.closeFile();
}

/*!
  Add the representation of a frame.
  \param fMo : desired position of the frame
  \param zoom : Zoom factor.
*/
void vpSimulator::addFrame(const vpHomogeneousMatrix &fMo, float zoom)
{

  SoScale *taille = new SoScale;
  taille->scaleFactor.setValue(zoom, zoom, zoom);

  SoSeparator *frame = new SoSeparator;
  frame->ref();
  frame->addChild(taille);
  frame->addChild(createFrame(LONGUEUR_FLECHE * zoom, PROPORTION_FLECHE * zoom, RAYON_FLECHE * zoom));
  this->addObject(frame, fMo, externalRoot);
  // frame->unref();
}

/*!
  \brief Add the representation of the absolute frame

  \param zoom : Zoom factor.
*/
void vpSimulator::addAbsoluteFrame(float zoom)
{
  scene->addChild(createFrame(LONGUEUR_FLECHE * zoom, PROPORTION_FLECHE * zoom, RAYON_FLECHE * zoom));
}

/*!
  \brief Add a new object in the scene graph
  \param iv_filename : name of.iv file to load
  \param fMo       : position of the object wrt the reference frame
*/
void vpSimulator::load(const char *iv_filename, const vpHomogeneousMatrix &fMo)
{

  SoInput in;
  SoSeparator *newObject;

  if (!in.openFile(iv_filename)) {
    vpERROR_TRACE("Erreur lors de la lecture du fichier %s.", iv_filename);
  }

  newObject = SoDB::readAll(&in);
  if (NULL == newObject) {
    vpERROR_TRACE("Problem reading data for file <%s>.", iv_filename);
  }

  try {
    this->addObject(newObject, fMo);
  } catch (...) {
    vpERROR_TRACE("Error adding object from file <%s> ", iv_filename);
    throw;
  }
}

/*!
  \brief Add a new object in the scene graph
  \param newObject : pointer toward the new object
  \param fMo       : position of the object wrt the reference frame
*/
void vpSimulator::addObject(SoSeparator *newObject, const vpHomogeneousMatrix &fMo)
{
  try {
    this->addObject(newObject, fMo, scene);
  } catch (...) {
    vpERROR_TRACE("Error adding object in scene graph ");
    throw;
  }
}

/*!
  \brief Add an object in a sub scene graph
  \param object : pointer toward the new object
  \param fMo    : position of the object wrt the reference frame
  \param root : pointer toward the subscene graph
*/

void vpSimulator::addObject(SoSeparator *object, const vpHomogeneousMatrix &fMo, SoSeparator *root)
{

  bool identity = true;
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      if (i == j) {
        if (fabs(fMo[i][j] - 1) > 1e-6)
          identity = false;
      } else {
        if (fabs(fMo[i][j]) > 1e-6)
          identity = false;
      }
    }
  }

  if (identity == true) {
    root->addChild(object);
  } else {
    SbMatrix matrix;
    SbRotation rotation;
    for (unsigned int i = 0; i < 4; i++)
      for (unsigned int j = 0; j < 4; j++)
        matrix[(int)j][(int)i] = (float)fMo[i][j];

    //  matrix= matrix.inverse();
    rotation.setValue(matrix);

    SoTransform *displacement = new SoTransform;
    SoSeparator *newNode = new SoSeparator;

    displacement->rotation.setValue(rotation);
    displacement->translation.setValue(matrix[3][0], matrix[3][1], matrix[3][2]);

    root->addChild(newNode);
    newNode->addChild(displacement);
    newNode->addChild(object);
  }
}

//! init the main program thread
void vpSimulator::initApplication(void *(*start_routine)(void *))
{
  // pthread_create (&mainThread, NULL, start_routine, (void *)this);
  mainThread = SbThread::create(start_routine, (void *)this);
}

/*!
  Set the function used for the simulation loop and the data to pass to this
  function. As the data are represented using a generic pointer, care should
  be taken to ensure there is no memory corruption.

  \param start_routine : A pointer to the function used as a main simulation
  loop for the simulation.
  \param data : The data to pass to the main loop.
*/
void vpSimulator::initApplication(void *(*start_routine)(void *), void *data)
{
  mainThread = SbThread::create(start_routine, (void *)data);
}

//! performed some initialization in the main program thread
//! should be locate at the beginning of the main program
void vpSimulator::initMainApplication()
{
  // pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL );
  // pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  vpTime::wait(1000);
}
//! performed some thread destruction in the main program thread
//! should be locate at the end of the main program
void vpSimulator::closeMainApplication()
{
  vpViewer::exitMainLoop();
  // pthread_exit (NULL);
}

/* Initialise le SoOffScreenRenderer si necessaire, puis realise le rendu.
 * Quand la fonction rend la main, le buffer est pret et n'a plus qu'a etre
 * enregistre ou passe a l'utilisateur.
 * INPUT:
 *   - vueInterne est vrai ssi il faut rendre la vue interne, faux ssi
 * il faut rendre la vue externe.
 * OUTPUT:
 *   - width : largeur de l'image dans le buffer.
 *   - height : hauteur de l'image dans le buffer.
 */
void vpSimulator::offScreenRendering(vpSimulatorViewType view, int *width, int *height)
{

  SbVec2s size(320, 200);
  SoNode *thisroot;

  {
    if (view == vpSimulator::INTERNAL) {
      size = this->internalView->getViewportRegion().getWindowSize();
      thisroot = this->internalView->getSceneManager()->getSceneGraph();
    } else {
      size = this->externalView->getViewportRegion().getWindowSize();
      thisroot = this->externalView->getSceneManager()->getSceneGraph();
    }
  }
  SbViewportRegion myViewPort(size);

  // Creation du rendu si necessaire.
  if (NULL == this->offScreenRenderer) {
    // Init du SoOffscreenRenderer
    this->offScreenRenderer = new SoOffscreenRenderer(myViewPort);
  } else {
    // Redefini le view port
    this->offScreenRenderer->setViewportRegion(myViewPort);
  }

  // Rendu offscreen
  if (!this->offScreenRenderer->render(thisroot)) {
    vpERROR_TRACE("La scene n'a pas pu etre rendue offscreen.");
    delete this->offScreenRenderer;
    this->offScreenRenderer = NULL;
  } else {

    /*
      if (view==vpSimulator::INTERNAL)
      {
      //Recopie du buffer contenant l'image, dans bufferView
      int length = 3*size [0]*size[1];
      delete [] bufferView;
      bufferView = new unsigned char [length];
      for(int i=0; i<length; i++)
      {
      bufferView[i] = this ->offScreenRenderer->getBuffer()[i];
      }
      }*/
  }

  //  exit(1) ;
  if (NULL != width) {
    *width = size[0];
  }
  if (NULL != height) {
    *height = size[1];
  }
}

/* Enregistre l'image de vue interne ou externe dans un fichier RGB.
 * Effectue le rendu dans un buffer plutot qu'a l'ecran, puis sauvegarde
 * ce buffer au format PS (copie directe).
 * INPUT
 *   - fileName: nom du fichier dans lequel placer le resultat.
 * OUTPUT
 *   - RETURN : Code d'erreur CODE_OK si tout s'est bien passe.
 */

#ifdef VISP_HAVE_MODULE_IO
void vpSimulator::write(const char *fileName)
{

  while (get == 0) {
    vpTRACE("%d ", get);
  }
  get = 2;
  /*  FILE *fp = fopen(fileName, "w");
      fprintf(fp,"P6 \n %d %d \n 255",internal_width,internal_height) ;
      fwrite(bufferView, sizeof(unsigned char),
     internal_width*internal_height*3, fp) ;*/
  vpImage<vpRGBa> I(internal_height, internal_width);

  for (unsigned int i = 0; i < internal_height; i++)
    for (unsigned int j = 0; j < internal_width; j++) {
      unsigned char r, g, b;
      unsigned int index = 3 * ((internal_height - i - 1) * internal_width + j);
      r = *(bufferView + index);
      g = *(bufferView + index + 1);
      b = *(bufferView + index + 2);
      I[i][j].R = r;
      I[i][j].G = g;
      I[i][j].B = b;
    }
  vpImageIo::write(I, fileName);
  // fclose (fp);
  get = 1;
}
#endif

void vpSimulator::getSizeInternalView(int &width, int &height)
{
  SbVec2s size = this->internalView->getViewportRegion().getWindowSize();
  width = size[0];
  height = size[1];
}

/*!
  Make a copy of the current internal view
  \param I : destination image
 */

void vpSimulator::getInternalImage(vpImage<vpRGBa> &I)
{
  // while (get==0) {;}
  get = 2;
  I.resize(internal_height, internal_width);
  vpImageConvert::RGBToRGBa(bufferView, (unsigned char *)I.bitmap, internal_width, internal_height, true);
  get = 1;
}

/*!
  Make a copy of the current internal view
  \param I : destination image
 */
void vpSimulator::getInternalImage(vpImage<unsigned char> &I)
{
  // while (get==0) {;}
  get = 2;
  I.resize(internal_height, internal_width);
  vpImageConvert::RGBToGrey(bufferView, I.bitmap, internal_width, internal_height, true);
  get = 1;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_ar.a(vpSimulator.cpp.o) has no
// symbols
void dummy_vpSimulator(){};
#endif
