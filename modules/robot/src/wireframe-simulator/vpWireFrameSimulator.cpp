/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Wire frame simulator
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpWireFrameSimulator.cpp
  \brief Implementation of a wire frame simulator.
*/

#include <visp3/robot/vpWireFrameSimulator.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <vector>

#include <visp3/core/vpException.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpIoTools.h>

//Inventor includes
#if defined(VISP_HAVE_COIN3D)
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedFaceSet.h>
#include <Inventor/VRMLnodes/SoVRMLIndexedLineSet.h>
#include <Inventor/VRMLnodes/SoVRMLCoordinate.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoSearchAction.h>
#include <Inventor/misc/SoChildList.h>
#include <Inventor/actions/SoGetMatrixAction.h>
#include <Inventor/actions/SoGetPrimitiveCountAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/VRMLnodes/SoVRMLShape.h>
#endif

extern Point2i *point2i;
extern Point2i *listpoint2i;

typedef enum
{
  BND_MODEL,
  WRL_MODEL,
  UNKNOWN_MODEL
} Model_3D;

Model_3D getExtension(const char* file);
void set_scene_wrl (const char* str, Bound_scene *sc, float factor);

/*
  Get the extension of the file and return it
*/
Model_3D
getExtension(const char* file)
{
  std::string sfilename(file);

  size_t bnd = sfilename.find("bnd");
  size_t BND = sfilename.find("BND");
  size_t wrl = sfilename.find("wrl");
  size_t WRL = sfilename.find("WRL");
  
  size_t size = sfilename.size();

  if ((bnd>0 && bnd<size ) || (BND>0 && BND<size))
    return BND_MODEL;
  else if ((wrl>0 && wrl<size) || ( WRL>0 && WRL<size))
  {
#if defined(VISP_HAVE_COIN3D)
    return WRL_MODEL;
#else
    std::cout << "Coin not installed, cannot read VRML files" << std::endl;
    throw std::string("Coin not installed, cannot read VRML files");
#endif
  }
  return UNKNOWN_MODEL;
}

/*
   Enable to initialize the scene
*/
void set_scene (const char* str, Bound_scene *sc, float factor)
{
  FILE  *fd;

  //if ((fd = fopen (str, 0)) == -1)
  if ((fd = fopen (str, "r")) == NULL)
  {
    std::string error = "The file " + std::string(str) + " can not be opened";

    throw(vpException(vpException::ioError, error.c_str())) ;
  }
  open_keyword (keyword_tbl);
  open_lex ();
  open_source (fd, str);
  malloc_Bound_scene (sc, str,(Index)BOUND_NBR);
  parser (sc);
  
  //if (factor != 1)
  if (std::fabs(factor) > std::numeric_limits<double>::epsilon())
  {
    for (int i = 0; i < sc->bound.nbr; i++)
    {
      for (int j = 0; j < sc->bound.ptr[i].point.nbr; j++)
      {
        sc->bound.ptr[i].point.ptr[j].x = sc->bound.ptr[i].point.ptr[j].x*factor;
        sc->bound.ptr[i].point.ptr[j].y = sc->bound.ptr[i].point.ptr[j].y*factor;
        sc->bound.ptr[i].point.ptr[j].z = sc->bound.ptr[i].point.ptr[j].z*factor;
      }
    }
  }
  
  close_source ();
  close_lex ();
  close_keyword ();
  fclose(fd);
}

#if defined(VISP_HAVE_COIN3D)

#ifndef DOXYGEN_SHOULD_SKIP_THIS
typedef struct indexFaceSet
{
  indexFaceSet() : nbPt(0), pt(), nbIndex(0), index() {};
  int nbPt;
  std::vector<vpPoint> pt;
  int nbIndex;
  std::vector<int> index;
} indexFaceSet;
#endif // DOXYGEN_SHOULD_SKIP_THIS

void extractFaces(SoVRMLIndexedFaceSet*, indexFaceSet *ifs);
void ifsToBound (Bound*, std::list<indexFaceSet*> &);
void destroyIfs(std::list<indexFaceSet*> &);
  

void set_scene_wrl (const char* str, Bound_scene *sc, float factor)
{
  //Load the sceneGraph
  SoDB::init();
  SoInput in;
  SbBool ok = in.openFile(str);
  SoSeparator  *sceneGraph;
  SoVRMLGroup  *sceneGraphVRML2;
  
  if (!ok) {
    vpERROR_TRACE("can't open file \"%s\" \n Please check the Marker_Less.ini file", str);
    exit(1);
  }
  
  if(!in.isFileVRML2())
  {
    sceneGraph = SoDB::readAll(&in);
    if (sceneGraph == NULL) { /*return -1;*/ }
    sceneGraph->ref();

    SoToVRML2Action tovrml2;
    tovrml2.apply(sceneGraph);
    sceneGraphVRML2 =tovrml2.getVRML2SceneGraph();
    sceneGraphVRML2->ref();
    sceneGraph->unref();
  }
  else
  {
    sceneGraphVRML2	= SoDB::readAllVRML(&in);
    if (sceneGraphVRML2 == NULL) {
      /*return -1;*/
      throw(vpException(vpException::notInitialized, "Cannot read VRML file"));
    }
    sceneGraphVRML2->ref();
  }
  
  in.closeFile();

  int nbShapes = sceneGraphVRML2->getNumChildren();

  SoNode * child;
  
  malloc_Bound_scene (sc, str,(Index)BOUND_NBR);
  
  int iterShapes = 0;
  for (int i = 0; i < nbShapes; i++)
  {
    int nbFaces = 0;
    child = sceneGraphVRML2->getChild(i);
    if (child->getTypeId() == SoVRMLShape::getClassTypeId())
    {
      std::list<indexFaceSet*> ifs_list;
      SoChildList * child2list = child->getChildren();
      for (int j = 0; j < child2list->getLength(); j++)
      {
        if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedFaceSet::getClassTypeId())
        {
          indexFaceSet *ifs = new indexFaceSet;
          SoVRMLIndexedFaceSet * face_set;
          face_set = (SoVRMLIndexedFaceSet*)child2list->get(j);
          extractFaces(face_set,ifs);
          ifs_list.push_back(ifs);
          nbFaces++;
        }
//         if (((SoNode*)child2list->get(j))->getTypeId() == SoVRMLIndexedLineSet::getClassTypeId())
//         {
//           std::cout << "> We found a line" << std::endl;
//           SoVRMLIndexedLineSet * line_set;
//           line_set = (SoVRMLIndexedLineSet*)child2list->get(j);
//           extractLines(line_set);
//         }
      }
      sc->bound.nbr++;
      ifsToBound (&(sc->bound.ptr[iterShapes]), ifs_list);
      destroyIfs(ifs_list);
      iterShapes++;
    }
  }
  
  //if (factor != 1)
  if (std::fabs(factor) > std::numeric_limits<double>::epsilon())
  {
    for (int i = 0; i < sc->bound.nbr; i++)
    {
      for (int j = 0; j < sc->bound.ptr[i].point.nbr; j++)
      {
        sc->bound.ptr[i].point.ptr[j].x = sc->bound.ptr[i].point.ptr[j].x*factor;
        sc->bound.ptr[i].point.ptr[j].y = sc->bound.ptr[i].point.ptr[j].y*factor;
        sc->bound.ptr[i].point.ptr[j].z = sc->bound.ptr[i].point.ptr[j].z*factor;
      }
    }
  }
}


void
extractFaces(SoVRMLIndexedFaceSet* face_set, indexFaceSet *ifs)
{
//   vpList<vpPoint> pointList;
//   pointList.kill();
  SoVRMLCoordinate *coord = (SoVRMLCoordinate *)(face_set->coord.getValue());
  int coordSize = coord->point.getNum();
  
  ifs->nbPt = coordSize;
  for (int i = 0; i < coordSize; i++)
  {
    SbVec3f point(0,0,0);
    point[0]=coord->point[i].getValue()[0];
    point[1]=coord->point[i].getValue()[1];
    point[2]=coord->point[i].getValue()[2];
    vpPoint pt(point[0],point[1],point[2]);
    ifs->pt.push_back(pt);
  }
  
  SoMFInt32 indexList = face_set->coordIndex;
  int indexListSize = indexList.getNum();  
  
  ifs->nbIndex = indexListSize;
  for (int i = 0; i < indexListSize; i++)
  {
    int index = face_set->coordIndex[i];
    ifs->index.push_back(index);
  }
}

void ifsToBound (Bound* bptr, std::list<indexFaceSet*> &ifs_list)
{
  int nbPt = 0;
  for(std::list<indexFaceSet*>::const_iterator it=ifs_list.begin(); it!=ifs_list.end(); ++it){
    nbPt += (*it)->nbPt;
  }
  bptr->point.nbr = (Index)nbPt;
  bptr->point.ptr = (Point3f *) malloc ((unsigned int)nbPt * sizeof (Point3f));
  
  ifs_list.front();
  unsigned int iter = 0;
  for (std::list<indexFaceSet*>::const_iterator it=ifs_list.begin(); it!=ifs_list.end(); ++it)
  {
    indexFaceSet* ifs = *it;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbPt; j++)
    {
      bptr->point.ptr[iter].x = (float)ifs->pt[j].get_oX();
      bptr->point.ptr[iter].y = (float)ifs->pt[j].get_oY();
      bptr->point.ptr[iter].z = (float)ifs->pt[j].get_oZ();
      iter++;
    }
  }
  
  unsigned int nbFace = 0;
  ifs_list.front();
  std::list<int> indSize;
  int indice = 0;
  for (std::list<indexFaceSet*>::const_iterator it=ifs_list.begin(); it!=ifs_list.end(); ++it)
  {
    indexFaceSet* ifs = *it;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbIndex; j++)
    {
      if(ifs->index[j] == -1) 
      {
        nbFace++;
        indSize.push_back(indice);
        indice = 0;
      }
      else indice++;
    }
  }
  
  bptr->face.nbr = (Index)nbFace;
  bptr->face.ptr = (Face *) malloc (nbFace * sizeof (Face));
  
  
  std::list<int>::const_iterator iter_indSize = indSize.begin();
  for (unsigned int i = 0; i < indSize.size(); i++)
  {
    bptr->face.ptr[i].vertex.nbr = (Index)*iter_indSize;
    bptr->face.ptr[i].vertex.ptr = (Index *) malloc ((unsigned int)*iter_indSize * sizeof (Index));
    ++iter_indSize;
  }
  
  int offset = 0;
  indice = 0;
  for (std::list<indexFaceSet*>::const_iterator it=ifs_list.begin(); it!=ifs_list.end(); ++it)
  {
    indexFaceSet* ifs = *it;
    iter = 0;
    for (unsigned int j = 0; j < (unsigned int)ifs->nbIndex; j++)
    {
      if(ifs->index[j] != -1)
      {
        bptr->face.ptr[indice].vertex.ptr[iter] = (Index)(ifs->index[j] + offset);
        iter++;
      }
      else
      {
        iter = 0;
        indice++;
      }
    }
    offset += ifs->nbPt;
  }
}

void destroyIfs(std::list<indexFaceSet*> &ifs_list)
{
  for(std::list<indexFaceSet*>::const_iterator it=ifs_list.begin(); it!=ifs_list.end(); ++it){
    delete *it;
  }
  ifs_list.clear();
}
#else
void set_scene_wrl (const char* /*str*/, Bound_scene* /*sc*/, float /*factor*/)
{
}
#endif


/*
  Convert the matrix format to deal with the one in the simulator
*/
void vp2jlc_matrix (const vpHomogeneousMatrix vpM, Matrix &jlcM)
{
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 0; j < 4; j++) 
      jlcM[j][i] = (float)vpM[i][j];
  }
}

/*
  Copy the scene corresponding to the registeresd parameters in the image.
*/
void
vpWireFrameSimulator::display_scene(Matrix mat, Bound_scene &sc, const vpImage<vpRGBa> &I, const vpColor &color)
{
 // extern Bound *clipping_Bound ();
  Bound *bp, *bend;
  Bound *clip; /* surface apres clipping */
  Byte b  = (Byte) *get_rfstack ();
  Matrix m;

  //bcopy ((char *) mat, (char *) m, sizeof (Matrix));
  memmove((char *) m, (char *) mat, sizeof (Matrix));
  View_to_Matrix (get_vwstack (), *(get_tmstack ()));
  postmult_matrix (m, *(get_tmstack ()));
  bp   = sc.bound.ptr;
  bend = bp + sc.bound.nbr;
  for (; bp < bend; bp++)
  {
    if ((clip = clipping_Bound (bp, m)) != NULL)
    {
      Face *fp   = clip->face.ptr;
      Face *fend = fp + clip->face.nbr;

      set_Bound_face_display (clip, b); //regarde si is_visible

      point_3D_2D (clip->point.ptr, clip->point.nbr,(int)I.getWidth(),(int)I.getHeight(),point2i);
      for (; fp < fend; fp++)
      {
        if (fp->is_visible)
        {
          wireframe_Face (fp, point2i);
          Point2i *pt = listpoint2i;
          for (int i = 1; i < fp->vertex.nbr; i++)
          {
            vpDisplay::displayLine(I,vpImagePoint((pt)->y,(pt)->x),vpImagePoint((pt+1)->y,(pt+1)->x),color,thickness_);
            pt++;
          }
          if (fp->vertex.nbr > 2)
          {
            vpDisplay::displayLine(I,vpImagePoint((listpoint2i)->y,(listpoint2i)->x),vpImagePoint((pt)->y,(pt)->x),color,thickness_);
          }
        }
      }
    }
  }
}

/*
  Copy the scene corresponding to the registeresd parameters in the image.
*/
void
vpWireFrameSimulator::display_scene(Matrix mat, Bound_scene &sc, const vpImage<unsigned char> &I, const vpColor &color)
{
  //extern Bound *clipping_Bound ();

  Bound *bp, *bend;
  Bound *clip; /* surface apres clipping */
  Byte b  = (Byte) *get_rfstack ();
  Matrix m;

  //bcopy ((char *) mat, (char *) m, sizeof (Matrix));
  memmove((char *) m, (char *) mat, sizeof (Matrix));
  View_to_Matrix (get_vwstack (), *(get_tmstack ()));
  postmult_matrix (m, *(get_tmstack ()));
  bp   = sc.bound.ptr;
  bend = bp + sc.bound.nbr;
  for (; bp < bend; bp++)
  {
    if ((clip = clipping_Bound (bp, m)) != NULL)
    {
      Face *fp   = clip->face.ptr;
      Face *fend = fp + clip->face.nbr;

      set_Bound_face_display (clip, b); //regarde si is_visible

      point_3D_2D (clip->point.ptr, clip->point.nbr,(int)I.getWidth(),(int)I.getHeight(),point2i);
      for (; fp < fend; fp++)
      {
        if (fp->is_visible)
        {
          wireframe_Face (fp, point2i);
          Point2i *pt = listpoint2i;
          for (int i = 1; i < fp->vertex.nbr; i++)
          {
            vpDisplay::displayLine(I,vpImagePoint((pt)->y,(pt)->x),vpImagePoint((pt+1)->y,(pt+1)->x),color,thickness_);
            pt++;
          }
          if (fp->vertex.nbr > 2)
          {
            vpDisplay::displayLine(I,vpImagePoint((listpoint2i)->y,(listpoint2i)->x),vpImagePoint((pt)->y,(pt)->x),color,thickness_);
          }
        }
      }
    }
  }
}

/*************************************************************************************************************/

/*!
  Basic constructor.
  
  Set the path to the scene files (*.bnd and *.sln) used by the
  simulator.  If the path set in vpConfig.h in VISP_SCENES_DIR macro is
  not valid, the path is set from the VISP_SCENES_DIR environment
  variable that the user has to set.
*/
vpWireFrameSimulator::vpWireFrameSimulator()
  : scene(), desiredScene(), camera(), objectImage(), fMo(), fMc(), camMf(),
    refMo(), cMo(), cdMo(), object(PLATE), desiredObject(D_STANDARD),
    camColor(vpColor::green), camTrajColor(vpColor::green), curColor(vpColor::blue),
    desColor(vpColor::red), sceneInitialized(false), displayCameraTrajectory(true),
    cameraTrajectory(), poseList(), fMoList(), nbrPtLimit(1000), old_iPr(), old_iPz(),
    old_iPt(), blockedr(false), blockedz(false), blockedt(false), blocked(false),
    camMf2(), f2Mf(), px_int(1), py_int(1), px_ext(1), py_ext(1), displayObject(false),
    displayDesiredObject(false), displayCamera(false), displayImageSimulator(false),
    cameraFactor(1.), camTrajType(CT_LINE), extCamChanged(false), rotz(), thickness_(1), scene_dir()
{
  // set scene_dir from #define VISP_SCENE_DIR if it exists
  // VISP_SCENES_DIR may contain multiple locations separated by ";"
  std::vector<std::string> scene_dirs = vpIoTools::splitChain(std::string(VISP_SCENES_DIR), std::string(";"));
  bool sceneDirExists = false;
  for(size_t i=0; i < scene_dirs.size(); i++)
  if (vpIoTools::checkDirectory(scene_dirs[i]) == true) { // directory exists
    scene_dir = scene_dirs[i];
    sceneDirExists = true;
    break;
  }
  if (! sceneDirExists) {
    try {
      scene_dir = vpIoTools::getenv("VISP_SCENES_DIR");
      std::cout << "The simulator uses data from VISP_SCENES_DIR=" << scene_dir << std::endl;
    }
    catch (...) {
      std::cout << "Cannot get VISP_SCENES_DIR environment variable" << std::endl;
    }
  }
    
  open_display();
  open_clipping();

  old_iPr = vpImagePoint(-1,-1);
  old_iPz = vpImagePoint(-1,-1);
  old_iPt = vpImagePoint(-1,-1);
   
  rotz.buildFrom(0,0,0,0,0,vpMath::rad(180));
  
  scene.name = NULL;
  scene.bound.ptr = NULL;
  scene.bound.nbr = 0;

  desiredScene.name = NULL;
  desiredScene.bound.ptr = NULL;
  desiredScene.bound.nbr = 0;

  camera.name = NULL;
  camera.bound.ptr = NULL;
  camera.bound.nbr = 0;
}


/*!
  Basic destructor
*/
vpWireFrameSimulator::~vpWireFrameSimulator()
{
  if(sceneInitialized)
  {
    if(displayObject)
      free_Bound_scene (&(this->scene));
    if(displayCamera){
      free_Bound_scene (&(this->camera));
    }
    if(displayDesiredObject)
      free_Bound_scene (&(this->desiredScene));
  }
  close_clipping();
  close_display ();

  cameraTrajectory.clear();
  poseList.clear();
  fMoList.clear();
}


/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  It exists several default scenes you can use. Use the vpSceneObject and the vpSceneDesiredObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current position.
  \param desired_object : Type of scene used to display the object at the desired pose (in the internal view).
*/
void
vpWireFrameSimulator::initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desired_object)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = obj;
  this->desiredObject = desired_object;

  const char *scene_dir_ = scene_dir.c_str();
  if (strlen(scene_dir_) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the camera name"));
  }

  strcpy(name_cam, scene_dir_);
  if (desiredObject != D_TOOL) 
  {
    strcat(name_cam,"/camera.bnd");
    set_scene(name_cam,&camera,cameraFactor);
  }
  else
  {
    strcat(name_cam,"/tool.bnd");
    set_scene(name_cam,&(this->camera),1.0);
  }

  strcpy(name, scene_dir_);
  switch (obj)
  {
    case THREE_PTS : {strcat(name,"/3pts.bnd"); break; }
    case CUBE : { strcat(name, "/cube.bnd"); break; }
    case PLATE : { strcat(name, "/plate.bnd"); break; }
    case SMALL_PLATE : { strcat(name, "/plate_6cm.bnd"); break; }
    case RECTANGLE : { strcat(name, "/rectangle.bnd"); break; }
    case SQUARE_10CM : { strcat(name, "/square10cm.bnd"); break; }
    case DIAMOND : { strcat(name, "/diamond.bnd"); break; }
    case TRAPEZOID : { strcat(name, "/trapezoid.bnd"); break; }
    case THREE_LINES : { strcat(name, "/line.bnd"); break; }
    case ROAD : { strcat(name, "/road.bnd"); break; }
    case TIRE : { strcat(name, "/circles2.bnd"); break; }
    case PIPE : { strcat(name, "/pipe.bnd"); break; }
    case CIRCLE : { strcat(name, "/circle.bnd"); break; }
    case SPHERE : { strcat(name, "/sphere.bnd"); break; }
    case CYLINDER : { strcat(name, "/cylinder.bnd"); break; }
    case PLAN: { strcat(name, "/plan.bnd"); break; }
    case POINT_CLOUD: { strcat(name, "/point_cloud.bnd"); break; }
  }
  set_scene(name,&(this->scene),1.0);

  scene_dir_ = scene_dir.c_str();
  if (strlen(scene_dir_) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the desired object name"));
  }

  switch (desiredObject)
  {
    case D_STANDARD : { break; }
    case D_CIRCLE : { 
      strcpy(name, scene_dir_);
      strcat(name, "/circle_sq2.bnd");
      break; }
    case D_TOOL : { 
      strcpy(name, scene_dir_);
      strcat(name, "/tool.bnd");
      break; }
  }
  set_scene(name, &(this->desiredScene), 1.0);

  if (obj == PIPE) load_rfstack(IS_INSIDE);
  else add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
  displayObject = true;
  displayDesiredObject = true;
  displayCamera = true;
  displayImageSimulator = true;
}

/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.

  It exists several default scenes you can use. Use the vpSceneObject and the vpSceneDesiredObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  It is also possible to add a list of vpImageSimulator instances. They will be automatically projected into the image. The position of the four corners have to be given in the object frame.

  \param obj : Type of scene used to display the object at the current position.
  \param desired_object : Type of scene used to display the object at the desired pose (in the internal view).
  \param imObj : A list of vpImageSimulator instances.
*/
void
vpWireFrameSimulator::initScene(const vpSceneObject &obj, const vpSceneDesiredObject &desired_object, const std::list<vpImageSimulator> &imObj)
{
  initScene(obj, desired_object);
  objectImage = imObj;
  displayImageSimulator = true;
}


/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  Here you can use the scene you want. You have to set the path to a .bnd or a .wrl file which is a 3D model file.

  \param obj : Path to the scene file you want to use.
  \param desired_object : Path to the scene file you want to use.
*/
void
vpWireFrameSimulator::initScene(const char* obj, const char* desired_object)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = THREE_PTS;
  this->desiredObject = D_STANDARD;
  
  const char *scene_dir_ = scene_dir.c_str();
  if (strlen(scene_dir_) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the camera name"));
  }

  strcpy(name_cam, scene_dir_);
  strcat(name_cam,"/camera.bnd");
  set_scene(name_cam,&camera,cameraFactor);

  if (strlen(obj) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the name"));
  }

  strcpy(name,obj);
  Model_3D model;
  model = getExtension(obj);
  if (model == BND_MODEL)
    set_scene(name,&(this->scene),1.0);
  else if (model == WRL_MODEL)
    set_scene_wrl(name,&(this->scene),1.0);
  else if (model == UNKNOWN_MODEL)
  {
    vpERROR_TRACE("Unknown file extension for the 3D model");
  }

  if (strlen(desired_object) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the camera name"));
  }

  strcpy(name,desired_object);
  model = getExtension(desired_object);
  if (model == BND_MODEL)
    set_scene(name,&(this->desiredScene),1.0);
  else if (model == WRL_MODEL)
    set_scene_wrl(name,&(this->desiredScene),1.0);
  else if (model == UNKNOWN_MODEL)
  {
    vpERROR_TRACE("Unknown file extension for the 3D model");
  }

  add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
  displayObject = true;
  displayDesiredObject = true;
  displayCamera = true;
}

/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.

  Here you can use the scene you want. You have to set the path to a .bnd or a .wrl file which is a 3D model file.

  It is also possible to add a list of vpImageSimulator instances. They will be automatically projected into the image. The position of the four corners have to be given in the object frame.

  \param obj : Path to the scene file you want to use.
  \param desired_object : Path to the scene file you want to use.
  \param imObj : A list of vpImageSimulator instances.
*/
void
vpWireFrameSimulator::initScene(const char* obj, const char* desired_object, const std::list<vpImageSimulator> &imObj)
{
  initScene(obj, desired_object);
  objectImage = imObj;
  displayImageSimulator = true;
}

/*!
  Initialize the simulator. It enables to choose the type of object which will be used to display the object
  at the current position. The object at the desired position is not displayed.
  
  It exists several default scenes you can use. Use the vpSceneObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current position.
*/
void
vpWireFrameSimulator::initScene(const vpSceneObject &obj)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = obj;

  const char *scene_dir_ = scene_dir.c_str();
  if (strlen(scene_dir_) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the camera name"));
  }

  strcpy(name_cam, scene_dir_);
  strcat(name_cam,"/camera.bnd");
  set_scene(name_cam,&camera,cameraFactor);

  strcpy(name, scene_dir_);
  switch (obj)
  {
    case THREE_PTS : {strcat(name,"/3pts.bnd"); break; }
    case CUBE : { strcat(name, "/cube.bnd"); break; }
    case PLATE : { strcat(name, "/plate.bnd"); break; }
    case SMALL_PLATE : { strcat(name, "/plate_6cm.bnd"); break; }
    case RECTANGLE : { strcat(name, "/rectangle.bnd"); break; }
    case SQUARE_10CM : { strcat(name, "/square10cm.bnd"); break; }
    case DIAMOND : { strcat(name, "/diamond.bnd"); break; }
    case TRAPEZOID : { strcat(name, "/trapezoid.bnd"); break; }
    case THREE_LINES : { strcat(name, "/line.bnd"); break; }
    case ROAD : { strcat(name, "/road.bnd"); break; }
    case TIRE : { strcat(name, "/circles2.bnd"); break; }
    case PIPE : { strcat(name, "/pipe.bnd"); break; }
    case CIRCLE : { strcat(name, "/circle.bnd"); break; }
    case SPHERE : { strcat(name, "/sphere.bnd"); break; }
    case CYLINDER : { strcat(name, "/cylinder.bnd"); break; }
    case PLAN: { strcat(name, "/plan.bnd"); break; }
    case POINT_CLOUD: { strcat(name, "/point_cloud.bnd"); break; }
  }
  set_scene(name,&(this->scene),1.0);

  if (obj == PIPE) load_rfstack(IS_INSIDE);
  else add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
  displayObject = true;
  displayCamera = true;

  displayDesiredObject = false;
  displayImageSimulator = false;
}

/*!
  Initialize the simulator. It enables to choose the type of object which will be used to display the object
  at the current position. The object at the desired position is not displayed.

  It exists several default scenes you can use. Use the vpSceneObject attributes to use them in this method. The corresponding files are stored in the "data" folder which is in the ViSP build directory.

  It is also possible to add a list of vpImageSimulator instances. They will be automatically projected into the image. The position of the four corners have to be given in the object frame.

  \param obj : Type of scene used to display the object at the current position.
  \param imObj : A list of vpImageSimulator instances.
*/
void
vpWireFrameSimulator::initScene(const vpSceneObject &obj, const std::list<vpImageSimulator> &imObj)
{
  initScene(obj);
  objectImage = imObj;
  displayImageSimulator = true;
}

/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position. The object at the desired position is not displayed.
  
  Here you can use the scene you want. You have to set the path to a .bnd or a .wrl file which is a 3D model file.

  \param obj : Path to the scene file you want to use.
*/
void
vpWireFrameSimulator::initScene(const char* obj)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = THREE_PTS;
  
  const char *scene_dir_ = scene_dir.c_str();
  if (strlen(scene_dir_) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the camera name"));
  }

  strcpy(name_cam, scene_dir_);
  strcat(name_cam,"/camera.bnd");
  set_scene(name_cam,&camera,cameraFactor);

  if (strlen(obj) >= FILENAME_MAX) {
    throw(vpException(vpException::memoryAllocationError,
                      "Not enough memory to intialize the name"));
  }

  strcpy(name,obj);
  Model_3D model;
  model = getExtension(obj);
  if (model == BND_MODEL)
    set_scene(name,&(this->scene),1.0);
  else if (model == WRL_MODEL)
    set_scene_wrl(name,&(this->scene),1.0);
  else if (model == UNKNOWN_MODEL)
  {
    vpERROR_TRACE("Unknown file extension for the 3D model");
  }

  add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
  displayObject = true;
  displayCamera = true;
}

/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position. The object at the desired position is not displayed.

  Here you can use the scene you want. You have to set the path to a .bnd or a .wrl file which is a 3D model file.

  It is also possible to add a list of vpImageSimulator instances. They will be automatically projected into the image. The position of the four corners have to be given in the object frame.

  \param obj : Path to the scene file you want to use.
  \param imObj : A list of vpImageSimulator instances.
*/
void
vpWireFrameSimulator::initScene(const char* obj, const std::list<vpImageSimulator> &imObj)
{
  initScene(obj);
  objectImage = imObj;
  displayImageSimulator = true;
}

/*!
  Get the internal view ie the view of the camera.

  \param I : The image where the internal view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpWireFrameSimulator::getInternalImage(vpImage<vpRGBa> &I)
{
  if (!sceneInitialized)
    throw(vpException(vpException::notInitialized,"The scene has to be initialized")) ;

  double u;
  double v;
  //if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if( (std::fabs(px_int-1.) > vpMath::maximum(px_int,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_int-1) > vpMath::maximum(py_int,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_int);
    v = (double)I.getHeight()/(2*py_int);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float o44c[4][4],o44cd[4][4],x,y,z;
  Matrix id = IDENTITY_MATRIX;

  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);
  
  if (displayImageSimulator)
  {
    I = 255;

    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*cMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }

    if (I.display != NULL)
      vpDisplay::display(I);
  }

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayObject)
    display_scene(id,this->scene,I, curColor);


  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayDesiredObject)
  {
    if (desiredObject == D_TOOL) display_scene(o44cd,desiredScene,I, vpColor::red);
    else display_scene(id,desiredScene,I, desColor);
  }
}


/*!
  Get the external view. It corresponds to the view of the scene from a reference frame you have to set.

  \param I : The image where the external view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/

void
vpWireFrameSimulator::getExternalImage(vpImage<vpRGBa> &I)
{
  bool changed = false;
  vpHomogeneousMatrix displacement = navigation(I,changed);

  //if (displacement[2][3] != 0 /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
  if (std::fabs(displacement[2][3]) > std::numeric_limits<double>::epsilon() /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
      camMf2 = camMf2*displacement;

  f2Mf = camMf2.inverse()*camMf;

  camMf = camMf2* displacement * f2Mf;

  double u;
  double v;
  //if(px_ext != 1 && py_ext != 1)
  // we assume px_ext and py_ext > 0
  if( (std::fabs(px_ext-1.) > vpMath::maximum(px_ext,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_ext-1) > vpMath::maximum(py_ext,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;

  vp2jlc_matrix(camMf.inverse(),w44cext);
  vp2jlc_matrix(fMc,w44c);
  vp2jlc_matrix(fMo,w44o);


  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if ((object == CUBE) || (object == SPHERE))
  {
    add_vwstack ("start","type", PERSPECTIVE);
  }
  
  if (displayImageSimulator)
  {
    I = 255;

    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*camMf*fMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }

    if (I.display != NULL)
      vpDisplay::display(I);
  }
  
  if (displayObject)
    display_scene(w44o,this->scene,I, curColor);

  if (displayCamera)
    display_scene(w44c,camera, I, camColor);

  if (displayCameraTrajectory)
  {
    vpImagePoint iP;
    vpImagePoint iP_1;
    poseList.push_back(cMo);
    fMoList.push_back(fMo);
  
    int iter = 0;

    if (changed || extCamChanged)
    {
      cameraTrajectory.clear();
      std::list<vpHomogeneousMatrix>::const_iterator iter_poseList = poseList.begin();
      std::list<vpHomogeneousMatrix>::const_iterator iter_fMoList = fMoList.begin();

      while ((iter_poseList != poseList.end()) && (iter_fMoList != fMoList.end()))
      {
        iP = projectCameraTrajectory(I, *iter_poseList, *iter_fMoList);
        cameraTrajectory.push_back(iP);
        if (camTrajType == CT_LINE)
        {
          if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,camTrajColor, thickness_);
        }
        else if (camTrajType == CT_POINT)
          vpDisplay::displayPoint(I,iP,camTrajColor);
        ++iter_poseList;
        ++iter_fMoList;
        iter++;
        iP_1 = iP;
      }
      extCamChanged = false;
    }
    else
    {
      iP = projectCameraTrajectory(I, cMo, fMo);
      cameraTrajectory.push_back(iP);

      for(std::list<vpImagePoint>::const_iterator it=cameraTrajectory.begin(); it!=cameraTrajectory.end(); ++it){
        if (camTrajType == CT_LINE)
        {
          if (iter != 0) vpDisplay::displayLine(I, iP_1, *it, camTrajColor, thickness_);
        }
        else if (camTrajType == CT_POINT)
          vpDisplay::displayPoint(I, *it, camTrajColor);
        iter++;
        iP_1 = *it;
      }
    }

    if (poseList.size() > nbrPtLimit)
    {
      poseList.pop_front();
    }
    if (fMoList.size() > nbrPtLimit)
    {
      fMoList.pop_front();
    }
    if (cameraTrajectory.size() > nbrPtLimit)
    {
      cameraTrajectory.pop_front();
    }
  }
}


/*!
  Get an external view. The point of view is set thanks to the pose between the camera camMf and the fixed world frame.

  \param I : The image where the external view is displayed.
  \param cam_Mf : The pose between the point of view and the fixed world frame.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpWireFrameSimulator::getExternalImage(vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cam_Mf)
{
  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;
  
  vpHomogeneousMatrix camMft = rotz * cam_Mf;

  double u;
  double v;
  //if(px_ext != 1 && py_ext != 1)
  // we assume px_ext and py_ext > 0
  if( (std::fabs(px_ext-1.) > vpMath::maximum(px_ext,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_ext-1) > vpMath::maximum(py_ext,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  vp2jlc_matrix(camMft.inverse(),w44cext);
  vp2jlc_matrix(fMo*cMo.inverse(),w44c);
  vp2jlc_matrix(fMo,w44o);

  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  
  if (displayImageSimulator)
  {
    I = 255;

    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*cam_Mf*fMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }

    if (I.display != NULL)
      vpDisplay::display(I);
  }
  
  if (displayObject)
    display_scene(w44o,this->scene,I, curColor);
  if (displayCamera)
    display_scene(w44c,camera, I, camColor);
}


/*!
  Get the internal view ie the view of the camera.

  \param I : The image where the internal view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpWireFrameSimulator::getInternalImage(vpImage<unsigned char> &I)
{
  if (!sceneInitialized)
    throw(vpException(vpException::notInitialized,"The scene has to be initialized")) ;

  double u;
  double v;
  //if(px_int != 1 && py_int != 1)
  // we assume px_int and py_int > 0
  if( (std::fabs(px_int-1.) > vpMath::maximum(px_int,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_int-1) > vpMath::maximum(py_int,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_int);
    v = (double)I.getHeight()/(2*py_int);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float o44c[4][4],o44cd[4][4],x,y,z;
  Matrix id = IDENTITY_MATRIX;

  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);
  
  if (displayImageSimulator)
  {
    I = 255;

    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*camMf*fMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }

    if (I.display != NULL)
      vpDisplay::display(I);
  }

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayObject)
    display_scene(id,this->scene,I, curColor);


  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (displayDesiredObject)
  {
    if (desiredObject == D_TOOL) display_scene(o44cd,desiredScene,I, vpColor::red);
    else display_scene(id,desiredScene,I, desColor);
  }
}


/*!
  Get the external view. It corresponds to the view of the scene from a reference frame you have to set.

  \param I : The image where the external view is displayed.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/

void
vpWireFrameSimulator::getExternalImage(vpImage<unsigned char> &I)
{
  bool changed = false;
  vpHomogeneousMatrix displacement = navigation(I,changed);

  //if (displacement[2][3] != 0 /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
  if (std::fabs(displacement[2][3]) > std::numeric_limits<double>::epsilon() /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
      camMf2 = camMf2*displacement;

  f2Mf = camMf2.inverse()*camMf;

  camMf = camMf2* displacement * f2Mf;

  double u;
  double v;
  //if(px_ext != 1 && py_ext != 1)
  // we assume px_ext and py_ext > 0
  if( (std::fabs(px_ext-1.) > vpMath::maximum(px_ext,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_ext-1) > vpMath::maximum(py_ext,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;

  vp2jlc_matrix(camMf.inverse(),w44cext);
  vp2jlc_matrix(fMc,w44c);
  vp2jlc_matrix(fMo,w44o);


  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if ((object == CUBE) || (object == SPHERE))
  {
    add_vwstack ("start","type", PERSPECTIVE);
  }
  
  if (displayImageSimulator)
  {
    I = 255;
    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*camMf*fMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }
    if (I.display != NULL)
      vpDisplay::display(I);
  }
  
  if (displayObject)
    display_scene(w44o,this->scene,I, curColor);

  if (displayCamera)
    display_scene(w44c,camera, I, camColor);

  if (displayCameraTrajectory)
  {
    vpImagePoint iP;
    vpImagePoint iP_1;
    poseList.push_back(cMo);
    fMoList.push_back(fMo);
  
    int iter = 0;

    if (changed || extCamChanged)
    {
      cameraTrajectory.clear();
      std::list<vpHomogeneousMatrix>::const_iterator iter_poseList = poseList.begin();
      std::list<vpHomogeneousMatrix>::const_iterator iter_fMoList = fMoList.begin();

      while ((iter_poseList!=poseList.end()) && (iter_fMoList!=fMoList.end()) )
      {
        iP = projectCameraTrajectory(I, *iter_poseList, *iter_fMoList);
        cameraTrajectory.push_back(iP);
        //vpDisplay::displayPoint(I,cameraTrajectory.value(),vpColor::green);
        if (camTrajType == CT_LINE)
        {
          if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,camTrajColor, thickness_);
        }
        else if (camTrajType == CT_POINT)
          vpDisplay::displayPoint(I,iP,camTrajColor);
        ++iter_poseList;
        ++iter_fMoList;
        iter++;
        iP_1 = iP;
      }
      extCamChanged = false;
    }
    else
    {
      iP = projectCameraTrajectory(I, cMo,fMo);
      cameraTrajectory.push_back(iP);

      for(std::list<vpImagePoint>::const_iterator it=cameraTrajectory.begin(); it!=cameraTrajectory.end(); ++it){
        if (camTrajType == CT_LINE){
          if (iter != 0) vpDisplay::displayLine(I,iP_1,*it,camTrajColor, thickness_);
        }
        else if(camTrajType == CT_POINT)
          vpDisplay::displayPoint(I, *it, camTrajColor);
        iter++;
        iP_1 = *it;
      }
    }

    if (poseList.size() > nbrPtLimit)
    {
      poseList.pop_front();
    }
    if (fMoList.size() > nbrPtLimit)
    {
      fMoList.pop_front();
    }
    if (cameraTrajectory.size() > nbrPtLimit)
    {
      cameraTrajectory.pop_front();
    }
  }
}


/*!
  Get an external view. The point of view is set thanks to the pose between the camera camMf and the fixed world frame.

  \param I : The image where the external view is displayed.
  \param cam_Mf : The pose between the point of view and the fixed world frame.
  
  \warning : The objects are displayed thanks to overlays. The image I is not modified.
*/
void
vpWireFrameSimulator::getExternalImage(vpImage<unsigned char> &I, const vpHomogeneousMatrix &cam_Mf)
{
  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;

  vpHomogeneousMatrix camMft = rotz * cam_Mf;
  
  double u;
  double v;
  //if(px_ext != 1 && py_ext != 1)
  // we assume px_ext and py_ext > 0
  if( (std::fabs(px_ext-1.) > vpMath::maximum(px_ext,1.)*std::numeric_limits<double>::epsilon()) 
      && (std::fabs(py_ext-1) > vpMath::maximum(py_ext,1.)*std::numeric_limits<double>::epsilon()))
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  vp2jlc_matrix(camMft.inverse(),w44cext);
  vp2jlc_matrix(fMo*cMo.inverse(),w44c);
  vp2jlc_matrix(fMo,w44o);

  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  
  if (displayImageSimulator)
  {
    I = 255;
    for(std::list<vpImageSimulator>::iterator it=objectImage.begin(); it!=objectImage.end(); ++it){
      vpImageSimulator* imSim = &(*it);
      imSim->setCameraPosition(rotz*cam_Mf*fMo);
      imSim->getImage(I,getInternalCameraParameters(I));
    }
    if (I.display != NULL)
      vpDisplay::display(I);
  }
  
  if (displayObject)
    display_scene(w44o,this->scene,I, curColor);
  if (displayCamera)
    display_scene(w44c,camera, I, camColor);
}

/*!
  Display a trajectory thanks to a list of homogeneous matrices which give the position of the camera relative to the object and the position of the object relative to the world reference frame. The trajectory is projected into the view of an external camera whose position is given in parameter.

  The two lists must have the same size of homogeneous matrices must have the same size.

  \param I : The image where the trajectory is displayed.
  \param list_cMo : The homogeneous matrices list containing the position of the camera relative to the object.
  \param list_fMo : The homogeneous matrices list containing the position of the object relative to the world reference frame.
  \param cMf : A homogeneous matrix which gives the position of the external camera (used to project the trajectory) relative to the world reference frame.
*/
void
vpWireFrameSimulator::displayTrajectory (const vpImage<unsigned char> &I, const std::list<vpHomogeneousMatrix> &list_cMo,
                                         const std::list<vpHomogeneousMatrix> &list_fMo, const vpHomogeneousMatrix &cMf)
{
  if (list_cMo.size() != list_fMo.size())
    throw(vpException(vpException::dimensionError ,"The two lists must have the same size")) ;

  vpImagePoint iP;
  vpImagePoint iP_1;
  int iter = 0;

  std::list<vpHomogeneousMatrix>::const_iterator it_cMo = list_cMo.begin();
  std::list<vpHomogeneousMatrix>::const_iterator it_fMo = list_fMo.begin();

  while ((it_cMo != list_cMo.end()) && (it_fMo != list_fMo.end()))
  {
    iP = projectCameraTrajectory(I, rotz * (*it_cMo), (*it_fMo), rotz * cMf);
    if (camTrajType == CT_LINE)
    {
      if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,camTrajColor,thickness_);
    }
    else if (camTrajType == CT_POINT)
      vpDisplay::displayPoint(I,iP,camTrajColor);
    ++it_cMo;
    ++it_fMo;
    iter++;
    iP_1 = iP;
  }
}

/*!
  Display a trajectory thanks to a list of homogeneous matrices which give the position of the camera relative to the object and the position of the object relative to the world reference frame. The trajectory is projected into the view of an external camera whose position is given in parameter.

  The two lists must have the same size of homogeneous matrices must have the same size.

  \param I : The image where the trajectory is displayed.
  \param list_cMo : The homogeneous matrices list containing the position of the camera relative to the object.
  \param list_fMo : The homogeneous matrices list containing the position of the object relative to the world reference frame.
  \param cMf : A homogeneous matrix which gives the position of the external camera (used to project the trajectory) relative to the world reference frame.
*/
void
vpWireFrameSimulator::displayTrajectory (const vpImage<vpRGBa> &I, const std::list<vpHomogeneousMatrix> &list_cMo,
                                         const std::list<vpHomogeneousMatrix> &list_fMo, const vpHomogeneousMatrix &cMf)
{
  if (list_cMo.size() != list_fMo.size())
    throw(vpException(vpException::dimensionError ,"The two lists must have the same size")) ;

  vpImagePoint iP;
  vpImagePoint iP_1;
  int iter = 0;

  std::list<vpHomogeneousMatrix>::const_iterator it_cMo = list_cMo.begin();
  std::list<vpHomogeneousMatrix>::const_iterator it_fMo = list_fMo.begin();

  while ((it_cMo != list_cMo.end()) && (it_fMo != list_fMo.end()))
  {
    iP = projectCameraTrajectory(I, rotz * (*it_cMo), (*it_fMo), rotz * cMf);
    if (camTrajType == CT_LINE)
    {
      if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,camTrajColor,thickness_);
    }
    else if (camTrajType == CT_POINT)
      vpDisplay::displayPoint(I,iP,camTrajColor);
    ++it_cMo;
    ++it_fMo;
    iter++;
    iP_1 = iP;
  }
}


/*!
  Enables to change the external camera position.
*/
vpHomogeneousMatrix
vpWireFrameSimulator::navigation(const vpImage<vpRGBa> &I, bool &changed)
{
  double width = vpMath::minimum(I.getWidth(),I.getHeight());
  vpImagePoint iP;
  vpImagePoint trash;
  bool clicked = false;
  bool clickedUp = false;
  vpMouseButton::vpMouseButtonType b = vpMouseButton::button1;

  vpHomogeneousMatrix mov(0,0,0,0,0,0);
  changed = false;
  
  //if(!blocked) vpDisplay::getClickUp(I,trash, b,false);

  if(!blocked)clicked = vpDisplay::getClick(I,trash,b,false);

  if(blocked)clickedUp = vpDisplay::getClickUp(I,trash, b,false);

  if(clicked)
  {
    if (b == vpMouseButton::button1) blockedr = true;
    if (b == vpMouseButton::button2) blockedz = true;
    if (b == vpMouseButton::button3) blockedt = true;
    blocked = true;
  }
  if(clickedUp)
  {
    if (b == vpMouseButton::button1)
    {
      old_iPr = vpImagePoint(-1,-1);
      blockedr = false;
    }
    if (b == vpMouseButton::button2)
    {
      old_iPz = vpImagePoint(-1,-1);
      blockedz = false;
    }
    if (b == vpMouseButton::button3)
    {
      old_iPt = vpImagePoint(-1,-1);
      blockedt = false;
    }
    if (!(blockedr || blockedz || blockedt))
    {
      blocked = false;
      while (vpDisplay::getClick(I,trash,b,false)) {};
    }
  }
    
  vpDisplay::getPointerPosition(I,iP);

  double anglei = 0;
  double anglej = 0;

  if (old_iPr != vpImagePoint(-1,-1) && blockedr)
  {
    double diffi = iP.get_i() - old_iPr.get_i();
    double diffj = iP.get_j() - old_iPr.get_j();
    //cout << "delta :" << diffj << endl;;
    anglei = diffi*360/width;
    anglej = diffj*360/width;
    mov.buildFrom(0,0,0,vpMath::rad(-anglei),vpMath::rad(anglej),0);
    changed = true;
  }

  if (blockedr) old_iPr = iP;

  if (old_iPz != vpImagePoint(-1,-1) && blockedz)
  {
    double diffi = iP.get_i() - old_iPz.get_i();
    mov.buildFrom(0,0,diffi*0.01,0,0,0);
    changed = true;
  }

  if (blockedz) old_iPz = iP;

  if (old_iPt != vpImagePoint(-1,-1) && blockedt)
  {
    double diffi = iP.get_i() - old_iPt.get_i();
    double diffj = iP.get_j() - old_iPt.get_j();
    mov.buildFrom(diffj*0.01,diffi*0.01,0,0,0,0);
    changed = true;
  }

  if (blockedt) old_iPt = iP;

  return mov;
}


/*!
  Enables to change the external camera position.
*/
vpHomogeneousMatrix
vpWireFrameSimulator::navigation(const vpImage<unsigned char> &I, bool &changed)
{
  double width = vpMath::minimum(I.getWidth(),I.getHeight());
  vpImagePoint iP;
  vpImagePoint trash;
  bool clicked = false;
  bool clickedUp = false;
  vpMouseButton::vpMouseButtonType b = vpMouseButton::button1;

  vpHomogeneousMatrix mov(0,0,0,0,0,0);
  changed = false;

  //if(!blocked) vpDisplay::getClickUp(I,trash, b,false);
  
  if(!blocked)clicked = vpDisplay::getClick(I,trash,b,false);

  if(blocked)clickedUp = vpDisplay::getClickUp(I,trash, b,false);

  if(clicked)
  {
    if (b == vpMouseButton::button1) blockedr = true;
    if (b == vpMouseButton::button2) blockedz = true;
    if (b == vpMouseButton::button3) blockedt = true;
    blocked = true;
  }
  if(clickedUp)
  {
    if (b == vpMouseButton::button1)
    {
      old_iPr = vpImagePoint(-1,-1);
      blockedr = false;
    }
    if (b == vpMouseButton::button2)
    {
      old_iPz = vpImagePoint(-1,-1);
      blockedz = false;
    }
    if (b == vpMouseButton::button3)
    {
      old_iPt = vpImagePoint(-1,-1);
      blockedt = false;
    }
    if (!(blockedr || blockedz || blockedt))
    {
      blocked = false;
      while (vpDisplay::getClick(I,trash,b,false)) {};
    }
  }
  
  vpDisplay::getPointerPosition(I,iP);
  
  //std::cout << "point : " << iP << std::endl;

  double anglei = 0;
  double anglej = 0;

  if (old_iPr != vpImagePoint(-1,-1) && blockedr)
  {
    double diffi = iP.get_i() - old_iPr.get_i();
    double diffj = iP.get_j() - old_iPr.get_j();
    //cout << "delta :" << diffj << endl;;
    anglei = diffi*360/width;
    anglej = diffj*360/width;
    mov.buildFrom(0,0,0,vpMath::rad(-anglei),vpMath::rad(anglej),0);
    changed = true;
  }

  if (blockedr) old_iPr = iP;

  if (old_iPz != vpImagePoint(-1,-1) && blockedz)
  {
    double diffi = iP.get_i() - old_iPz.get_i();
    mov.buildFrom(0,0,diffi*0.01,0,0,0);
    changed = true;
  }

  if (blockedz) old_iPz = iP;

  if (old_iPt != vpImagePoint(-1,-1) && blockedt)
  {
    double diffi = iP.get_i() - old_iPt.get_i();
    double diffj = iP.get_j() - old_iPt.get_j();
    mov.buildFrom(diffj*0.01,diffi*0.01,0,0,0,0);
    changed = true;
  }

  if (blockedt) old_iPt = iP;

  return mov;
}

/*!
  Project the center of the internal camera into the external camera view.
*/
vpImagePoint
vpWireFrameSimulator::projectCameraTrajectory (const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                               const vpHomogeneousMatrix &fMo_)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(rotz*(camMf*fMo_*cMo_.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}

/*!
  Project the center of the internal camera into the external camera view.
*/
vpImagePoint
vpWireFrameSimulator::projectCameraTrajectory (const vpImage<unsigned char> &I,
                                               const vpHomogeneousMatrix &cMo_,
                                               const vpHomogeneousMatrix &fMo_)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(rotz*(camMf*fMo_*cMo_.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}

/*!
  Project the center of the internal camera into the external camera view.
*/
vpImagePoint
vpWireFrameSimulator::projectCameraTrajectory (const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                                               const vpHomogeneousMatrix &fMo_, const vpHomogeneousMatrix &cMf)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(rotz*(cMf*fMo_*cMo_.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}

/*!
  Project the center of the internal camera into the external camera view.
*/
vpImagePoint
vpWireFrameSimulator::projectCameraTrajectory (const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                                               const vpHomogeneousMatrix &fMo_, const vpHomogeneousMatrix &cMf)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(rotz*(cMf*fMo_*cMo_.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}

