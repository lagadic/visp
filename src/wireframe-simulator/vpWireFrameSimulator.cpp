/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Wire frame simulator
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \file vpWireFrameSimulator.h
  \brief Implementation of a wire frame simulator.
*/

#include <visp/vpWireFrameSimulator.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <visp/vpSimulatorException.h>
#include <visp/vpPoint.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMeterPixelConversion.h>

#if defined(WIN32)
#define bcopy(b1,b2,len) (memmove((b2), (b1), (len)), (void) 0) 
#endif

extern "C"{extern Point2i *point2i;}
extern "C"{extern Point2i *listpoint2i;}

/*
   Enable to initialize the scene
*/
void set_scene (const char* str, Bound_scene *sc)
{
  FILE  *fd;

  //if ((fd = fopen (str, 0)) == -1)
  if ((fd = fopen (str, "r")) == NULL)
  {std::cout << "Le short c'est le top " << std::endl;
    char strerr[80];
    strcpy (strerr,"The file ");
    strcat (strerr,str);
    strcat (strerr," can not be opened");
    throw(vpException(vpSimulatorException::ioError,strerr)) ;
  }
  open_keyword (keyword_tbl);
  open_lex ();
  open_source (fd, str);
  malloc_Bound_scene (sc, str,(Index)BOUND_NBR);
  parser (sc);
  close_source ();
  close_lex ();
  close_keyword ();
}

/*
  Convert the matrix format to deal with the one in the simulator
*/
void vp2jlc_matrix (const vpHomogeneousMatrix vpM, Matrix &jlcM)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++) jlcM[j][i] = (float)vpM[i][j];
  }
}

/*
  Copy the scene corresponding to the registeresd parameters in the image.
*/
void
vpWireFrameSimulator::display_scene(Matrix mat, Bound_scene sc, vpImage<vpRGBa> &I, vpColor color)
{
  extern Bound *clipping_Bound ();
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

      point_3D_2D (clip->point.ptr, clip->point.nbr,I.getWidth(),I.getHeight(),point2i);
      for (; fp < fend; fp++)
      {
        if (fp->is_visible)
	{
	  wireframe_Face (fp, point2i);
	  Point2i *pt = listpoint2i;
	  for (int i = 1; i < fp->vertex.nbr; i++)
	  {
	    vpDisplay::displayLine(I,vpImagePoint((pt)->y,(pt)->x),vpImagePoint((pt+1)->y,(pt+1)->x),color,1);
	    pt++;
	  }
	  if (fp->vertex.nbr > 2)
	  {
	    vpDisplay::displayLine(I,vpImagePoint((listpoint2i)->y,(listpoint2i)->x),vpImagePoint((pt)->y,(pt)->x),color,1);
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
vpWireFrameSimulator::display_scene(Matrix mat, Bound_scene sc, vpImage<unsigned char> &I, vpColor color)
{
  extern Bound *clipping_Bound ();
//  extern Point2i *point2i;
//  extern Point2i *listpoint2i;
  Bound *bp, *bend;
  Bound *clip; /* surface apres clipping */
  Byte b  = (Byte) *get_rfstack ();
  Matrix m;


  bcopy ((char *) mat, (char *) m, sizeof (Matrix));
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

      point_3D_2D (clip->point.ptr, clip->point.nbr,I.getWidth(),I.getHeight(),point2i);
      for (; fp < fend; fp++)
      {
        if (fp->is_visible)
	{
	  wireframe_Face (fp, point2i);
	  Point2i *pt = listpoint2i;
	  for (int i = 1; i < fp->vertex.nbr; i++)
	  {
	    vpDisplay::displayLine(I,vpImagePoint((pt)->y,(pt)->x),vpImagePoint((pt+1)->y,(pt+1)->x),color,1);
	    pt++;
	  }
	  if (fp->vertex.nbr > 2)
	  {
	    vpDisplay::displayLine(I,vpImagePoint((listpoint2i)->y,(listpoint2i)->x),vpImagePoint((pt)->y,(pt)->x),color,1);
	  }
	}
      }
    }
  }
}

vpImagePoint getCameraPosition(Matrix mat, vpImage<vpRGBa> &I)
{
  Matrix m;

  bcopy ((char *) mat, (char *) m, sizeof (Matrix));
  View_to_Matrix (get_vwstack (), *(get_tmstack ()));
  postmult_matrix (m, *(get_tmstack ()));

  Point3f p3[1];
  Point2i p2[1];
  Point4f p4[1];

  p3[0].x = 0;//mat[3][0];
  p3[0].y = 0;//mat[3][1];
  p3[0].z = 0;//mat[3][2];

  point_3D_4D (p3, 1, m, p4);
  p3->x = p4->x / p4->w;
  p3->y = p4->y / p4->w;
  p3->z = p4->z / p4->w;

  int size = vpMath::minimum(I.getWidth(),I.getHeight());
  point_3D_2D (p3, 1,size,size,p2);

  vpImagePoint iP((p2)->y,(p2)->x);

  //printf ( "vali : %f\n valj : %f\n\n", iP.get_i(),iP.get_j());
  return iP;
}

vpImagePoint getCameraPosition(Matrix mat, vpImage<unsigned char> &I)
{
  Matrix m;

  bcopy ((char *) mat, (char *) m, sizeof (Matrix));
  View_to_Matrix (get_vwstack (), *(get_tmstack ()));
  postmult_matrix (m, *(get_tmstack ()));

  Point3f p3[1];
  Point2i p2[1];
  Point4f p4[1];

  p3[0].x = 0;//mat[3][0];
  p3[0].y = 0;//mat[3][1];
  p3[0].z = 0;//mat[3][2];

  point_3D_4D (p3, 1, m, p4);
  p3->x = p4->x / p4->w;
  p3->y = p4->y / p4->w;
  p3->z = p4->z / p4->w;

  int size = vpMath::minimum(I.getWidth(),I.getHeight());
  point_3D_2D (p3, 1,size,size,p2);

  vpImagePoint iP((p2)->y,(p2)->x);

  //printf ( "vali : %f\n valj : %f\n\n", iP.get_i(),iP.get_j());
  return iP;
}


/*************************************************************************************************************/

/*!
  Basic constructor
*/
vpWireFrameSimulator::vpWireFrameSimulator()
{
  open_display();
  open_clipping();

  camColor = vpColor::green;
  curColor = vpColor::blue;
  desColor = vpColor::red;

  sceneInitialized = false;

  displayCameraTrajectory = true;
  cameraTrajectory.kill();
  poseList.kill();
  wMoList.kill();

  wMo.setIdentity();

  old_iPr = vpImagePoint(-1,-1);
  old_iPz = vpImagePoint(-1,-1);
  old_iPt = vpImagePoint(-1,-1);
  blockedr = false;
  blockedz = false;
  blockedt = false;
  blocked = false;

  nbrPtLimit = 1000;
  
  px_int = 1;
  py_int = 1;
  px_ext = 1;
  py_ext = 1;
}


/*!
  Basic destructor
*/
vpWireFrameSimulator::~vpWireFrameSimulator()
{
  if(sceneInitialized)
  {
    free_Bound_scene (&(this->scene));
    free_Bound_scene (&(this->camera));
    free_Bound_scene (&(this->motif));
  }
  close_display ();
  close_clipping ();

  cameraTrajectory.kill();
  poseList.kill();
  wMoList.kill();
}


/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  It exists several default scenes you can use. Use the vpSceneObject and the vpSceneMotif attributes to use them in this method. The corresponding files are stored in the data folder which is in the ViSP build directory.

  \param obj : Type of scene used to display the object at the current position.
  \param motif : Type of scene used to display the object at the desired pose (in the internal view).
*/
void
vpWireFrameSimulator::initScene(vpSceneObject obj, vpSceneMotif motif)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = obj;
  this->motifType = motif;

  strcpy(name_cam,VISP_SCENES_DIR);
  if (motif != MOTIF_TOOL) 
  {
    strcat(name_cam,"/camera.bnd");
    set_scene(name_cam,&camera);
  }
  else
  {
    strcat(name_cam,"/tool.bnd");
    set_scene(name_cam,&(this->camera));
  }

  strcpy(name,VISP_SCENES_DIR);
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
  }
  set_scene(name,&(this->scene));

  switch (motif)
  {
    case MOTIF_STANDARD : { break; }
    case MOTIF_CIRCLE : { 
      strcpy(name,VISP_SCENES_DIR);
      strcat(name, "/cercle_sq2.bnd");
      break; }
    case MOTIF_TOOL : { 
      strcpy(name,VISP_SCENES_DIR);
      strcat(name, "/tool.bnd");
      break; }
  }
  set_scene(name,&(this->motif));

  if (obj == PIPE) load_rfstack(IS_INSIDE);
  else add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
}

/*!
  Initialize the simulator. It enables to choose the type of scene which will be used to display the object
  at the current position and at the desired position.
  
  Here you can use the scene you want. You have to set the path to the .bnd file which is a scene file.

  \param obj : Path to the scene file you want to use.
  \param motif : Path to the scene file you want to use.
*/
void
vpWireFrameSimulator::initScene(const char* obj, const char* motif)
{
  char name_cam[FILENAME_MAX];
  char name[FILENAME_MAX];

  object = THREE_PTS;
  this->motifType = MOTIF_STANDARD;
  
  strcpy(name_cam,VISP_SCENES_DIR);
  strcat(name_cam,"/camera.bnd");
  set_scene(name_cam,&camera);

  strcpy(name,obj);
  set_scene(name,&(this->scene));

  strcpy(name,motif);
  set_scene(name,&(this->motif));

  add_rfstack(IS_BACK);

  add_vwstack ("start","depth", 0.0, 100.0);
  add_vwstack ("start","window", -0.1,0.1,-0.1,0.1);
  add_vwstack ("start","type", PERSPECTIVE);

  sceneInitialized = true;
}


/*!
  Get the internal view ie the view of the camera.

  \param I : The image where the internal view is stored.
*/
void
vpWireFrameSimulator::getInternalImage(vpImage<vpRGBa> &I)
{
  if (!sceneInitialized)
    throw(vpException(vpSimulatorException::notInitializedError,"The scene has to be initialized")) ;

  double u;
  double v;
  if(px_int != 1 && py_int != 1)
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
  Matrix id = IDENTITY_MATRIX;/*{ {1.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0} } ;*/

  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  display_scene(id,this->scene,I, curColor);


  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (motifType == MOTIF_TOOL) display_scene(o44cd,motif,I, vpColor::red);
  else display_scene(id,motif,I, desColor);

}


/*!
  Get the external view. It corresponds to the view of the scene from a reference frame you have to set.

  \param I : The image where the external view is stored.
*/

void
vpWireFrameSimulator::getExternalImage(vpImage<vpRGBa> &I)
{
  bool changed = false;
  vpHomogeneousMatrix displacement = navigation(I,changed);

  if (displacement[2][3] != 0 /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
      camMw2 = camMw2*displacement;

  w2Mw = camMw2.inverse()*camMw;

  camMw = camMw2* displacement * w2Mw;

  double u;
  double v;
  if(px_ext != 1 && py_ext != 1)
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

  vp2jlc_matrix(camMw.inverse(),w44cext);
  vp2jlc_matrix(wMo*cMo.inverse(),w44c);
  vp2jlc_matrix(wMo,w44o);


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
   // add_vwstack ("start","window", -0.1, 0.1, -0.1, 0.1);
  }
  display_scene(w44o,this->scene,I, curColor);

  display_scene(w44c,camera, I, camColor);

  if (displayCameraTrajectory)
  {
    vpImagePoint iP;
    vpImagePoint iP_1;
    poseList.end();
    poseList.addRight(cMo);
    wMoList.end();
    wMoList.addRight(wMo);
  
    int iter = 0;

    if (changed)
    {
      cameraTrajectory.kill();
      poseList.front();
      wMoList.front();
      while (!poseList.outside() && !wMoList.outside())
      {
        iP = projectCameraTrajectory(I, poseList.value(),wMoList.value());
        cameraTrajectory.addRight(iP);
        //vpDisplay::displayPoint(I,cameraTrajectory.value(),vpColor::green);
        if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,vpColor::green);
        poseList.next();
        wMoList.next();
        iter++;
        iP_1 = iP;
      }
    }
    else
    {
      iP = projectCameraTrajectory(I, poseList.value(),wMoList.value());
      cameraTrajectory.end();
      cameraTrajectory.addRight(iP);
      cameraTrajectory.front();
      while (!cameraTrajectory.outside())
      {
        //vpDisplay::displayPoint(I,cameraTrajectory.value(),vpColor::green);
        if (iter != 0) vpDisplay::displayLine(I,iP_1,cameraTrajectory.value(),vpColor::green);
        iter++;
        iP_1 = cameraTrajectory.value();
        cameraTrajectory.next();
      }
    }

    if (poseList.nbElement() > nbrPtLimit)
    {
      poseList.front();
      poseList.suppress();
    }
    if (wMoList.nbElement() > nbrPtLimit)
    {
      wMoList.front();
      wMoList.suppress();
    }
    if (cameraTrajectory.nbElement() > nbrPtLimit)
    {
      cameraTrajectory.front();
      cameraTrajectory.suppress();
    }
  }
}


/*!
  Get an external view. The point of view is set thanks to the pose between the camera camMw and the fixed world frame.

  \param I : The image where the external view is stored.
  \param camMw : The pose between the point of view and the fixed world frame.
*/
void
vpWireFrameSimulator::getExternalImage(vpImage<vpRGBa> &I, vpHomogeneousMatrix camMw)
{
  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;
  
  vpHomogeneousMatrix camMwt = vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180)) * camMw;

  double u;
  double v;
  if(px_ext != 1 && py_ext != 1)
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  vp2jlc_matrix(camMwt.inverse(),w44cext);
  vp2jlc_matrix(wMo*cMo.inverse(),w44c);
  vp2jlc_matrix(wMo,w44o);

  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  
  display_scene(w44o,this->scene,I, curColor);
  display_scene(w44c,camera, I, camColor);
}


/*!
  Get the internal view ie the view of the camera.

  \param I : The image where the internal view is stored.
*/
void
vpWireFrameSimulator::getInternalImage(vpImage<unsigned char> &I)
{
  if (!sceneInitialized)
    throw(vpException(vpSimulatorException::notInitializedError,"The scene has to be initialized")) ;

  double u;
  double v;
  if(px_int != 1 && py_int != 1)
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
  Matrix id = IDENTITY_MATRIX;/*{ {1.0, 0.0, 0.0, 0.0},
                {0.0, 1.0, 0.0, 0.0},
		{0.0, 0.0, 1.0, 0.0},
		{0.0, 0.0, 0.0, 1.0} } ;*/

  vp2jlc_matrix(cMo.inverse(),o44c);
  vp2jlc_matrix(cdMo.inverse(),o44cd);

  add_vwstack ("start","cop", o44c[3][0],o44c[3][1],o44c[3][2]);
  x = o44c[2][0] + o44c[3][0];
  y = o44c[2][1] + o44c[3][1];
  z = o44c[2][2] + o44c[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44c[2][0],o44c[2][1],o44c[2][2]);
  add_vwstack ("start","vup", o44c[1][0],o44c[1][1],o44c[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  display_scene(id,this->scene,I, curColor);


  add_vwstack ("start","cop", o44cd[3][0],o44cd[3][1],o44cd[3][2]);
  x = o44cd[2][0] + o44cd[3][0];
  y = o44cd[2][1] + o44cd[3][1];
  z = o44cd[2][2] + o44cd[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", o44cd[2][0],o44cd[2][1],o44cd[2][2]);
  add_vwstack ("start","vup", o44cd[1][0],o44cd[1][1],o44cd[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  if (motifType == MOTIF_TOOL) display_scene(o44cd,motif,I, vpColor::red);
  else display_scene(id,motif,I, desColor);

}


/*!
  Get the external view. It corresponds to the view of the scene from a reference frame you have to set.

  \param I : The image where the external view is stored.
*/

void
vpWireFrameSimulator::getExternalImage(vpImage<unsigned char> &I)
{
  bool changed = false;
  vpHomogeneousMatrix displacement = navigation(I,changed);

  if (displacement[2][3] != 0 /*|| rotation[0][3] != 0 || rotation[1][3] != 0*/)
      camMw2 = camMw2*displacement;

  w2Mw = camMw2.inverse()*camMw;

  camMw = camMw2* displacement * w2Mw;

  double u;
  double v;
  if(px_ext != 1 && py_ext != 1)
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

  vp2jlc_matrix(camMw.inverse(),w44cext);
  vp2jlc_matrix(wMo*cMo.inverse(),w44c);
  vp2jlc_matrix(wMo,w44o);


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
  display_scene(w44o,this->scene,I, curColor);

  display_scene(w44c,camera, I, camColor);

  if (displayCameraTrajectory)
  {
    vpImagePoint iP;
    vpImagePoint iP_1;
    poseList.end();
    poseList.addRight(cMo);
    wMoList.end();
    wMoList.addRight(wMo);
  
    int iter = 0;

    if (changed)
    {
      cameraTrajectory.kill();
      poseList.front();
      wMoList.front();
      while (!poseList.outside() && !wMoList.outside())
      {
        iP = projectCameraTrajectory(I, poseList.value(),wMoList.value());
        cameraTrajectory.addRight(iP);
        //vpDisplay::displayPoint(I,cameraTrajectory.value(),vpColor::green);
        if (iter != 0) vpDisplay::displayLine(I,iP_1,iP,vpColor::green);
        poseList.next();
        wMoList.next();
        iter++;
        iP_1 = iP;
      }
    }
    else
    {
      iP = projectCameraTrajectory(I, poseList.value(),wMoList.value());
      cameraTrajectory.end();
      cameraTrajectory.addRight(iP);
      cameraTrajectory.front();
      while (!cameraTrajectory.outside())
      {
        //vpDisplay::displayPoint(I,cameraTrajectory.value(),vpColor::green);
        if (iter != 0) vpDisplay::displayLine(I,iP_1,cameraTrajectory.value(),vpColor::green);
        iter++;
        iP_1 = cameraTrajectory.value();
        cameraTrajectory.next();
      }
    }

    if (poseList.nbElement() > nbrPtLimit)
    {
      poseList.front();
      poseList.suppress();
    }
    if (wMoList.nbElement() > nbrPtLimit)
    {
      wMoList.front();
      wMoList.suppress();
    }
    if (cameraTrajectory.nbElement() > nbrPtLimit)
    {
      cameraTrajectory.front();
      cameraTrajectory.suppress();
    }
  }
}


/*!
  Get an external view. The point of view is set thanks to the pose between the camera camMw and the fixed world frame.

  \param I : The image where the external view is stored.
  \param camMw : The pose between the point of view and the fixed world frame.
*/
void
vpWireFrameSimulator::getExternalImage(vpImage<unsigned char> &I, vpHomogeneousMatrix camMw)
{
  float w44o[4][4],w44cext[4][4],w44c[4][4],x,y,z;

  vpHomogeneousMatrix camMwt = vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180)) * camMw;
  
  double u;
  double v;
  if(px_ext != 1 && py_ext != 1)
  {
    u = (double)I.getWidth()/(2*px_ext);
    v = (double)I.getHeight()/(2*py_ext);
  }
  else
  {
    u = (double)I.getWidth()/(vpMath::minimum(I.getWidth(),I.getHeight()));
    v = (double)I.getHeight()/(vpMath::minimum(I.getWidth(),I.getHeight()));
  }

  vp2jlc_matrix(camMwt.inverse(),w44cext);
  vp2jlc_matrix(wMo*cMo.inverse(),w44c);
  vp2jlc_matrix(wMo,w44o);

  add_vwstack ("start","cop", w44cext[3][0],w44cext[3][1],w44cext[3][2]);
  x = w44cext[2][0] + w44cext[3][0];
  y = w44cext[2][1] + w44cext[3][1];
  z = w44cext[2][2] + w44cext[3][2];
  add_vwstack ("start","vrp", x,y,z);
  add_vwstack ("start","vpn", w44cext[2][0],w44cext[2][1],w44cext[2][2]);
  add_vwstack ("start","vup", w44cext[1][0],w44cext[1][1],w44cext[1][2]);
  add_vwstack ("start","window", -u, u, -v, v);
  
  display_scene(w44o,this->scene,I, curColor);
  display_scene(w44c,camera, I, camColor);
}

/*!
  Enables to change the external camera position.
*/
vpHomogeneousMatrix
vpWireFrameSimulator::navigation(vpImage<vpRGBa> &I, bool &changed)
{
  double width = vpMath::minimum(I.getWidth(),I.getHeight());
  vpImagePoint iP;
  vpImagePoint trash;
  bool clicked = false;
  bool clickedUp = false;
  vpMouseButton::vpMouseButtonType b = vpMouseButton::button1;

  vpHomogeneousMatrix mov(0,0,0,0,0,0);
  changed = false;
  
  if(!blocked) vpDisplay::getClickUp(I,trash, b,false);

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
  
//   std::cout << "clicked : " << clicked << std::endl;
//   std::cout << "clickedUp : " << clickedUp << std::endl;
//   std::cout << "blockedr : " << blockedr << std::endl;
//   std::cout << "blockedz : " << blockedz << std::endl;
//   std::cout << "blockedt : " << blockedt << std::endl;
//   std::cout << "blocked : " << blocked << std::endl;
  
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
  Enables to change the external camera position.
*/
vpHomogeneousMatrix
vpWireFrameSimulator::navigation(vpImage<unsigned char> &I, bool &changed)
{
  double width = vpMath::minimum(I.getWidth(),I.getHeight());
  vpImagePoint iP;
  vpImagePoint trash;
  bool clicked = false;
  bool clickedUp = false;
  vpMouseButton::vpMouseButtonType b = vpMouseButton::button1;

  vpHomogeneousMatrix mov(0,0,0,0,0,0);
  changed = false;

  if(!blocked) vpDisplay::getClickUp(I,trash, b,false);
  
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
vpWireFrameSimulator::projectCameraTrajectory (vpImage<vpRGBa> &I, vpHomogeneousMatrix cMo, vpHomogeneousMatrix wMo)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180))*(camMw*wMo*cMo.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}

/*!
  Project the center of the internal camera into the external camera view.
*/
vpImagePoint
vpWireFrameSimulator::projectCameraTrajectory (vpImage<unsigned char> &I, vpHomogeneousMatrix cMo, vpHomogeneousMatrix wMo)
{
  vpPoint point;
  point.setWorldCoordinates(0,0,0);

  point.track(vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180))*(camMw*wMo*cMo.inverse())) ;

  vpImagePoint iP;

  vpMeterPixelConversion::convertPoint ( getExternalCameraParameters(I), point.get_x(), point.get_y(),iP );

  return iP;
}
