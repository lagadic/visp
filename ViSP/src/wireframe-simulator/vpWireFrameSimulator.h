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


#ifndef vpWireFrameSimulator_HH
#define vpWireFrameSimulator_HH

/*!
  \file vpWireFrameSimulator.h
  \brief Implementation of a wire frame simulator.
*/

#include <stdio.h>

extern "C" {
#include <visp/vpMy.h>
#include <visp/vpArit.h>
#include <visp/vpBound.h>
#include <visp/vpView.h>
#include <visp/vpToken.h>
#include <visp/vpTmstack.h>
#include <visp/vpVwstack.h>

int open_display();
int close_display();
int open_clipping();
int close_clipping();
int open_keyword (Keyword *kwp);
int open_lex ();
int open_source (FILE *fd, const char *str);
int malloc_Bound_scene (Bound_scene *bsp, const char *name,Index bn);
int free_Bound_scene (Bound_scene *bsp);
int parser (Bound_scene *bsp);
int close_source ();
int close_lex ();
int close_keyword ();
void add_rfstack (int i);
void load_rfstack (int i);
void add_vwstack (const char* path, ... );
void display_scene(Matrix mat, Bound_scene sc);
int * get_rfstack ();
Matrix	* get_tmstack ();
int View_to_Matrix (View_parameters *vp, Matrix m);
void postmult_matrix (Matrix a, Matrix b);
Bound *clipping_Bound (Bound *bp, Matrix m);
int set_Bound_face_display (Bound *bp, Byte b);
int point_3D_2D (Point3f *p3, Index size, int xsize, int ysize, Point2i *p2);
void point_3D_4D (Point3f *p3, int size, Matrix m, Point4f *p4);
int wireframe_Face (Face *fp, Point2i *pp);
}

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDisplay.h>
#include <visp/vpList.h>
#include <visp/vpImagePoint.h>


/*!
  \class vpWireFrameSimulator

  \ingroup VisuRobotSimu
  
  \brief Implementation of a wire frame simulator. Compared to the vpSimulator class, it does not
  require third party libraries to be used.
  
  The simulator uses several frames to display the scene in the image. There are:
  
  - The world frame : This is a fixed frame used to compute the position of the other frames against each other. By default this frame is positionned at the same location as the initial object frame position.
  
  - The object frame : It is the frame linked to the object.
  
  - The camera frame : It is the frame linked to the main camera.
  
  - The external camera frame : It is the frame which corresponds to one external point of view.
  
  The most used homogeneous matrices which describes the displacement between two frames are :
  
  - wMo which is the displacement between the world frame and the object frame.
  
  - cMo which is the displacement between the main camera frame and the object frame.
  
  - cextMw which is the displacement between one external camera frame and the world frame.
  
  The following picture summarize all the previous informations.
  
  \image html vpWireFrameSimulator.jpeg
  \image latex vpWireFrameSimulator.ps
  
  The simulator uses .bnd files as 3D scene descriptors. Several scenes can be found in the ViSP-data archive which is dowloadable from the ViSP web site.
  
  You can move the main external view while clicking in the image. The left click enables to turn, the middle button enables to zoom and the left to translate along x and y.
  
  The simulator is able to take into account to camera parameters. You can set the internal and external cameras parameters thanks to a vpCameraParameters.
  
  The following example shows how it is easy to use.
  
  \code
  
  #include <visp/vpImage.h>
  #include <visp/vpWireFrameSimulator.h>
  
  int main()
  {
    vpWireFrameSimulator sim;
    
    vpImage<vpRGBa> Iint(480,640,255);
    vpImage<vpRGBa> Iext(480,640,255);
    
    //Set the type of scene to use
    sim.initScene(vpWireFrameSimulator::PLAQUE, vpWireFrameSimulator::MOTIF_STANDARD);
    
    //Set the initial pose of the camera
    sim.setCameraPosition(vpHomogeneousMatrix(0,0,0.5,vpMath::rad(0),vpMath::rad(10),0));
    //Set the desired pose of the camera (for the internal view)
    sim.setDesiredCameraPosition(vpHomogeneousMatrix(0.0,0,0.5,0,0,0));
    //Set the pose of the reference frame (for the external view)
    sim.setExternalCameraPosition(vpHomogeneousMatrix(0.1,0,0.2,0,0,0));
    
    //Set the camera parameters
    vpCameraParameters camera(1000,1000,320,240);
    sim.setInternalCameraParameters(camera);
    sim.setExternalCameraParameters(camera);
    
    //Get the internal view
    sim.getInternalImage(Iint);
    
    //Get the external view
    sim.getExternalImage(Iext);
    
    return 0;
  }
  \endcode
*/

class VISP_EXPORT vpWireFrameSimulator
{
  public:
    
    /*!
      Type of scene used to display the object at the current position.
    */
    typedef enum  
    {
      THREE_PTS,
      CUBE,
      PLATE,
      SMALL_PLATE,
      RECTANGLE,
      SQUARE_5CM,
      DIAMOND,
      TRAPEZOID,
      THREE_LINES,
      ROAD,
      TIRE,
      PIPE,
      CIRCLE,
      SPHERE,
      CYLINDER,
      PLAN
    } vpSceneObject;
    
    /*!
      Type of scene used to display the object at the desired pose (in the internal view).
      
      - MOTIF_STANDARD will use the vpSceneObject used to be the object at the current position.
      - MOTIF_OUTIL will display a tool which is attached to the camera.
    */
    typedef enum  
    {
      MOTIF_STANDARD,
      MOTIF_CIRCLE,
      MOTIF_TOOL
    } vpSceneMotif;
    
    
  private:
    Bound_scene scene;
    Bound_scene camera;
    Bound_scene motif;
    
    vpHomogeneousMatrix wMo;
    vpHomogeneousMatrix camMw;
    vpHomogeneousMatrix refMo;
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix cdMo;
    
    vpSceneObject object;
    vpSceneMotif motifType;
    
    vpColor camColor;
    vpColor curColor;
    vpColor desColor;
    
    bool sceneInitialized;
    
    bool displayCameraTrajectory;
    vpList<vpImagePoint> cameraTrajectory;
    vpList<vpHomogeneousMatrix> poseList;
    vpList<vpHomogeneousMatrix> wMoList;
    int nbrPtLimit;
    
    
    vpImagePoint old_iPr;
    vpImagePoint old_iPz;
    vpImagePoint old_iPt;
    bool blockedr;
    bool blockedz;
    bool blockedt;
    bool blocked;
    
    vpHomogeneousMatrix camMw2;
    vpHomogeneousMatrix  w2Mw;
    
    double px_int;
    double py_int;
    double px_ext;
    double py_ext;
  
  public:
    vpWireFrameSimulator();
    virtual ~vpWireFrameSimulator();
    
    void initScene(vpSceneObject obj, vpSceneMotif motif);
    void initScene(const char* obj, const char* motif);
    
    /*!
      Set the position of the camera relative to the object.
      
      \param cMo : The pose of the camera.
    */
    void setCameraPosition(const vpHomogeneousMatrix cMo) {this->cMo = vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180)) * cMo;}
    
    /*!
      Set the desired position of the camera relative to the object.
      
      \param cdMo : The desired pose of the camera.
    */
    void setDesiredCameraPosition(const vpHomogeneousMatrix cdMo) {this->cdMo = vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180)) * cdMo;}
    
    /*!
      Set the external camera point of view.
      
      \param camMw : The pose of the external camera relative to the world reference frame.
    */
    void setExternalCameraPosition(const vpHomogeneousMatrix camMw) 
    {
      this->camMw = vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180)) * camMw;
      vpTranslationVector T;
      this->camMw.extract (T);
      this->camMw2.buildFrom(0,0,T[2],0,0,0);
      w2Mw =camMw2.inverse()*this->camMw;
    }
    
    /*!
      Set the color used to display the camera in the external view.
      
      \param col : The desired color.
    */
    void setCameraColor(const vpColor col) {camColor = col;}
    
    /*!
      Set the color used to display the object at the current position.
      
      \param col : The desired color.
    */
    void setCurrentViewColor(const vpColor col) {curColor = col;}
    
    /*!
      Set the color used to display the object at the desired position.
      
      \param col : The desired color.
    */
    void setDesiredViewColor(const vpColor col) {desColor = col;}
    
    /*!
      Enable or disable the displaying of the camera trajectory in the main external camera view.
      
      By default the trajectory is displayed.
      
      \param displayCameraTrajectory : Set to true to display the camera trajectory.
    */
    void setDisplayCameraTrajectory (const bool displayCameraTrajectory) {this->displayCameraTrajectory = displayCameraTrajectory;}
    
    /*!
      Get the pose between the external camera and the fixed world frame.
      
      \return It returns the desired pose as a vpHomogeneousMatrix
    */
    vpHomogeneousMatrix get_cMw() const {return vpHomogeneousMatrix(0,0,0,vpMath::rad(0),vpMath::rad(0),vpMath::rad(180))*camMw;}
    
    /*!
      Get the parameters of the virtual internal camera.
      
      \param I : The image used to display the view of the camera.
      
      \return It returns the camera parameters.
    */
    vpCameraParameters getInternalCameraParameters(const vpImage<vpRGBa> &I) const {
      if(px_int != 1 && py_int != 1)
        return vpCameraParameters(px_int,py_int,I.getWidth()/2,I.getHeight()/2);
      else
      {
        int size = vpMath::minimum(I.getWidth(),I.getHeight())/2;
        return vpCameraParameters(size,size,I.getWidth()/2,I.getHeight()/2);
      }
    }
    
    /*!
      Get the parameters of the virtual internal camera.
      
      \param I : The image used to display the view of the camera.
      
      \return It returns the camera parameters.
    */
    vpCameraParameters getInternalCameraParameters(const vpImage<unsigned char> &I) const {
      if(px_int != 1 && py_int != 1)
        return vpCameraParameters(px_int,py_int,I.getWidth()/2,I.getHeight()/2);
      else
      {
        int size = vpMath::minimum(I.getWidth(),I.getHeight())/2;
        return vpCameraParameters(size,size,I.getWidth()/2,I.getHeight()/2);
      }
    }
    
    /*!
      Get the parameters of the virtual external camera.
      
      \param I : The image used to display the view of the camera.
      
      \return It returns the camera parameters.
    */
    vpCameraParameters getExternalCameraParameters(const vpImage<vpRGBa> &I) const {
      if(px_ext != 1 && py_ext != 1)
        return vpCameraParameters(px_ext,py_ext,I.getWidth()/2,I.getHeight()/2);
      else
      {
        int size = vpMath::minimum(I.getWidth(),I.getHeight())/2;
        return vpCameraParameters(size,size,I.getWidth()/2,I.getHeight()/2);
      }
    }
    
    /*!
      Get the parameters of the virtual external camera.
      
      \param I : The image used to display the view of the camera.
      
      \return It returns the camera parameters.
    */
    vpCameraParameters getExternalCameraParameters(const vpImage<unsigned char> &I) const {
      if(px_ext != 1 && py_ext != 1)
        return vpCameraParameters(px_ext,py_ext,I.getWidth()/2,I.getHeight()/2);
      else
      {
        int size = vpMath::minimum(I.getWidth(),I.getHeight())/2;
        return vpCameraParameters(size,size,I.getWidth()/2,I.getHeight()/2);
      }
    }
    
    /*!
      Set the pose between the object and the fixed world frame.
      
      \param wMo : The pose between the object and the fixed world frame.
    */
    void moveObject(const vpHomogeneousMatrix &wMo) {this->wMo = wMo;}
    
    /*!
      Get the pose between the object and the fixed world frame.
      
      \return The pose between the object and the fixed world frame.
    */
    vpHomogeneousMatrix get_wMo() const {return wMo;}
    
    /*!
      Set the internal camera parameters.
      
      \param cam : The desired camera parameters.
    */
    inline void setInternalCameraParameters(const vpCameraParameters cam) {
      px_int = cam.get_px();
      py_int = cam.get_py();
    }
    
    /*!
      Set the internal camera parameters.
      
      \param cam : The desired camera parameters.
    */
    inline void setExternalCameraParameters(const vpCameraParameters cam) {
      px_ext = cam.get_px();
      py_ext = cam.get_py();
    }
    
    void getInternalImage(vpImage<vpRGBa> &I);
    void getExternalImage(vpImage<vpRGBa> &I);
    void getExternalImage(vpImage<vpRGBa> &I, vpHomogeneousMatrix camMw);
    
    void getInternalImage(vpImage<unsigned char> &I);
    void getExternalImage(vpImage<unsigned char> &I);
    void getExternalImage(vpImage<unsigned char> &I, vpHomogeneousMatrix camMw);
    
  private:
    void display_scene(Matrix mat, Bound_scene sc, vpImage<vpRGBa> &I, vpColor color);
    void display_scene(Matrix mat, Bound_scene sc, vpImage<unsigned char> &I, vpColor color);
    vpHomogeneousMatrix navigation(vpImage<vpRGBa> &I, bool &changed);
    vpHomogeneousMatrix navigation(vpImage<unsigned char> &I, bool &changed);
    vpImagePoint projectCameraTrajectory (vpImage<vpRGBa> &I, vpHomogeneousMatrix cMo, vpHomogeneousMatrix wMo);
    vpImagePoint projectCameraTrajectory (vpImage<unsigned char> &I, vpHomogeneousMatrix cMo, vpHomogeneousMatrix wMo);

};

#endif