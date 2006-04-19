/*                                                                -*-c++-*-
    Copyright (C) 2005  IRISA-INRIA Rennes Vista Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

*/

#ifndef vpSimulator_HH
#define vpSimulator_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT

/*   KNOWN ISSUE DEALING WITH X11 and QT
     If you get a strange compiler error on the line with None,
     it's probably because you're also including X11 headers,
     which #define the symbol None. Put the X11 includes after
     the Qt includes to solve this problem.
 */

// Qt and Coin stuff
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/SoOffscreenRenderer.h>
#include <Inventor/fields/SoSFTime.h>

// open GL
#include <GL/gl.h>

// thread
#include <pthread.h>

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>


#include <visp/vpViewer.h>

class vpSimulator
{
private:
  //! perform some initialization
  void init() ;
  //! perform some destruction
  void kill() ;

public:
  //! constructor
  vpSimulator() ;
  ~vpSimulator() ;

private:
  //! main Widget
  QWidget * mainWindow ;
  bool mainWindowInitialized ;

  //! open the Qt application
  void initSoQt() ;

public:
  //! activate the mainloop
  void mainLoop() ;

private:
  //! view from the camera
  vpViewer *internalView ;
  //! view from an external camera
  vpViewer *externalView ;

public:
  //! initialize the camera view
  void initInternalViewer(int nlig, int ncol) ;
  //! initialize the external view
  void initExternalViewer(int nlig, int ncol) ;

private:
  //! thread with the main program
  pthread_t mainThread;

public:
  //! begin the main program
  void initApplication(void *(*start_routine)(void *)) ;
  //! perform some initialization in the main program thread
  void initMainApplication() ;
  void closeMainApplication() ;

  //----------------------------------------------------
  // scene description
private:
  int internal_width;
  int internal_height;
  int external_width;
  int external_height;

  //! root node of the scene : contains everything except stuff specific to
  //! each view
  SoSeparator *scene;
  //! root node of the internal view
  SoSeparator *internalRoot;
  //! root node of the external view
  SoSeparator *externalRoot;

  //! camera
  SoPerspectiveCamera *internalCamera ;

  //! internal camera position
  SoTransform *internalCameraPosition ;

  //! external camera position
  SoTransform *extrenalCameraPosition ;

  //! representation of the camera in the external view
  SoSeparator *internalCameraObject;

  //! initialize the scene graph
  void initSceneGraph() ;

  //!Add a new object in the scene graph ad a given location
  void addObject(SoSeparator * object, const vpHomogeneousMatrix &fMo,
		 SoSeparator * root) ;
  //!Add a new object in the scene graph ad a given location
  void addObject(SoSeparator * newObject, const vpHomogeneousMatrix &fMo) ;

public:
  //! display the scene (handle with care)
  void redraw() ;
  //! load an iv file
  void load(const char *file_name) ;
  //! load an iv file, set the location of this scene
  void load(const char *iv_filename,const vpHomogeneousMatrix &fMo) ;
  //! save the scene in an iv file
  void save(const char *name,bool binary=false) ;

  //!   Add the representation of the absolute frame
  void addAbsoluteFrame (double zoom=1) ;
  //!   Add the representation of a frame
  void addFrame (const vpHomogeneousMatrix &fMo, double zoom=1) ;
  //! set the size of the camera/frame
  void setZoomFactor (const double zoom) ;

private:
  double zoomFactor ;
  //---------------------------------------------------
  //  camera description
private:
  bool cameraPositionInitialized ;
  //! internal camera position
  vpHomogeneousMatrix cMf ;
  //! internal camera parameters
  vpCameraParameters internalCameraParameters ;

public:
  //! set the camera position (from an homogeneous matrix)
  void setCameraPosition(vpHomogeneousMatrix &cMf) ;
  //! get the camera position (from an homogeneous matrix)
  void getCameraPosition(vpHomogeneousMatrix &_cMf) { _cMf = cMf ; }
  //! modify the position of the camera in the scene graph
  void moveInternalCamera(vpHomogeneousMatrix &cMf) ;
  //! set internal camera parameters
  void setInternalCameraParameters(vpCameraParameters &cam) ;
  //! set external camera parameters
  void setExternalCameraParameters(vpCameraParameters &cam) ;
  //! get the external camera position
  void getExternalCameraPosition(vpHomogeneousMatrix &cMf) ;


  /* --- Off screen rendering  --- */
public:
  enum viewEnum { INTERNAL, EXTERNAL } ;
  void  write(viewEnum view, const char * fileName);
private:
  SbTime * realtime ;
  SoOffscreenRenderer * offScreenRenderer ;
  void offScreenRendering (viewEnum view=vpSimulator::EXTERNAL,
			   int * width = NULL,
			   int * height = NULL);
private:
  //! image of the internal view
  unsigned char * bufferView;

public:
  //! get the image corresponding to the internal view
  unsigned char* getBufferedOffScreenRenderer()
  {
    return bufferView;
  }

  //! get the size of the internal view
  void getSizeInternalView(int& width, int& height);

  //! get the intrinsic parameters of the camera
  void getCameraParameters(vpCameraParameters &cam)
  {
    cam = internalCameraParameters;
  }

} ;


#endif
#endif
