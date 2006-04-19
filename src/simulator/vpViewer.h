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

#ifndef vpViewer_HH
#define vpViewer_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT


// Qt and Coin stuff
#include <Inventor/Qt/SoQt.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
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
#include <GL/gl.h>

// thread
#include <pthread.h>

// visp
#include <visp/vpDebug.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

class vpSimulator;

class vpViewer : public SoQtExaminerViewer {

  friend class vpSimulator ;

public:
  vpViewer(QWidget * parent,  vpSimulator *simu);
  ~vpViewer();
  void  resize(int x, int y) ;


public:
  virtual void actualRedraw(void);

private:
  vpSimulator *simu ;
  SbBool processSoEvent(const SoEvent * const event) ;


};


#endif
#endif
