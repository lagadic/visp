/****************************************************************************
 *
 * $Id: vpViewer.cpp,v 1.7 2007-03-05 10:21:40 marchand Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Simulator based on SoQt.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_SOQT
#include <visp/vpViewer.h>
#include <visp/vpSimulator.h>
#include <visp/vpRA.h>


#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/nodes/SoEventCallback.h>



vpViewer::vpViewer(QWidget * parent,  vpSimulator *_simu)
{

  SoQtExaminerViewer::SoQtExaminerViewer(parent,(char *)NULL,false) ;

  this->simu = _simu ;

  // Coin should not clear the pixel-buffer, so the background image
  // is not removed.
  this->setClearBeforeRender(FALSE, TRUE);
  //  this->setAntialiasing(true, 2) ;

}


vpViewer::~vpViewer()
{

}

void
vpViewer::actualRedraw(void)
{

   const SbViewportRegion vp = this->getViewportRegion();
   SbVec2s origin = vp.getViewportOriginPixels();
   SbVec2s size = vp.getViewportSizePixels();
   glViewport(origin[0], origin[1], size[0], size[1]);

   const SbColor col = this->getBackgroundColor();
   glClearColor(col[0], col[1], col[2], 0.0f);
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   // this should be used only with the vpRA:vpSimulator
   // to diplay an image background
   if (simu->image_background != NULL)
   {
     glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
     if (simu->typeImage == vpSimulator::grayImage)
       glDrawPixels(simu->getInternalWidth(), simu->getInternalHeight(),
		    (GLenum)GL_LUMINANCE,
		    GL_UNSIGNED_BYTE,
		    simu->image_background );
     else
       glDrawPixels(simu->getInternalWidth(), simu->getInternalHeight(),
		    (GLenum)GL_RGB,
		    GL_UNSIGNED_BYTE,
		    simu->image_background );

     glEnable(GL_DEPTH_TEST);
     glClear(GL_DEPTH_BUFFER_BIT);     // clear the z-buffer
     glClearDepth(100.0);              // Profondeur du Z-Buf
   }

   // Render normal scenegraph.
   SoQtExaminerViewer::actualRedraw();
   glSwapBuffers() ;

}

/*!

\param x : width
\param y : height

*/
void
vpViewer::resize(int x, int y)
{
  SbVec2s size(x,y) ;
  //  setGlxSize(size) ;
  setSize(size) ;
}

SbBool
vpViewer::processSoEvent(const SoEvent * const event)
{
  if ( this->isViewing() &&
       event->getTypeId() == SoKeyboardEvent::getClassTypeId() )
  {
    SoKeyboardEvent * kbevent = (SoKeyboardEvent *) event;
    switch ( kbevent->getKey() ) {
    case SoKeyboardEvent::H:
      if ( kbevent->getState() == SoButtonEvent::DOWN )
      {
	cout << "H : this help "<<endl ;
	cout << "M : get and save the external camera location (matrix)"<<endl;
	cout << "V : get and save the external camera location (vector)"<<endl;
	cout << "M : load camera location (vector)"<<endl;
	cout << "P : get external camera location and set the internal one"<<endl;
      }
      return TRUE;

    case SoKeyboardEvent::M:
      if ( kbevent->getState() == SoButtonEvent::DOWN )
      {
	vpHomogeneousMatrix cMf ;
	simu->getExternalCameraPosition(cMf) ;
	ofstream f("cMf.dat") ;
	cMf.save(f) ;
	f.close() ;
      }
      return TRUE;
    case SoKeyboardEvent::V:
      if ( kbevent->getState() == SoButtonEvent::DOWN )
      {
	vpHomogeneousMatrix cMf ;
	simu->getExternalCameraPosition(cMf) ;
	vpPoseVector vcMf(cMf) ;
	ofstream f("vcMf.dat") ;
	vcMf.save(f) ;
	f.close() ;
      }
      return TRUE;
    case SoKeyboardEvent::L:
      if ( kbevent->getState() == SoButtonEvent::DOWN )
      {
	vpPoseVector vcMf;
	ifstream f("vcMf.dat") ;
	vcMf.load(f) ;
	f.close() ;
	vpHomogeneousMatrix cMf(vcMf) ;
	simu->setCameraPosition(cMf) ;
	simu->moveInternalCamera(cMf) ;
      }
      return TRUE;
    case SoKeyboardEvent::P:
      if ( kbevent->getState() == SoButtonEvent::DOWN )
      {
	vpHomogeneousMatrix cMf ;
	simu->getExternalCameraPosition(cMf) ;
	vpPoseVector vcMf(cMf) ;
	vcMf.print() ;
	simu->setCameraPosition(cMf) ;
	simu->moveInternalCamera(cMf) ;
      }
      return TRUE;
    default:
      break;
    }
  }
  return  SoQtExaminerViewer::processSoEvent(event);
}

#endif
