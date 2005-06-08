#include <visp/vpViewer.h>
#include <visp/vpSimulator.h>


#include <Inventor/events/SoKeyboardEvent.h>
#include <Inventor/nodes/SoEventCallback.h>



vpViewer::vpViewer(QWidget * parent,  vpSimulator *_simu)
{

  SoQtExaminerViewer::SoQtExaminerViewer(parent,(char *)NULL,false) ;

  simu = _simu ;

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


   // Render normal scenegraph.
   SoQtExaminerViewer::actualRedraw();
   glSwapBuffers() ;

}


void
vpViewer::resize(int x, int y)
{
  SbVec2s size(x,y) ;
  setGlxSize(size) ;
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

