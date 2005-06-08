

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      grasping-v0.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: grasping-v0.cpp,v 1.1.1.1 2005-06-08 07:08:03 fspindle Exp $
 *
 * Description
 * ============
 *   2 1/2 D visual servoing
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example grasping-v0.cpp
  \brief 2 1/D visual servoing experiment on 7 points
*/


#include <visp/vpIcComp.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpGenericFeature.h>
#include <visp/vpPoint.h>
#include <visp/vpPose.h>
#include <visp/vpServo.h>
#include <visp/vpFeatureBuilder.h>

#include <visp/vpRobotAfma6.h>
#include <visp/afma_main.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpServoDisplay.h>

// Debug trace
#include <visp/vpDebug.h>

#include <visp/vpDot.h>

int
main()
{
  vpImage<unsigned char> I ;
  int i ;

  vpIcCompGrabber g(2) ;
  TRACE(" ") ;
  g.open(I) ;
  TRACE(" ") ;

  try{
    g.acquire(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpImageIo::writePGM(I,"test.pgm") ;
  TRACE(" ") ;

  vpDisplayX display(I,100,100,"Grasping") ;
  TRACE(" ") ;

  try{
    vpDisplay::display(I) ;
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  TRACE(" ") ;
  vpServo task ;
  TRACE(" ") ;

  vpRobotAfma6 robot ;
  TRACE(" ") ;


  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Grapsing experiment "  <<endl ;
  cout << " Eye-in-hand task control, velocity computed in the camera frame" << endl ;
  cout << " 2 1/2 D control law " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  int nbPoint = 7 ;
  int indexOfReferencePoint = 6 ;

  vpDot dot[nbPoint] ;

  try{
    for (i=0 ; i < nbPoint ; i++)
    {
      dot[i].setGraphics(true) ;
      dot[i].initTracking(I) ;
      dot[i].track(I) ;
      dot[i].setGraphics(false) ;
      vpDisplay::flush(I) ;

    }

  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

  vpCameraParameters cam ;

  TRACE("sets the current position of the visual feature ") ;



#define L 0.006
#define D 0

  /*------------------------------------------------------------------
    --  Build the structure to compute the pose
    ------------------------------------------------------------------
  */
  vpPose pose ;
  vpHomogeneousMatrix cMo ;  // computed pose
  vpHomogeneousMatrix cdMo ; // desired pose

  // Compute the pose 3D model
  vpPoint point[nbPoint] ;
  point[0].setWorldCoordinates(-L,-L,D) ;
  point[1].setWorldCoordinates(L,-L,D) ;
  point[2].setWorldCoordinates(L,L,D) ;
  point[3].setWorldCoordinates(-L,L,D) ;
  point[4].setWorldCoordinates(2*L,3*L,D) ; //revoir
  point[5].setWorldCoordinates(0,3*L,D) ;
  point[6].setWorldCoordinates(-2*L,3*L,D) ;

  for (i=0 ; i < nbPoint ; i++)
  {
    double x,y ;
    vpPixelMeterConversion::convertPoint(cam,
					 dot[i].J(), dot[i].I(),
					 x,y)  ;
    point[i].set_x(x) ;
    point[i].set_y(y) ;
    pose.addPoint(point[i]) ;
  }

  pose.printPoint() ;
  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method
  pose.computePose(vpPose::DEMENTHON_LOWE, cMo) ;

  /*------------------------------------------------------------------
    --  Learning the desired position
    --  or reading the desired position
    ------------------------------------------------------------------
  */
  cout << " Learning 0/1 " <<endl ;
  char name[FILENAME_MAX] ;
  sprintf(name,"cdMo.dat") ;
  int learning ;
  cin >> learning ;
  if (learning ==1)
  {
    // save the object position
    TRACE("Save the location of the object in a file cdMo.dat") ;
    ofstream f(name) ;
    cMo.save(f) ;
    f.close() ;
    exit(1) ;
  }
  else
  {
    TRACE("Loading desired location from cdMo.dat") ;
    ifstream f(name) ;
    cdMo.load(f) ;
    f.close() ;
  }


 //------------------------------------------------------------------
  TRACE("1st feature (x,y)");
  // 2D point
  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,cam, dot[indexOfReferencePoint])  ;
  p.set_Z(point[indexOfReferencePoint].get_Z()) ;
  vpFeaturePoint pd ;
  {
    vpColVector cP, p ;
    point[indexOfReferencePoint].changeFrame(cdMo, cP) ;
    point[indexOfReferencePoint].project(cP, p) ;

    pd.set_x(p[0]) ;
    pd.set_y(p[1]) ;
  }



  //------------------------------------------------------------------
  TRACE("2nd feature (logZ/Z*)") ;
  vpGenericFeature logZ(1) ;
  double Zd,Z ;
  {
    vpColVector cP ;

    //current Z
    point[indexOfReferencePoint].changeFrame(cMo, cP) ;
    Z = cP[2] ;

    // desired Z
    point[indexOfReferencePoint].changeFrame(cdMo, cP) ;
    Zd = cP[2] ;
  }

  logZ.set_s(log(Z/Zd)) ;


  //------------------------------------------------------------------
  TRACE("3rd feature ThetaU") ;
  TRACE("\tcompute the rotation that the camera has to realize "  ) ;
  vpHomogeneousMatrix cdMc ;
  cdMc = cdMo*cMo.inverse() ;

  vpFeatureThetaU tu ;
  tu.buildFrom(cdMc) ;


  //------------------------------------------------------------------

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;
  task.addFeature(p,pd) ;
  task.addFeature(logZ) ;
  task.addFeature(tu) ;


  cout << "------------------------------------------------------ " <<endl ;

  double lambda_av =0.05;
  double alpha = 0 ; //1 ;
  double beta =0 ; //3 ;

  cout << "Gain adaptatif g =" <<alpha<<" *  exp (-"<<beta<<" * err_max ) + "<<lambda_av <<endl ;

  cout << "Modify the gains 0/1" <<endl ;
  int modify ;
  cin >> modify ;


  if (modify==1)
  {
    cout << "  Lambda (0.1)" << endl ;
    cin >> lambda_av ;
    alpha = 0 ;
    cout << "  Alpha  (gain maximal 1)" << endl ;
    cin >> alpha ;
    cout << "  Beta  (3)" << endl ;
    cin >> beta ;
  }



  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL) ;
  while(1)
  {
    cout << "---------------------------------------------" << iter <<endl ;

    g.acquire(I) ;
    vpDisplay::display(I) ;
    vpDisplay::displayCharString(I,280,150,
				 "IRISA-INRIA Rennes, Lagadic project",
				 vpColor::green) ;
    try
      {
	for (i=0 ; i < nbPoint ; i++)
	  dot[i].track(I) ;
      }
    catch(...)
      {
	TRACE("Error detected while tracking visual features") ;
	robot.stopMotion() ;
	exit(1) ;
      }

    // compute the position of the reference point
    vpFeatureBuilder::create(p,cam, dot[indexOfReferencePoint])  ;
   // compute the initial pose using  a non linear minimisation method
    pose.clearPoint() ;
    for (i=0 ; i < nbPoint ; i++)
      {
	double x,y ;
	vpPixelMeterConversion::convertPoint(cam,
					     dot[i].J(), dot[i].I(),
					     x,y)  ;
	point[i].set_x(x) ;
	point[i].set_y(y) ;
	pose.addPoint(point[i]) ;

	point[i].display(I,cMo,cam, vpColor::red) ;
	point[i].display(I,cdMo,cam, vpColor::blue) ;
      }
    pose.computePose(vpPose::LOWE, cMo) ;
    vpDisplay::flush(I) ;


    

    // display the pose 
    pose.display(I,cMo,cam, 0.025, vpColor::red) ;
    // display the pose 
    pose.display(I,cdMo,cam, 0.025, vpColor::blue) ;

    //current Z
    {
      vpColVector cP ;
      point[indexOfReferencePoint].changeFrame(cMo, cP) ;
      Z = cP[2] ;   
      p.set_Z(Z) ;

    }
    
    cout << "Z " << Z << "  " << Zd << endl ;
    p.print() ;

    // compute log (Z/Z*) anf the corresponding interaction matrix
    logZ.set_s(log(Z/Zd)) ;
    vpMatrix LlogZ(1,6) ;
    LlogZ[0][0] = LlogZ[0][1] = LlogZ[0][5] = 0 ;
    LlogZ[0][2] = -1/Z ;
    LlogZ[0][3] = -p.get_y() ;
    LlogZ[0][4] =  p.get_x() ;

    logZ.setInteractionMatrix(LlogZ) ;



    // Compute the displacement to achieve
    cdMc = cdMo*cMo.inverse() ;
    tu.buildFrom(cdMc) ;




    task.print() ;

    // Compute the adaptative gain (speed up the convergence)
    double gain ;
    {
      if (alpha == 0) gain = lambda_av ;
      else
      {
	gain = alpha * exp (-beta * task.error.sumSquare() ) +  lambda_av ;
      }
    }
    task.setLambda(gain) ;

    vpColVector v ;
    v = task.computeControlLaw() ;

    vpServoDisplay::display(task,cam,I) ;
    cout << v.t() ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    TRACE("\t\t || s - s* || = %f ", task.error.sumSquare()) ;
  }

  TRACE("Display task information " ) ;
  task.print() ;
}

