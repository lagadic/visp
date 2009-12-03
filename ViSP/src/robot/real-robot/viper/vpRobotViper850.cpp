/****************************************************************************
 *
 * $Id: vpRobotAfma6.cpp 2261 2009-09-08 07:47:42Z nmelchio $
 *
 * Copyright (C) 1998-2009 Inria. All rights reserved.
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
 * Interface for the Irisa's Viper S850 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_VIPER850

#include <signal.h>
#include <stdlib.h>

#include <visp/vpRobotException.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpDebug.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpThetaUVector.h>
#include <visp/vpRobot.h>
#include <visp/vpRobotViper850.h>

/* ---------------------------------------------------------------------- */
/* --- STATIC ----------------------------------------------------------- */
/* ---------------------------------------------------------------------- */

bool vpRobotViper850::robotAlreadyCreated = false;

/*!

  Default positioning velocity in percentage of the maximum
  velocity. This value is set to 15. The member function
  setPositioningVelocity() allows to change this value.

*/
const double vpRobotViper850::defaultPositioningVelocity = 15.0;

/* ---------------------------------------------------------------------- */
/* --- EMERGENCY STOP --------------------------------------------------- */
/* ---------------------------------------------------------------------- */

/*!

  Emergency stops the robot if the program is interrupted by a SIGINT
  (CTRL C), SIGSEGV (segmentation fault), SIGBUS (bus error), SIGKILL
  or SIGQUIT signal.

*/
void emergencyStopViper850(int signo)
{
  std::cout << "Stop the Viper850 application by signal (" 
	    << signo << "): " << (char)7 ;
  switch(signo)
    {
    case SIGINT:
      std::cout << "SIGINT (stop by ^C) " << std::endl ; break ;
    case SIGBUS:
      std::cout <<"SIGBUS (stop due to a bus error) " << std::endl ; break ;
    case SIGSEGV:
      std::cout <<"SIGSEGV (stop due to a segmentation fault) " << std::endl ; break ;
    case SIGKILL:
      std::cout <<"SIGKILL (stop by CTRL \\) " << std::endl ; break ;
    case SIGQUIT:
      std::cout <<"SIGQUIT " << std::endl ; break ;
    default :
      std::cout << signo << std::endl ;
    }
  //std::cout << "Emergency stop called\n";
  //  PrimitiveESTOP_Viper850();
  PrimitiveSTOP_Viper850();
  std::cout << "Robot was stopped\n";

  // Free allocated ressources
  //  ShutDownConnection(); // Some times cannot exit here when Ctrl-C

  fprintf(stdout, "Application ");
  fflush(stdout);
  kill(getpid(), SIGKILL);
  exit(1) ;
}

/* ---------------------------------------------------------------------- */
/* --- CONSTRUCTOR ------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

/*!

  The only available constructor.

  This contructor calls init() to initialise the connection with the
  MotionBox or low level controller, send the default \f$^e{\bf
  M}_c\f$ homogeneous matrix and power on the robot.

  It also set the robot state to vpRobot::STATE_STOP.

  To set the extrinsic camera parameters related to the \f$^e{\bf
  M}_c\f$ matrix obtained with a camera perspective projection model
  including the distorsion, use the code below:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpCameraParameters.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective 
  // projection model including a distorsion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA,
	     vpCameraParameters::perspectiveProjWithDistortion);
  \endcode

  Now, you can get the intrinsic camera parameters associated to an
  image acquired by the camera attached to the robot, with:

  \code
  vpImage<unsigned char> I(480, 640);

  // Get an image from the camera attached to the robot
#ifdef VISP_HAVE_DC1394_2
  vp1394TwoGrabber g;
  g.acquire(I);
#endif
  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model
  // with distorsion.
#endif
}
  \endcode

  \sa vpCameraParameters, init(vpViper850::vpViper850CameraRobotType,
  vpCameraParameters::vpCameraParametersProjType)

*/
vpRobotViper850::vpRobotViper850 (void)
  :
  vpViper850 (),
  vpRobot ()
{

  /*
    #define	SIGHUP	1	// hangup
    #define	SIGINT	2	// interrupt (rubout)
    #define	SIGQUIT	3	// quit (ASCII FS)
    #define	SIGILL	4	// illegal instruction (not reset when caught)
    #define	SIGTRAP	5	// trace trap (not reset when caught)
    #define	SIGIOT	6	// IOT instruction
    #define	SIGABRT 6	// used by abort, replace SIGIOT in the future
    #define	SIGEMT	7	// EMT instruction
    #define	SIGFPE	8	// floating point exception
    #define	SIGKILL	9	// kill (cannot be caught or ignored)
    #define	SIGBUS	10	// bus error
    #define	SIGSEGV	11	// segmentation violation
    #define	SIGSYS	12	// bad argument to system call
    #define	SIGPIPE	13	// write on a pipe with no one to read it
    #define	SIGALRM	14	// alarm clock
    #define	SIGTERM	15	// software termination signal from kill
  */

  signal(SIGINT, emergencyStopViper850);
  signal(SIGBUS, emergencyStopViper850) ;
  signal(SIGSEGV, emergencyStopViper850) ;
  signal(SIGKILL, emergencyStopViper850);
  signal(SIGQUIT, emergencyStopViper850);

  std::cout << "Open communication with MotionBlox.\n";
  try {
    this->init();
    this->setRobotState(vpRobot::STATE_STOP) ;
  }
  catch(...) {
    //  vpERROR_TRACE("Error caught") ;
    throw ;
  }
  positioningVelocity  = defaultPositioningVelocity ;

  vpRobotViper850::robotAlreadyCreated = true;

  return ;
}


/* ------------------------------------------------------------------------ */
/* --- INITIALISATION ----------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Initialise the connection with the MotionBox or low level
  controller, send the default eMc homogeneous matrix, power on the
  robot and wait 1 sec before returning to be sure the initialisation
  is done.

  \warning This method sets the camera extrinsic parameters (matrix
  eMc) to the one obtained by calibration with a camera projection
  model without distorsion by calling
  init(vpViper850::defaultCameraRobot). If you want to set the extrinsic
  camera parameters to those obtained with a camera perspective model
  including the distorsion you have to call the
  init(vpViper850::vpViper850CameraRobotType,
  vpCameraParameters::vpCameraParametersProjType) method.

  \sa vpCameraParameters, init(vpViper850::vpViper850CameraRobotType,
  vpCameraParameters::vpCameraParametersProjType)
*/
void
vpRobotViper850::init (void)
{
  InitTry;

  // Initialise private variables used to compute the measured velocities
  q_prev_getvel.resize(6);
  q_prev_getvel = 0;
  time_prev_getvel = 0;
  first_time_getvel = true;

  // Initialise private variables used to compute the measured displacement
  q_prev_getdis.resize(6);
  q_prev_getdis = 0;
  first_time_getdis = true;


  // Initialize the firewire connection
  Try( InitializeConnection() );

  // Connect to the servoboard using the servo board GUID
  Try( InitializeNode_Viper850() );

  Try( PrimitiveRESET_Viper850() );

  // Update the eMc matrix in the low level controller
  init(vpViper850::defaultTool);

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  UInt32 EStopStatus;
  Try( PrimitiveSTATUS_Viper850(NULL, NULL, &EStopStatus, NULL, NULL, NULL, 
			     &HIPowerStatus));
  CAL_Wait(0.1);

  switch(EStopStatus) {
  case ESTOP_AUTO: break;
  case ESTOP_MANUAL: break;
  case ESTOP_ACTIVATED: 
    std::cout << "Emergency stop is activated! \n"
	      << "Check the emergency stop button and push the yellow button before continuing. \n"
	      << "We quit now the application. See you soon..." << std::endl;
    // Free allocated ressources
    ShutDownConnection();
    exit(0);

    break;
  default: 
    std::cout << "Sorry there is an error on the emergency chain." << std::endl;
    std::cout << "You have to call Adept for maintenance..." << std::endl;
    // Free allocated ressources
    ShutDownConnection();
    exit(0);
  }

  if (HIPowerStatus == 0) {
    fprintf(stdout, "\nPower ON the Viper850 robot in the next 10 second...\n");
    fflush(stdout);
    Try( PrimitivePOWERON_Viper850() );
  }
  fprintf(stdout, "Viper850 power is ON. We continue...\n");
  fflush(stdout);

  // get real joint min/max from the MotionBlox
  Try( PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data) );
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

//   for (int i=0; i < njoint; i++) {
//     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i], joint_max[i]);
//   }

  // If an error occur in the low level controller, goto here
  //CatchPrint();
  Catch();

  // Test if an error occurs
  if (TryStt == -20001)
    printf("No connection detected. Check if the robot is powered on \n"
	   "and if the firewire link exist between the MotionBlox and this computer.\n");
  else if (TryStt == -675)
    printf(" Timeout enabling power...\n");

  if (TryStt < 0) {
    // Power off the robot
    PrimitivePOWEROFF_Viper850();
    // Free allocated ressources
    ShutDownConnection();

    std::cout << "Cannot open connexion with the motionblox..." << std::endl;
    throw vpRobotException (vpRobotException::constructionError,
  			  "Cannot open connexion with the motionblox");
  }
  return ;
}

/*!

  Initialize the robot kinematics with the extrinsic calibration
  parameters associated to a specific camera (set the eMc homogeneous
  parameters in the low level controller) and also get the joint
  limits from the low-level controller.

  The eMc parameters depend on the camera and the projection model in use.

  \param tool : Tool to use.

  \param projModel : Projection model associated to the camera.

 To set the extrinsic camera parameters related to the \f$^e{\bf
  M}_c\f$ matrix obtained with a camera perspective projection model
  including the distorsion, use the code below:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpImage.h>
#include <visp/vp1394TwoGrabber.h>
#include <visp/vpCameraParameters.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective 
  // projection model including a distorsion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA,
	     vpCameraParameters::perspectiveProjWithDistortion);
  \endcode

  Now, you can get the intrinsic camera parameters associated to an
  image acquired by the camera attached to the robot, with:

  \code
  vpImage<unsigned char> I(480, 640);

  // Get an image from the camera attached to the robot
#ifdef VISP_HAVE_DC1394_2
  vp1394TwoGrabber g;
  g.acquire(I);
#endif
  vpCameraParameters cam;
  robot.getCameraParameters(cam, I);
  // In cam, you get the intrinsic parameters of the projection model
  // with distorsion.
#endif
}
  \endcode

  \sa vpCameraParameters, init()
*/
void
vpRobotViper850::init (vpViper850::vpToolType tool,
		       vpCameraParameters::vpCameraParametersProjType projModel)
{

  InitTry;
  // Read the robot constants from files
  // - joint [min,max], coupl_56, long_56
  // - camera extrinsic parameters relative to eMc
  vpViper850::init(tool, projModel);

 // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (int i=0; i < 3; i ++) {
    eMc_pose[i] = etc[i];   // translation in meters
    eMc_pose[i+3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try( PrimitiveCONST_Viper850(eMc_pose) );

  // get real joint min/max from the MotionBlox
  Try( PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data) );
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

//   for (int i=0; i < njoint; i++) {
//     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i], joint_max[i]);
//   }

  setToolType(tool);

  CatchPrint();
  return ;
}

/* ------------------------------------------------------------------------ */
/* --- DESTRUCTOR --------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Destructor.

  Free allocated ressources.
*/
vpRobotViper850::~vpRobotViper850 (void)
{
  InitTry;

  setRobotState(vpRobot::STATE_STOP) ;

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try( PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL,
			     &HIPowerStatus));
  CAL_Wait(0.1);

//   if (HIPowerStatus == 1) {
//     fprintf(stdout, "Power OFF the robot\n");
//     fflush(stdout);

//     Try( PrimitivePOWEROFF_Viper850() );
//   }

  // Free allocated ressources
  ShutDownConnection();

  vpRobotViper850::robotAlreadyCreated = false;

  CatchPrint();
  return;
}




/*!

Change the robot state.

\param newState : New requested robot state.
*/
vpRobot::vpRobotStateType
vpRobotViper850::setRobotState(vpRobot::vpRobotStateType newState)
{
  InitTry;

  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState ()) {
      Try( PrimitiveSTOP_Viper850() );
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState ()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      Try( PrimitiveSTOP_Viper850() );
    }
    else {
      //std::cout << "Change the control mode from stop to position control.\n";
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    break;
  }
  default:
    break ;
  }

  CatchPrint();

  return vpRobot::setRobotState (newState);
}


/* ------------------------------------------------------------------------ */
/* --- STOP --------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Stop the robot and set the robot state to vpRobot::STATE_STOP.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.
*/
void
vpRobotViper850::stopMotion(void)
{
  if (getRobotState() != vpRobot::STATE_VELOCITY_CONTROL)
    return;

  InitTry;
  Try( PrimitiveSTOP_Viper850() );
  setRobotState (vpRobot::STATE_STOP);

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot stop robot motion");
    throw vpRobotException (vpRobotException::lowLevelError,
			      "Cannot stop robot motion.");
  }
}

/*!

  Power on the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot power on.

  \sa powerOff(), getPowerState()
*/
void
vpRobotViper850::powerOn(void)
{
  InitTry;

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try( PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, 
			     &HIPowerStatus));
  CAL_Wait(0.1);

  if (HIPowerStatus == 0) {
    fprintf(stdout, "Power ON the Viper850 robot\n");
    fflush(stdout);

    Try( PrimitivePOWERON_Viper850() );
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot power on the robot");
    throw vpRobotException (vpRobotException::lowLevelError,
			      "Cannot power off the robot.");
  }
}

/*!

  Power off the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.

  \sa powerOn(), getPowerState()
*/
void
vpRobotViper850::powerOff(void)
{
  InitTry;

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try( PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, 
			     &HIPowerStatus));
  CAL_Wait(0.1);

  if (HIPowerStatus == 1) {
    fprintf(stdout, "Power OFF the Viper850 robot\n");
    fflush(stdout);

    Try( PrimitivePOWEROFF_Viper850() );
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot power off the robot");
    throw vpRobotException (vpRobotException::lowLevelError,
			      "Cannot power off the robot.");
  }
}

/*!

  Get the robot power state indication if power is on or off.

  \return true if power is on. false if power is off.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error.

  \sa powerOn(), powerOff()
*/
bool
vpRobotViper850::getPowerState(void)
{
  InitTry;
  bool status = false;
  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try( PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, 
			     &HIPowerStatus));
  CAL_Wait(0.1);

  if (HIPowerStatus == 1) {
    status = true;
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot get the power status");
    throw vpRobotException (vpRobotException::lowLevelError,
			      "Cannot get the power status.");
  }
  return status;
}

/*!

  Get the twist transformation \f$^c{\bf V}_e\f$ from camera frame to
  end-effector frame.  This transformation allows to compute a
  velocity expressed in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void
vpRobotViper850::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  vpViper850::get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;
}

/*!

  Get the geometric transformation \f$^c{\bf M}_e\f$ between the
  camera frame and the end-effector frame. This transformation is
  constant and correspond to the extrinsic camera parameters estimated
  by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void
vpRobotViper850::get_cMe(vpHomogeneousMatrix &cMe)
{
  vpViper850::get_cMe(cMe) ;
}


/*!

  Get the robot jacobian expressed in the end-effector frame.

  To compute \f$^e{\bf J}_e\f$, we communicate with the low level
  controller to get the joint position of the robot.

  \param eJe : Robot jacobian \f$^e{\bf J}_e\f$ expressed in the
  end-effector frame.

*/
void
vpRobotViper850::get_eJe(vpMatrix &eJe)
{

  double position[6];

  InitTry;
  Try( PrimitiveACQ_POS_J_Viper850(position) );
  CatchPrint();

  vpColVector q(6);
  for (int i=0; i < njoint; i++)
    q[i] = vpMath::rad(position[i]);

  try
    {
      vpViper850::get_eJe(q, eJe) ;
    }
  catch(...)
    {
      vpERROR_TRACE("catch exception ") ;
      throw ;
    }
}
/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame.

  To compute \f$^f{\bf J}_e\f$, we communicate with the low level
  controller to get the joint position of the robot.

  \param fJe : Robot jacobian \f$^f{\bf J}_e\f$ expressed in the
  reference frame.
*/

  void
  vpRobotViper850::get_fJe(vpMatrix &fJe)
{

  double position[6];

  InitTry;
  Try( PrimitiveACQ_POS_Viper850(position) );
  CatchPrint();

  vpColVector q(6);
  for (int i=0; i < njoint; i++)
    q[i] = position[i];

  try
    {
      vpViper850::get_fJe(q, fJe) ;
    }
  catch(...)
    {
      vpERROR_TRACE("Error caught");
      throw ;
    }
}

/*!

  Set the maximal velocity percentage to use for a positionning task.

  The default positioning velocity is defined by
  vpRobotViper850::defaultPositioningVelocity. This method allows to
  change this default positioning velocity

  \param velocity : Percentage of the maximal velocity. Values should
  be in ]0:100].

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpColVector position(6);
  position = 0; // position in rad

  vpRobotViper850 robot;

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Set the max velocity to 20%
  robot.setPositioningVelocity(20);

  // Moves the robot to the joint position [0,0,0,0,0,0]
  robot.setPosition(vpRobot::ARTICULAR_FRAME, position);
#endif
}
  \endcode

  \sa getPositioningVelocity()
*/
void
vpRobotViper850::setPositioningVelocity (const double velocity)
{
  positioningVelocity = velocity;
}

/*!
  Get the maximal velocity percentage used for a positionning task.

  \sa setPositioningVelocity()
*/
double
vpRobotViper850::getPositioningVelocity (void)
{
  return positioningVelocity;
}


/*!

  Move to an absolute position with a given percent of max velocity.
  The percent of max velocity is to set with setPositioningVelocity().
  The position to reach can be specified in joint coordinates, in the
  camera frame or in the reference frame.

  \warning This method is blocking. It returns only when the position
  is reached by the robot.

  \param position : A six dimension vector corresponding to the
  position to reach. All the positions are expressed in meters for the
  translations and radians for the rotations. If the position is out
  of range, an exception is provided.

  \param frame : Frame in which the position is expressed.

  - In the joint space, positions are respectively X, Y, Z, A, B, C,
  with X,Y,Z the translations, and A,B,C the rotations of the
  end-effector.

  - In the camera and the reference frame, rotations are
  represented by a vpRxyzVector.

  - Mixt frame is not implemented. By mixt frame we mean, translations
  expressed in the reference frame, and rotations in the camera
  frame.

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME not
  implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpColVector position(6);
  // Set positions in the camera frame
  position[0] = 0.1;    // x axis, in meter
  position[1] = 0.2;    // y axis, in meter
  position[2] = 0.3;    // z axis, in meter
  position[3] = M_PI/8; // rotation around x axis, in rad
  position[4] = M_PI/4; // rotation around y axis, in rad
  position[5] = M_PI;   // rotation around z axis, in rad

  vpRobotViper850 robot;

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Set the max velocity to 20%
  robot.setPositioningVelocity(20);

  // Moves the robot in the camera frame
  robot.setPosition(vpRobot::CAMERA_FRAME, position);
#endif
}
  \endcode

  To catch the exception if the position is out of range, modify the code like:

  \code
  try {
    robot.setPosition(vpRobot::CAMERA_FRAME, position);
  }
  catch (vpRobotException e) {
    if (e.getCode() == vpRobotException::positionOutOfRangeError) {
    std::cout << "The position is out of range" << std::endl;
  }
  \endcode

*/
void
vpRobotViper850::setPosition (const vpRobot::vpControlFrameType frame,
			      const vpColVector & position )
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState ())
    {
      vpERROR_TRACE ("Robot was not in position-based control\n"
		     "Modification of the robot state");
      setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
    }

  vpColVector destination(njoint);
  int error = 0;

  InitTry;
  switch(frame) {
  case vpRobot::CAMERA_FRAME : {
    vpColVector q(njoint);
    Try( PrimitiveACQ_POS_Viper850(q.data) );

    // Convert degrees into rad
    q.deg2rad();

    // Get fMc from the inverse kinematics
    vpHomogeneousMatrix fMc;
    vpViper850::get_fMc(q, fMc);

    // Set cMc from the input position
    vpTranslationVector txyz;
    vpRxyzVector rxyz;
    for (int i=0; i < 3; i++) {
      txyz[i] = position[i];
      rxyz[i] = position[i+3];
    }

    // Compute cMc2
    vpRotationMatrix cRc2(rxyz);
    vpHomogeneousMatrix cMc2(txyz, cRc2);

    // Compute the new position to reach: fMc*cMc2
    vpHomogeneousMatrix fMc2 = fMc * cMc2;

    // Compute the corresponding joint position from the inverse kinematics
    bool nearest = true;
    int solution = this->getInverseKinematics(fMc2, q, nearest);
    if (solution) { // Position is reachable
      destination = q;
      // convert rad to deg requested for the low level controller
      destination.rad2deg();
      Try( PrimitiveMOVE_J_Viper850(destination.data, positioningVelocity) );
      Try( WaitState_Viper850(ETAT_ATTENTE_AFMA6, 1000) );
    }
    else {
      // Cartesian position is out of range
      error = -1;
    }

    break ;
  }
  case vpRobot::ARTICULAR_FRAME: {
    destination = position;
    // convert rad to deg requested for the low level controller
    destination.rad2deg();

    //std::cout << "Joint destination (deg): " << destination.t() << std::endl;
    Try( PrimitiveMOVE_J_Viper850(destination.data, positioningVelocity) );
    Try( WaitState_Viper850(ETAT_ATTENTE_AFMA6, 1000) );
    break ;

  }
  case vpRobot::REFERENCE_FRAME: {
    // Convert angles from Rxyz representation to Rzyz representation
    vpRxyzVector rxyz(position[3],position[4],position[5]);
    vpRotationMatrix R(rxyz);
    vpRzyzVector rzyz(R);

    for (int i=0; i <3; i++) {
      destination[i] = position[i];
      destination[i+3] = vpMath::deg(rzyz[i]); // convert also angles in deg
    }
    int configuration = 0; // keep the actual configuration

    //std::cout << "Base frame destination Rzyz (deg): " << destination.t() << std::endl;
    Try( PrimitiveMOVE_C_Viper850(destination.data, configuration,
				  positioningVelocity) );
    Try( WaitState_Viper850(ETAT_ATTENTE_AFMA6, 1000) );

    break ;
  }
  case vpRobot::MIXT_FRAME:
    {
      vpERROR_TRACE ("Positionning error. Mixt frame not implemented");
      throw vpRobotException (vpRobotException::lowLevelError,
			      "Positionning error: "
			      "Mixt frame not implemented.");
      break ;
    }
  }

  CatchPrint();
  if (TryStt == InvalidPosition || TryStt == -1023)
    std::cout << " : Position out of range.\n";
  else if (TryStt == -3019) {
    if (frame == vpRobot::ARTICULAR_FRAME)
      std::cout << " : Joint position out of range.\n";    
    else
      std::cout << " : Cartesian position leads to a joint position out of range.\n";
  }
  else if (TryStt < 0)
    std::cout << " : Unknown error (see Fabien).\n";
  else if (error == -1)
     std::cout << "Position out of range.\n";

  if (TryStt < 0 || error < 0) {
     vpERROR_TRACE ("Positionning error.");
     throw vpRobotException (vpRobotException::positionOutOfRangeError,
 			    "Position out of range.");
  }

  return ;
}

/*!
  Move to an absolute position with a given percent of max velocity.
  The percent of max velocity is to set with setPositioningVelocity().
  The position to reach can be specified in joint coordinates, in the
  camera frame or in the reference frame.

  This method owerloads setPosition(const
  vpRobot::vpControlFrameType, const vpColVector &).

  \warning This method is blocking. It returns only when the position
  is reached by the robot.

  \param pos1, pos2, pos3, pos4, pos5, pos6 : The six coordinates of
  the position to reach. All the positions are expressed in meters for
  the translations and radians for the rotations.

  \param frame : Frame in which the position is expressed.

  - In the joint space, positions are respectively X (pos1), Y (pos2),
  Z (pos3), A (pos4), B (pos5), C (pos6), with X,Y,Z the
  translations, and A,B,C the rotations of the end-effector.

  - In the camera and the reference frame, rotations [pos4, pos5, pos6] are
  represented by a vpRxyzVector.

  - Mixt frame is not implemented. By mixt frame we mean, translations
  expressed in the reference frame, and rotations in the camera
  frame.

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME not
  implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  // Set positions in the camera frame
  double pos1 = 0.1;    // x axis, in meter
  double pos2 = 0.2;    // y axis, in meter
  double pos3 = 0.3;    // z axis, in meter
  double pos4 = M_PI/8; // rotation around x axis, in rad
  double pos5 = M_PI/4; // rotation around y axis, in rad
  double pos6 = M_PI;   // rotation around z axis, in rad

  vpRobotViper850 robot;

  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Set the max velocity to 20%
  robot.setPositioningVelocity(20);

  // Moves the robot in the camera frame
  robot.setPosition(vpRobot::CAMERA_FRAME, pos1, pos2, pos3, pos4, pos5, pos6);
#endif
}
  \endcode

  \sa setPosition()
*/
void vpRobotViper850::setPosition (const vpRobot::vpControlFrameType frame,
				   const double pos1,
				   const double pos2,
				   const double pos3,
				   const double pos4,
				   const double pos5,
				   const double pos6)
{
  try{
    vpColVector position(6) ;
    position[0] = pos1 ;
    position[1] = pos2 ;
    position[2] = pos3 ;
    position[3] = pos4 ;
    position[4] = pos5 ;
    position[5] = pos6 ;

    setPosition(frame, position) ;
  }
  catch(...)
    {
      vpERROR_TRACE("Error caught");
      throw ;
    }
}

/*!

  Move to an absolute joint position with a given percent of max
  velocity. The robot state is set to position control.  The percent
  of max velocity is to set with setPositioningVelocity(). The
  position to reach is defined in the position file.

  \param filename : Name of the position file to read. The
  readPosFile() documentation shows a typical content of such a
  position file.

  This method has the same behavior than the sample code given below;
  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpColVector q;
  vpRobotViper850 robot;

  robot.readPosFile("MyPositionFilename.pos", q);
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
#endif
}
  \endcode

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME not
  implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \sa setPositioningVelocity()

*/
void vpRobotViper850::setPosition(const char *filename)
{
  vpColVector q;
  bool ret;

  ret = this->readPosFile(filename, q);

  if (ret == false) {
    vpERROR_TRACE ("Bad position in \"%s\"", filename);
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Bad position in filename.");
  }
  this->setRobotState(vpRobot::STATE_POSITION_CONTROL);
  this->setPosition(vpRobot::ARTICULAR_FRAME, q);
}

/*!

  Get the current position of the robot.

  \param frame : Control frame type in which to get the position, either :
  - in the camera cartesien frame,
  - joint (articular) coordinates of each axes
  - in a reference or fixed cartesien frame attached to the robot base
  - in a mixt cartesien frame (translation in reference
  frame, and rotation in camera frame)

  \param position : Measured position of the robot:
  - in camera cartesien frame, a 6 dimension vector, set to 0.

  - in articular, a 6 dimension vector corresponding to the articular
  position of each dof, first the 3 translations, then the 3
  articular rotation positions represented by a vpRxyzVector.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector). The code
  below show how to convert this position into a vpHomogenousMatrix:

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  vpColVector position;
  robot.getPosition(vpRobot::REFERENCE_FRAME, position);

  vpTranslationVector ftc; // reference frame to camera frame translations
  vpRxyzVector frc; // reference frame to camera frame rotations

  // Update the transformation between reference frame and camera frame
  for (int i=0; i < 3; i++) {
    ftc[i] = position[i];   // tx, ty, tz
    frc[i] = position[i+3]; // ry, ry, rz
  }

  // Create a rotation matrix from the Rxyz rotation angles
  vpRotationMatrix fRc(frc); // reference frame to camera frame rotation matrix

  // Create the camera to fix frame transformation in terms of a
  // homogenous matrix
  vpHomogeneousMatrix fMc(fRc, ftc);
#endif
}
  \endcode

  \exception vpRobotException::lowLevelError : If the position cannot
  be get from the low level controller.

  \sa setPosition(const vpRobot::vpControlFrameType frame, const
  vpColVector & r)

*/
void
vpRobotViper850::getPosition (const vpRobot::vpControlFrameType frame,
			      vpColVector & position)
{

  InitTry;

  position.resize (6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME : {
    position = 0;
    return;
  }
  case vpRobot::ARTICULAR_FRAME : {
    Try( PrimitiveACQ_POS_J_Viper850(position.data) );
    //vpCTRACE << "Get joint position (deg)" << position.t() << std::endl;
    position.deg2rad();

    return;
  }
  case vpRobot::REFERENCE_FRAME : {
    Try( PrimitiveACQ_POS_C_Viper850(position.data) );
    //    vpCTRACE << "Get cartesian position " << position.t() << std::endl;
    // 1=tx, 2=ty, 3=tz in meters; 4=Rz 5=Ry 6=Rz in deg
    // Convert Euler Rzyz angles from deg to rad
    for (int i=3; i <6; i++)
      position[i] = vpMath::rad(position[i]);
    // Convert Rzyz angles into Rxyz representation
    vpRzyzVector rzyz(position[3], position[4], position[5]);
    vpRotationMatrix R(rzyz);
    vpRxyzVector rxyz(R);

    // Update the position using Rxyz representation
    for (int i=0; i <3; i++)
      position[i+3] = rxyz[i];
//     vpCTRACE << "Cartesian position Rxyz (deg)" 
// 	     << position[0] << " " << position[1] << " " << position[2] << " "  
// 	     << vpMath::deg(position[3]) << " "
// 	     << vpMath::deg(position[4]) << " "
// 	     << vpMath::deg(position[5]) << std::endl;

    break ;
  }
  case vpRobot::MIXT_FRAME: {
    vpERROR_TRACE ("Cannot get position in mixt frame: not implemented");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position in mixt frame: "
			    "not implemented");
    break ;
  }
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot get position.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get position.");
  }

  return;
}
/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpColVector &)

  The difference is here that the position is returned using a ThetaU
  representation.

  \sa getPosition(const vpRobot::vpControlFrameType frame, vpColVector &)
*/
void 
vpRobotViper850::getPosition (const vpRobot::vpControlFrameType frame,   
                            vpPoseVector &position)
{
  vpColVector posRxyz;
  //recupere  position en Rxyz
  this->getPosition(frame,posRxyz);
  vpRxyzVector RxyzVect;
  for(int j=0;j<3;j++)
    RxyzVect[j]=posRxyz[j+3];
  //recupere le vecteur thetaU correspondant
  vpThetaUVector RtuVect(RxyzVect);

  //remplit le vpPoseVector avec translation et rotation ThetaU
  for(int j=0;j<3;j++)
  {
    position[j]=posRxyz[j];
    position[j+3]=RtuVect[j];
  }
}

/*!
  Apply a velocity to the robot.

  \param frame : Control frame in which the velocity is expressed. Velocities
  could be expressed in articular, camera frame, reference frame or mixt frame.

  \param vel : Velocity vector. Translation velocities are expressed
  in m/s while rotation velocities in rad/s. The size of this vector
  is always 6.

  - In articular, \f$ vel = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
  \dot{q}_5, \dot{q}_6]^t \f$ correspond to joint velocities.

  - In camera frame, \f$ vel = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c}
  \omega_x, ^{c} \omega_y, ^{c} \omega_z]^t \f$ is expressed in the
  camera frame.

  - In reference frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{r}
  \omega_x, ^{r} \omega_y, ^{r} \omega_z]^t \f$ is expressed in the
  reference frame.

  - In mixt frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{c} \omega_x,
  ^{c} \omega_y, ^{c} \omega_z]^t \f$.  In mixt frame, translations \f$ v_x,
  v_y, v_z \f$ are expressed in the reference frame and rotations \f$
  \omega_x, \omega_y, \omega_z \f$ in the camera frame.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \warning Velocities could be saturated if one of them exceed the
  maximal autorized speed (see vpRobot::maxTranslationVelocity and
  vpRobot::maxRotationVelocity). To change these values use
  setMaxTranslationVelocity() and setMaxRotationVelocity().

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  vpColVector qvel(6);
  // Set a joint velocity
  qvel[0] = 0.1;             // Joint 1 velocity in rad/s
  qvel[1] = vpMath::rad(15); // Joint 2 velocity in rad/s
  qvel[2] = 0;               // Joint 3 velocity in rad/s
  qvel[3] = M_PI/8;          // Joint 4 velocity in rad/s
  qvel[4] = 0;               // Joint 5 velocity in rad/s
  qvel[5] = 0;               // Joint 6 velocity in rad/s

  // Initialize the controller to position control
  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  while (1) {
    // Apply a velocity in the joint space
    robot.setVelocity(vpRobot::ARTICULAR_FRAME, qvel);

    // Compute new velocities qvel...
  }

  // Stop the robot
  robot.setRobotState(vpRobot::STATE_STOP);
#endif
}
  \endcode
*/
void
vpRobotViper850::setVelocity (const vpRobot::vpControlFrameType frame,
			   const vpColVector & vel)
{

  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState ())
    {
      vpERROR_TRACE ("Cannot send a velocity to the robot "
		     "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
      throw vpRobotException (vpRobotException::wrongStateError,
			      "Cannot send a velocity to the robot "
			      "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    }

  vpColVector vel_sat(6);

  double scale_trans_sat = 1;
  double scale_rot_sat   = 1;
  double scale_sat       = 1;
  double vel_trans_max = getMaxTranslationVelocity();
  double vel_rot_max   = getMaxRotationVelocity(); 

  double vel_abs; // Absolute value

  // Velocity saturation
  switch(frame) {
    // saturation in cartesian space
  case vpRobot::CAMERA_FRAME :
  case vpRobot::REFERENCE_FRAME :  
  case vpRobot::MIXT_FRAME : {
    for (int i = 0 ; i < 3; ++ i) {
      vel_abs = fabs (vel[i]);
      if (vel_abs > vel_trans_max) {
	vel_trans_max = vel_abs;
	vpERROR_TRACE ("Excess velocity %g m/s in TRANSLATION "
		       "(axis nr. %d).", vel[i], i+1);
      }
      
      vel_abs = fabs (vel[i+3]);
      if (vel_abs > vel_rot_max) {
	vel_rot_max = vel_abs;
	vpERROR_TRACE ("Excess velocity %g rad/s in ROTATION "
		       "(axis nr. %d).", vel[i+3], i+4);
      }
    }
    
    if (vel_trans_max > getMaxTranslationVelocity())                     
      scale_trans_sat = getMaxTranslationVelocity() / vel_trans_max;
    
    if (vel_rot_max > getMaxRotationVelocity())
      scale_rot_sat = getMaxRotationVelocity() / vel_rot_max; 
    
    if ( (scale_trans_sat < 1) || (scale_rot_sat < 1) ) {
      if (scale_trans_sat < scale_rot_sat)  
	scale_sat = scale_trans_sat;                    
      else                        
	scale_sat = scale_rot_sat;
    }
    break;
  }
    // saturation in joint space
  case vpRobot::ARTICULAR_FRAME : {
    for (int i = 0 ; i < 6; ++ i) {
      vel_abs = fabs (vel[i]);
      if (vel_abs > vel_rot_max) {
	vel_rot_max = vel_abs;
	vpERROR_TRACE ("Excess velocity %g rad/s in ROTATION "
		       "(axis nr. %d).", vel[i], i+1);
      }
    }
    if (vel_rot_max > getMaxRotationVelocity())
      scale_rot_sat = getMaxRotationVelocity() / vel_rot_max; 
    if ( scale_rot_sat < 1 ) {
      scale_sat = scale_rot_sat;
    }

  }
  }
  vel_sat = vel * scale_sat;

  InitTry;

  switch(frame) {
  case vpRobot::CAMERA_FRAME : {
    // Send velocities in m/s and rad/s
    // std::cout << "Vitesse cam appliquee: " << vel_sat.t();
    Try( PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPCAM) );
    break ;
  }
  case vpRobot::ARTICULAR_FRAME : {
    // Convert all the velocities from rad/s into deg/s
    vel_sat.rad2deg();
    //std::cout << "Vitesse appliquee: " << vel_sat.t();
    //Try( PrimitiveMOVESPEED_CART(vel_sat.data, REPART) );
    Try( PrimitiveMOVESPEED_Viper850(vel_sat.data) );
    break ;
  }
  case vpRobot::REFERENCE_FRAME : {
    // Send velocities in m/s and rad/s
   std::cout << "Vitesse ref appliquee: " << vel_sat.t();
    Try( PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPFIX) );
    break ;
  }
  case vpRobot::MIXT_FRAME : {
    //Try( PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPMIX) );
    break ;
  }
  default: {
    vpERROR_TRACE ("Error in spec of vpRobot. "
		   "Case not taken in account.");
    return;
  }
  }

  Catch();
  if (TryStt < 0) {
    if (TryStt == VelStopOnJoint) {
      UInt32 axisInJoint[njoint];
      PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, axisInJoint, NULL);
      for (int i=0; i < njoint; i ++) {
	if (axisInJoint[i])
	  std::cout << "\nWarning: Velocity control stopped: axis "
		    << i+1 << " on joint limit!" <<std::endl;
      }
    }
    else {
      printf("\n%s(%d): Error %d", __FUNCTION__, TryLine, TryStt);
      if (TryString != NULL) {
	// The statement is in TryString, but we need to check the validity
	printf(" Error sentence %s\n", TryString); // Print the TryString
      }
      else {
	printf("\n");
      }
    }
  }

  return;
}






/* ------------------------------------------------------------------------ */
/* --- GET ---------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */


/*!

  Get the robot velocities.

  \param frame : Frame in wich velocities are mesured.

  \param velocity : Measured velocities. Translations are expressed in m/s
  and rotations in rad/s.

  \warning In camera frame, reference frame and mixt frame, the representation
  of the rotation is ThetaU. In that cases, \f$velocity = [\dot x, \dot y, \dot
  z, \dot {\theta U}_x, \dot {\theta U}_y, \dot {\theta U}_z]\f$.

  \warning The first time this method is called, \e velocity is set to 0. The
  first call is used to intialise the velocity computation for the next call.

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  // Set requested joint velocities
  vpColVector q_dot(6);
  q_dot[0] = 0.1;    // Joint 1 velocity in rad/s
  q_dot[1] = 0.2;    // Joint 2 velocity in rad/s
  q_dot[2] = 0.3;    // Joint 3 velocity in rad/s
  q_dot[3] = M_PI/8; // Joint 4 velocity in rad/s
  q_dot[4] = M_PI/4; // Joint 5 velocity in rad/s
  q_dot[5] = M_PI/16;// Joint 6 velocity in rad/s

  vpRobotViper850 robot;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  // Moves the joint in velocity
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);

  // Initialisation of the velocity measurement
  vpColVector q_dot_mes; // Measured velocities
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes); // q_dot_mes =0
  // q_dot_mes is resized to 6, the number of joint

  while (1) {
    robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes);
     vpTime::wait(40); // wait 40 ms
     // here q_dot_mes is equal to [0.1, 0.2, 0.3, M_PI/8, M_PI/4, M_PI/16]
  }
#endif
}
  \endcode
*/
void
vpRobotViper850::getVelocity (const vpRobot::vpControlFrameType frame,
			   vpColVector & velocity)
{

  velocity.resize (6);
  velocity = 0;

  vpColVector q_cur(6);
  vpHomogeneousMatrix fMc_cur;
  vpHomogeneousMatrix cMc; // camera displacement

  InitTry;

  // Get the actual time
  double time_cur = vpTime::measureTimeSecond();

  // Get the current joint position
  Try( PrimitiveACQ_POS_J_Viper850(q_cur.data) );
  q_cur.deg2rad();

  // Get the camera pose from the direct kinematics
  vpViper850::get_fMc(q_cur, fMc_cur);

  if ( ! first_time_getvel ) {

    switch (frame) {
    case vpRobot::CAMERA_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      velocity = vpExponentialMap::inverse(cMc, time_cur - time_prev_getvel);

      break ;
    }

    case vpRobot::ARTICULAR_FRAME: {
      velocity = (q_cur - q_prev_getvel)
	/ (time_cur - time_prev_getvel);
      break ;
    }

    case vpRobot::REFERENCE_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      vpColVector v;
      v = vpExponentialMap::inverse(cMc, time_cur - time_prev_getvel);

      // Express this velocity in the reference frame
      vpTwistMatrix fVc(fMc_cur);
      velocity = fVc * v;

      break ;
    }

    case vpRobot::MIXT_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the ThetaU representation for the rotation
      vpRotationMatrix cRc;
      cMc.extract(cRc);
      vpThetaUVector thetaU;
      thetaU.buildFrom(cRc);

      for (int i=0; i < 3; i++) {
	// Compute the translation displacement in the reference frame
	velocity[i] = fMc_prev_getvel[i][3] - fMc_cur[i][3];
	// Update the rotation displacement in the camera frame
	velocity[i+3] = thetaU[i];
      }

      // Compute the velocity
      velocity /= (time_cur - time_prev_getvel);
      break ;
    }
    }
  }
  else {
    first_time_getvel = false;
  }

  // Memorize the camera pose for the next call
  fMc_prev_getvel = fMc_cur;

  // Memorize the joint position for the next call
  q_prev_getvel = q_cur;

  // Memorize the time associated to the joint position for the next call
  time_prev_getvel = time_cur;


  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot get velocity.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get velocity.");
  }
}




/*!

  Get the robot velocities.

  \param frame : Frame in wich velocities are mesured.

  \return Measured velocities. Translations are expressed in m/s
  and rotations in rad/s.

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  // Set requested joint velocities
  vpColVector q_dot(6);
  q_dot[0] = 0.1;    // Joint 1 velocity in rad/s
  q_dot[1] = 0.2;    // Joint 2 velocity in rad/s
  q_dot[2] = 0.3;    // Joint 3 velocity in rad/s
  q_dot[3] = M_PI/8; // Joint 4 velocity in rad/s
  q_dot[4] = M_PI/4; // Joint 5 velocity in rad/s
  q_dot[5] = M_PI/16;// Joint 6 velocity in rad/s

  vpRobotViper850 robot;

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

  // Moves the joint in velocity
  robot.setVelocity(vpRobot::ARTICULAR_FRAME, q_dot);

  // Initialisation of the velocity measurement
  vpColVector q_dot_mes; // Measured velocities
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, q_dot_mes); // q_dot_mes =0
  // q_dot_mes is resized to 6, the number of joint

  while (1) {
     q_dot_mes = robot.getVelocity(vpRobot::ARTICULAR_FRAME);
     vpTime::wait(40); // wait 40 ms
     // here q_dot_mes is equal to [0.1, 0.2, 0.3, M_PI/8, M_PI/4, M_PI/16]
  }
#endif
}
  \endcode
*/
vpColVector
vpRobotViper850::getVelocity (vpRobot::vpControlFrameType frame)
{
  vpColVector velocity;
  getVelocity (frame, velocity);

  return velocity;
}

/*!

Read joint positions in a specific Viper850 position file.

This position file has to start with a header. The six joint positions
are given after the "R:" keyword. The first 3 values correspond to the
joint translations X,Y,Z expressed in meters. The 3 last values
correspond to the joint rotations A,B,C expressed in degres to be more
representative for the user. Theses values are then converted in
radians in \e q. The character "#" starting a line indicates a
comment.

A typical content of such a file is given below:

\code
#Viper - Position - Version 1.0
# file: "myposition.pos "
#
# R: A B C D E F
# Joint position in degrees
#

R: 0.1 0.3 -0.25 -80.5 80 0
\endcode

\param filename : Name of the position file to read.

\param q : The six joint positions. Values are expressed in radians.

\return true if the positions were successfully readen in the file. false, if
an error occurs.

The code below shows how to read a position from a file and move the robot to this position.
\code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Enable the position control of the robot
  robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

  // Get the current robot joint positions
  vpColVector q;        // Current joint position
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q); 

  // Save this position in a file named "current.pos"
  robot.savePosFile("current.pos", q); 

  // Get the position from a file and move to the registered position
  robot.readPosFile("current.pos", q); // Set the joint position from the file
  
  robot.setPositioningVelocity(5); // Positioning velocity set to 5%
  robot.setPosition(vpRobot::ARTICULAR_FRAME, q); // Move to the joint position
#endif
}
\endcode

\sa savePosFile()
*/

bool
vpRobotViper850::readPosFile(const char *filename, vpColVector &q)
{

  FILE * fd ;
  fd = fopen(filename, "r") ;
  if (fd == NULL)
    return false;

  char line[FILENAME_MAX];
  char dummy[FILENAME_MAX];
  char head[] = "R:";
  bool sortie = false;

  do {
    // Saut des lignes commencant par #
    if (fgets (line, FILENAME_MAX, fd) != NULL) {
      if ( strncmp (line, "#", 1) != 0) {
	// La ligne n'est pas un commentaire
	if ( strncmp (line, head, sizeof(head)-1) == 0) {
	  sortie = true; 	// Position robot trouvee.
	}
// 	else
// 	  return (false); // fin fichier sans position robot.
      }
    }
    else {
      return (false);		/* fin fichier 	*/
    }

  }
  while ( sortie != true );

  // Lecture des positions
  q.resize(njoint);
  sscanf(line, "%s %lf %lf %lf %lf %lf %lf",
	 dummy,
	 &q[0], &q[1], &q[2],
	 &q[3], &q[4], &q[5]);

  // converts rotations from degrees into radians
  q.deg2rad();

  fclose(fd) ;
  return (true);
}

/*!

  Save joint (articular) positions in a specific Viper850 position file.

  This position file starts with a header on the first line. After
  convertion of the rotations in degrees, the joint position \e q is
  written on a line starting with the keyword "R: ". See readPosFile()
  documentation for an example of such a file.

  \param filename : Name of the position file to create.

  \param q : The six joint positions to save in the
  filename. Values are expressed in radians.

  \warning All the six joint rotations written in the file are converted
  in degrees to be more representative for the user.

  \return true if the positions were successfully saved in the file. false, if
  an error occurs.

  \sa readPosFile()
*/

bool
vpRobotViper850::savePosFile(const char *filename, const vpColVector &q)
{

  FILE * fd ;
  fd = fopen(filename, "w") ;
  if (fd == NULL)
    return false;

  fprintf(fd, "\
#Viper - Position - Version 1.0\n\
#\n\
# R: A B C D E F\n\
# Joint position in degrees\n\
#\n\
#\n\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf\n",
	  q[0],
	  q[1],
	  q[2],
	  vpMath::deg(q[3]),
	  vpMath::deg(q[4]),
	  vpMath::deg(q[5]));

  fclose(fd) ;
  return (true);
}

/*!

  Moves the robot to the joint position specified in the filename. The
  positioning velocity is set to 10% of the robot maximal velocity.

  \param filename: File containing a joint position.

  \sa readPosFile

*/
void
vpRobotViper850::move(const char *filename)
{
  vpColVector q;

  try {
    this->readPosFile(filename, q)  ;
    this->setRobotState(vpRobot::STATE_POSITION_CONTROL) ;
    this->setPositioningVelocity(10);
    this->setPosition ( vpRobot::ARTICULAR_FRAME,  q) ;
  }
  catch(...) {
    throw;
  }
}

/*!

  Get the robot displacement expressed in the camera frame since the last call
  of this method.

  \param displacement : The measured displacement in the camera frame. The
  dimension of \e displacement is 6 (tx, ty, ty, rx, ry,
  rz). Translations are expressed in meters, rotations in radians with
  the Euler Rxyz representation.

  \sa getDisplacement(), getArticularDisplacement()

*/
void
vpRobotViper850::getCameraDisplacement(vpColVector &displacement)
{
  getDisplacement(vpRobot::CAMERA_FRAME, displacement);
}
/*!

  Get the robot joint displacement since the last call of this method.

  \param displacement : The measured joint displacement. The dimension
  of \e displacement is 6 (the robot joint number). All the values are
  expressed in radians.

  \sa getDisplacement(), getCameraDisplacement()

*/
void
vpRobotViper850::getArticularDisplacement(vpColVector  &displacement)
{
  getDisplacement(vpRobot::ARTICULAR_FRAME, displacement);
}

/*!

  Get the robot displacement since the last call of this method.

  \warning This functionnality is not implemented for the moment in the
  cartesian space. It is only available in the joint space
  (vpRobot::ARTICULAR_FRAME).

  \param frame : The frame in which the measured displacement is expressed.

  \param displacement : The measured displacement since the last call
  of this method. The dimension of \e displacement is always
  6. Translations are expressed in meters, rotations in radians.

  In camera or reference frame, rotations are expressed with the
  Euler Rxyz representation.

  \sa getArticularDisplacement(), getCameraDisplacement()

*/
void
vpRobotViper850::getDisplacement(vpRobot::vpControlFrameType frame,
				 vpColVector &displacement)
{
  displacement.resize (6);
  displacement = 0;

  double q[6];
  vpColVector q_cur(6);

  InitTry;

  // Get the current joint position
  Try( PrimitiveACQ_POS_Viper850(q) );
  for (int i=0; i < njoint; i ++) {
    q_cur[i] = q[i];
  }

  if ( ! first_time_getdis ) {
    switch (frame) {
    case vpRobot::CAMERA_FRAME: {
      std::cout << "getDisplacement() CAMERA_FRAME not implemented\n";
      return;
      break ;
    }

    case vpRobot::ARTICULAR_FRAME: {
      displacement = q_cur - q_prev_getdis;
      break ;
    }

    case vpRobot::REFERENCE_FRAME: {
      std::cout << "getDisplacement() REFERENCE_FRAME not implemented\n";
      return;
      break ;
    }

    case vpRobot::MIXT_FRAME: {
      std::cout << "getDisplacement() MIXT_FRAME not implemented\n";
      return;
      break ;
    }
    }
  }
  else {
    first_time_getdis = false;
  }

  // Memorize the joint position for the next call
  q_prev_getdis = q_cur;

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE ("Cannot get velocity.");
    throw vpRobotException (vpRobotException::lowLevelError,
			    "Cannot get velocity.");
  }
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

