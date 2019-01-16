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
 * Interface for the Irisa's Viper S850 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_VIPER850

#include <signal.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/robot/vpRobot.h>
#include <visp3/robot/vpRobotException.h>
#include <visp3/robot/vpRobotViper850.h>

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
  std::cout << "Stop the Viper850 application by signal (" << signo << "): " << (char)7;
  switch (signo) {
  case SIGINT:
    std::cout << "SIGINT (stop by ^C) " << std::endl;
    break;
  case SIGBUS:
    std::cout << "SIGBUS (stop due to a bus error) " << std::endl;
    break;
  case SIGSEGV:
    std::cout << "SIGSEGV (stop due to a segmentation fault) " << std::endl;
    break;
  case SIGKILL:
    std::cout << "SIGKILL (stop by CTRL \\) " << std::endl;
    break;
  case SIGQUIT:
    std::cout << "SIGQUIT " << std::endl;
    break;
  default:
    std::cout << signo << std::endl;
  }
  // std::cout << "Emergency stop called\n";
  //  PrimitiveESTOP_Viper850();
  PrimitiveSTOP_Viper850();
  std::cout << "Robot was stopped\n";

  // Free allocated resources
  //  ShutDownConnection(); // Some times cannot exit here when Ctrl-C

  fprintf(stdout, "Application ");
  fflush(stdout);
  kill(getpid(), SIGKILL);
  exit(1);
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
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective
  // projection model including a distorsion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
  \endcode

  Now, you can get the intrinsic camera parameters associated to an
  image acquired by the camera attached to the robot, with:

  \code
  vpImage<unsigned char> I(480, 640);

  // Get an image from the camera attached to the robot
#ifdef VISP_HAVE_DC1394
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

  \sa vpCameraParameters, init(vpViper850::vpToolType,
  vpCameraParameters::vpCameraParametersProjType)

*/
vpRobotViper850::vpRobotViper850(bool verbose) : vpViper850(), vpRobot()
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
  signal(SIGBUS, emergencyStopViper850);
  signal(SIGSEGV, emergencyStopViper850);
  signal(SIGKILL, emergencyStopViper850);
  signal(SIGQUIT, emergencyStopViper850);

  setVerbose(verbose);
  if (verbose_)
    std::cout << "Open communication with MotionBlox.\n";
  try {
    this->init();
    this->setRobotState(vpRobot::STATE_STOP);
  } catch (...) {
    //  vpERROR_TRACE("Error caught") ;
    throw;
  }
  positioningVelocity = defaultPositioningVelocity;

  maxRotationVelocity_joint6 = maxRotationVelocity;

  vpRobotViper850::robotAlreadyCreated = true;

  return;
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
  vpCameraParameters::vpCameraParametersProjType) method. If you want to set
  custom extrinsic camera parameters you have to call the
  init(vpViper850::vpToolType, const vpHomogeneousMatrix&) method.

  \sa vpCameraParameters, init(vpViper850::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpViper850::vpToolType, const vpHomogeneousMatrix&),
  init(vpViper850::vpToolType, const std::string&)

*/
void vpRobotViper850::init(void)
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

#if defined(USE_ATI_DAQ) && defined(VISP_HAVE_COMEDI)
  std::string calibfile;
#ifdef VISP_HAVE_VIPER850_DATA
  calibfile = std::string(VISP_VIPER850_DATA_PATH) + std::string("/ati/FT17824.cal");
  if (!vpIoTools::checkFilename(calibfile))
    throw(vpException(vpException::ioError, "ATI F/T calib file \"%s\" doesn't exist", calibfile.c_str()));
#else
  throw(vpException(vpException::ioError, "You don't have access to Viper850 "
                                          "data to retrive ATI F/T calib "
                                          "file"));
#endif
  ati.setCalibrationFile(calibfile);
  ati.open();
#endif

  // Initialize the firewire connection
  Try(InitializeConnection(verbose_));

  // Connect to the servoboard using the servo board GUID
  Try(InitializeNode_Viper850());

  Try(PrimitiveRESET_Viper850());

  // Enable the joint limits on axis 6
  Try(PrimitiveREMOVE_JOINT6_LIMITS_Viper850(0));

  // Update the eMc matrix in the low level controller
  init(vpViper850::defaultTool);

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  UInt32 EStopStatus;
  Try(PrimitiveSTATUS_Viper850(NULL, NULL, &EStopStatus, NULL, NULL, NULL, &HIPowerStatus));
  CAL_Wait(0.1);

  // Print the robot status
  if (verbose_) {
    std::cout << "Robot status: ";
    switch (EStopStatus) {
    case ESTOP_AUTO:
      controlMode = AUTO;
      if (HIPowerStatus == 0)
        std::cout << "Power is OFF" << std::endl;
      else
        std::cout << "Power is ON" << std::endl;
      break;

    case ESTOP_MANUAL:
      controlMode = MANUAL;
      if (HIPowerStatus == 0)
        std::cout << "Power is OFF" << std::endl;
      else
        std::cout << "Power is ON" << std::endl;
      break;
    case ESTOP_ACTIVATED:
      controlMode = ESTOP;
      std::cout << "Emergency stop is activated" << std::endl;
      break;
    default:
      std::cout << "Sorry there is an error on the emergency chain." << std::endl;
      std::cout << "You have to call Adept for maintenance..." << std::endl;
      // Free allocated resources
    }
    std::cout << std::endl;
  }
  // get real joint min/max from the MotionBlox
  Try(PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data));
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

  //   for (unsigned int i=0; i < njoint; i++) {
  //     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i],
  //     joint_max[i]);
  //   }

  // If an error occur in the low level controller, goto here
  // CatchPrint();
  Catch();

  // Test if an error occurs
  if (TryStt == -20001)
    printf("No connection detected. Check if the robot is powered on \n"
           "and if the firewire link exist between the MotionBlox and this "
           "computer.\n");
  else if (TryStt == -675)
    printf(" Timeout enabling power...\n");

  if (TryStt < 0) {
    // Power off the robot
    PrimitivePOWEROFF_Viper850();
    // Free allocated resources
    ShutDownConnection();

    std::cout << "Cannot open connection with the motionblox..." << std::endl;
    throw vpRobotException(vpRobotException::constructionError, "Cannot open connection with the motionblox");
  }
  return;
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
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/robot/vpRobotViper850.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the extrinsic camera parameters obtained with a perpective
  // projection model including a distorsion parameter
  robot.init(vpViper850::TOOL_MARLIN_F033C_CAMERA, vpCameraParameters::perspectiveProjWithDistortion);
  \endcode

  Now, you can get the intrinsic camera parameters associated to an
  image acquired by the camera attached to the robot, with:

  \code
  vpImage<unsigned char> I(480, 640);

  // Get an image from the camera attached to the robot
#ifdef VISP_HAVE_DC1394
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

  \sa vpCameraParameters,
  init(vpViper850::vpToolType, const vpHomogeneousMatrix&),
  init(vpViper850::vpToolType, const std::string&)
*/
void vpRobotViper850::init(vpViper850::vpToolType tool, vpCameraParameters::vpCameraParametersProjType projModel)
{
  // Read the robot constants from files
  // - joint [min,max], coupl_56, long_56
  // - camera extrinsic parameters relative to eMc
  vpViper850::init(tool, projModel);

  InitTry;

  // get real joint min/max from the MotionBlox
  Try(PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data));
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

  //   for (unsigned int i=0; i < njoint; i++) {
  //     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i],
  //     joint_max[i]);
  //   }

  // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (unsigned int i = 0; i < 3; i++) {
    eMc_pose[i] = etc[i];     // translation in meters
    eMc_pose[i + 3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try(PrimitiveCONST_Viper850(eMc_pose));

  CatchPrint();
  return;
}

/*!

  Initialize the robot kinematics (set the eMc homogeneous
  parameters in the low level controller) from a file and
  also get the joint limits from the low-level controller.

  \param tool : Tool to use.

  \param filename : Path of the configuration file containing the
  transformation between the end-effector frame and the tool frame.

  To set the transformation parameters related to the \f$^e{\bf
  M}_c\f$ matrix, use the code below:

  \code
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the transformation between the end-effector frame
  // and the tool frame from a file
  std::string filename("./EffectorToolTransformation.cnf");

  robot.init(vpViper850::TOOL_CUSTOM, filename);
#endif
}
  \endcode

  The configuration file should have the form below:

  \code
# Start with any number of consecutive lines
# beginning with the symbol '#'
#
# The 3 following lines contain the name of the camera,
# the rotation parameters of the geometric transformation
# using the Euler angles in degrees with convention XYZ and
# the translation parameters expressed in meters
CAMERA CameraName
eMc_ROT_XYZ 10.0 -90.0 20.0
eMc_TRANS_XYZ  0.05 0.01 0.06
    \endcode

  \sa init(), init(vpViper850::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpViper850::vpToolType, const vpHomogeneousMatrix&)
*/
void vpRobotViper850::init(vpViper850::vpToolType tool, const std::string &filename)
{
  vpViper850::init(tool, filename);

  InitTry;

  // Get real joint min/max from the MotionBlox
  Try(PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data));
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

  //   for (unsigned int i=0; i < njoint; i++) {
  //     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i],
  //     joint_max[i]);
  //   }

  // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (unsigned int i = 0; i < 3; i++) {
    eMc_pose[i] = etc[i];     // translation in meters
    eMc_pose[i + 3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try(PrimitiveCONST_Viper850(eMc_pose));

  CatchPrint();
  return;
}

/*!

  Initialize the robot kinematics with user defined parameters
  (set the eMc homogeneous parameters in the low level controller)
  and also get the joint limits from the low-level controller.

  \param tool : Tool to use.

  \param eMc_ : Transformation between the end-effector frame
  and the tool frame.

  To set the transformation parameters related to the \f$^e{\bf
  M}_c\f$ matrix, use the code below:

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Set the transformation between the end-effector frame
  // and the tool frame.
  vpHomogeneousMatrix eMc(0.001, 0.0, 0.1, 0.0, 0.0, M_PI/2);

  robot.init(vpViper850::TOOL_CUSTOM, eMc);
#endif
}
  \endcode

  \sa vpCameraParameters, init(), init(vpViper850::vpToolType,
  vpCameraParameters::vpCameraParametersProjType),
  init(vpViper850::vpToolType, const std::string&)
*/
void vpRobotViper850::init(vpViper850::vpToolType tool, const vpHomogeneousMatrix &eMc_)
{
  vpViper850::init(tool, eMc_);

  InitTry;

  // Get real joint min/max from the MotionBlox
  Try(PrimitiveJOINT_MINMAX_Viper850(joint_min.data, joint_max.data));
  // Convert units from degrees to radians
  joint_min.deg2rad();
  joint_max.deg2rad();

  //   for (unsigned int i=0; i < njoint; i++) {
  //     printf("axis %d: joint min %lf, max %lf\n", i, joint_min[i],
  //     joint_max[i]);
  //   }

  // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (unsigned int i = 0; i < 3; i++) {
    eMc_pose[i] = etc[i];     // translation in meters
    eMc_pose[i + 3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try(PrimitiveCONST_Viper850(eMc_pose));

  CatchPrint();
  return;
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame in the low level controller.

  \warning This function overwrite the transformation parameters that were
  potentially set using one of the init functions

  \param eMc_ : Transformation between the end-effector frame
  and the tool frame.
*/
void vpRobotViper850::set_eMc(const vpHomogeneousMatrix &eMc_)
{
  this->vpViper850::set_eMc(eMc_);

  InitTry;

  // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (unsigned int i = 0; i < 3; i++) {
    eMc_pose[i] = etc[i];     // translation in meters
    eMc_pose[i + 3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try(PrimitiveCONST_Viper850(eMc_pose));

  CatchPrint();

  return;
}

/*!

  Set the geometric transformation between the end-effector frame and
  the tool frame in the low level controller.

  \warning This function overwrite the transformation parameters that were
  potentially set using one of the init functions.

  \param etc_ : Translation between the end-effector frame and the tool frame.
  \param erc_ : Rotation between the end-effector frame and the tool frame
  using the Euler angles in radians with the XYZ convention.
*/
void vpRobotViper850::set_eMc(const vpTranslationVector &etc_, const vpRxyzVector &erc_)
{
  this->vpViper850::set_eMc(etc_, erc_);

  InitTry;

  // Set the camera constant (eMc pose) in the MotionBlox
  double eMc_pose[6];
  for (unsigned int i = 0; i < 3; i++) {
    eMc_pose[i] = etc[i];     // translation in meters
    eMc_pose[i + 3] = erc[i]; // rotation in rad
  }
  // Update the eMc pose in the low level controller
  Try(PrimitiveCONST_Viper850(eMc_pose));

  CatchPrint();

  return;
}

/* ------------------------------------------------------------------------ */
/* --- DESTRUCTOR --------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Destructor.

  Free allocated resources.
*/
vpRobotViper850::~vpRobotViper850(void)
{
#if defined(USE_ATI_DAQ) && defined(VISP_HAVE_COMEDI)
  ati.close();
#endif

  InitTry;

  setRobotState(vpRobot::STATE_STOP);

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try(PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, &HIPowerStatus));
  CAL_Wait(0.1);

  //   if (HIPowerStatus == 1) {
  //     fprintf(stdout, "Power OFF the robot\n");
  //     fflush(stdout);

  //     Try( PrimitivePOWEROFF_Viper850() );
  //   }

  // Free allocated resources
  ShutDownConnection();

  vpRobotViper850::robotAlreadyCreated = false;

  CatchPrint();
  return;
}

/*!

Change the robot state.

\param newState : New requested robot state.
*/
vpRobot::vpRobotStateType vpRobotViper850::setRobotState(vpRobot::vpRobotStateType newState)
{
  InitTry;

  switch (newState) {
  case vpRobot::STATE_STOP: {
    // Start primitive STOP only if the current state is Velocity
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      Try(PrimitiveSTOP_Viper850());
      vpTime::sleepMs(100); // needed to ensure velocity task ends up on low level
    }
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL == getRobotState()) {
      std::cout << "Change the control mode from velocity to position control.\n";
      Try(PrimitiveSTOP_Viper850());
    } else {
      // std::cout << "Change the control mode from stop to position
      // control.\n";
    }
    this->powerOn();
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control.\n";
    }
    this->powerOn();
    break;
  }
  default:
    break;
  }

  CatchPrint();

  return vpRobot::setRobotState(newState);
}

/* ------------------------------------------------------------------------ */
/* --- STOP --------------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/*!

  Stop the robot and set the robot state to vpRobot::STATE_STOP.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.
*/
void vpRobotViper850::stopMotion(void)
{
  if (getRobotState() != vpRobot::STATE_VELOCITY_CONTROL)
    return;

  InitTry;
  Try(PrimitiveSTOP_Viper850());
  setRobotState(vpRobot::STATE_STOP);

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot stop robot motion");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot stop robot motion.");
  }
}

/*!

  Power on the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot power on.

  \sa powerOff(), getPowerState()
*/
void vpRobotViper850::powerOn(void)
{
  InitTry;

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  UInt32 EStopStatus;
  bool firsttime = true;
  unsigned int nitermax = 10;

  for (unsigned int i = 0; i < nitermax; i++) {
    Try(PrimitiveSTATUS_Viper850(NULL, NULL, &EStopStatus, NULL, NULL, NULL, &HIPowerStatus));
    if (EStopStatus == ESTOP_AUTO) {
      controlMode = AUTO;
      break; // exit for loop
    } else if (EStopStatus == ESTOP_MANUAL) {
      controlMode = MANUAL;
      break; // exit for loop
    } else if (EStopStatus == ESTOP_ACTIVATED) {
      controlMode = ESTOP;
      if (firsttime) {
        std::cout << "Emergency stop is activated! \n"
                  << "Check the emergency stop button and push the yellow "
                     "button before continuing."
                  << std::endl;
        firsttime = false;
      }
      fprintf(stdout, "Remaining time %us  \r", nitermax - i);
      fflush(stdout);
      CAL_Wait(1);
    } else {
      std::cout << "Sorry there is an error on the emergency chain." << std::endl;
      std::cout << "You have to call Adept for maintenance..." << std::endl;
      // Free allocated resources
      ShutDownConnection();
      exit(0);
    }
  }

  if (EStopStatus == ESTOP_ACTIVATED)
    std::cout << std::endl;

  if (EStopStatus == ESTOP_ACTIVATED) {
    std::cout << "Sorry, cannot power on the robot." << std::endl;
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot power on the robot.");
  }

  if (HIPowerStatus == 0) {
    fprintf(stdout, "Power ON the Viper850 robot\n");
    fflush(stdout);

    Try(PrimitivePOWERON_Viper850());
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot power on the robot");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot power off the robot.");
  }
}

/*!

  Power off the robot.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error during robot stopping.

  \sa powerOn(), getPowerState()
*/
void vpRobotViper850::powerOff(void)
{
  InitTry;

  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try(PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, &HIPowerStatus));
  CAL_Wait(0.1);

  if (HIPowerStatus == 1) {
    fprintf(stdout, "Power OFF the Viper850 robot\n");
    fflush(stdout);

    Try(PrimitivePOWEROFF_Viper850());
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot power off the robot");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot power off the robot.");
  }
}

/*!

  Get the robot power state indication if power is on or off.

  \return true if power is on. false if power is off.

  \exception vpRobotException::lowLevelError : If the low level
  controller returns an error.

  \sa powerOn(), powerOff()
*/
bool vpRobotViper850::getPowerState(void) const
{
  InitTry;
  bool status = false;
  // Look if the power is on or off
  UInt32 HIPowerStatus;
  Try(PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, NULL, &HIPowerStatus));
  CAL_Wait(0.1);

  if (HIPowerStatus == 1) {
    status = true;
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot get the power status");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get the power status.");
  }
  return status;
}

/*!

  Get the twist transformation \f$^c{\bf V}_e\f$ from camera frame to
  end-effector frame.  This transformation allows to compute a
  velocity expressed in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void vpRobotViper850::get_cVe(vpVelocityTwistMatrix &cVe) const
{
  vpHomogeneousMatrix cMe;
  vpViper850::get_cMe(cMe);

  cVe.buildFrom(cMe);
}

/*!

  Get the geometric transformation \f$^c{\bf M}_e\f$ between the
  camera frame and the end-effector frame. This transformation is
  constant and correspond to the extrinsic camera parameters estimated
  by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void vpRobotViper850::get_cMe(vpHomogeneousMatrix &cMe) const { vpViper850::get_cMe(cMe); }

/*!

  Get the robot jacobian expressed in the end-effector frame.

  To compute \f$^e{\bf J}_e\f$, we communicate with the low level
  controller to get the joint position of the robot.

  \param eJe : Robot jacobian \f$^e{\bf J}_e\f$ expressed in the
  end-effector frame.

*/
void vpRobotViper850::get_eJe(vpMatrix &eJe)
{

  double position[6];
  double timestamp;

  InitTry;
  Try(PrimitiveACQ_POS_J_Viper850(position, &timestamp));
  CatchPrint();

  vpColVector q(6);
  for (unsigned int i = 0; i < njoint; i++)
    q[i] = vpMath::rad(position[i]);

  try {
    vpViper850::get_eJe(q, eJe);
  } catch (...) {
    vpERROR_TRACE("catch exception ");
    throw;
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

void vpRobotViper850::get_fJe(vpMatrix &fJe)
{

  double position[6];
  double timestamp;

  InitTry;
  Try(PrimitiveACQ_POS_Viper850(position, &timestamp));
  CatchPrint();

  vpColVector q(6);
  for (unsigned int i = 0; i < njoint; i++)
    q[i] = position[i];

  try {
    vpViper850::get_fJe(q, fJe);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Set the maximal velocity percentage to use for a position control.

  The default positioning velocity is defined by
  vpRobotViper850::defaultPositioningVelocity. This method allows to
  change this default positioning velocity

  \param velocity : Percentage of the maximal velocity. Values should
  be in ]0:100].

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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
void vpRobotViper850::setPositioningVelocity(const double velocity) { positioningVelocity = velocity; }

/*!
  Get the maximal velocity percentage used for a position control.

  \sa setPositioningVelocity()
*/
double vpRobotViper850::getPositioningVelocity(void) const { return positioningVelocity; }

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

  - In the joint space, positions are the six joint rotations starting
    from the base to the end-effector.

  - In the camera and the reference frame, positions are respectively
  X,Y,Z translations and 3 rotations arround the X, Y and Z
  axis. Rotations are represented by a vpRxyzVector.

  - Mixt frame is not implemented. By mixt frame we mean, translations
  expressed in the reference frame, and rotations in the camera
  frame.

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME
  and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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

  To catch the exception if the position is out of range, modify the code
like:

  \code
  try {
    robot.setPosition(vpRobot::CAMERA_FRAME, position);
  }
  catch (vpRobotException &e) {
    if (e.getCode() == vpRobotException::positionOutOfRangeError) {
    std::cout << "The position is out of range" << std::endl;
  }
  \endcode

*/
void vpRobotViper850::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &position)
{

  if (vpRobot::STATE_POSITION_CONTROL != getRobotState()) {
    vpERROR_TRACE("Robot was not in position-based control\n"
                  "Modification of the robot state");
    setRobotState(vpRobot::STATE_POSITION_CONTROL);
  }

  vpColVector destination(njoint);
  int error = 0;
  double timestamp;

  InitTry;
  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    vpColVector q(njoint);
    Try(PrimitiveACQ_POS_Viper850(q.data, &timestamp));

    // Convert degrees into rad
    q.deg2rad();

    // Get fMc from the inverse kinematics
    vpHomogeneousMatrix fMc;
    vpViper850::get_fMc(q, fMc);

    // Set cMc from the input position
    vpTranslationVector txyz;
    vpRxyzVector rxyz;
    for (unsigned int i = 0; i < 3; i++) {
      txyz[i] = position[i];
      rxyz[i] = position[i + 3];
    }

    // Compute cMc2
    vpRotationMatrix cRc2(rxyz);
    vpHomogeneousMatrix cMc2(txyz, cRc2);

    // Compute the new position to reach: fMc*cMc2
    vpHomogeneousMatrix fMc2 = fMc * cMc2;

    // Compute the corresponding joint position from the inverse kinematics
    unsigned int solution = this->getInverseKinematics(fMc2, q);
    if (solution) { // Position is reachable
      destination = q;
      // convert rad to deg requested for the low level controller
      destination.rad2deg();
      Try(PrimitiveMOVE_J_Viper850(destination.data, positioningVelocity));
      Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));
    } else {
      // Cartesian position is out of range
      error = -1;
    }

    break;
  }
  case vpRobot::ARTICULAR_FRAME: {
    destination = position;
    // convert rad to deg requested for the low level controller
    destination.rad2deg();

    // std::cout << "Joint destination (deg): " << destination.t() <<
    // std::endl;
    Try(PrimitiveMOVE_J_Viper850(destination.data, positioningVelocity));
    Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    // Convert angles from Rxyz representation to Rzyz representation
    vpRxyzVector rxyz(position[3], position[4], position[5]);
    vpRotationMatrix R(rxyz);
    vpRzyzVector rzyz(R);

    for (unsigned int i = 0; i < 3; i++) {
      destination[i] = position[i];
      destination[i + 3] = vpMath::deg(rzyz[i]); // convert also angles in deg
    }
    int configuration = 0; // keep the actual configuration

    // std::cout << "Base frame destination Rzyz (deg): " << destination.t()
    // << std::endl;
    Try(PrimitiveMOVE_C_Viper850(destination.data, configuration, positioningVelocity));
    Try(WaitState_Viper850(ETAT_ATTENTE_VIPER850, 1000));

    break;
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::lowLevelError, "Positionning error: "
                                                            "Mixt frame not implemented.");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::lowLevelError, "Positionning error: "
                                                            "End-effector frame not implemented.");
  }
  }

  CatchPrint();
  if (TryStt == InvalidPosition || TryStt == -1023)
    std::cout << " : Position out of range.\n";
  else if (TryStt == -3019) {
    if (frame == vpRobot::ARTICULAR_FRAME)
      std::cout << " : Joint position out of range.\n";
    else
      std::cout << " : Cartesian position leads to a joint position out of "
                   "range.\n";
  } else if (TryStt < 0)
    std::cout << " : Unknown error (see Fabien).\n";
  else if (error == -1)
    std::cout << "Position out of range.\n";

  if (TryStt < 0 || error < 0) {
    vpERROR_TRACE("Positionning error.");
    throw vpRobotException(vpRobotException::positionOutOfRangeError, "Position out of range.");
  }

  return;
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

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME
  and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \code
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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
void vpRobotViper850::setPosition(const vpRobot::vpControlFrameType frame, const double pos1, const double pos2,
                                  const double pos3, const double pos4, const double pos5, const double pos6)
{
  try {
    vpColVector position(6);
    position[0] = pos1;
    position[1] = pos2;
    position[2] = pos3;
    position[3] = pos4;
    position[4] = pos5;
    position[5] = pos6;

    setPosition(frame, position);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
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
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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

  \exception vpRobotException::lowLevelError : vpRobot::MIXT_FRAME
  and vpRobot::END_EFFECTOR_FRAME not implemented.

  \exception vpRobotException::positionOutOfRangeError : The requested
  position is out of range.

  \sa setPositioningVelocity()

*/
void vpRobotViper850::setPosition(const std::string &filename)
{
  vpColVector q;
  bool ret;

  ret = this->readPosFile(filename, q);

  if (ret == false) {
    vpERROR_TRACE("Bad position in \"%s\"", filename.c_str());
    throw vpRobotException(vpRobotException::lowLevelError, "Bad position in filename.");
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

  - in articular, a 6 dimension vector corresponding to the joint
  position of each dof in radians.

  - in reference frame, a 6 dimension vector, the first 3 values correspond to
  the translation tx, ty, tz in meters (like a vpTranslationVector), and the
  last 3 values to the rx, ry, rz rotation (like a vpRxyzVector). The code
  below show how to convert this position into a vpHomogeneousMatrix:

  \param timestamp : Time in second since last robot power on.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  vpColVector position;
  double timestamp;
  robot.getPosition(vpRobot::REFERENCE_FRAME, position, timestamp);

  vpTranslationVector ftc; // reference frame to camera frame translations
  vpRxyzVector frc; // reference frame to camera frame rotations

  // Update the transformation between reference frame and camera frame
  for (unsigned int i=0; i < 3; i++) {
    ftc[i] = position[i];   // tx, ty, tz
    frc[i] = position[i+3]; // ry, ry, rz
  }

  // Create a rotation matrix from the Rxyz rotation angles
  vpRotationMatrix fRc(frc); // reference frame to camera frame rotation matrix

  // Create the camera to fix frame transformation in terms of a
  // homogeneous matrix
  vpHomogeneousMatrix fMc(ftc, fRc);
#endif
}
  \endcode

  \exception vpRobotException::lowLevelError : If the position cannot
  be get from the low level controller.

  \sa setPosition(const vpRobot::vpControlFrameType frame, const
  vpColVector & r)

*/
void vpRobotViper850::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position, double &timestamp)
{

  InitTry;

  position.resize(6);

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    position = 0;
    return;
  }
  case vpRobot::ARTICULAR_FRAME: {
    Try(PrimitiveACQ_POS_J_Viper850(position.data, &timestamp));
    // vpCTRACE << "Get joint position (deg)" << position.t() << std::endl;
    position.deg2rad();

    return;
  }
  case vpRobot::REFERENCE_FRAME: {
    vpColVector q(njoint);
    Try(PrimitiveACQ_POS_J_Viper850(q.data, &timestamp));

    // Compute fMc
    vpHomogeneousMatrix fMc = vpViper850::get_fMc(q);

    // From fMc extract the pose
    vpRotationMatrix fRc;
    fMc.extract(fRc);
    vpRxyzVector rxyz;
    rxyz.buildFrom(fRc);

    for (unsigned int i = 0; i < 3; i++) {
      position[i] = fMc[i][3];   // translation x,y,z
      position[i + 3] = rxyz[i]; // Euler rotation x,y,z
    }

    break;
  }
  case vpRobot::MIXT_FRAME: {
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position in mixt frame: "
                                                            "not implemented");
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position in end-effector frame: "
                                                            "not implemented");
  }
  }

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot get position.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get position.");
  }

  return;
}

/*!
  Returns the robot controller current time (in second) since last robot power
  on.
*/
double vpRobotViper850::getTime() const
{
  double timestamp;
  PrimitiveACQ_TIME_Viper850(&timestamp);
  return timestamp;
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpColVector
  &, double &).

  The difference is here that the timestamp is not used.

*/
void vpRobotViper850::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &position)
{
  double timestamp;
  getPosition(frame, position, timestamp);
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpColVector
  &, double &).

  The difference is here that the position is returned using a \f$ \theta {\bf
  u}\f$ representation.

*/
void vpRobotViper850::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position, double &timestamp)
{
  vpColVector posRxyz;
  // recupere  position en Rxyz
  this->getPosition(frame, posRxyz, timestamp);
  vpRxyzVector RxyzVect;
  for (unsigned int j = 0; j < 3; j++)
    RxyzVect[j] = posRxyz[j + 3];
  // recupere le vecteur thetaU correspondant
  vpThetaUVector RtuVect(RxyzVect);

  // remplit le vpPoseVector avec translation et rotation ThetaU
  for (unsigned int j = 0; j < 3; j++) {
    position[j] = posRxyz[j];
    position[j + 3] = RtuVect[j];
  }
}

/*!

  Get the current position of the robot.

  Similar as getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector
  &, double &).

  The difference is here that the timestamp is not used.

*/
void vpRobotViper850::getPosition(const vpRobot::vpControlFrameType frame, vpPoseVector &position)
{
  vpColVector posRxyz;
  double timestamp;
  // recupere  position en Rxyz
  this->getPosition(frame, posRxyz, timestamp);
  vpRxyzVector RxyzVect;
  for (unsigned int j = 0; j < 3; j++)
    RxyzVect[j] = posRxyz[j + 3];
  // recupere le vecteur thetaU correspondant
  vpThetaUVector RtuVect(RxyzVect);

  // remplit le vpPoseVector avec translation et rotation ThetaU
  for (unsigned int j = 0; j < 3; j++) {
    position[j] = posRxyz[j];
    position[j + 3] = RtuVect[j];
  }
}

/*!
  Apply a velocity to the robot.

  \param frame : Control frame in which the velocity is expressed. Velocities
  could be expressed in articular, camera frame, reference frame or mixt
frame.

  \param vel : Velocity vector. Translation velocities are expressed
  in m/s while rotation velocities in rad/s. The size of this vector
  is always 6.

  - In articular, \f$ vel = [\dot{q}_1, \dot{q}_2, \dot{q}_3, \dot{q}_4,
  \dot{q}_5, \dot{q}_6]^t \f$ correspond to joint velocities in rad/s.

  - In camera frame, \f$ vel = [^{c} v_x, ^{c} v_y, ^{c} v_z, ^{c}
  \omega_x, ^{c} \omega_y, ^{c} \omega_z]^t \f$ is a velocity twist vector
expressed in the camera frame, with translations velocities \f$ ^{c} v_x, ^{c}
v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
\omega_y, ^{c} \omega_z \f$ in rad/s.

  - In reference frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{r}
  \omega_x, ^{r} \omega_y, ^{r} \omega_z]^t \f$ is a velocity twist vector
expressed in the reference frame, with translations velocities \f$ ^{c} v_x,
^{c} v_y, ^{c} v_z \f$ in m/s and rotation velocities \f$ ^{c}\omega_x, ^{c}
\omega_y, ^{c} \omega_z \f$ in rad/s.

  - In mixt frame, \f$ vel = [^{r} v_x, ^{r} v_y, ^{r} v_z, ^{c} \omega_x,
  ^{c} \omega_y, ^{c} \omega_z]^t \f$ is a velocity twist vector where,
translations \f$ ^{r} v_x, ^{r} v_y, ^{r} v_z \f$ are expressed in the
reference frame in m/s and rotations \f$ ^{c} \omega_x, ^{c} \omega_y, ^{c}
\omega_z \f$ in the camera frame in rad/s.

  \exception vpRobotException::wrongStateError : If a the robot is not
  configured to handle a velocity. The robot can handle a velocity only if the
  velocity control mode is set. For that, call setRobotState(
  vpRobot::STATE_VELOCITY_CONTROL) before setVelocity().

  \warning Velocities could be saturated if one of them exceed the
  maximal autorized speed (see vpRobot::maxTranslationVelocity and
  vpRobot::maxRotationVelocity). To change these values use
  setMaxTranslationVelocity() and setMaxRotationVelocity().

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpRobotViper850.h>

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
void vpRobotViper850::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != getRobotState()) {
    vpERROR_TRACE("Cannot send a velocity to the robot "
                  "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot "
                           "use setRobotState(vpRobot::STATE_VELOCITY_CONTROL) first) ");
  }

  vpColVector vel_sat(6);

  // Velocity saturation
  switch (frame) {
  // saturation in cartesian space
  case vpRobot::CAMERA_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    break;
  }
  // saturation in joint space
  case vpRobot::ARTICULAR_FRAME: {
    vpColVector vel_max(6);

    // if (getMaxRotationVelocity() == getMaxRotationVelocityJoint6()) {
    if (std::fabs(getMaxRotationVelocity() - getMaxRotationVelocityJoint6()) < std::numeric_limits<double>::epsilon()) {

      for (unsigned int i = 0; i < 6; i++)
        vel_max[i] = getMaxRotationVelocity();
    } else {
      for (unsigned int i = 0; i < 5; i++)
        vel_max[i] = getMaxRotationVelocity();
      vel_max[5] = getMaxRotationVelocityJoint6();
    }

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);
  }
  }

  InitTry;

  switch (frame) {
  case vpRobot::CAMERA_FRAME: {
    // Send velocities in m/s and rad/s
    // std::cout << "Vitesse cam appliquee: " << vel_sat.t();
    Try(PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPCAM_VIPER850));
    break;
  }
  case vpRobot::END_EFFECTOR_FRAME: {
    // Transform in camera frame
    vpHomogeneousMatrix cMe;
    this->get_cMe(cMe);
    vpVelocityTwistMatrix cVe(cMe);
    vpColVector v_c = cVe * vel_sat;
    // Send velocities in m/s and rad/s
    Try(PrimitiveMOVESPEED_CART_Viper850(v_c.data, REPCAM_VIPER850));
    break;
  }
  case vpRobot::ARTICULAR_FRAME: {
    // Convert all the velocities from rad/s into deg/s
    vel_sat.rad2deg();
    // std::cout << "Vitesse appliquee: " << vel_sat.t();
    // Try( PrimitiveMOVESPEED_CART(vel_sat.data, REPART_VIPER850) );
    Try(PrimitiveMOVESPEED_Viper850(vel_sat.data));
    break;
  }
  case vpRobot::REFERENCE_FRAME: {
    // Send velocities in m/s and rad/s
    //std::cout << "Vitesse ref appliquee: " << vel_sat.t();
    Try(PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPFIX_VIPER850));
    break;
  }
  case vpRobot::MIXT_FRAME: {
    // Try( PrimitiveMOVESPEED_CART_Viper850(vel_sat.data, REPMIX_VIPER850) );
    break;
  }
  default: {
    vpERROR_TRACE("Error in spec of vpRobot. "
                  "Case not taken in account.");
    return;
  }
  }

  Catch();
  if (TryStt < 0) {
    if (TryStt == VelStopOnJoint) {
      UInt32 axisInJoint[njoint];
      PrimitiveSTATUS_Viper850(NULL, NULL, NULL, NULL, NULL, axisInJoint, NULL);
      for (unsigned int i = 0; i < njoint; i++) {
        if (axisInJoint[i])
          std::cout << "\nWarning: Velocity control stopped: axis " << i + 1 << " on joint limit!" << std::endl;
      }
    } else {
      printf("\n%s(%d): Error %d", __FUNCTION__, TryLine, TryStt);
      if (TryString != NULL) {
        // The statement is in TryString, but we need to check the validity
        printf(" Error sentence %s\n", TryString); // Print the TryString
      } else {
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

  \param timestamp : Time in second since last robot power on.

  \warning In camera frame, reference frame and mixt frame, the representation
  of the rotation is \f$ \theta {\bf u}\f$. In that cases, \f$velocity = [\dot
x, \dot y, \dot z, \dot {\theta u}_x, \dot {\theta u}_y, \dot {\theta
u}_z]\f$.

  \warning The first time this method is called, \e velocity is set to 0. The
  first call is used to intialise the velocity computation for the next call.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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
void vpRobotViper850::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity, double &timestamp)
{

  velocity.resize(6);
  velocity = 0;

  vpColVector q_cur(6);
  vpHomogeneousMatrix fMc_cur;
  vpHomogeneousMatrix cMc; // camera displacement
  double time_cur;

  InitTry;

  // Get the current joint position
  Try(PrimitiveACQ_POS_J_Viper850(q_cur.data, &timestamp));
  time_cur = timestamp;
  q_cur.deg2rad();

  // Get the camera pose from the direct kinematics
  vpViper850::get_fMc(q_cur, fMc_cur);

  if (!first_time_getvel) {

    switch (frame) {
    case vpRobot::CAMERA_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      velocity = vpExponentialMap::inverse(cMc, time_cur - time_prev_getvel);

      break;
    }

    case vpRobot::ARTICULAR_FRAME: {
      velocity = (q_cur - q_prev_getvel) / (time_cur - time_prev_getvel);
      break;
    }

    case vpRobot::REFERENCE_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the velocity of the camera from this displacement
      vpColVector v;
      v = vpExponentialMap::inverse(cMc, time_cur - time_prev_getvel);

      // Express this velocity in the reference frame
      vpVelocityTwistMatrix fVc(fMc_cur);
      velocity = fVc * v;

      break;
    }

    case vpRobot::MIXT_FRAME: {
      // Compute the displacement of the camera since the previous call
      cMc = fMc_prev_getvel.inverse() * fMc_cur;

      // Compute the ThetaU representation for the rotation
      vpRotationMatrix cRc;
      cMc.extract(cRc);
      vpThetaUVector thetaU;
      thetaU.buildFrom(cRc);

      for (unsigned int i = 0; i < 3; i++) {
        // Compute the translation displacement in the reference frame
        velocity[i] = fMc_prev_getvel[i][3] - fMc_cur[i][3];
        // Update the rotation displacement in the camera frame
        velocity[i + 3] = thetaU[i];
      }

      // Compute the velocity
      velocity /= (time_cur - time_prev_getvel);
      break;
    }
    default: {
      throw(vpException(vpException::functionNotImplementedError,
                        "vpRobotViper850::getVelocity() not implemented in end-effector"));
    }
    }
  } else {
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
    vpERROR_TRACE("Cannot get velocity.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get velocity.");
  }
}

/*!

  Get robot velocities.

  The behavior is the same than getVelocity(const vpRobot::vpControlFrameType,
  vpColVector &, double &) except that the timestamp is not returned.

  */
void vpRobotViper850::getVelocity(const vpRobot::vpControlFrameType frame, vpColVector &velocity)
{
  double timestamp;
  getVelocity(frame, velocity, timestamp);
}

/*!

  Get the robot velocities.

  \param frame : Frame in wich velocities are mesured.

  \param timestamp : Time in second since last robot power on.

  \return Measured velocities. Translations are expressed in m/s
  and rotations in rad/s.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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

  double timestamp;
  while (1) {
     q_dot_mes = robot.getVelocity(vpRobot::ARTICULAR_FRAME, timestamp);
     vpTime::wait(40); // wait 40 ms
     // here q_dot_mes is equal to [0.1, 0.2, 0.3, M_PI/8, M_PI/4, M_PI/16]
  }
#endif
}
  \endcode
*/
vpColVector vpRobotViper850::getVelocity(vpRobot::vpControlFrameType frame, double &timestamp)
{
  vpColVector velocity;
  getVelocity(frame, velocity, timestamp);

  return velocity;
}

/*!

  Get robot velocities.

  The behavior is the same than getVelocity(const vpRobot::vpControlFrameType,
  double &) except that the timestamp is not returned.

  */
vpColVector vpRobotViper850::getVelocity(vpRobot::vpControlFrameType frame)
{
  vpColVector velocity;
  double timestamp;
  getVelocity(frame, velocity, timestamp);

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

The code below shows how to read a position from a file and move the robot to
this position.
\code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotViper850.h>

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

bool vpRobotViper850::readPosFile(const std::string &filename, vpColVector &q)
{
  std::ifstream fd(filename.c_str(), std::ios::in);

  if (!fd.is_open()) {
    return false;
  }

  std::string line;
  std::string key("R:");
  std::string id("#Viper850 - Position");
  bool pos_found = false;
  int lineNum = 0;

  q.resize(njoint);

  while (std::getline(fd, line)) {
    lineNum++;
    if (lineNum == 1) {
      if (!(line.compare(0, id.size(), id) == 0)) { // check if Viper850 position file
        std::cout << "Error: this position file " << filename << " is not for Viper850 robot" << std::endl;
        return false;
      }
    }
    if ((line.compare(0, 1, "#") == 0)) { // skip comment
      continue;
    }
    if ((line.compare(0, key.size(), key) == 0)) { // decode position
      // check if there are at least njoint values in the line
      std::vector<std::string> chain = vpIoTools::splitChain(line, std::string(" "));
      if (chain.size() < njoint + 1) // try to split with tab separator
        chain = vpIoTools::splitChain(line, std::string("\t"));
      if (chain.size() < njoint + 1)
        continue;

      std::istringstream ss(line);
      std::string key_;
      ss >> key_;
      for (unsigned int i = 0; i < njoint; i++)
        ss >> q[i];
      pos_found = true;
      break;
    }
  }

  // converts rotations from degrees into radians
  q.deg2rad();

  fd.close();

  if (!pos_found) {
    std::cout << "Error: unable to find a position for Viper850 robot in " << filename << std::endl;
    return false;
  }

  return true;
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

bool vpRobotViper850::savePosFile(const std::string &filename, const vpColVector &q)
{

  FILE *fd;
  fd = fopen(filename.c_str(), "w");
  if (fd == NULL)
    return false;

  fprintf(fd, "\
#Viper850 - Position - Version 1.00\n\
#\n\
# R: A B C D E F\n\
# Joint position in degrees\n\
#\n\
#\n\n");

  // Save positions in mm and deg
  fprintf(fd, "R: %lf %lf %lf %lf %lf %lf\n", vpMath::deg(q[0]), vpMath::deg(q[1]), vpMath::deg(q[2]),
          vpMath::deg(q[3]), vpMath::deg(q[4]), vpMath::deg(q[5]));

  fclose(fd);
  return (true);
}

/*!

  Moves the robot to the joint position specified in the filename. The
  positioning velocity is set to 10% of the robot maximal velocity.

  \param filename: File containing a joint position.

  \sa readPosFile

*/
void vpRobotViper850::move(const std::string &filename)
{
  vpColVector q;

  try {
    this->readPosFile(filename, q);
    this->setRobotState(vpRobot::STATE_POSITION_CONTROL);
    this->setPositioningVelocity(10);
    this->setPosition(vpRobot::ARTICULAR_FRAME, q);
  } catch (...) {
    throw;
  }
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

*/
void vpRobotViper850::getDisplacement(vpRobot::vpControlFrameType frame, vpColVector &displacement)
{
  displacement.resize(6);
  displacement = 0;

  double q[6];
  vpColVector q_cur(6);
  double timestamp;

  InitTry;

  // Get the current joint position
  Try(PrimitiveACQ_POS_Viper850(q, &timestamp));
  for (unsigned int i = 0; i < njoint; i++) {
    q_cur[i] = q[i];
  }

  if (!first_time_getdis) {
    switch (frame) {
    case vpRobot::CAMERA_FRAME: {
      std::cout << "getDisplacement() CAMERA_FRAME not implemented\n";
      return;
    }

    case vpRobot::ARTICULAR_FRAME: {
      displacement = q_cur - q_prev_getdis;
      break;
    }

    case vpRobot::REFERENCE_FRAME: {
      std::cout << "getDisplacement() REFERENCE_FRAME not implemented\n";
      return;
    }

    case vpRobot::MIXT_FRAME: {
      std::cout << "getDisplacement() MIXT_FRAME not implemented\n";
      return;
    }
    case vpRobot::END_EFFECTOR_FRAME: {
      std::cout << "getDisplacement() END_EFFECTOR_FRAME not implemented\n";
      return;
    }
    }
  } else {
    first_time_getdis = false;
  }

  // Memorize the joint position for the next call
  q_prev_getdis = q_cur;

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot get velocity.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get velocity.");
  }
}

/*!

  Bias the force/torque sensor.

  \sa unbiasForceTorqueSensor(), getForceTorque()

*/
void vpRobotViper850::biasForceTorqueSensor()
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  ati.bias();
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. Try sudo "
                                             "apt-get install libcomedi-dev"));
#endif
#else // Use serial link
  InitTry;

  Try(PrimitiveTFS_BIAS_Viper850());

  // Wait 500 ms to be sure the next measures take into account the bias
  vpTime::wait(500);

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot bias the force/torque sensor.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot bias the force/torque sensor.");
  }
#endif
}

/*!

  Unbias the force/torque sensor.

  \sa biasForceTorqueSensor(), getForceTorque()

*/
void vpRobotViper850::unbiasForceTorqueSensor()
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  ati.unbias();
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. Try sudo "
                                             "apt-get install libcomedi-dev"));
#endif
#else // Use serial link
// Not implemented
#endif
}

/*!

  Get the rough force/torque sensor measures.

  \param H: [Fx, Fy, Fz, Tx, Ty, Tz] Forces/torques measured by the sensor.

  The code below shows how to get the force/torque measures after a sensor
bias.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpColVector  H; // force/torque measures [Fx, Fy, Fz, Tx, Ty, Tz]

  vpRobotViper850 robot;

  // Bias the force/torque sensor
  robot.biasForceTorqueSensor();

  for (unsigned int i=0; i< 10; i++) {
    robot.getForceTorque(H) ;
    std::cout << "Measured force/torque: " << H.t() << std::endl;
    vpTime::wait(5);
  }
#endif
}
  \endcode

  \exception vpRobotException::lowLevelError : If the force/torque measures
  cannot be get from the low level controller.

  \sa biasForceTorqueSensor(), unbiasForceTorqueSensor()

*/
void vpRobotViper850::getForceTorque(vpColVector &H) const
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  H = ati.getForceTorque();
#else
  (void)H;
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. Try sudo "
                                             "apt-get install libcomedi-dev"));
#endif
#else // Use serial link
  InitTry;

  H.resize(6);

  Try(PrimitiveTFS_ACQ_Viper850(H.data));

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot get the force/torque measures.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get force/torque measures.");
  }
#endif
}

/*!

  Get the rough force/torque sensor measures.

  \return [Fx, Fy, Fz, Tx, Ty, Tz] Forces/torques measured by the sensor.

  The code below shows how to get the force/torque measures after a sensor
bias.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpTime.h>
#include <visp3/robot/vpRobotViper850.h>

int main()
{
#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;

  // Bias the force/torque sensor
  robot.biasForceTorqueSensor();

  for (unsigned int i=0; i< 10; i++) {
    vpColVector H = robot.getForceTorque(); // force/torque measures [Fx, Fy, Fz, Tx, Ty, Tz]
    std::cout << "Measured force/torque: " << H.t() << std::endl;
    vpTime::wait(5);
  }
#endif
}
  \endcode

  \exception vpRobotException::lowLevelError : If the force/torque measures
  cannot be get from the low level controller.

  \sa biasForceTorqueSensor(), unbiasForceTorqueSensor()

*/
vpColVector vpRobotViper850::getForceTorque() const
{
#if defined(USE_ATI_DAQ)
#if defined(VISP_HAVE_COMEDI)
  vpColVector H = ati.getForceTorque();
  return H;
#else
  throw(vpException(vpException::fatalError, "Cannot use ATI F/T if comedi is not installed. Try sudo "
                                             "apt-get install libcomedi-dev"));
#endif
#else // Use serial link
  InitTry;

  vpColVector H(6);

  Try(PrimitiveTFS_ACQ_Viper850(H.data));
  return H;

  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot get the force/torque measures.");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot get force/torque measures.");
  }
  return H; // Here to avoid a warning, but should never be called
#endif
}

/*!

  Open the pneumatic two fingers gripper.

  \sa closeGripper()
*/
void vpRobotViper850::openGripper()
{
  InitTry;
  Try(PrimitivePneumaticGripper_Viper850(1));
  std::cout << "Open the pneumatic gripper..." << std::endl;
  CatchPrint();
  if (TryStt < 0) {
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot open the gripper.");
  }
}

/*!

  Close the pneumatic two fingers gripper.

  \sa openGripper()

*/
void vpRobotViper850::closeGripper() const
{
  InitTry;
  Try(PrimitivePneumaticGripper_Viper850(0));
  std::cout << "Close the pneumatic gripper..." << std::endl;
  CatchPrint();
  if (TryStt < 0) {
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot close the gripper.");
  }
}

/*!
  Enable the joint limits on axis number 6. This is the default.

  \sa disbleJoint6Limits()
*/
void vpRobotViper850::enableJoint6Limits() const
{
  InitTry;
  Try(PrimitiveREMOVE_JOINT6_LIMITS_Viper850(0));
  std::cout << "Enable joint limits on axis 6..." << std::endl;
  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot enable joint limits on axis 6");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot enable joint limits on axis 6.");
  }
}

/*!
  \warning Each call to this function should be done carefully.

  Disable the joint limits on axis number 6. When joint 6 is outside the
  limits, a call to this function allows to bring the robot to a position
  inside the limits. Don't forget then to call enableJoint6Limits() to reduce
  the working space for joint 6.

  \sa enableJoint6Limits()
*/
void vpRobotViper850::disableJoint6Limits() const
{
  InitTry;
  Try(PrimitiveREMOVE_JOINT6_LIMITS_Viper850(1));
  std::cout << "Warning: Disable joint limits on axis 6..." << std::endl;
  CatchPrint();
  if (TryStt < 0) {
    vpERROR_TRACE("Cannot disable joint limits on axis 6");
    throw vpRobotException(vpRobotException::lowLevelError, "Cannot disable joint limits on axis 6.");
  }
}

/*!

  Set the maximal rotation velocity that can be sent to the robot  during a
  velocity control.

  \param w_max : Maximum rotation velocity expressed in rad/s.
*/

void vpRobotViper850::setMaxRotationVelocity(double w_max)
{
  vpRobot::setMaxRotationVelocity(w_max);
  setMaxRotationVelocityJoint6(w_max);

  return;
}

/*!

  Set the maximal rotation velocity on joint 6 that is used only during
  velocity joint control.

  This function affects only the velocities that are sent as joint velocities.

  \code
  vpRobotViper850 robot;
  robot.setMaxRotationVelocity( vpMath::rad(20) );
  robot.setMaxRotationVelocityJoint6( vpMath::rad(50) );

  robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
  robot.setVelocity(ARTICULAR_FRAME, v);
  \endcode


  \param w6_max : Maximum rotation velocity expressed in rad/s on joint 6.
*/

void vpRobotViper850::setMaxRotationVelocityJoint6(const double w6_max)
{
  maxRotationVelocity_joint6 = w6_max;
  return;
}

/*!

  Get the maximal rotation velocity on joint 6 that is used only during
  velocity joint control.

  \return Maximum rotation velocity on joint 6 expressed in rad/s.
*/
double vpRobotViper850::getMaxRotationVelocityJoint6() const { return maxRotationVelocity_joint6; }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpRobotViper850.cpp.o) has
// no symbols
void dummy_vpRobotViper850(){};
#endif
