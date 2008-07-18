/****************************************************************************
 *
 * $Id: vpAfma6.cpp,v 1.27 2008-07-18 09:51:52 fspindle Exp $
 *
 * Copyright (C) 1998-2008 Inria. All rights reserved.
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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpAfma6.cpp

  Control of Irisa's gentry robot named Afma6.

*/


#include <visp/vpDebug.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpRobotException.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpAfma6.h>

/* ----------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ---------------------------------------------------------------------- */

static char *opt_Afma6[] = {"JOINT_MAX","JOINT_MIN","LONG_56","COUPL_45",
			    "CAMERA", "eMc_ROT_XYZ","eMc_TRANS_XYZ",			    
			    NULL};

const char * const vpAfma6::CONST_AFMA6_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_Afma6.cnf";

const char * const vpAfma6::CONST_EMC_DRAGONFLY2_WITHOUT_DISTORTION_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_eMc_Dragonfly2_without_distortion_Afma6.cnf";
const char * const vpAfma6::CONST_EMC_DRAGONFLY2_WITH_DISTORTION_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_eMc_Dragonfly2_with_distortion_Afma6.cnf";

const char * const vpAfma6::CONST_CAMERA_AFMA6_FILENAME
= "/udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml";

const char * const vpAfma6::CONST_LABEL_DRAGONFLY2 = "Dragonfly2-8mm";

const vpAfma6::vpAfma6CameraRobotType vpAfma6::defaultCameraRobot = CAMERA_DRAGONFLY2_8MM;

const int vpAfma6::njoint = 6;

/*!

  Default constructor.

*/
vpAfma6::vpAfma6()
{
  //
  // Geometric model constant parameters
  //
  // coupling between join 4 and 5
  this->_coupl_45 = 0; 
  // distance between join 5 and 6
  this->_long_56  = 0; 
  // Camera extrinsic parameters: effector to camera frame
  this->_eMc.setIdentity(); 
  // Maximal value of the joints
  for (int i=0; i < 6; i ++)
    this->_joint_max[i] = 0; 
  // Minimal value of the joints
  for (int i=0; i < 6; i ++)
    this->_joint_min[i] = 0; 

  init();

}

/*!

  Initialize the robot with the default camera vpAfma6::defaultCameraRobot.
 */
void 
vpAfma6::init (void)
{
  this->init ( vpAfma6::defaultCameraRobot);
  return;
}

/*!

  Read files containing the constant parameters related to the robot
  kinematics and to the end-effector to camera transformation.

  \param paramAfma6 : Filename containing the constant parameters of
  the robot kinematics.

  \param paramCamera : Filename containing the camera extrinsic parameters.

*/
void 
vpAfma6::init (const char * paramAfma6,
	       const char * paramCamera)
{
  //  vpTRACE ("Parsage fichier robot.");
  this->parseConfigFile (paramAfma6);

  //vpTRACE ("Parsage fichier camera.");
  this->parseConfigFile (paramCamera);

  return ;
}

/*!
 
  Get the constant parameters related to the robot kinematics and to
  the end-effector to camera transformation (eMc) correponding to the
  camera extrinsic parameters. These last parameters depend on the
  camera and projection model in use.

  \param camera : Camera in use.

  \param projModel : Projection model of the camera.
 
*/
void 
vpAfma6::init (vpAfma6::vpAfma6CameraRobotType camera,
	       vpCameraParameters::vpCameraParametersProjType projModel)
{
  char filename_eMc [FILENAME_MAX];
  this->projModel = projModel;
  switch (camera)
  {
  case vpAfma6::CAMERA_DRAGONFLY2_8MM:
    {   
      switch(projModel){   
      case vpCameraParameters::perspectiveProjWithoutDistortion :
#ifdef UNIX
        snprintf(filename_eMc, FILENAME_MAX, "%s",
	         CONST_EMC_DRAGONFLY2_WITHOUT_DISTORTION_FILENAME);
#else // WIN32
        _snprintf(filename_eMc, FILENAME_MAX, "%s",
	         CONST_EMC_DRAGONFLY2_WITHOUT_DISTORTION_FILENAME);
#endif
        break;
      case vpCameraParameters::perspectiveProjWithDistortion :
#ifdef UNIX
        snprintf(filename_eMc, FILENAME_MAX, "%s",
           CONST_EMC_DRAGONFLY2_WITH_DISTORTION_FILENAME);
#else // WIN32
        _snprintf(filename_eMc, FILENAME_MAX, "%s",
           CONST_EMC_DRAGONFLY2_WITH_DISTORTION_FILENAME);
#endif
        break;    
      }   
      break;
    }
  default:
    {
      vpERROR_TRACE ("Cette erreur ne peut pas arriver.");
      vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
		   "que les specs de la classe ont ete modifiee, "
		   "et que le code n'a pas ete mis a jour "
		   "correctement.");
      vpERROR_TRACE ("Verifiez les valeurs possibles du type "
		   "vpAfma6::vpAfma6CameraRobotType, et controlez que "
		   "tous les cas ont ete pris en compte dans la "
		   "fonction init(camera).");
      break;
    }
  }

  this->init (vpAfma6::CONST_AFMA6_FILENAME, filename_eMc);

  setCameraRobotType(camera);
  return ;
}


/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the six joints. 

  This method is the same than get_fMc(const vpColVector & q).

  \param q : Articular position of the six joints: q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix 
vpAfma6::getForwardKinematics(const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Compute the inverse kinematics (inverse geometric model).

  By inverse kinematics we mean here the six articular values of the joint
  positions given the position and the orientation of the camera frame
  relative to the base frame. 

  \param fMc : Homogeneous matrix describing the transformation from
  base frame to the camera frame.

  \param q : In input, the current articular joint position of the
  robot. In output, the solution of the inverse kinematics. Articular
  position of the six joints: q[0], q[1], q[2] correspond to the first
  3 translations expressed in meter, while q[3], q[4] and q[5]
  correspond to the 3 succesives rotations expressed in radians.

  \param nearest : true to return the nearest solution to q. false to
  return the farest.

  \return The number of solutions (1 or 2) of the inverse geometric
  model. O no solution can be found.

  The code below shows how to compute the inverse geometric model:

  \code
  vpColVector q1(6), q2(6);
  vpHomogeneousMatrix fMc;

  a6RobotAfma6 robot;

  // Get the current articular position of the robot
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q1);

  // Compute the pose of the camera in the reference frame using the
  // direct geometric model
  fMc = robot.getForwardKinematics(q1);
  // this is similar to  fMc = robot.get_fMc(q1);
  // or robot.get_fMc(q1, fMc);


  // Compute the inverse geometric model
  int nbsol; // number of solutions (0, 1 or 2) of the inverse geometric model
  // get the nearest solution to the current articular position
  nbsol = robot.getInverseKinematics(fMc, q1, true);

  if (nbsol == 0)
    std::cout << "No solution of the inverse geometric model " << std::endl;
  else if (nbsol >= 1)
    std::cout << "First solution: " << q1 << std::endl;

  if (nbsol == 2) {
    // Compute the other solution of the inverse geometric model
    q2 = q1;
    robot.getInverseKinematics(fMc, q2, false);
    std::cout << "Second solution: " << q2 << std::endl;
  }
  \endcode

sr	  \sa getForwardKinematics()

*/
int 
vpAfma6::getInverseKinematics(const vpHomogeneousMatrix & fMc, 
			      vpColVector & q, const bool &nearest)
{
  vpHomogeneousMatrix fMe;
  double q_[2][6],d[2],t;
  int ok[2];
  double cord[6];

  int nbsol = 0;

  q.resize(6);

//   for(int i=0;i<3;i++) {
//     fMe[i][3] = fMc[i][3];
//     for(int j=0;j<3;j++) {
//       fMe[i][j] = 0.0;
//       for (int k=0;k<3;k++) fMe[i][j] += fMc[i][k]*rpi[j][k];
//       fMe[i][3] -= fMe[i][j]*rpi[j][3]; 
//     }
//   }
  
  std::cout << "\n\nfMc: " << fMc;
  std::cout << "\n\neMc: " << _eMc;

  fMe = fMc * this->_eMc.inverse();
  std::cout << "\n\nfMe: " << fMe;

  if (fMe[2][2] >= .99999f)
  {
    vpTRACE("singularity\n");
    q_[0][4] = q_[1][4] = M_PI/2.f;
    t = atan2(fMe[0][0],fMe[0][1]);
    q_[1][3] = q_[0][3] = q[3];
    q_[1][5] = q_[0][5] = t - q_[0][3];

    while  ((q_[1][5]+vpMath::rad(2)) >= this->_joint_max[5])
      /*			-> a cause du couplage 4/5	*/
    {
      q_[1][5] -= vpMath::rad(10);
      q_[1][3] += vpMath::rad(10);
    }
    while  (q_[1][5] <= this->_joint_min[5])
    {
      q_[1][5] += vpMath::rad(10);
      q_[1][3] -= vpMath::rad(10);
    }
  }
  else if (fMe[2][2] <= -.99999)
  {
    vpTRACE("singularity\n");
    q_[0][4] = q_[1][4] = -M_PI/2;
    t = atan2(fMe[1][1],fMe[1][0]);
    q_[1][3] = q_[0][3] = q[3];
    q_[1][5] = q_[0][5] = q_[0][3] - t;
    while  ((q_[1][5]+vpMath::rad(2)) >= this->_joint_max[5])
      /*			-> a cause du couplage 4/5	*/
    {
      q_[1][5] -= vpMath::rad(10);
      q_[1][3] -= vpMath::rad(10);
    }
    while  (q_[1][5] <= this->_joint_min[5])
    {
      q_[1][5] += vpMath::rad(10);
      q_[1][3] += vpMath::rad(10);
    }
  }
  else
  {
    q_[0][3] = atan2(-fMe[0][2],fMe[1][2]);
    if (q_[0][3] >= 0.0) q_[1][3] = q_[0][3] - M_PI;
    else q_[1][3] = q_[0][3] + M_PI;

    q_[0][4] = asin(fMe[2][2]);
    if (q_[0][4] >= 0.0) q_[1][4] = M_PI - q_[0][4];
    else q_[1][4] = -M_PI - q_[0][4];

    q_[0][5] = atan2(-fMe[2][1],fMe[2][0]);
    if (q_[0][5] >= 0.0) q_[1][5] = q_[0][5] - M_PI;
    else q_[1][5] = q_[0][5] + M_PI;
  }
  q_[0][0] = fMe[0][3] - this->_long_56*cos(q_[0][3]);
  q_[1][0] = fMe[0][3] - this->_long_56*cos(q_[1][3]);
  q_[0][1] = fMe[1][3] - this->_long_56*sin(q_[0][3]);
  q_[1][1] = fMe[1][3] - this->_long_56*sin(q_[1][3]);
  q_[0][2] = q_[1][2] = fMe[2][3];

  /* prise en compte du couplage axes 5/6	*/
  q_[0][5] += this->_coupl_45*q_[0][4];
  q_[1][5] += this->_coupl_45*q_[1][4];

  for (int j=0;j<2;j++)
  {
    ok[j] = 1;
    // test is position is reachable
    for (int i=0;i<6;i++) {
      if (q_[j][i] < this->_joint_min[i] || q_[j][i] > this->_joint_max[i])
	ok[j] = 0;
    }
  }
  if (ok[0] == 0)
  {
    if (ok[1] == 0) {
      std::cout << "No solution..." << std::endl;
      nbsol = 0;
      return nbsol;
    }
    else if (ok[1] == 1) {
      for (int i=0;i<6;i++) cord[i] = q_[1][i];
      nbsol = 1;
    }
  }
  else
  {
    if (ok[1] == 0) {
      for (int i=0;i<6;i++) cord[i] = q_[0][i];
      nbsol = 1;
    }
    else
    {
      nbsol = 2;
      //vpTRACE("2 solutions\n");
      for (int j=0;j<2;j++)
      {
	d[j] = 0.0;
	for (int i=3;i<6;i++)
	  d[j] += (q_[j][i] - q[i]) * (q_[j][i] - q[i]);
      }
      if (nearest == true)
      {
	if (d[0] <= d[1])
	  for (int i=0;i<6;i++) cord[i] = q_[0][i];
	else
	  for (int i=0;i<6;i++) cord[i] = q_[1][i];
      }
      else
      {
	if (d[0] <= d[1])
	  for (int i=0;i<6;i++) cord[i] = q_[1][i];
	else
	  for (int i=0;i<6;i++) cord[i] = q_[0][i];
      }
    }
  }
  for(int i=0; i<6; i++)
    q[i] = cord[i] ;

  return nbsol;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the six joints.

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Articular position of the six joints: q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa getForwardKinematics(const vpColVector & q)
*/
vpHomogeneousMatrix 
vpAfma6::get_fMc (const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the end effector with respect to the base frame given the
  articular positions of all the six joints.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param fMe The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  end effector frame (fMe).

*/
void 
vpAfma6::get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe)
{
  double            q0 = q[0]; // meter
  double            q1 = q[1]; // meter
  double            q2 = q[2]; // meter

  /* Decouplage liaisons 2 et 3. */
  double            q5 = q[5] - this->_coupl_45 * q[4];

  double            c1 = cos(q[3]);
  double            s1 = sin(q[3]);
  double            c2 = cos(q[4]);
  double            s2 = sin(q[4]);
  double            c3 = cos(q5);
  double            s3 = sin(q5);

  // Compute the direct geometric model: fMe = transformation betwee
  // fix and end effector frame.
  fMe[0][0] = s1*s2*c3 + c1*s3;
  fMe[0][1] = -s1*s2*s3 + c1*c3;
  fMe[0][2] = -s1*c2;
  fMe[0][3] = q0 + this->_long_56*c1;

  fMe[1][0] = -c1*s2*c3 + s1*s3;
  fMe[1][1] = c1*s2*s3 + s1*c3;
  fMe[1][2] = c1*c2;
  fMe[1][3] = q1 + this->_long_56*s1;

  fMe[2][0] = c2*c3;
  fMe[2][1] = -c2*s3;
  fMe[2][2] = s2;
  fMe[2][3] = q2;

  fMe[3][0] = 0;
  fMe[3][1] = 0;
  fMe[3][2] = 0;
  fMe[3][3] = 1;

  //  vpCTRACE << "Effector position fMe: " << std::endl << fMe;

  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the articular positions of all
  the six joints.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param fMc The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  camera frame (fMc).

*/
void 
vpAfma6::get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc)
{
  
  // Compute the direct geometric model: fMe = transformation betwee
  // fix and end effector frame.
  vpHomogeneousMatrix fMe;

  get_fMe(q, fMe);

  fMc = fMe * this->_eMc;

  return;
}

/*!  

  Get the geometric transformation between the camera frame and the
  end-effector frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

*/
void 
vpAfma6::get_cMe(vpHomogeneousMatrix &cMe)
{
  cMe = this->_eMc.inverse();
}

/*!

  Get the twist transformation from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.

  \param cVe : Twist transformation.

*/
void
vpAfma6::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;

  return;
}

/*!

  Get the robot jacobian expressed in the end-effector frame.

  \param q : Articular joint position of the robot. q[0], q[1], q[2]
  correspond to the first 3 translations expressed in meter, while
  q[3], q[4] and q[5] correspond to the 3 succesives rotations expressed in
  radians.

  \param eJe : Robot jacobian expressed in the end-effector frame.

*/
void
vpAfma6::get_eJe(const vpColVector &q, vpMatrix &eJe)
{

  eJe.resize(6,6) ;

  double s3,c3,s4,c4,s5,c5 ;

  s3=sin(q[3]); c3=cos(q[3]);
  s4=sin(q[4]); c4=cos(q[4]);
  s5=sin(q[5]); c5=cos(q[5]);

  eJe = 0;
  eJe[0][0] = s3*s4*c5+c3*s5;
  eJe[0][1] = -c3*s4*c5+s3*s5;
  eJe[0][2] = c4*c5;
  eJe[0][3] = - this->_long_56*s4*c5;

  eJe[1][0] = -s3*s4*s5+c3*c5;
  eJe[1][1] = c3*s4*s5+s3*c5;
  eJe[1][2] = -c4*s5;
  eJe[1][3] = this->_long_56*s4*s5;

  eJe[2][0] = -s3*c4;
  eJe[2][1] = c3*c4;
  eJe[2][2] = s4;
  eJe[2][3] = this->_long_56*c4;

  eJe[3][3] = c4*c5;
  eJe[3][4] = s5;

  eJe[4][3] = -c4*s5;
  eJe[4][4] = c5;

  eJe[5][3] = s4;
  eJe[5][5] = 1;

  return;
}


/*!

  Get the robot jacobian expressed in the robot reference frame also
  called fix frame.

  \f[
  {^f}J_e = \left(\begin{array}{cccccc}
  1  &   0  &   0  & -Ls4 &   0  &   0   \\
  0  &   1  &   0  &  Lc4 &   0  &   0   \\
  0  &   0  &   1  &   0  &   0  &   0   \\ 
  0  &   0  &   0  &   0  &   c4 & -s4c5 \\
  0  &   0  &   0  &   0  &   s4 &  c4c5 \\
  0  &   0  &   0  &   1  &   0  &  s5   \\
  \end{array}
  \right)
  \f]
*/

void
vpAfma6::get_fJe(const vpColVector &q, vpMatrix &fJe)
{

  fJe.resize(6,6) ;

  // block superieur gauche
  fJe[0][0] = fJe[1][1] = fJe[2][2] = 1 ;

  double s4 = sin(q[4]) ;
  double c4 = cos(q[4]) ;


  // block superieur droit
  fJe[0][3] = - this->_long_56*s4 ;
  fJe[1][3] =   this->_long_56*c4 ;


  double s5 = sin(q[5]) ;
  double c5 = cos(q[5]) ;
  // block inferieur droit
  fJe[3][4] = c4 ;     fJe[3][5] = -s4*c5 ;
  fJe[4][4] = s4 ;     fJe[4][5] = c4*c5 ;
  fJe[5][3] = 1 ;      fJe[5][5] = s5 ;

  return;
}


/*! 
  Get min joint values.

  \return Minimal joint values for the 6 dof
  X,Y,Z,A,B,C. Translation X,Y,Z are expressed in meters. Rotation
  A,B,C in radians.

*/
vpColVector 
vpAfma6::getJointMin()
{
  vpColVector qmin(6);
  for (int i=0; i < 6; i ++)
    qmin[i] = this->_joint_min[i];
  return qmin;
}

/*! 
  Get max joint values.

  \return Maximal joint values for the 6 dof
  X,Y,Z,A,B,C. Translation X,Y,Z are expressed in meters. Rotation
  A,B,C in radians.

*/
vpColVector 
vpAfma6::getJointMax() 
{
  vpColVector qmax(6);
  for (int i=0; i < 6; i ++)
    qmax[i] = this->_joint_max[i];
  return qmax;
}

/*! 

  Return the coupling factor between join 4 and 5.

  \return Coupling factor between join 4 and 5.
*/
double 
vpAfma6::getCoupl45()
{
  return _coupl_45;
}

/*! 

  Return the distance between join 5 and 6.

  \return Distance between join 5 and 6.
*/
double 
vpAfma6::getLong56()
{
  return _long_56;
}

void 
vpAfma6::parseConfigFile (const char * filename)
{
  int               dim;
  int               code;
  char              Ligne[100];
  char              namoption[10];
  float             val;
  FILE *            fdtask;
  int               numLn = 0;
  double rot_eMc[3]; // rotation
  double trans_eMc[3]; // translation
  bool get_rot_eMc = false;
  bool get_trans_eMc = false;

  //vpTRACE("Read the config file for constant parameters %s.", filename);
  if ((fdtask = fopen(filename, "r" )) == NULL)
  {
    vpERROR_TRACE ("Impossible to read the config file %s.",
		 filename);
    fclose(fdtask);
    throw vpRobotException (vpRobotException::readingParametersError,
			    "Impossible to read the config file.");
  }

  while (fgets(Ligne,100,fdtask) != NULL) {
    numLn ++;
    if ('#' == Ligne[0]) { continue; }
    sscanf(Ligne, "%s %f", namoption, &val);
    dim = strlen(namoption);

    for (code = 0;
	 NULL != opt_Afma6[code];
	 ++ code)
    {
      if (strncmp(opt_Afma6[code], namoption, dim) == 0)
      {
	break;
      }
    }

    switch(code)
    {
    case 0:
      sscanf(Ligne, "%s %lf %lf %lf %lf %lf %lf",
	     namoption,
	     &this->_joint_max[0], &this->_joint_max[1],
	     &this->_joint_max[2], &this->_joint_max[3],
	     &this->_joint_max[4], &this->_joint_max[5]);
      break;

    case 1:
      sscanf(Ligne, "%s %lf %lf %lf %lf %lf %lf", namoption,
	     &this->_joint_min[0], &this->_joint_min[1],
	     &this->_joint_min[2], &this->_joint_min[3],
	     &this->_joint_min[4], &this->_joint_min[5]);
      break;

    case 2:
      sscanf(Ligne, "%s %lf", namoption, &this->_long_56);
      break;

    case 3:
      sscanf(Ligne, "%s %lf", namoption, &this->_coupl_45);
      break;

    case 4:
      break; // Nothing to do: camera name

    case 5:
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &rot_eMc[0], 
	     &rot_eMc[1],
	     &rot_eMc[2]);

      // Convert rotation from degrees to radians
      rot_eMc[0] *= M_PI / 180.0;
      rot_eMc[1] *= M_PI / 180.0;
      rot_eMc[2] *= M_PI / 180.0;
      get_rot_eMc = true;
      break;

    case 6:
      sscanf(Ligne, "%s %lf %lf %lf", namoption,
	     &trans_eMc[0], 
	     &trans_eMc[1],
	     &trans_eMc[2]);
      get_trans_eMc = true;
      break;

    default:
      vpERROR_TRACE ("Bad configuration file %s  "
		     "ligne #%d.", filename, numLn);
    } /* SWITCH */
  } /* WHILE */

  fclose (fdtask);

  // Compute the eMc matrix from the translations and rotations
  if (get_rot_eMc && get_trans_eMc) {
    for (int i=0; i < 3; i ++) {
      _erc[i] = rot_eMc[i];
      _etc[i] = trans_eMc[i]; 
    }

    vpRotationMatrix eRc(_erc);
    this->_eMc.buildFrom(_etc, eRc);
  }

  return;
}

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  \warning This method needs XML library to parse the file containing
  the camera parameters. If XML is detected by ViSP, VISP_HAVE_XML2
  macro is defined in include/visp/vpConfig.h file.

  Camera parameters are read from
  /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml

  \param cam : In output, camera parameters to fill.
  \param image_width : Image width used to compute camera calibration.
  \param image_height : Image height used to compute camera calibration.

  The code below shows how to get the camera parameters of the camera
  attached to the robot.

  \code
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;

  a6RobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  // Camera parameters are read from
  // /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml
  robot.getCameraParameters (cam, I.getWidth(), I.getHeight());
  \endcode
*/

#ifndef VISP_HAVE_XML2
void 
vpAfma6::getCameraParameters (vpCameraParameters &,
			      const unsigned int &,
			      const unsigned int &)
{
  vpTRACE("Not implemented, since XML library is not installed !");
  return;
}
#else
void 
vpAfma6::getCameraParameters (vpCameraParameters &cam,
			      const unsigned int &image_width,
			      const unsigned int &image_height)
{
  vpXmlParserCamera parser;
  switch (getCameraRobotType())
  {
  case vpAfma6::CAMERA_DRAGONFLY2_8MM:
    {
      parser.parse(cam,
		   vpAfma6::CONST_CAMERA_AFMA6_FILENAME,
		   vpAfma6::CONST_LABEL_DRAGONFLY2,
		   projModel,  
		   image_width, image_height);
      break;
    }
  default:
    {
      vpERROR_TRACE ("Cette erreur ne peut pas arriver.");
      vpERROR_TRACE ("Si elle survient malgre tout, c'est sans doute "
       "que les specs de la classe ont ete modifiee, "
       "et que le code n'a pas ete mis a jour "
       "correctement.");
      vpERROR_TRACE ("Verifiez les valeurs possibles du type "
       "vpAfma6::vpAfma6CameraRobotType, et controlez que "
       "tous les cas ont ete pris en compte dans la "
       "fonction init(camera).");
      break;
    }
  }
  return;
}
#endif

/*!
  Get the current intrinsic camera parameters obtained by calibration.

  Camera parameters are read from
  /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml

  \param cam : In output, camera parameters to fill.
  \param I : A B&W image send by the current camera in use.

  \code
  vpImage<unsigned char> I;
  vp1394TwoGrabber g;

  // Acquire an image to update image structure
  g.acquire(I) ;

  a6RobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  robot.getCameraParameters (cam, I);
  \endcode

*/
void 
vpAfma6::getCameraParameters (vpCameraParameters &cam, 
			      const vpImage<unsigned char> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}
/*!
  \brief Get the current intrinsic camera parameters obtained by calibration.

  Camera parameters are read from
  /udd/fspindle/robot/Afma6/current/include/const_camera_Afma6.xml

  \param cam : In output, camera parameters to fill.
  \param I : A color image send by the current camera in use.

  \code
  vpImage<vpRGBa> I;
  vp1394TwoGrabber g;

  // Acquire a color image to update image structure
  g.acquire(I) ;

  a6RobotAfma6 robot;
  vpCameraParameters cam ;
  // Get the intrinsic camera parameters depending on the image size
  robot.getCameraParameters (cam, I);
  \endcode
*/

void 
vpAfma6::getCameraParameters (vpCameraParameters &cam, 
			      const vpImage<vpRGBa> &I)
{
  getCameraParameters(cam,I.getWidth(),I.getHeight());
}



std::ostream & operator << (std::ostream & os,
			    const vpAfma6 & afma6)
{
  vpRotationMatrix eRc;
  afma6._eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  os
    << "Joint Max:" << std::endl 
    << "\t" << afma6._joint_max[0]
    << "\t" << afma6._joint_max[1]
    << "\t" << afma6._joint_max[2]
    << "\t" << afma6._joint_max[3]
    << "\t" << afma6._joint_max[4]
    << "\t" << afma6._joint_max[5]
    << "\t" << std::endl 

    << "Joint Min: " << std::endl 
    << "\t" << afma6._joint_min[0]
    << "\t" << afma6._joint_min[1]
    << "\t" << afma6._joint_min[2]
    << "\t" << afma6._joint_min[3]
    << "\t" << afma6._joint_min[4]
    << "\t" << afma6._joint_min[5]
    << "\t" << std::endl 

    << "Long 5-6: " << std::endl 
    << "\t" << afma6._long_56
    << "\t" << std::endl 

    << "Coupling 4-5:" << std::endl 
    << "\t" << afma6._coupl_45
    << "\t" << std::endl 

    << "eMc: "<< std::endl 
    << "\tTranslation (m): " 
    << afma6._eMc[0][3] << " " 
    << afma6._eMc[1][3] << " " 
    << afma6._eMc[2][3]
    << "\t" << std::endl 
    << "\tRotation (rad) : " 
    << rxyz[0] << " " 
    << rxyz[1] << " " 
    << rxyz[2]
    << "\t" << std::endl
    << "\tRotation (deg) : " 
    << vpMath::deg(rxyz[0])  << " " 
    << vpMath::deg(rxyz[1])  << " " 
    << vpMath::deg(rxyz[2])
    << "\t" << std::endl;

  return os;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
