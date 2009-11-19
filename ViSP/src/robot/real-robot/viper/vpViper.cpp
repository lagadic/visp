/****************************************************************************
 *
 * $Id: vpAfma6.cpp 2158 2009-05-07 07:24:51Z fspindle $
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
 * Interface for a  generic ADEPT Viper (either 650 or 850) robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \file vpViper.cpp

  Control of Irisa's gentry robot named Afma6.

*/


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>
#include <visp/vpTwistMatrix.h>
#include <visp/vpRobotException.h>
#include <visp/vpXmlParserCamera.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpViper.h>


const int vpViper::njoint = 6;

/*!

  Default constructor.

*/
vpViper::vpViper()
{
  // Default values are initialized

  // Denavit Hartenberg parameters
  a1 = 0.075;
  a2 = 0.365;
  a3 = 0.090;
  d1 = 0.335;
  d4 = 0.405;
  d6 = 0.080;
  c56 = -341.33 / 9102.22;

  // Software joint limits in radians
  joint_min.resize(njoint);
  joint_min[0] = vpMath::rad(-170);
  joint_min[1] = vpMath::rad(-190);
  joint_min[2] = vpMath::rad(-29);
  joint_min[3] = vpMath::rad(-190);
  joint_min[4] = vpMath::rad(-120);
  joint_min[5] = vpMath::rad(-360);
  joint_max.resize(njoint);
  joint_max[0] = vpMath::rad(170);
  joint_max[1] = vpMath::rad(45);
  joint_max[2] = vpMath::rad(256);
  joint_max[3] = vpMath::rad(190);
  joint_max[4] = vpMath::rad(120);
  joint_max[5] = vpMath::rad(360);

  // End effector to camera transformation
  eMc.setIdentity(); 
}




/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the six joint positions.

  This method is the same than get_fMc(const vpColVector & q).

  \param q : A six dimension vector corresponding to the robot joint
  positions expressed in radians.

  \return The homogeneous matrix \f$^f{\bf M}_c \f$ corresponding to
  the direct geometric model which expresses the transformation
  between the base frame and the camera frame.

  \sa get_fMc(const vpColVector & q)
  \sa getInverseKinematics()

*/
vpHomogeneousMatrix
vpViper::getForwardKinematics(const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  fMc = get_fMc(q);

  return fMc;
}

/*!

  Compute the inverse kinematics (inverse geometric model).

  \warning Not implemented yet.

  By inverse kinematics we mean here the six joint values given the
  position and the orientation of the camera frame relative to the
  base frame.

  \param fMc : Homogeneous matrix \f$^f{\bf M}_c \f$ describing the
  transformation from base frame to the camera frame.

  \param q : In input, a six dimension vector corresponding to the
  current joint positions expressed in radians. In output, the
  solution of the inverse kinematics, ie. the joint positions
  corresponding to \f$^f{\bf M}_c \f$.

  \param nearest : true to return the nearest solution to q. false to
  return the farest.

  \return The number of solutions (1 or 2) of the inverse geometric
  model. O, if no solution can be found.

  The code below shows how to compute the inverse geometric model:

  \code
  vpColVector q1(6), q2(6);
  vpHomogeneousMatrix fMc;

  vpRobotAfma6 robot;

  // Get the current joint position of the robot
  robot.getPosition(vpRobot::ARTICULAR_FRAME, q1);

  // Compute the pose of the camera in the reference frame using the
  // direct geometric model
  fMc = robot.getForwardKinematics(q1);
  // this is similar to  fMc = robot.get_fMc(q1);
  // or robot.get_fMc(q1, fMc);


  // Compute the inverse geometric model
  int nbsol; // number of solutions (0, 1 or 2) of the inverse geometric model
  // get the nearest solution to the current joint position
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

  \sa getForwardKinematics()

*/
int
vpViper::getInverseKinematics(const vpHomogeneousMatrix & fMc,
			      vpColVector & q, const bool &/*nearest*/)
{
  vpHomogeneousMatrix fMe;
//   double q_[2][6],d[2],t;
//   int ok[2];
//   double cord[6];

  int nbsol = 0;

  if (q.getRows() != njoint)
    q.resize(6);


//   for(int i=0;i<3;i++) {
//     fMe[i][3] = fMc[i][3];
//     for(int j=0;j<3;j++) {
//       fMe[i][j] = 0.0;
//       for (int k=0;k<3;k++) fMe[i][j] += fMc[i][k]*rpi[j][k];
//       fMe[i][3] -= fMe[i][j]*rpi[j][3];
//     }
//   }

//   std::cout << "\n\nfMc: " << fMc;
//   std::cout << "\n\neMc: " << _eMc;

  fMe = fMc * this->eMc.inverse();
  std::cout << "\n\nfMe: " << fMe;

//   if (fMe[2][2] >= .99999f)
//   {
//     vpTRACE("singularity\n");
//     q_[0][4] = q_[1][4] = M_PI/2.f;
//     t = atan2(fMe[0][0],fMe[0][1]);
//     q_[1][3] = q_[0][3] = q[3];
//     q_[1][5] = q_[0][5] = t - q_[0][3];

//     while  ((q_[1][5]+vpMath::rad(2)) >= this->_joint_max[5])
//       /*			-> a cause du couplage 4/5	*/
//     {
//       q_[1][5] -= vpMath::rad(10);
//       q_[1][3] += vpMath::rad(10);
//     }
//     while  (q_[1][5] <= this->_joint_min[5])
//     {
//       q_[1][5] += vpMath::rad(10);
//       q_[1][3] -= vpMath::rad(10);
//     }
//   }
//   else if (fMe[2][2] <= -.99999)
//   {
//     vpTRACE("singularity\n");
//     q_[0][4] = q_[1][4] = -M_PI/2;
//     t = atan2(fMe[1][1],fMe[1][0]);
//     q_[1][3] = q_[0][3] = q[3];
//     q_[1][5] = q_[0][5] = q_[0][3] - t;
//     while  ((q_[1][5]+vpMath::rad(2)) >= this->_joint_max[5])
//       /*			-> a cause du couplage 4/5	*/
//     {
//       q_[1][5] -= vpMath::rad(10);
//       q_[1][3] -= vpMath::rad(10);
//     }
//     while  (q_[1][5] <= this->_joint_min[5])
//     {
//       q_[1][5] += vpMath::rad(10);
//       q_[1][3] += vpMath::rad(10);
//     }
//   }
//   else
//   {
//     q_[0][3] = atan2(-fMe[0][2],fMe[1][2]);
//     if (q_[0][3] >= 0.0) q_[1][3] = q_[0][3] - M_PI;
//     else q_[1][3] = q_[0][3] + M_PI;

//     q_[0][4] = asin(fMe[2][2]);
//     if (q_[0][4] >= 0.0) q_[1][4] = M_PI - q_[0][4];
//     else q_[1][4] = -M_PI - q_[0][4];

//     q_[0][5] = atan2(-fMe[2][1],fMe[2][0]);
//     if (q_[0][5] >= 0.0) q_[1][5] = q_[0][5] - M_PI;
//     else q_[1][5] = q_[0][5] + M_PI;
//   }
//   q_[0][0] = fMe[0][3] ;
//   q_[1][0] = fMe[0][3] ;
//   q_[0][1] = fMe[1][3] ;
//   q_[1][1] = fMe[1][3] ;
//   q_[0][2] = q_[1][2] = fMe[2][3];

//   /* prise en compte du couplage axes 5/6	*/
//   q_[0][5] += this->_coupl_56*q_[0][4];
//   q_[1][5] += this->_coupl_56*q_[1][4];

//   for (int j=0;j<2;j++)
//   {
//     ok[j] = 1;
//     // test is position is reachable
//     for (int i=0;i<6;i++) {
//       if (q_[j][i] < this->_joint_min[i] || q_[j][i] > this->_joint_max[i])
// 	ok[j] = 0;
//     }
//   }
//   if (ok[0] == 0)
//   {
//     if (ok[1] == 0) {
//       std::cout << "No solution..." << std::endl;
//       nbsol = 0;
//       return nbsol;
//     }
//     else if (ok[1] == 1) {
//       for (int i=0;i<6;i++) cord[i] = q_[1][i];
//       nbsol = 1;
//     }
//   }
//   else
//   {
//     if (ok[1] == 0) {
//       for (int i=0;i<6;i++) cord[i] = q_[0][i];
//       nbsol = 1;
//     }
//     else
//     {
//       nbsol = 2;
//       //vpTRACE("2 solutions\n");
//       for (int j=0;j<2;j++)
//       {
// 	d[j] = 0.0;
// 	for (int i=3;i<6;i++)
// 	  d[j] += (q_[j][i] - q[i]) * (q_[j][i] - q[i]);
//       }
//       if (nearest == true)
//       {
// 	if (d[0] <= d[1])
// 	  for (int i=0;i<6;i++) cord[i] = q_[0][i];
// 	else
// 	  for (int i=0;i<6;i++) cord[i] = q_[1][i];
//       }
//       else
//       {
// 	if (d[0] <= d[1])
// 	  for (int i=0;i<6;i++) cord[i] = q_[1][i];
// 	else
// 	  for (int i=0;i<6;i++) cord[i] = q_[0][i];
//       }
//     }
//   }
//   for(int i=0; i<6; i++)
//     q[i] = cord[i] ;

  return nbsol;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the joint positions of all
  the six joints.

  \f[
  ^f{\bf M}_c = ^f{\bf M}_e \; ^e{\bf M}_c
  \f]

  This method is the same than getForwardKinematics(const vpColVector & q).

  \param q : Vector of six joint positions expressed in
  radians.

  \return The homogeneous matrix corresponding to the direct geometric
  model which expresses the transformation between the base frame and the
  camera frame (fMc).

  \sa getForwardKinematics(const vpColVector & q), get_fMe(), get_eMc()

*/
vpHomogeneousMatrix
vpViper::get_fMc (const vpColVector & q)
{
  vpHomogeneousMatrix fMc;
  get_fMc(q, fMc);

  return fMc;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix.

  By forward kinematics we mean here the position and the orientation
  of the camera relative to the base frame given the six joint positions.

  \f[
  ^f{\bf M}_c = ^f{\bf M}_e \; {^e}{\bf M}_c
  \f]

  \param q : Vector of six joint positions expressed in
  radians.

  \param fMc The homogeneous matrix \f$^f{\bf M}_c\f$corresponding to
  the direct geometric model which expresses the transformation
  between the fix frame and the camera frame.

  \sa get_fMe(), get_eMc()
*/
void
vpViper::get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc)
{

  // Compute the direct geometric model: fMe = transformation between
  // fix and end effector frame.
  vpHomogeneousMatrix fMe;

  get_fMe(q, fMe);

  fMc = fMe * this->eMc;

  return;
}

/*!

  Compute the forward kinematics (direct geometric model) as an
  homogeneous matrix \f${^f}{\bf M}_e\f$.

  By forward kinematics we mean here the position and the orientation
  of the end effector with respect to the base frame given the
  motor positions of all the six joints.

  \f[
  {^f}M_e = \left(\begin{array}{cccc}
  r_{11} & r_{12} & r_{13} & t_x  \\
  r_{21} & r_{22} & r_{23} & t_y  \\
  r_{31} & r_{32} & r_{33} & t_z  \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  r_{11} = c1(c23(c4c5c6-s4s6)-s23s5c6)-s1(s4c5c6+c4s6) \\
  r_{21} = -s1(c23(-c4c5c6+s4s6)+s23s5c6)+c1(s4c5c6+c4s6) \\
  r_{31} = s23(s4s6-c4c5c6)-c23s5c6 \\
  \\
  r_{12} = -c1(c23(c4c5s6+s4c6)-s23s5s6)+s1(s4c5s6-c4c6)\\
  r_{22} = -s1(c23(c4c5s6+s4c6)-s23s5s6)-c1(s4c5s6-c4c6)\\
  r_{32} = s23(c4c5s6+s4c6)+c23s5s6\\
  \\
  r_{13} = c1(c23c4s5+s23c5)-s1s4s5\\
  r_{23} = s1(c23c4s5+s23c5)+c1s4s5\\
  r_{33} = -s23c4s5+c23c5\\
  \\
  t_x = c1(c23(c4s5d6-a3)+s23(c5d6+d4)+a1+a2c2)-s1s4s5d6\\
  t_y = s1(c23(c4s5d6-a3)+s23(c5d6+d4)+a1+a2c2)+c1s4s5d6\\
  t_z = s23(a3-c4s5d6)+c23(c5d6+d4)-a2s2+d1\\
  \end{array}
  \f]

  \param q : A 6-dimension vector that contains the 6 joint positions
  expressed in radians.

  \param fMe The homogeneous matrix \f${^f}{\bf M}_e\f$ corresponding to the direct geometric
  model which expresses the transformation between the fix frame and the
  end effector frame.

  Note that this transformation can also be computed by considering the wrist
  frame \f${^f}{\bf M}_e = {^f}{\bf M}_w *{^w}{\bf M}_e\f$.

  \code
#include <visp/vpViper.h>

int main()
{
  vpViper robot;
  vpColVector q(6); // The measured six joint positions

  vpHomogeneousMatrix fMe; // Transformation from fix frame to end-effector
  robot.get_fMe(q, fMe); // Get the forward kinematics

  // The forward kinematics can also be computed by considering the wrist frame
  vpHomogeneousMatrix fMw; // Transformation from fix frame to wrist frame
  robot.get_fMw(q, fMw);
  vpHomogeneousMatrix wMe; // Transformation from wrist frame to end-effector
  robot.get_wMe(wMe); // Constant transformation

  // Compute the forward kinematics
  fMe = fMw * wMe;
}
  \endcode

*/
void
vpViper::get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe)
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
  double q6 = q[5];
  //  We turn off the coupling since the measured positions are joint position
  //  taking into account the coupling factor. The coupling factor is relevant
  //  if positions are motor position.
  // double q6 = q[5] + c56 * q[4];

//   std::cout << "q6 motor: " << q[5] << " rad " 
// 	    << vpMath::deg(q[5]) << " deg" << std::endl;
//   std::cout << "q6 joint: " << q6 << " rad " 
// 	    << vpMath::deg(q6) << " deg" << std::endl;

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  //double c3 = cos(q3);
  //double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c6 = cos(q6);
  double s6 = sin(q6);
  double c23 = cos(q2+q3);
  double s23 = sin(q2+q3);

  fMe[0][0] = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-s1*(s4*c5*c6+c4*s6);
  fMe[1][0] = -s1*(c23*(-c4*c5*c6+s4*s6)+s23*s5*c6)+c1*(s4*c5*c6+c4*s6);
  fMe[2][0] = s23*(s4*s6-c4*c5*c6)-c23*s5*c6;

  fMe[0][1] = -c1*(c23*(c4*c5*s6+s4*c6)-s23*s5*s6)+s1*(s4*c5*s6-c4*c6);
  fMe[1][1] = -s1*(c23*(c4*c5*s6+s4*c6)-s23*s5*s6)-c1*(s4*c5*s6-c4*c6);
  fMe[2][1] = s23*(c4*c5*s6+s4*c6)+c23*s5*s6;

  fMe[0][2] = c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
  fMe[1][2] = s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
  fMe[2][2] = -s23*c4*s5+c23*c5;

  fMe[0][3] = c1*(c23*(c4*s5*d6-a3)+s23*(c5*d6+d4)+a1+a2*c2)-s1*s4*s5*d6;
  fMe[1][3] = s1*(c23*(c4*s5*d6-a3)+s23*(c5*d6+d4)+a1+a2*c2)+c1*s4*s5*d6;
  fMe[2][3] = s23*(a3-c4*s5*d6)+c23*(c5*d6+d4)-a2*s2+d1;

  // std::cout << "Effector position fMe: " << std::endl << fMe;

  return;
}
/*!

  Compute the transformation between the fix frame and the wrist frame. The
  wrist frame is located on the intersection of the 3 last rotations.

  \param q : A 6-dimension vector that contains the 6 joint positions
  expressed in radians.

  \param fMw The homogeneous matrix corresponding to the transformation between
  the fix frame and the wrist frame (fMw).

  \f[
  {^f}M_w = \left(\begin{array}{cccc}
  r_{11} & r_{12} & r_{13} & t_x  \\
  r_{21} & r_{22} & r_{23} & t_y  \\
  r_{31} & r_{32} & r_{33} & t_z  \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  r_{11} = c1(c23(c4c5c6-s4s6)-s23s5c6)-s1(s4c5c6+c4s6) \\
  r_{21} = -s1(c23(-c4c5c6+s4s6)+s23s5c6)+c1(s4c5c6+c4s6) \\
  r_{31} = s23(s4s6-c4c5c6)-c23s5c6 \\
  \\
  r_{12} = -c1(c23(c4c5s6+s4c6)-s23s5s6)+s1(s4c5s6-c4c6)\\
  r_{22} = -s1(c23(c4c5s6+s4c6)-s23s5s6)-c1(s4c5s6-c4c6)\\
  r_{32} = s23(c4c5s6+s4c6)+c23s5s6\\
  \\
  r_{13} = c1(c23c4s5+s23c5)-s1s4s5\\
  r_{23} = s1(c23c4s5+s23c5)+c1s4s5\\
  r_{33} = -s23c4s5+c23c5\\
  \\
  t_x = c1(-c23a3+s23d4+a1+a2c2)\\
  t_y = s1(-c23a3+s23d4+a1+a2c2)\\
  t_z = s23a3+c23d4-a2s2+d1\\
  \end{array}
  \f]
  
*/
void
vpViper::get_fMw(const vpColVector & q, vpHomogeneousMatrix & fMw)
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];
  double q6 = q[5];
  //  We turn off the coupling since the measured positions are joint position
  //  taking into account the coupling factor. The coupling factor is relevant
  //  if positions are motor position.
  // double q6 = q[5] + c56 * q[4];

//   std::cout << "q6 motor: " << q[5] << " rad " 
// 	    << vpMath::deg(q[5]) << " deg" << std::endl;
//   std::cout << "q6 joint: " << q6 << " rad " 
// 	    << vpMath::deg(q6) << " deg" << std::endl;

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  //  double c3 = cos(q3);
  //double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c6 = cos(q6);
  double s6 = sin(q6);
  double c23 = cos(q2+q3);
  double s23 = sin(q2+q3);


  fMw[0][0] = c1*(c23*(c4*c5*c6-s4*s6)-s23*s5*c6)-s1*(s4*c5*c6+c4*s6);
  fMw[1][0] = -s1*(c23*(-c4*c5*c6+s4*s6)+s23*s5*c6)+c1*(s4*c5*c6+c4*s6);
  fMw[2][0] = s23*(s4*s6-c4*c5*c6)-c23*s5*c6;

  fMw[0][1] = -c1*(c23*(c4*c5*s6+s4*c6)-s23*s5*s6)+s1*(s4*c5*s6-c4*c6);
  fMw[1][1] = -s1*(c23*(c4*c5*s6+s4*c6)-s23*s5*s6)-c1*(s4*c5*s6-c4*c6);
  fMw[2][1] = s23*(c4*c5*s6+s4*c6)+c23*s5*s6;

  fMw[0][2] = c1*(c23*c4*s5+s23*c5)-s1*s4*s5;
  fMw[1][2] = s1*(c23*c4*s5+s23*c5)+c1*s4*s5;
  fMw[2][2] = -s23*c4*s5+c23*c5;

  fMw[0][3] = c1*(-c23*a3+s23*d4+a1+a2*c2);
  fMw[1][3] = s1*(-c23*a3+s23*d4+a1+a2*c2);
  fMw[2][3] = s23*a3+c23*d4-a2*s2+d1;

  //std::cout << "Wrist position fMw: " << std::endl << fMw;

  return;
}

/*!

  Return the transformation between the wrist frame and the end-effector. The
  wrist frame is located on the intersection of the 3 last rotations.


  \param wMe The homogeneous matrix corresponding to the transformation between
  the wrist frame and the end-effector frame (wMe).

*/
void
vpViper::get_wMe(vpHomogeneousMatrix & wMe)
{
  // Set the rotation as identity
  wMe.setIdentity();

  // Set the translation
  wMe[2][3] = d6;
}

/*!

  Get the geometric transformation between the end-effector frame and
  the camera frame. This transformation is constant and correspond to
  the extrinsic camera parameters estimated by calibration.

  \param eMc : Transformation between the the
  end-effector frame and the camera frame.

  \sa get_cMe()
*/
void
vpViper::get_eMc(vpHomogeneousMatrix &eMc)
{
  eMc = this->eMc;
}

/*!

  Get the geometric transformation between the camera frame and the
  end-effector frame. This transformation is constant and correspond
  to the extrinsic camera parameters estimated by calibration.

  \param cMe : Transformation between the camera frame and the
  end-effector frame.

  \sa get_eMc()
*/
void
vpViper::get_cMe(vpHomogeneousMatrix &cMe)
{
  cMe = this->eMc.inverse();
}

/*!

  Get the twist transformation \f$^c{\bf V}_e\f$ from camera frame to end-effector
  frame.  This transformation allows to compute a velocity expressed
  in the end-effector frame into the camera frame.
  \f[
  ^c{\bf V}_e = \left(\begin{array}{cc}
  ^c{\bf R}_e & [^c{\bf t}_e]_\times ^c{\bf R}_e\\
  {\bf 0}_{3\times 3} & ^c{\bf R}_e
  \end{array}
  \right)
  \f]
  \param cVe : Twist transformation \f$^c{\bf V}_e\f$.

*/
void
vpViper::get_cVe(vpTwistMatrix &cVe)
{
  vpHomogeneousMatrix cMe ;
  get_cMe(cMe) ;

  cVe.buildFrom(cMe) ;

  return;
}

/*!

  Get the robot jacobian \f${^e}{\bf J}_e\f$ which gives the velocity
  of the origin of the end-effector frame expressed in end-effector frame.

  \f[
  {^e}{\bf J}_e = \left[\begin{array}{cc}
  {^e}{\bf R}_f &  {[{^e}{\bf t}_w}]_\times \; {^e}{\bf R}_f \\
  0_{3\times3} & {^e}{\bf R}_f
  \end{array}
  \right] \;
  {^f}{\bf J}_w
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param eJe : Robot jacobian \f${^e}{\bf J}_e\f$ that express the
  velocity of the end-effector in the robot end-effector frame.

  \sa get_fJw()
*/
void
vpViper::get_eJe(const vpColVector &q, vpMatrix &eJe)
{
#if 1
  vpMatrix V(6,6);
  V = 0;
  // Compute the first and last block of V
  vpHomogeneousMatrix fMe;
  get_fMe(q, fMe);
  vpRotationMatrix fRe;
  fMe.extract(fRe);
  vpRotationMatrix eRf;
  eRf = fRe.inverse();
  for (int i=0; i<3; i++ ) {
    for (int j=0; j<3; j++ ) {
      V[i][j] = V[i+3][j+3] = eRf[i][j];
    }
  }
  // Compute the second block of V
  vpHomogeneousMatrix wMe;
  get_wMe(wMe);
  vpHomogeneousMatrix eMw;
  eMw = wMe.inverse();
  vpTranslationVector etw;
  eMw.extract(etw);
  vpMatrix block2 = etw.skew()*eRf;
  for (int i=0; i<3; i++ ) {
    for (int j=0; j<3; j++ ) {
      V[i][j+3] = block2[i][j];
    }
  }
  // V.print(std::cout, 5, "V=eVw*wVf calcule ViSP:");
  // Compute eJe
  vpMatrix fJw;
  get_fJw(q, fJw);  
  eJe = V * fJw;


#else
  // We need here to compute eVw * wVf * fJe
  eJe.resize(6,6) ;


  vpMatrix fJw;
  // We recall that for simplification, fJw is computed with d6 set to zero in
  // the modelization, to come in the wrist frame
  get_fJw(q, fJw);

  // fix frame to wrist transformation
  vpHomogeneousMatrix fMw;
  get_fMw(q, fMw);
  vpHomogeneousMatrix wMf;
  wMf = fMw.inverse();

  // the transformation between wrist frame and end effector
  vpHomogeneousMatrix wMe;
  get_wMe(wMe);

  vpTranslationVector t;
  t=0;
  vpRotationMatrix wRf;
  wMf.extract(wRf);
  
  vpTwistMatrix wVf(t, wRf);

  vpHomogeneousMatrix eMw;
  eMw = wMe.inverse();
  vpTwistMatrix eVw(eMw);

//   {
//     vpTwistMatrix fVw(t, wRf.t()); 
//     vpTwistMatrix wVe(eMw.inverse());
//     fVw.print(std::cout, 5, "fVw vpViper:");
//     wVe.print(std::cout, 5, "wVe vpViper:");

//   }
   wVf.print(std::cout, 5, "wVf:");
   eVw.print(std::cout, 5, "eVw:");

  eJe = eVw*wVf*fJw;
#endif
  return;
}


/*!

  Get the robot jacobian \f${^f}{\bf J}_w\f$ which express the
  velocity of the origin of the wrist frame in the robot reference
  frame also called fix frame.

  \f[
  {^f}J_w = \left(\begin{array}{cccccc}
  J_{11} & J_{12} & J_{13} &   0     &   0    &   0    \\
  J_{21} & J_{22} & J_{23} &   0     &   0    &   0    \\
  0      & J_{32} & J_{33} &   0     &   0    &   0    \\
  0      &   -s1  &   -s1  &  c1s23 & J_{45} & J_{46} \\
  0      &    c1  &    c1  &  s1s23 & J_{55} & J_{56} \\
  1      &    0   &    0   &  c23    & s23s4 & J_{56} \\
  \end{array}
  \right)
  \f]

  with
  \f[
  \begin{array}{l}
  J_{11} = -s1(-c23a3+s23d4+a1+a2c2) \\
  J_{21} = c1(-c23a3+s23d4+a1+a2c2) \\
  J_{12} = c1(s23a3+c23d4-a2s2) \\
  J_{22} = s1(s23a3+c23d4-a2s2) \\
  J_{32} = c23a3-s23d4-a2c2 \\
  J_{13} = c1(a3(s2c3+c2s3)+(-s2s3+c2c3)d4)\\
  J_{23} = s1(a3(s2c3+c2s3)+(-s2s3+c2c3)d4)\\
  J_{33} = -a3(s2s3-c2c3)-d4(s2c3+c2s3)\\
  J_{45} = -c23c1s4-s1c4\\
  J_{55} = c1c4-c23s1s4\\
  J_{46} = (c1c23c4-s1s4)s5+c1s23c5\\
  J_{56} = (s1c23c4+c1s4)s5+s1s23c5\\
  J_{66} = -s23c4s5+c23c5\\
  \end{array}
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param fJw : Robot jacobian \f${^f}{\bf J}_w\f$ that express the
  velocity of the point \e w (origin of the wrist frame) in the robot
  reference frame.

  \sa get_fJe(), get_eJe()
*/

void
vpViper::get_fJw(const vpColVector &q, vpMatrix &fJw)
{
  double q1 = q[0];
  double q2 = q[1];
  double q3 = q[2];
  double q4 = q[3];
  double q5 = q[4];

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);
  double c3 = cos(q3);
  double s3 = sin(q3);
  double c4 = cos(q4);
  double s4 = sin(q4);
  double c5 = cos(q5);
  double s5 = sin(q5);
  double c23 = cos(q2+q3);
  double s23 = sin(q2+q3);

  vpColVector J1(6);
  vpColVector J2(6);
  vpColVector J3(6);
  vpColVector J4(6);
  vpColVector J5(6);
  vpColVector J6(6);

  // Jacobian when d6 is set to zero
  J1[0] = -s1*(-c23*a3+s23*d4+a1+a2*c2);
  J1[1] =  c1*(-c23*a3+s23*d4+a1+a2*c2);
  J1[2] = 0;
  J1[3] = 0;
  J1[4] = 0;
  J1[5] = 1;

  J2[0] = c1*(s23*a3+c23*d4-a2*s2);
  J2[1] = s1*(s23*a3+c23*d4-a2*s2);
  J2[2] = c23*a3-s23*d4-a2*c2;
  J2[3] = -s1;
  J2[4] = c1;
  J2[5] = 0;

  J3[0] = c1*(a3*(s2*c3+c2*s3)+(-s2*s3+c2*c3)*d4);
  J3[1] = s1*(a3*(s2*c3+c2*s3)+(-s2*s3+c2*c3)*d4);
  J3[2] = -a3*(s2*s3-c2*c3)-d4*(s2*c3+c2*s3);
  J3[3] = -s1;
  J3[4] = c1;
  J3[5] = 0;

  J4[0] = 0;
  J4[1] = 0;
  J4[2] = 0;
  J4[3] = c1*s23;
  J4[4] = s1*s23;
  J4[5] = c23;

  J5[0] = 0;
  J5[1] = 0;
  J5[2] = 0;
  J5[3] = -c23*c1*s4-s1*c4;
  J5[4] = c1*c4-c23*s1*s4;
  J5[5] = s23*s4;

  J6[0] = 0;
  J6[1] = 0;
  J6[2] = 0;
  J6[3] = (c1*c23*c4-s1*s4)*s5+c1*s23*c5;
  J6[4] = (s1*c23*c4+c1*s4)*s5+s1*s23*c5;
  J6[5] = -s23*c4*s5+c23*c5;

  fJw.resize(6,6) ;
  for (int i=0;i<6;i++) {
    fJw[i][0] = J1[i];
    fJw[i][1] = J2[i];
    fJw[i][2] = J3[i];
    fJw[i][3] = J4[i];
    fJw[i][4] = J5[i];
    fJw[i][5] = J6[i];
  }
  return;
}
/*!

  Get the robot jacobian \f${^f}{\bf J}_e\f$ which gives the velocity
  of the origin of the end-effector frame expressed in the robot
  reference frame also called fix frame.

  \f[
  {^f}{\bf J}_e = \left[\begin{array}{cc}
  I_{3\times3} & [{^f}{\bf R}_e \; {^e}{\bf t}_w]_\times \\
  0_{3\times3} & I_{3\times3} 
  \end{array}
  \right]
  {^f}{\bf J}_w
  \f]

  \param q : A six-dimension vector that contains the joint positions
  of the robot expressed in radians.

  \param fJe : Robot jacobian \f${^f}{\bf J}_e\f$ that express the
  velocity of the end-effector in the robot reference frame.

  \sa get_fJw
*/
void
vpViper::get_fJe(const vpColVector &q, vpMatrix &fJe)
{
  vpMatrix V(6,6);
  V = 0;
  // Set the first and last block to identity
  for (int i=0; i<6; i++ )
    V[i][i] = 1;
  
  // Compute the second block of V
  vpHomogeneousMatrix fMe;
  get_fMe(q, fMe);
  vpRotationMatrix fRe;
  fMe.extract(fRe);
  vpHomogeneousMatrix wMe;
  get_wMe(wMe);
  vpHomogeneousMatrix eMw;
  eMw = wMe.inverse();
  vpTranslationVector etw;
  eMw.extract(etw);
  vpMatrix block2 = (fRe*etw).skew();
  // Set the second block
  for (int i=0; i<3; i++ )
    for (int j=0; j<3; j++ )
      V[i][j+3] = block2[i][j];

  // Compute fJe
  vpMatrix fJw;
  get_fJw(q, fJw);  
  fJe = V * fJw;

  return;
}


/*!
  Get minimal joint values.

  \return A 6-dimension vector that contains the minimal joint values
  for the 6 dof. All the values are expressed in radians.

*/
vpColVector
vpViper::getJointMin()
{
  return joint_min;
}

/*!
  Get maximal joint values.

  \return A 6-dimension vector that contains the maximal joint values
  for the 6 dof. All the values are expressed in radians.

*/
vpColVector
vpViper::getJointMax()
{
  return joint_max;
}

/*!

  Return the coupling factor between join 5 and joint 6.

  This factor should be only useful when motor positions are
  considered.  Since the positions returned by the robot are joint
  positions which takes into account the coupling factor, it has not to
  be considered in the modelization of the robot.

*/
double
vpViper::getCoupl56()
{
  return c56;
}



/*!

  Print on the output stream \e os the robot parameters (joint
  min/max, coupling factor between axis 5 and 6, hand-to-eye constant
  homogeneous matrix \f$^e{\bf M}_c \f$.

  \param os : Output stream.
  \param viper : Robot parameters.
*/
std::ostream & operator << (std::ostream & os, const vpViper & viper)
{
  vpRotationMatrix eRc;
  viper.eMc.extract(eRc);
  vpRxyzVector rxyz(eRc);

  // Convert joint limits in degrees
  vpColVector jmax = viper.joint_max;
  vpColVector jmin = viper.joint_min;
  jmax.rad2deg();
  jmin.rad2deg();

  os
    << "Joint Max (deg):" << std::endl
    << "\t" << jmax.t() << std::endl

    << "Joint Min (deg): " << std::endl
    << "\t" << jmin.t() << std::endl

    << "Coupling 5-6:" << std::endl
    << "\t" << viper.c56 << std::endl

    << "eMc: "<< std::endl
    << "\tTranslation (m): "
    << viper.eMc[0][3] << " "
    << viper.eMc[1][3] << " "
    << viper.eMc[2][3]
    << "\t" << std::endl
    << "\tRotation Rxyz (rad) : "
    << rxyz[0] << " "
    << rxyz[1] << " "
    << rxyz[2]
    << "\t" << std::endl
    << "\tRotation Rxyz (deg) : "
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
