/****************************************************************************
 *
 * $Id: vpAfma6.h,v 1.15 2008-07-17 20:14:58 fspindle Exp $
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

#ifndef __vpAfma6_h
#define __vpAfma6_h

/*!

  \file vpAfma6.h

  Control of Irisa's gentry robot named Afma6.

*/

/*!

  \class vpAfma6 

  Control of Irisa's gentry robot named Afma6.

*/

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTwistMatrix.h>


class VISP_EXPORT vpAfma6
{
 public:
  //! File where constant parameters in relation with the robot are
  //! stored: joint max, min, couplig factor between 4 ant 5 joint,
  //! distance between 5 and 6 joint.
  static const char * const CONST_AFMA6_FILENAME;
  static const char * const CONST_EMC_DRAGONFLY2_WITHOUT_DISTORTION_FILENAME;
  static const char * const CONST_EMC_DRAGONFLY2_WITH_DISTORTION_FILENAME;
  static const char * const CONST_CAMERA_AFMA6_FILENAME;
  //! Name of the firewire Dragonfly2 camera
  static const char * const CONST_LABEL_DRAGONFLY2 ;

  //! List of cameras intalled on the end-effector
  typedef enum 
    {
      CAMERA_DRAGONFLY2_8MM,
    } vpAfma6CameraRobotType;

  //! Default camera
  static const vpAfma6CameraRobotType defaultCameraRobot;

 public:
  vpAfma6();

  void init (void);
  void init (const char * paramAfma6, const char * paramCamera);
  void init (vpAfma6::vpAfma6CameraRobotType camera,
	     vpCameraParameters::vpCameraParametersProjType projModel =
	     vpCameraParameters::perspectiveProjWithoutDistortion);

  vpHomogeneousMatrix getForwardKinematics(const vpColVector & q);
  int getInverseKinematics(const vpHomogeneousMatrix & fMc, 
			   vpColVector & q, const bool &nearest=true);
  vpHomogeneousMatrix get_fMc (const vpColVector & q);
  void get_fMe(const vpColVector & q, vpHomogeneousMatrix & fMe);
  void get_fMc(const vpColVector & q, vpHomogeneousMatrix & fMc);

  void get_cMe(vpHomogeneousMatrix &cMe) ;
  void get_cVe(vpTwistMatrix &cVe) ;
  void get_eJe(const vpColVector &q, vpMatrix &eJe)  ;
  void get_fJe(const vpColVector &q, vpMatrix &fJe)  ;

  void parseConfigFile (const char * filename);

  //! Get the current used camera
  vpAfma6CameraRobotType getCameraRobotType(){
    return camera_current;
  };
  //! Set the current used camera
  void setCameraRobotType(vpAfma6::vpAfma6CameraRobotType camera){
    camera_current = camera;
  };

  void getCameraParameters(vpCameraParameters &cam,
			   const unsigned int &image_width,
			   const unsigned int &image_height);
  void getCameraParameters(vpCameraParameters &cam,
			   const vpImage<unsigned char> &I);
  void getCameraParameters(vpCameraParameters &cam, const vpImage<vpRGBa> &I);

  friend std::ostream & operator << (std::ostream & os,
				     const vpAfma6 & afma6);

  vpColVector getJointMin();
  vpColVector getJointMax();
  double getCoupl45();
  double getLong56();

 public:
   
  static const int njoint; ///< Number of joint. 


 protected:
  double _coupl_45; // coupling between join 4 and 5
  double _long_56;  // distance between join 5 and 6
  double _joint_max[6]; // Maximal value of the joints
  double _joint_min[6]; // Minimal value of the joints
  // Minimal representation of _eMc
  vpTranslationVector _etc; // meters
  vpRxyzVector        _erc; // radian

  vpHomogeneousMatrix _eMc; // Camera extrinsic parameters: effector to camera

protected: 
  //! Current camera in use
  vpAfma6CameraRobotType camera_current;
  // Used projection model
  vpCameraParameters::vpCameraParametersProjType projModel;

};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

