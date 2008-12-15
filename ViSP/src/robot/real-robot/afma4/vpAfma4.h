/****************************************************************************
 *
 * $Id: vpAfma4.h,v 1.7 2008-12-15 17:19:22 fspindle Exp $
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
 * Interface for the Irisa's Afma4 robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef __vpAfma4_h
#define __vpAfma4_h

/*!

  \file vpAfma4.h

  Modelisation of Irisa's cylindrical robot named Afma4.

*/

/*!

  \class vpAfma4

  \ingroup Afma4

  \brief Modelisation of Irisa's cylindrical robot named Afma4.

*/

#include <visp/vpConfig.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTwistMatrix.h>


class VISP_EXPORT vpAfma4
{
 public:
  vpAfma4();

  void init (void);

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

  friend VISP_EXPORT std::ostream & operator << (std::ostream & os,
						 const vpAfma4 & afma4);

  vpColVector getJointMin();
  vpColVector getJointMax();
  double getLong23();
  double getlong23();

 public:

  static const int njoint; ///< Number of joint.


 protected:
  double _long_23; // distance between join 2 and 3
  double _Long_23; // distance between join 2 and 3
  double _joint_max[4]; // Maximal value of the joints
  double _joint_min[4]; // Minimal value of the joints

  // Minimal representation of _eMc
  vpTranslationVector _etc; // meters
  vpRxyzVector        _erc; // radian

  vpHomogeneousMatrix _eMc; // Camera extrinsic parameters: effector to camera
};

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

#endif

