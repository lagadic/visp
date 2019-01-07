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
 * 3D translation visual feature.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

#include <visp3/core/vpMath.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

/*!
  \file vpFeatureTranslation.cpp
  \brief class that defines 3D translation visual feature
*/
/*

attributes and members directly related to the vpBasicFeature needs
other functionalities are useful but not mandatory

*/

/*!

  Initialise the memory space requested for 3D translation visual
  feature.
*/
void vpFeatureTranslation::init()
{
  // feature dimension
  dim_s = 3;
  nbParameters = 1;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;
}

/*!
  Default constructor that builds a visual feature and initialize it to zero.
  The type of the translation feature will be vpFeatureTranslation::cdMc
  by default. Use the function setFeatureTranslationType() to set the
  desired type of feature.
*/
vpFeatureTranslation::vpFeatureTranslation() : f2Mf1(), translation(vpFeatureTranslation::cdMc) { init(); }

/*!
  Default constructor that builds a visual feature and initialize it to zero
  specifying the type.

  \param r : Type of considered 3D translation feature.

*/
vpFeatureTranslation::vpFeatureTranslation(vpFeatureTranslationRepresentationType r) : f2Mf1(), translation(r)
{
  init();
}

/*!

  Constructor that builds a 3D visual feature from an homogeneous
  matrix \f$ ^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$ that represent the 3D
  transformation between two frames \f${\cal{F}}_1\f$ and \f${\cal{F}}_2\f$.

  \param f2Mf1_ [in] : 3D displacement that the camera has to achieve to
  move from the frame \f${\cal{F}}_2\f$ to the frame \f${\cal{F}}_1\f$ (\f$
  ^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$). \param r : type of feature. It can be
  vpFeature::cdMc or vpFeature::cMo.

*/
vpFeatureTranslation::vpFeatureTranslation(vpHomogeneousMatrix &f2Mf1_, vpFeatureTranslationRepresentationType r)
  : f2Mf1(), translation(r)
{
  init();

  buildFrom(f2Mf1_);
}

/*!
  Build a 3D translation visual feature from an homogeneous
  matrix \f$ ^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$ that represent the 3D
  transformation between two frames \f${\cal{F}}_1\f$ and \f${\cal{F}}_2\f$.

  \param f2Mf1_ [in] : 3D displacement that the camera has to achieve to
  move from the frame \f${\cal{F}}_2\f$ to the frame \f${\cal{F}}_1\f$ (\f$
  ^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$).
*/
void vpFeatureTranslation::buildFrom(const vpHomogeneousMatrix &f2Mf1_)
{
  this->f2Mf1 = f2Mf1_;
  s[0] = f2Mf1[0][3];
  s[1] = f2Mf1[1][3];
  s[2] = f2Mf1[2][3];

  flags[0] = true;
}

/*!

  Set the type of translation feature.

  \param r : type of translation feature. It can be
  vpFeatureTranslation::cdMc, vpFeatureTranslation::cMcd or
  vpFeatureTranslation::cMo. \sa getFeatureTranslationType()

*/
void vpFeatureTranslation::setFeatureTranslationType(const vpFeatureTranslationRepresentationType r)
{
  translation = r;
}

/*!

  Initialise the \f$t_x \f$ subset value of the 3D
  visual feature \f$ s\f$.

  \param t_x : \f$t_x \f$ subset value to initialize.
  \sa get_Tx()

*/
void vpFeatureTranslation::set_Tx(const double t_x) { s[0] = t_x; }
/*!

  Initialise the \f$t_y \f$ subset value of the 3D
  visual feature \f$ s\f$.

  \param t_y : \f$t_y \f$ subset value to initialize.
  \sa get_Ty()

*/
void vpFeatureTranslation::set_Ty(const double t_y) { s[1] = t_y; }
/*!

  Initialise the \f$t_z \f$ subset value of the 3D
  visual feature \f$ s\f$.

  \param t_z : \f$t_z \f$ subset value to initialize.
  \sa get_Tz()

*/
void vpFeatureTranslation::set_Tz(const double t_z) { s[2] = t_z; }

/*!

  Get the type of translation feature.

  \return Type of translation feature. It can be vpFeatureTranslation::cdMc,
  vpFeatureTranslation::cMcd or vpFeatureTranslation::cMo. \sa
  setFeatureTranslationType()

*/
vpFeatureTranslation::vpFeatureTranslationRepresentationType vpFeatureTranslation::getFeatureTranslationType() const
{
  return translation;
}

/*!
  Return the \f$t_x \f$ subset value of the visual feature
  \f$s\f$.

*/
double vpFeatureTranslation::get_Tx() const { return s[0]; }

/*!
  Return the \f$t_y \f$ subset value of the visual feature
  \f$s\f$.

*/
double vpFeatureTranslation::get_Ty() const { return s[1]; }

/*!
  Return the \f$t_z \f$ subset value of the visual feature
  \f$s\f$.

*/
double vpFeatureTranslation::get_Tz() const { return s[2]; }

/*!

  Compute and return the interaction matrix \f$ L \f$ from a subset
  \f$(t_x, t_y, t_z)\f$ of the possible translation features that
  represent the 3D transformation \f$ ^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$.

  As it exists three different features, the computation of the
  interaction matrix is diferent for each one.

  - With the feature type cdMc:

  \f[ L = [ ^{c^*}R_c \;\; 0_3] \f]

  where \f$^{c^*}R_c\f$ is the rotation the camera has to achieve to
  move from the desired camera frame to the current camera frame.

  - With the feature type cMcd:

  \f[ L = [ -I_3 \;\; [^{c}t_{c^*}]_\times] \f]

  where \f$^{c}R_{c^*}\f$ is the rotation the camera has to achieve to
  move from the current camera frame to the desired camera frame.

  - With the feature type cMo:

  \f[ L = [ -I_3 \;\; [^{c}t_o]_\times] \f]

  where \f$^{c}t_o \f$ is the position of
  the object frame relative to the current camera frame.

  \param select : Selection of a subset of the possible translation
  features.
  - To compute the interaction matrix for all the three translation
    subset features \f$(t_x,t_y,t_y)\f$ use vpBasicFeature::FEATURE_ALL. In
    that case the dimension of the interaction matrix is \f$ [3 \times
    6] \f$
  - To compute the interaction matrix for only one of the translation
    subset (\f$t_x, t_y, t_z\f$) use
    one of the corresponding function selectTx(), selectTy() or
    selectTz(). In that case the returned interaction matrix is \f$ [1
    \times 6] \f$ dimension.

  \return The interaction matrix computed from the translation
  features.

  The code below shows how to compute the interaction matrix
  associated to the visual feature \f$s = t_x \f$ using the cdMc feature type.

  \code
  vpHomogeneousMatrix cdMc;
  ...
  // Creation of the current feature s
  vpFeatureTranslation s(vpFeatureTranslation::cdMc);
  s.buildFrom(cdMc);

  vpMatrix L_x = s.interaction( vpFeatureTranslation::selectTx() );
  \endcode

  The code below shows how to compute the interaction matrix
  associated to the \f$s = (t_x, t_y) \f$
  subset visual feature:

  \code
  vpMatrix L_xy = s.interaction( vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy() );
  \endcode

  L_xy is here now a 2 by 6 matrix. The first line corresponds to
  the \f$ t_x \f$ visual feature while the second one to the \f$
  t_y \f$ visual feature.

  It is also possible to build the interaction matrix from all the
  translation components by:

  \code
  vpMatrix L_xyz = s.interaction( vpBasicFeature::FEATURE_ALL );
  \endcode

  In that case, L_xyz is a 3 by 6 interaction matrix where the last
  line corresponds to the \f$ t_z \f$ visual feature.

*/
vpMatrix vpFeatureTranslation::interaction(const unsigned int select)
{

  vpMatrix L;
  L.resize(0, 6);

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but f2Mf1 "
                  "was not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  if (translation == cdMc) {
    // This version is a simplification
    if (vpFeatureTranslation::selectTx() & select) {
      vpMatrix Lx(1, 6);

      for (int i = 0; i < 3; i++)
        Lx[0][i] = f2Mf1[0][i];
      Lx[0][3] = 0;
      Lx[0][4] = 0;
      Lx[0][5] = 0;

      L = vpMatrix::stack(L, Lx);
    }

    if (vpFeatureTranslation::selectTy() & select) {
      vpMatrix Ly(1, 6);

      for (int i = 0; i < 3; i++)
        Ly[0][i] = f2Mf1[1][i];
      Ly[0][3] = 0;
      Ly[0][4] = 0;
      Ly[0][5] = 0;

      L = vpMatrix::stack(L, Ly);
    }

    if (vpFeatureTranslation::selectTz() & select) {
      vpMatrix Lz(1, 6);

      for (int i = 0; i < 3; i++)
        Lz[0][i] = f2Mf1[2][i];
      Lz[0][3] = 0;
      Lz[0][4] = 0;
      Lz[0][5] = 0;

      L = vpMatrix::stack(L, Lz);
    }
  }
  if (translation == cMcd) {
    // This version is a simplification
    if (vpFeatureTranslation::selectTx() & select) {
      vpMatrix Lx(1, 6);
      Lx[0][0] = -1;
      Lx[0][1] = 0;
      Lx[0][2] = 0;
      Lx[0][3] = 0;
      Lx[0][4] = -s[2];
      Lx[0][5] = s[1];

      L = vpMatrix::stack(L, Lx);
    }

    if (vpFeatureTranslation::selectTy() & select) {
      vpMatrix Ly(1, 6);
      Ly[0][0] = 0;
      Ly[0][1] = -1;
      Ly[0][2] = 0;
      Ly[0][3] = s[2];
      Ly[0][4] = 0;
      Ly[0][5] = -s[0];

      L = vpMatrix::stack(L, Ly);
    }

    if (vpFeatureTranslation::selectTz() & select) {
      vpMatrix Lz(1, 6);
      Lz[0][0] = 0;
      Lz[0][1] = 0;
      Lz[0][2] = -1;
      Lz[0][3] = -s[1];
      Lz[0][4] = s[0];
      Lz[0][5] = 0;

      L = vpMatrix::stack(L, Lz);
    }
  }

  if (translation == cMo) {
    // This version is a simplification
    if (vpFeatureTranslation::selectTx() & select) {
      vpMatrix Lx(1, 6);
      Lx[0][0] = -1;
      Lx[0][1] = 0;
      Lx[0][2] = 0;
      Lx[0][3] = 0;
      Lx[0][4] = -s[2];
      Lx[0][5] = s[1];

      L = vpMatrix::stack(L, Lx);
    }

    if (vpFeatureTranslation::selectTy() & select) {
      vpMatrix Ly(1, 6);
      Ly[0][0] = 0;
      Ly[0][1] = -1;
      Ly[0][2] = 0;
      Ly[0][3] = s[2];
      Ly[0][4] = 0;
      Ly[0][5] = -s[0];

      L = vpMatrix::stack(L, Ly);
    }

    if (vpFeatureTranslation::selectTz() & select) {
      vpMatrix Lz(1, 6);
      Lz[0][0] = 0;
      Lz[0][1] = 0;
      Lz[0][2] = -1;
      Lz[0][3] = -s[1];
      Lz[0][4] = s[0];
      Lz[0][5] = 0;

      L = vpMatrix::stack(L, Lz);
    }
  }

  return L;
}

/*!
  Compute the error \f$ (s-s^*)\f$ between the current and the desired
  visual features from a subset of the possible features.

  - With the feature type cdMc:
  Since this visual feature \f$ s \f$ represent the 3D translation from the
  desired camera frame to the current one \f$^{c^*}t_{c} \f$, the desired
  visual feature \f$ s^* \f$ should be zero. Thus, the error is here
  equal to the current visual feature \f$ s \f$.

  - With the feature type cMo:
  In this case the desired feature is not necessary equal to zero. Thus, the
  error is here equal to \f$ s-s^* \f$.

  \param s_star : Desired visual feature.

  \param select : The error can be computed for a selection of a
  subset of the possible translation features.
  - To compute the error for all the three translation vector coordinates use
    vpBasicFeature::FEATURE_ALL. In that case the error vector is a 3
    dimension column vector.
  - To compute the error for only one of the translation vector coordinate
    feature \f$(t_x, t_y, t_z)\f$ use one of the
    corresponding function selectTx(), selectTy() or selectTz(). In
    that case the error vector is a 1 dimension column vector.

  \return The error \f$ (s-s^*)\f$ between the current and the desired
  visual feature.

  \exception vpFeatureException::badInitializationError : If the
  desired visual feature \f$ s^* \f$ is not equal to zero in the case of the
  feature type is cdMc or cMcd.

  The code below shows how to use this method to manipulate the \f$
  t_z \f$ subset in the case of the cdMc feature type. It can be used also
  with the cMo feature type. In that case just change
  vpFeatureTranslation::cdMc by vpFeatureTranslation::cMo during the
  declaration of the two vpFeatureTranslation features.

  \code
  // Creation of the current feature s
  vpFeatureTranslation s(vpFeatureTranslation::cdMc);
  s.set_TUz(0.3); // Initialization of the feature

  // Creation of the desired feature s*. By default this feature is
  // initialized to zero
  vpFeatureTranslation s_star(vpFeatureTranslation::cdMc);

  // Compute the interaction matrix for the t_z translation feature
  vpMatrix L_z = s.interaction( vpFeatureTranslation::selectTz() );

  // Compute the error vector (s-s*) for the t_z feature
  s.error(s_star, vpFeatureTranslation::selectTz());
  \endcode

  To manipulate the subset features \f$s=(t_y, t_z)\f$,
  the code becomes:
  \code
  // Compute the interaction matrix for the t_y, t_z features
  vpMatrix L_yz = s.interaction( vpFeatureTranslation::selectTy() | vpFeatureTranslation::selectTz() );

  // Compute the error vector e = (s-s*) for the t_y, t_z feature
  vpColVector e = s.error(s_star, vpFeatureTranslation::selectTy() | vpFeatureTranslation::selectTz());
  \endcode

*/
vpColVector vpFeatureTranslation::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  if (translation == cdMc || translation == cMcd) {
    if (s_star.get_s().sumSquare() > 1e-6) {
      vpERROR_TRACE("s* should be zero ! ");
      throw(vpFeatureException(vpFeatureException::badInitializationError, "s* should be zero !"));
    }
  }

  if (vpFeatureTranslation::selectTx() & select) {
    vpColVector ex(1);
    ex[0] = s[0] - s_star[0];
    e = vpColVector::stack(e, ex);
  }

  if (vpFeatureTranslation::selectTy() & select) {
    vpColVector ey(1);
    ey[0] = s[1] - s_star[1];
    e = vpColVector::stack(e, ey);
  }

  if (vpFeatureTranslation::selectTz() & select) {
    vpColVector ez(1);
    ez[0] = s[2] - s_star[2];
    e = vpColVector::stack(e, ez);
  }

  return e;
}

/*!
  Print to stdout the values of the current visual feature \f$ s \f$.

  \param select : Selection of a subset of the possible translation features.
  - To print all the three translation vector coordinates used as features use
  vpBasicFeature::FEATURE_ALL.
  - To print only one of the translation coordinate
  feature \f$(t_x, t_y, t_z)\f$ use one of the
  corresponding function selectTx(), selectTy() or selectTz().

  \code
  vpHomogeneousMatrix cdMc; // Homogeneous transformation between the desired
  camera frame and the current camera frame.

  // Creation of the current feature s
  vpFeatureTranslation s(vpFeatureTranslation::cdMc);
  s.buildFrom(cdMc);

  s.print(); // print all the 3 components of the translation feature
  s.print(vpBasicFeature::FEATURE_ALL); // same behavior then previous line
  s.print(vpFeatureTranslation::selectTz()); // print only the t_z component
  \endcode
*/
void vpFeatureTranslation::print(const unsigned int select) const
{
  std::cout << "Translation 3D: ";
  if (vpFeatureTranslation::selectTx() & select) {
    std::cout << s[0] << " ";
  }
  if (vpFeatureTranslation::selectTy() & select) {
    std::cout << s[1] << " ";
  }
  if (vpFeatureTranslation::selectTz() & select) {
    std::cout << s[2] << " ";
  }
  std::cout << std::endl;
}

/*!

  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  vpFeatureTranslation s(vpFeatureTranslation::cdMc); //or
  vpFeatureTranslation s(vpFeatureTranslation::cMo);
  s_star = s.duplicate();
  // s_star is now a vpFeatureTranslation
  \endcode

*/
vpFeatureTranslation *vpFeatureTranslation::duplicate() const
{
  vpFeatureTranslation *feature = NULL;
  if (translation == cdMc)
    feature = new vpFeatureTranslation(cdMc);
  if (translation == cMo)
    feature = new vpFeatureTranslation(cMo);
  if (translation == cMcd)
    feature = new vpFeatureTranslation(cMcd);
  return feature;
}

/*!

  Not implemented.

*/
void vpFeatureTranslation::display(const vpCameraParameters & /* cam */, const vpImage<unsigned char> & /* I */,
                                   const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}
/*!

  Not implemented.

*/
void vpFeatureTranslation::display(const vpCameraParameters & /* cam */, const vpImage<vpRGBa> & /* I */,
                                   const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Function used to select the \f$ t_x\f$ subset of the translation
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ t_x\f$.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  - With the feature type cdMc:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cdMc);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  - With the feature type cMcd:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMcd);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  - With the feature type cMo:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMo);
  vpFeatureTranslation t_star(vpFeatureTranslation::cMo);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, t_star, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  \sa selectTy(), selectTz()

*/
unsigned int vpFeatureTranslation::selectTx() { return FEATURE_LINE[0]; }

/*!

  Function used to select the \f$ t_y\f$ subset of the translation
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ t_y\f$.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  - With the feature type cdMc:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cdMc);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  - With the feature type cMcd:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMcd);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  - With the feature type cMo:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMo);
  vpFeatureTranslation t_star(vpFeatureTranslation::cMo);
  vpServo task;
  ...
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(t, t_star, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  \sa selectTx(), selectTz()
*/
unsigned int vpFeatureTranslation::selectTy() { return FEATURE_LINE[1]; }

/*!

  Function used to select the \f$ t_z\f$ subset of the translation
  visual feature.

  This function is to use in conjunction with interaction() in order
  to compute the interaction matrix associated to \f$ t_z\f$.

  See the interaction() method for an usage example.

  This function is also useful in the vpServo class to indicate that
  a subset of the visual feature is to use in the control law:

  - With the feature type cdMc:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cdMc);
  vpServo task;
  ...
  // Add the (tz) subset feature from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTz());
  \endcode

  - With the feature type cMcd:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMcd);
  vpServo task;
  ...
  // Add the (tz) subset feature from 3D translation to the task
  task.addFeature(t, vpFeatureTranslation::selectTz());
  \endcode

  - With the feature type cMo:
  \code
  vpFeatureTranslation t(vpFeatureTranslation::cMo);
  vpFeatureTranslation t_star(vpFeatureTranslation::cMo);
  vpServo task;
  ...
  // Add the (tz) subset feature from 3D translation to the task
  task.addFeature(t, t_star, vpFeatureTranslation::selectTz());
  \endcode

  \sa selectTx(), selectTy()
*/
unsigned int vpFeatureTranslation::selectTz() { return FEATURE_LINE[2]; }
