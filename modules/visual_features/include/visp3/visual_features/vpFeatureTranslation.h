/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#ifndef vpFeatureTranslation_H
#define vpFeatureTranslation_H

/*!
  \file vpFeatureTranslation.h
  \brief class that defines the translation visual feature.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/visual_features/vpBasicFeature.h>

/*!
  \class vpFeatureTranslation
  \ingroup group_visual_features

  \brief Class that defines the translation visual feature
  \f$s=(t_x,t_y,t_z)\f$.

  It is convenient to consider two coordinate frames noted here \f$
{\cal{F}}_1 \f$ and \f$
  {\cal{F}}_{2} \f$.

  Let \f$^{{\cal{F}}_2}M_{{\cal{F}}_1} \f$ be the homogeneous matrix that
gives the orientation and the translation of the frame \f$ {\cal{F}}_1 \f$
with respect to the frame \f$ {\cal{F}}_2 \f$.

  \f[
  ^{{\cal{F}}_2}M_{{\cal{F}}_1} = \left(\begin{array}{cc}
  ^{{\cal{F}}_2}R_{{\cal{F}}_1} & ^{{\cal{F}}_2}t_{{\cal{F}}_1}  \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right)
  \f]

  with \f$^{{\cal{F}}_2}R_{{\cal{F}}_1} \f$ the rotation matrix that gives the
orientation of the frame \f$ {\cal{F}}_1 \f$ relative to the frame \f$
{\cal{F}}_2 \f$ and \f$^{{\cal{F}}_2}t_{{\cal{F}}_1} \f$ the translation
vector that gives the position of the frame \f$ {\cal{F}}_1 \f$ relative to
the frame \f$ {\cal{F}}_2 \f$. To know more about homogeneous matrices see
vpHomogeneousMatrix documentation.

  This class can be used to manipulate three kind of visual features:

  -  This class can be used to manipulate the translation visual feature
  \f$s= ^{c^*}t_c\f$ which gives the position of
  the current camera frame relative to the desired camera frame. It is
composed by the three components \f$(t_x,t_y,t_z)\f$. The desired visual
feature \f$ s^* \f$ is equal to zero. The corresponding error is than equal to
\f$ e=(s-s^*) = ^{c^*}t_c \f$. In this case, the interaction matrix related to
\f$ s \f$ is given by \f[ L = [
  ^{c^*}R_c \;\; 0_3] \f]

  -  This class can also be used to manipulate the translation visual feature
  \f$s= ^{c}t_{c^*}\f$ which gives the position of
  the desired camera frame relative to the current camera frame. It is
composed by the three components \f$(t_x,t_y,t_z)\f$. The desired visual
feature \f$ s^* \f$ is equal to zero. The corresponding error is than equal to
\f$ e=(s-s^*) = ^{c}t_{c^*} \f$. In this case, the interaction matrix related
to \f$ s \f$ is given by \f[ L = [ -I_3 \;\; [^{c}t_{c^*}]_\times] \f]

  - Actually, this class can also be used to manipulate the
  translation visual feature \f$s= ^{c}t_o\f$ which gives the position
  of the object frame relative to the current camera frame. It is
  composed by the three components \f$(t_x,t_y,t_z)\f$ too. The
  desired visual feature \f$ s^* \f$ is the translation visual feature
  \f$s^*= ^{c^*}t_o\f$ which gives the position of the object frame
  relative to the desired camera frame. The corresponding error is
  than equal to \f$ e=(s-s^*) = ^{c}t_o - ^{c^*}t_o \f$. In this case,
  the interaction matrix related to \f$ s \f$ is given by \f[ L = [
  -I_3 \;\; [^{c}t_o]_\times] \f]

  To initialize the feature \f$(t_x, t_y, t_z)\f$ you may use member
  fonctions like set_Tx(), set_Ty(), set_Tz(), or also buildFrom()
  fonctions.

  The interaction() method allows to compute the interaction matrix
  \f$ L\f$ associated to the translation visual feature, while the
  error() method computes the error vector \f$(s - s^*)\f$ between the
  current visual feature and the desired one.

  The code below shows how to create a eye-in hand visual servoing
  task using a 3D translation feature \f$(t_x,t_y,t_z)\f$ that
  correspond to the 3D translation between the desired camera frame
  and the current camera frame. To control six degrees of freedom, at
  least three other features must be considered like vpFeatureThetaU
  visual features. First we create a current (\f$s\f$) and desired
  (\f$s^*\f$) 3D translation feature, set the task to use the
  interaction matrix associated to the current feature \f$L_s\f$ and
  than compute the camera velocity \f$v=-\lambda \; L_s^+ \;
  (s-s^*)\f$. The current feature \f$s\f$ is updated in the while() loop
  while \f$s^*\f$ is set to zero.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpHomogeneousMatrix cdMc;
  // ... cdMc need here to be initialized from for example a pose estimation.

  // Creation of the current visual feature s
  vpFeatureTranslation s(vpFeatureTranslation::cdMc);
  s.buildFrom(cdMc); // Initialization of the current feature s=(tx,ty,tz)

  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT);
  // Set the constant gain
  double lambda = 0.8;
  task.setLambda(lambda);

  // Add the 3D translation feature to the task
  task.addFeature(s); // s* is here considered as zero

  // Control loop
  for ( ; ; ) {
    // ... cdMc need here to be initialized from for example a pose estimation.

    // Update the current 3D translation visual feature
    s.buildFrom(cdMc);

    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
}
  \endcode

  If you want to deal only with the \f$(t_x,t_y)\f$ subset feature from the 3D
  translation, you have just to modify the addFeature() call in
  the previous example by the following line. In that case, the dimension
  of \f$s\f$ is two.

  \code
  // Add the (tx,ty) subset features from 3D translation to the task
  task.addFeature(s, vpFeatureTranslation::selectTx() | vpFeatureTranslation::selectTy());
  \endcode

  If you want to build your own control law, this other example shows
  how to create a current (\f$s\f$) and desired (\f$s^*\f$) 3D
  translation visual feature, compute the corresponding error
  vector \f$(s-s^*)\f$ and finally build the interaction matrix \f$L_s\f$.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpFeatureTranslation.h>

int main()
{
  vpHomogeneousMatrix cdMc;
  // ... cdMc need here to be initialized from for example a pose estimation.

  // Creation of the current feature s
  vpFeatureTranslation s(vpFeatureTranslation::cdMc);
  s.buildFrom(cdMc); // Initialization of the feature

  // Creation of the desired feature s*. By default this feature is
  // initialized to zero
  vpFeatureTranslation s_star(vpFeatureTranslation::cdMc);

  // Compute the interaction matrix for the translation feature
  vpMatrix L = s.interaction();

  // Compute the error vector (s-s*) for the translation feature
  vpColVector e = s.error(s_star); // e = (s-s*)
}
  \endcode

  The code below shows how to create an eye-in hand visual servoing
  task using a 3D translation feature \f$(t_x,t_y,t_z)\f$ that
  correspond to the 3D translation between the current camera frame
  and the object frame. Like with the previous examples, to
  control six degrees of freedom, at least three other features must be
  considered like vpFeatureThetaU visual features. The way to initialize
  the visual features is quite the same as before. The difference is that
  the cMo method must be precised and the desired feature is note
  necessary equal to zero.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpServo task; // Visual servoing task

  vpHomogeneousMatrix cdMo;
  // ... cdMo need here to be initialized from for example a pose estimation.

  // Creation of the desired visual feature s*
  vpFeatureTranslation s_star(vpFeatureTranslation::cMo);
  s_star.buildFrom(cdMo); // Initialization of the desired feature s*=(tx*,ty*,tz*)

  vpHomogeneousMatrix cMo;
  // ... cMo need here to be computed.

  // Creation of the current visual feature s
  vpFeatureTranslation s(vpFeatureTranslation::cMo);
  s.buildFrom(cMo); // Initialization of the current feature s=(tx,ty,tz)

  // Set eye-in-hand control law.
  // The computed velocities will be expressed in the camera frame
  task.setServo(vpServo::EYEINHAND_CAMERA);
  // Interaction matrix is computed with the current visual features s
  task.setInteractionMatrixType(vpServo::CURRENT);
  // Set the constant gain
  double lambda = 0.8;
  task.setLambda(lambda);

  // Add the 3D translation feature to the task
  task.addFeature(s, s_star); // s* is here considered as zero

  // Control loop
  for ( ; ; ) {
    // ... cMo need here to be computed from for example a pose estimation.

    // Update the current 3D translation visual feature
    s.buildFrom(cMo);

    // compute the control law
    vpColVector v = task.computeControlLaw(); // camera velocity
  }
}
  \endcode

*/
class VISP_EXPORT vpFeatureTranslation : public vpBasicFeature
{
public:
  /*!
    \enum vpFeatureTranslationRepresentationType
    Kind of implemented 3D translation feature.
   */
  typedef enum {
    /*! Selector used to manipulate the visual feature \f$s=
      ^{c^*}t_c\f$ which gives the position of the current camera frame
      relative to the desired camera frame.*/
    cdMc,
    /*! Selector used to manipulate the visual feature \f$s=
      ^{c}t_{c^*}\f$ which gives the position of the desired camera frame
      relative to the current camera frame.*/
    cMcd,
    /*! Selector used to manipulate the visual feature \f$s=
      ^{c}t_o\f$ which gives the position of the object frame relative to
      the current camera frame. */
    cMo
  } vpFeatureTranslationRepresentationType;

  // basic contructor
  vpFeatureTranslation();
  // basic constructor specifying the type of translation feature
  explicit vpFeatureTranslation(vpFeatureTranslationRepresentationType r);
  // constructor : build from an homogeneous matrix
  // cdMc is the displacement that the camera has to realize
  vpFeatureTranslation(vpHomogeneousMatrix &f2Mf1, vpFeatureTranslationRepresentationType r);
  //! Destructor. Does nothing.
  virtual ~vpFeatureTranslation() {}

  // build from an homogeneous matrix
  // cdMc is the displacement that the camera has to realize
  void buildFrom(const vpHomogeneousMatrix &f2Mf1);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  //! Feature duplication
  vpFeatureTranslation *duplicate() const;

  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);

  vpFeatureTranslationRepresentationType getFeatureTranslationType() const;

  double get_Tx() const;
  double get_Ty() const;
  double get_Tz() const;

  // basic construction
  void init();
  // compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  // print the name of the feature
  void print(const unsigned int select = FEATURE_ALL) const;

  void set_Tx(const double t_x);
  void set_Ty(const double t_y);
  void set_Tz(const double t_z);

  void setFeatureTranslationType(const vpFeatureTranslationRepresentationType r);

  // feature selection
  static unsigned int selectTx();
  static unsigned int selectTy();
  static unsigned int selectTz();

private:
  //! displacement that the camera has to realize
  vpHomogeneousMatrix f2Mf1;
  vpFeatureTranslationRepresentationType translation;
};

#endif
