/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Visual servoing control law.
 */

#include <sstream>

#include <visp3/core/vpException.h>
#include <visp3/core/vpDebug.h>
#include <visp3/vs/vpServo.h>

/*!
  \file vpServo.cpp
  \brief  Class required to compute the visual servoing control law
*/

vpServo::vpServo()
  : L(), error(), J1(), J1p(), s(), sStar(), e1(), e(), q_dot(), v(), servoType(vpServo::NONE), rankJ1(0),
  featureList(), desiredFeatureList(), featureSelectionList(), lambda(), signInteractionMatrix(1),
  interactionMatrixType(DESIRED), inversionType(PSEUDO_INVERSE), cVe(), init_cVe(false), cVf(), init_cVf(false),
  fVe(), init_fVe(false), eJe(), init_eJe(false), fJe(), init_fJe(false), errorComputed(false),
  interactionMatrixComputed(false), dim_task(0), taskWasKilled(false), forceInteractionMatrixComputation(false),
  WpW(), I_WpW(), P(), sv(), mu(4.), e1_initial(), iscJcIdentity(true), cJc(6, 6), m_first_iteration(true),
  m_pseudo_inverse_threshold(1e-6)
{
  cJc.eye();
}

vpServo::vpServo(vpServoType servo_type)
  : L(), error(), J1(), J1p(), s(), sStar(), e1(), e(), q_dot(), v(), servoType(servo_type), rankJ1(0), featureList(),
  desiredFeatureList(), featureSelectionList(), lambda(), signInteractionMatrix(1), interactionMatrixType(DESIRED),
  inversionType(PSEUDO_INVERSE), cVe(), init_cVe(false), cVf(), init_cVf(false), fVe(), init_fVe(false), eJe(),
  init_eJe(false), fJe(), init_fJe(false), errorComputed(false), interactionMatrixComputed(false), dim_task(0),
  taskWasKilled(false), forceInteractionMatrixComputation(false), WpW(), I_WpW(), P(), sv(), mu(4), e1_initial(),
  iscJcIdentity(true), cJc(6, 6), m_first_iteration(true)
{
  cJc.eye();
}

vpServo::~vpServo() { kill(); }

void vpServo::init()
{
  // type of visual servoing
  servoType = vpServo::NONE;

  // Twist transformation matrix
  init_cVe = false;
  init_cVf = false;
  init_fVe = false;
  // Jacobians
  init_eJe = false;
  init_fJe = false;

  dim_task = 0;

  featureList.clear();
  desiredFeatureList.clear();
  featureSelectionList.clear();

  signInteractionMatrix = 1;

  interactionMatrixType = DESIRED;
  inversionType = PSEUDO_INVERSE;

  interactionMatrixComputed = false;
  errorComputed = false;

  taskWasKilled = false;

  forceInteractionMatrixComputation = false;

  rankJ1 = 0;

  m_first_iteration = true;
}

void vpServo::kill()
{
  if (taskWasKilled == false) {
    // kill the current and desired feature lists

    // current list
    for (std::list<vpBasicFeature *>::iterator it = featureList.begin(); it != featureList.end(); ++it) {
      if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
        delete (*it);
        (*it) = NULL;
      }
    }
    // desired list
    for (std::list<vpBasicFeature *>::iterator it = desiredFeatureList.begin(); it != desiredFeatureList.end(); ++it) {
      if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
        delete (*it);
        (*it) = NULL;
      }
    }

    featureList.clear();
    desiredFeatureList.clear();
    taskWasKilled = true;
  }
}

void vpServo::setServo(const vpServoType &servo_type)
{
  this->servoType = servo_type;

  if ((servoType == EYEINHAND_CAMERA) || (servoType == EYEINHAND_L_cVe_eJe))
    signInteractionMatrix = 1;
  else
    signInteractionMatrix = -1;

  // when the control is directly compute in the camera frame
  // we relieve the end-user to initialize cVa and aJe
  if (servoType == EYEINHAND_CAMERA) {
    vpVelocityTwistMatrix _cVe;
    set_cVe(_cVe);

    vpMatrix _eJe;
    _eJe.eye(6);
    set_eJe(_eJe);
  };
}

void vpServo::setCameraDoF(const vpColVector &dof)
{
  if (dof.size() == 6) {
    iscJcIdentity = true;
    for (unsigned int i = 0; i < 6; i++) {
      if (std::fabs(dof[i]) > std::numeric_limits<double>::epsilon()) {
        cJc[i][i] = 1.0;
      }
      else {
        cJc[i][i] = 0.0;
        iscJcIdentity = false;
      }
    }
  }
}

void vpServo::print(const vpServo::vpServoPrintType displayLevel, std::ostream &os)
{
  switch (displayLevel) {
  case vpServo::ALL: {
    os << "Visual servoing task: " << std::endl;

    os << "Type of control law " << std::endl;
    switch (servoType) {
    case NONE:
      os << "Type of task have not been chosen yet ! " << std::endl;
      break;
    case EYEINHAND_CAMERA:
      os << "Eye-in-hand configuration " << std::endl;
      os << "Control in the camera frame " << std::endl;
      break;
    case EYEINHAND_L_cVe_eJe:
      os << "Eye-in-hand configuration " << std::endl;
      os << "Control in the articular frame " << std::endl;
      break;
    case EYETOHAND_L_cVe_eJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVe_eJe q_dot " << std::endl;
      break;
    case EYETOHAND_L_cVf_fVe_eJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVe_fVe_eJe q_dot " << std::endl;
      break;
    case EYETOHAND_L_cVf_fJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVf_fJe q_dot " << std::endl;
      break;
    }

    os << "List of visual features : s" << std::endl;
    std::list<vpBasicFeature *>::const_iterator it_s;
    std::list<vpBasicFeature *>::const_iterator it_s_star;
    std::list<unsigned int>::const_iterator it_select;

    for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end();
      ++it_s, ++it_select) {
      os << "";
      (*it_s)->print((*it_select));
    }

    os << "List of desired visual features : s*" << std::endl;
    for (it_s_star = desiredFeatureList.begin(), it_select = featureSelectionList.begin();
      it_s_star != desiredFeatureList.end(); ++it_s_star, ++it_select) {
      os << "";
      (*it_s_star)->print((*it_select));
    }

    os << "Interaction Matrix Ls " << std::endl;
    if (interactionMatrixComputed) {
      os << L << std::endl;
    }
    else {
      os << "not yet computed " << std::endl;
    }

    os << "Error vector (s-s*) " << std::endl;
    if (errorComputed) {
      os << error.t() << std::endl;
    }
    else {
      os << "not yet computed " << std::endl;
    }

    os << "Gain : " << lambda << std::endl;

    break;
  }

  case vpServo::CONTROLLER: {
    os << "Type of control law " << std::endl;
    switch (servoType) {
    case NONE:
      os << "Type of task have not been chosen yet ! " << std::endl;
      break;
    case EYEINHAND_CAMERA:
      os << "Eye-in-hand configuration " << std::endl;
      os << "Control in the camera frame " << std::endl;
      break;
    case EYEINHAND_L_cVe_eJe:
      os << "Eye-in-hand configuration " << std::endl;
      os << "Control in the articular frame " << std::endl;
      break;
    case EYETOHAND_L_cVe_eJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVe_eJe q_dot " << std::endl;
      break;
    case EYETOHAND_L_cVf_fVe_eJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVe_fVe_eJe q_dot " << std::endl;
      break;
    case EYETOHAND_L_cVf_fJe:
      os << "Eye-to-hand configuration " << std::endl;
      os << "s_dot = _L_cVf_fJe q_dot " << std::endl;
      break;
    }
    break;
  }

  case vpServo::FEATURE_CURRENT: {
    os << "List of visual features : s" << std::endl;

    std::list<vpBasicFeature *>::const_iterator it_s;
    std::list<unsigned int>::const_iterator it_select;

    for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end();
      ++it_s, ++it_select) {
      os << "";
      (*it_s)->print((*it_select));
    }
    break;
  }
  case vpServo::FEATURE_DESIRED: {
    os << "List of desired visual features : s*" << std::endl;

    std::list<vpBasicFeature *>::const_iterator it_s_star;
    std::list<unsigned int>::const_iterator it_select;

    for (it_s_star = desiredFeatureList.begin(), it_select = featureSelectionList.begin();
      it_s_star != desiredFeatureList.end(); ++it_s_star, ++it_select) {
      os << "";
      (*it_s_star)->print((*it_select));
    }
    break;
  }
  case vpServo::GAIN: {
    os << "Gain : " << lambda << std::endl;
    break;
  }
  case vpServo::INTERACTION_MATRIX: {
    os << "Interaction Matrix Ls " << std::endl;
    if (interactionMatrixComputed) {
      os << L << std::endl;
    }
    else {
      os << "not yet computed " << std::endl;
    }
    break;
  }

  case vpServo::ERROR_VECTOR:
  case vpServo::MINIMUM:

  {
    os << "Error vector (s-s*) " << std::endl;
    if (errorComputed) {
      os << error.t() << std::endl;
    }
    else {
      os << "not yet computed " << std::endl;
    }

    break;
  }
  }
}

void vpServo::addFeature(vpBasicFeature &s_cur, vpBasicFeature &s_star, unsigned int select)
{
  featureList.push_back(&s_cur);
  desiredFeatureList.push_back(&s_star);
  featureSelectionList.push_back(select);
}

void vpServo::addFeature(vpBasicFeature &s_cur, unsigned int select)
{
  featureList.push_back(&s_cur);

  // in fact we have a problem with s_star that is not defined
  // by the end user.

  // If the same user want to compute the interaction at the desired position
  // this "virtual feature" must exist

  // a solution is then to duplicate the current feature (s* = s)
  // and reinitialized s* to 0

  // it remains the deallocation issue therefore a flag that stipulates that
  // the feature has been allocated in vpServo is set

  // vpServo must deallocate the memory (see ~vpServo and kill() )

  vpBasicFeature *s_star;
  s_star = s_cur.duplicate();

  s_star->init();
  s_star->setDeallocate(vpBasicFeature::vpServo);

  desiredFeatureList.push_back(s_star);
  featureSelectionList.push_back(select);
}

unsigned int vpServo::getDimension() const
{
  unsigned int dim = 0;
  std::list<vpBasicFeature *>::const_iterator it_s;
  std::list<unsigned int>::const_iterator it_select;

  for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end();
    ++it_s, ++it_select) {
    dim += (*it_s)->getDimension(*it_select);
  }

  return dim;
}

void vpServo::setInteractionMatrixType(const vpServoIteractionMatrixType &interactionMatrix_type,
  const vpServoInversionType &interactionMatrixInversion)
{
  this->interactionMatrixType = interactionMatrix_type;
  this->inversionType = interactionMatrixInversion;
}

static void computeInteractionMatrixFromList(const std::list<vpBasicFeature *> &featureList,
  const std::list<unsigned int> &featureSelectionList, vpMatrix &L)
{
  if (featureList.empty()) {
    vpERROR_TRACE("feature list empty, cannot compute Ls");
    throw(vpServoException(vpServoException::noFeatureError, "feature list empty, cannot compute Ls"));
  }

  /* The matrix dimension is not known before the affectation loop.
   * It thus should be allocated on the flight, in the loop.
   * The first assumption is that the size has not changed. A double
   * reallocation (realloc(dim*2)) is done if necessary. In particular,
   * [log_2(dim)+1] reallocations are done for the first matrix computation.
   * If the allocated size is too large, a correction is done after the loop.
   * The algorithmic cost is linear in affectation, logarithmic in allocation
   * numbers and linear in allocation size.
   */

   /* First assumption: matrix dimensions have not changed. If 0, they are
    * initialized to dim 1.*/
  unsigned int rowL = L.getRows();
  const unsigned int colL = 6;
  if (0 == rowL) {
    rowL = 1;
    L.resize(rowL, colL);
  }

  /* vectTmp is used to store the return values of functions get_s() and
   * error(). */
  vpMatrix matrixTmp;

  /* The cursor are the number of the next case of the vector array to
   * be affected. A memory reallocation should be done when cursor
   * is out of the vector-array range.*/
  unsigned int cursorL = 0;

  std::list<vpBasicFeature *>::const_iterator it;
  std::list<unsigned int>::const_iterator it_select;

  for (it = featureList.begin(), it_select = featureSelectionList.begin(); it != featureList.end(); ++it, ++it_select) {
    /* Get s. */
    matrixTmp = (*it)->interaction(*it_select);
    unsigned int rowMatrixTmp = matrixTmp.getRows();
    unsigned int colMatrixTmp = matrixTmp.getCols();

    /* Check the matrix L size, and realloc if needed. */
    while (rowMatrixTmp + cursorL > rowL) {
      rowL *= 2;
      L.resize(rowL, colL, false);
      vpDEBUG_TRACE(15, "Realloc!");
    }

    /* Copy the temporarily matrix into L. */
    for (unsigned int k = 0; k < rowMatrixTmp; ++k, ++cursorL) {
      for (unsigned int j = 0; j < colMatrixTmp; ++j) {
        L[cursorL][j] = matrixTmp[k][j];
      }
    }
  }

  L.resize(cursorL, colL, false);

  return;
}

vpMatrix vpServo::computeInteractionMatrix()
{
  try {

    switch (interactionMatrixType) {
    case CURRENT: {
      try {
        computeInteractionMatrixFromList(this->featureList, this->featureSelectionList, L);
        dim_task = L.getRows();
        interactionMatrixComputed = true;
      }

      catch (...) {
        throw;
      }
    } break;
    case DESIRED: {
      try {
        if (interactionMatrixComputed == false || forceInteractionMatrixComputation == true) {
          computeInteractionMatrixFromList(this->desiredFeatureList, this->featureSelectionList, L);

          dim_task = L.getRows();
          interactionMatrixComputed = true;
        }

      }
      catch (...) {
        throw;
      }
    } break;
    case MEAN: {
      vpMatrix Lstar(L.getRows(), L.getCols());
      try {
        computeInteractionMatrixFromList(this->featureList, this->featureSelectionList, L);
        computeInteractionMatrixFromList(this->desiredFeatureList, this->featureSelectionList, Lstar);
      }
      catch (...) {
        throw;
      }
      L = (L + Lstar) / 2;

      dim_task = L.getRows();
      interactionMatrixComputed = true;
    } break;
    case USER_DEFINED:
      // dim_task = L.getRows() ;
      interactionMatrixComputed = false;
      break;
    }

  }
  catch (...) {
    throw;
  }
  return L;
}

vpColVector vpServo::computeError()
{
  if (featureList.empty()) {
    vpERROR_TRACE("feature list empty, cannot compute Ls");
    throw(vpServoException(vpServoException::noFeatureError, "feature list empty, cannot compute Ls"));
  }
  if (desiredFeatureList.empty()) {
    vpERROR_TRACE("feature list empty, cannot compute Ls");
    throw(vpServoException(vpServoException::noFeatureError, "feature list empty, cannot compute Ls"));
  }

  try {
    vpBasicFeature *current_s;
    vpBasicFeature *desired_s;

    /* The vector dimensions are not known before the affectation loop.
     * They thus should be allocated on the flight, in the loop.
     * The first assumption is that the size has not changed. A double
     * reallocation (realloc(dim*2)) is done if necessary. In particular,
     * [log_2(dim)+1] reallocations are done for the first error computation.
     * If the allocated size is too large, a correction is done after the
     * loop. The algorithmic cost is linear in affectation, logarithmic in
     * allocation numbers and linear in allocation size. No assumptions are
     * made concerning size of each vector: they are not said equal, and could
     * be different.
     */

     /* First assumption: vector dimensions have not changed. If 0, they are
      * initialized to dim 1.*/
    unsigned int dimError = error.getRows();
    unsigned int dimS = s.getRows();
    unsigned int dimSStar = sStar.getRows();
    if (0 == dimError) {
      dimError = 1;
      error.resize(dimError);
    }
    if (0 == dimS) {
      dimS = 1;
      s.resize(dimS);
    }
    if (0 == dimSStar) {
      dimSStar = 1;
      sStar.resize(dimSStar);
    }

    /* vectTmp is used to store the return values of functions get_s() and
     * error(). */
    vpColVector vectTmp;

    /* The cursor are the number of the next case of the vector array to
     * be affected. A memory reallocation should be done when cursor
     * is out of the vector-array range.*/
    unsigned int cursorS = 0;
    unsigned int cursorSStar = 0;
    unsigned int cursorError = 0;

    /* For each cell of the list, copy value of s, s_star and error. */
    std::list<vpBasicFeature *>::const_iterator it_s;
    std::list<vpBasicFeature *>::const_iterator it_s_star;
    std::list<unsigned int>::const_iterator it_select;

    for (it_s = featureList.begin(), it_s_star = desiredFeatureList.begin(), it_select = featureSelectionList.begin();
      it_s != featureList.end(); ++it_s, ++it_s_star, ++it_select) {
      current_s = (*it_s);
      desired_s = (*it_s_star);
      unsigned int select = (*it_select);

      /* Get s, and store it in the s vector. */
      vectTmp = current_s->get_s(select);
      unsigned int dimVectTmp = vectTmp.getRows();
      while (dimVectTmp + cursorS > dimS) {
        dimS *= 2;
        s.resize(dimS, false);
        vpDEBUG_TRACE(15, "Realloc!");
      }
      for (unsigned int k = 0; k < dimVectTmp; ++k) {
        s[cursorS++] = vectTmp[k];
      }

      /* Get s_star, and store it in the s vector. */
      vectTmp = desired_s->get_s(select);
      dimVectTmp = vectTmp.getRows();
      while (dimVectTmp + cursorSStar > dimSStar) {
        dimSStar *= 2;
        sStar.resize(dimSStar, false);
      }
      for (unsigned int k = 0; k < dimVectTmp; ++k) {
        sStar[cursorSStar++] = vectTmp[k];
      }

      /* Get error, and store it in the s vector. */
      vectTmp = current_s->error(*desired_s, select);
      dimVectTmp = vectTmp.getRows();
      while (dimVectTmp + cursorError > dimError) {
        dimError *= 2;
        error.resize(dimError, false);
      }
      for (unsigned int k = 0; k < dimVectTmp; ++k) {
        error[cursorError++] = vectTmp[k];
      }
    }

    /* If too much memory has been allocated, realloc. */
    s.resize(cursorS, false);
    sStar.resize(cursorSStar, false);
    error.resize(cursorError, false);

    /* Final modifications. */
    dim_task = error.getRows();
    errorComputed = true;
  }
  catch (...) {
    throw;
  }
  return error;
}

bool vpServo::testInitialization()
{
  switch (servoType) {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined");
    throw(vpServoException(vpServoException::servoError, "No control law have been yet defined"));
    break;
  case EYEINHAND_CAMERA:
    return true;
    break;
  case EYEINHAND_L_cVe_eJe:
  case EYETOHAND_L_cVe_eJe:
    if (!init_cVe)
      vpERROR_TRACE("cVe not initialized");
    if (!init_eJe)
      vpERROR_TRACE("eJe not initialized");
    return (init_cVe && init_eJe);
    break;
  case EYETOHAND_L_cVf_fVe_eJe:
    if (!init_cVf)
      vpERROR_TRACE("cVf not initialized");
    if (!init_fVe)
      vpERROR_TRACE("fVe not initialized");
    if (!init_eJe)
      vpERROR_TRACE("eJe not initialized");
    return (init_cVf && init_fVe && init_eJe);
    break;

  case EYETOHAND_L_cVf_fJe:
    if (!init_cVf)
      vpERROR_TRACE("cVf not initialized");
    if (!init_fJe)
      vpERROR_TRACE("fJe not initialized");
    return (init_cVf && init_fJe);
    break;
  }

  return false;
}

bool vpServo::testUpdated()
{
  switch (servoType) {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined");
    throw(vpServoException(vpServoException::servoError, "No control law have been yet defined"));
    break;
  case EYEINHAND_CAMERA:
    return true;
  case EYEINHAND_L_cVe_eJe:
    if (!init_eJe)
      vpERROR_TRACE("eJe not updated");
    return (init_eJe);
    break;
  case EYETOHAND_L_cVe_eJe:
    if (!init_cVe)
      vpERROR_TRACE("cVe not updated");
    if (!init_eJe)
      vpERROR_TRACE("eJe not updated");
    return (init_cVe && init_eJe);
    break;
  case EYETOHAND_L_cVf_fVe_eJe:
    if (!init_fVe)
      vpERROR_TRACE("fVe not updated");
    if (!init_eJe)
      vpERROR_TRACE("eJe not updated");
    return (init_fVe && init_eJe);
    break;

  case EYETOHAND_L_cVf_fJe:
    if (!init_fJe)
      vpERROR_TRACE("fJe not updated");
    return (init_fJe);
    break;
  }

  return false;
}

vpColVector vpServo::computeControlLaw()
{
  vpVelocityTwistMatrix cVa; // Twist transformation matrix
  vpMatrix aJe;              // Jacobian

  if (m_first_iteration) {
    if (testInitialization() == false) {
      vpERROR_TRACE("All the matrices are not correctly initialized");
      throw(vpServoException(vpServoException::servoError, "Cannot compute control law. "
                             "All the matrices are not correctly"
                             "initialized."));
    }
  }
  if (testUpdated() == false) {
    vpERROR_TRACE("All the matrices are not correctly updated");
  }

  // test if all the required initialization have been done
  switch (servoType) {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined");
    throw(vpServoException(vpServoException::servoError, "No control law have been yet defined"));
    break;
  case EYEINHAND_CAMERA:
  case EYEINHAND_L_cVe_eJe:
  case EYETOHAND_L_cVe_eJe:

    cVa = cVe;
    aJe = eJe;

    init_cVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fVe_eJe:
    cVa = cVf * fVe;
    aJe = eJe;
    init_fVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fJe:
    cVa = cVf;
    aJe = fJe;
    init_fJe = false;
    break;
  }

  computeInteractionMatrix();
  computeError();

  // compute  task Jacobian
  if (iscJcIdentity)
    J1 = L * cVa * aJe;
  else
    J1 = L * cJc * cVa * aJe;

  // handle the eye-in-hand eye-to-hand case
  J1 *= signInteractionMatrix;

  // pseudo inverse of the task Jacobian
  // and rank of the task Jacobian
  // the image of J1 is also computed to allows the computation
  // of the projection operator
  vpMatrix imJ1t, imJ1;
  bool imageComputed = false;

  if (inversionType == PSEUDO_INVERSE) {
    rankJ1 = J1.pseudoInverse(J1p, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);

    imageComputed = true;
  }
  else
    J1p = J1.t();

  if (rankJ1 == J1.getCols()) {
    /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
    e1 = J1p * error; // primary task

    WpW.eye(J1.getCols(), J1.getCols());
  }
  else {
    if (imageComputed != true) {
      vpMatrix Jtmp;
      // image of J1 is computed to allows the computation
      // of the projection operator
      rankJ1 = J1.pseudoInverse(Jtmp, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);
    }
    WpW = imJ1t.AAt();

#ifdef DEBUG
    std::cout << "rank J1: " << rankJ1 << std::endl;
    imJ1t.print(std::cout, 10, "imJ1t");
    imJ1.print(std::cout, 10, "imJ1");

    WpW.print(std::cout, 10, "WpW");
    J1.print(std::cout, 10, "J1");
    J1p.print(std::cout, 10, "J1p");
#endif
    e1 = WpW * J1p * error;
  }
  e = -lambda(e1) * e1;

  I.eye(J1.getCols());

  // Compute classical projection operator
  I_WpW = (I - WpW);

  m_first_iteration = false;
  return e;
}

vpColVector vpServo::computeControlLaw(double t)
{
  vpVelocityTwistMatrix cVa; // Twist transformation matrix
  vpMatrix aJe;              // Jacobian

  if (m_first_iteration) {
    if (testInitialization() == false) {
      vpERROR_TRACE("All the matrices are not correctly initialized");
      throw(vpServoException(vpServoException::servoError, "Cannot compute control law "
                             "All the matrices are not correctly"
                             "initialized"));
    }
  }
  if (testUpdated() == false) {
    vpERROR_TRACE("All the matrices are not correctly updated");
  }

  // test if all the required initialization have been done
  switch (servoType) {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined");
    throw(vpServoException(vpServoException::servoError, "No control law have been yet defined"));
    break;
  case EYEINHAND_CAMERA:
  case EYEINHAND_L_cVe_eJe:
  case EYETOHAND_L_cVe_eJe:

    cVa = cVe;
    aJe = eJe;

    init_cVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fVe_eJe:
    cVa = cVf * fVe;
    aJe = eJe;
    init_fVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fJe:
    cVa = cVf;
    aJe = fJe;
    init_fJe = false;
    break;
  }

  computeInteractionMatrix();
  computeError();

  // compute  task Jacobian
  J1 = L * cVa * aJe;

  // handle the eye-in-hand eye-to-hand case
  J1 *= signInteractionMatrix;

  // pseudo inverse of the task Jacobian
  // and rank of the task Jacobian
  // the image of J1 is also computed to allows the computation
  // of the projection operator
  vpMatrix imJ1t, imJ1;
  bool imageComputed = false;

  if (inversionType == PSEUDO_INVERSE) {
    rankJ1 = J1.pseudoInverse(J1p, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);

    imageComputed = true;
  }
  else
    J1p = J1.t();

  if (rankJ1 == J1.getCols()) {
    /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
    e1 = J1p * error; // primary task

    WpW.eye(J1.getCols());
  }
  else {
    if (imageComputed != true) {
      vpMatrix Jtmp;
      // image of J1 is computed to allows the computation
      // of the projection operator
      rankJ1 = J1.pseudoInverse(Jtmp, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);
    }
    WpW = imJ1t.AAt();

#ifdef DEBUG
    std::cout << "rank J1 " << rankJ1 << std::endl;
    std::cout << "imJ1t" << std::endl << imJ1t;
    std::cout << "imJ1" << std::endl << imJ1;

    std::cout << "WpW" << std::endl << WpW;
    std::cout << "J1" << std::endl << J1;
    std::cout << "J1p" << std::endl << J1p;
#endif
    e1 = WpW * J1p * error;
  }

  // memorize the initial e1 value if the function is called the first time
  // or if the time given as parameter is equal to 0.
  if (m_first_iteration || std::fabs(t) < std::numeric_limits<double>::epsilon()) {
    e1_initial = e1;
  }
  // Security check. If size of e1_initial and e1 differ, that means that
  // e1_initial was not set
  if (e1_initial.getRows() != e1.getRows())
    e1_initial = e1;

  e = -lambda(e1) * e1 + lambda(e1) * e1_initial * exp(-mu * t);

  I.eye(J1.getCols());

  // Compute classical projection operator
  I_WpW = (I - WpW);

  m_first_iteration = false;
  return e;
}

vpColVector vpServo::computeControlLaw(double t, const vpColVector &e_dot_init)
{
  vpVelocityTwistMatrix cVa; // Twist transformation matrix
  vpMatrix aJe;              // Jacobian

  if (m_first_iteration) {
    if (testInitialization() == false) {
      vpERROR_TRACE("All the matrices are not correctly initialized");
      throw(vpServoException(vpServoException::servoError, "Cannot compute control law "
                             "All the matrices are not correctly"
                             "initialized"));
    }
  }
  if (testUpdated() == false) {
    vpERROR_TRACE("All the matrices are not correctly updated");
  }

  // test if all the required initialization have been done
  switch (servoType) {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined");
    throw(vpServoException(vpServoException::servoError, "No control law have been yet defined"));
    break;
  case EYEINHAND_CAMERA:
  case EYEINHAND_L_cVe_eJe:
  case EYETOHAND_L_cVe_eJe:

    cVa = cVe;
    aJe = eJe;

    init_cVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fVe_eJe:
    cVa = cVf * fVe;
    aJe = eJe;
    init_fVe = false;
    init_eJe = false;
    break;
  case EYETOHAND_L_cVf_fJe:
    cVa = cVf;
    aJe = fJe;
    init_fJe = false;
    break;
  }

  computeInteractionMatrix();
  computeError();

  // compute  task Jacobian
  J1 = L * cVa * aJe;

  // handle the eye-in-hand eye-to-hand case
  J1 *= signInteractionMatrix;

  // pseudo inverse of the task Jacobian
  // and rank of the task Jacobian
  // the image of J1 is also computed to allows the computation
  // of the projection operator
  vpMatrix imJ1t, imJ1;
  bool imageComputed = false;

  if (inversionType == PSEUDO_INVERSE) {
    rankJ1 = J1.pseudoInverse(J1p, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);

    imageComputed = true;
  }
  else
    J1p = J1.t();

  if (rankJ1 == J1.getCols()) {
    /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
    e1 = J1p * error; // primary task

    WpW.eye(J1.getCols());
  }
  else {
    if (imageComputed != true) {
      vpMatrix Jtmp;
      // image of J1 is computed to allows the computation
      // of the projection operator
      rankJ1 = J1.pseudoInverse(Jtmp, sv, m_pseudo_inverse_threshold, imJ1, imJ1t);
    }
    WpW = imJ1t.AAt();

#ifdef DEBUG
    std::cout << "rank J1 " << rankJ1 << std::endl;
    std::cout << "imJ1t" << std::endl << imJ1t;
    std::cout << "imJ1" << std::endl << imJ1;

    std::cout << "WpW" << std::endl << WpW;
    std::cout << "J1" << std::endl << J1;
    std::cout << "J1p" << std::endl << J1p;
#endif
    e1 = WpW * J1p * error;
  }

  // memorize the initial e1 value if the function is called the first time
  // or if the time given as parameter is equal to 0.
  if (m_first_iteration || std::fabs(t) < std::numeric_limits<double>::epsilon()) {
    e1_initial = e1;
  }
  // Security check. If size of e1_initial and e1 differ, that means that
  // e1_initial was not set
  if (e1_initial.getRows() != e1.getRows())
    e1_initial = e1;

  e = -lambda(e1) * e1 + (e_dot_init + lambda(e1) * e1_initial) * exp(-mu * t);

  I.eye(J1.getCols());

  // Compute classical projection operator
  I_WpW = (I - WpW);

  m_first_iteration = false;
  return e;
}

void vpServo::computeProjectionOperators(const vpMatrix &J1_, const vpMatrix &I_, const vpMatrix &I_WpW_,
                                         const vpColVector &error_, vpMatrix &P_) const
{
  // Initialization
  unsigned int n = J1_.getCols();
  P_.resize(n, n);

  // Compute gain depending by the task error to ensure a smooth change
  // between the operators.
  double e0_ = 0.1;
  double e1_ = 0.7;
  double sig = 0.0;

  double norm_e = error_.frobeniusNorm();
  if (norm_e > e1_)
    sig = 1.0;
  else if (e0_ <= norm_e && norm_e <= e1_)
    sig = 1.0 / (1.0 + exp(-12.0 * ((norm_e - e0_) / ((e1_ - e0_))) + 6.0));
  else
    sig = 0.0;

  vpMatrix eT_J = error_.t() * J1_;
  vpMatrix eT_J_JT_e = eT_J.AAt();
  double pp = eT_J_JT_e[0][0];

  vpMatrix P_norm_e = I_ - (1.0 / pp) * eT_J.AtA();

  P_ = sig * P_norm_e + (1 - sig) * I_WpW_;

  return;
}

vpColVector vpServo::secondaryTask(const vpColVector &de2dt, const bool &useLargeProjectionOperator)
{
  vpColVector sec;

  if (!useLargeProjectionOperator) {
    if (rankJ1 == J1.getCols()) {
      vpERROR_TRACE("no degree of freedom is free, cannot use secondary task");
      throw(vpServoException(vpServoException::noDofFree, "no degree of freedom is free, cannot use secondary task"));
    }
    else {
#if 0
      // computed in computeControlLaw()
      vpMatrix I;

      I.resize(J1.getCols(), J1.getCols());
      I.setIdentity();
      I_WpW = (I - WpW);
#endif
      //    std::cout << "I-WpW" << std::endl << I_WpW <<std::endl ;
      sec = I_WpW * de2dt;
    }
  }

  else {
    computeProjectionOperators(J1, I, I_WpW, error, P);

    sec = P * de2dt;
  }

  return sec;
}

vpColVector vpServo::secondaryTask(const vpColVector &e2, const vpColVector &de2dt,
                                   const bool &useLargeProjectionOperator)
{
  vpColVector sec;

  if (!useLargeProjectionOperator) {
    if (rankJ1 == J1.getCols()) {
      vpERROR_TRACE("no degree of freedom is free, cannot use secondary task");
      throw(vpServoException(vpServoException::noDofFree, "no degree of freedom is free, cannot use secondary task"));
    }
    else {

#if 0
      // computed in computeControlLaw()
      vpMatrix I;

      I.resize(J1.getCols(), J1.getCols());
      I.setIdentity();

      I_WpW = (I - WpW);
#endif

      // To be coherent with the primary task the gain must be the same
      // between primary and secondary task.
      sec = -lambda(e1) * I_WpW * e2 + I_WpW * de2dt;
    }
  }
  else {
    computeProjectionOperators(J1, I, I_WpW, error, P);

    sec = -lambda(e1) * P * e2 + P * de2dt;
  }

  return sec;
}

vpColVector vpServo::secondaryTaskJointLimitAvoidance(const vpColVector &q, const vpColVector &dq,
                                                      const vpColVector &qmin, const vpColVector &qmax,
                                                      const double &rho, const double &rho1, const double &lambda_tune)
{
  unsigned int const n = J1.getCols();

  if (qmin.size() != n || qmax.size() != n) {
    std::stringstream msg;
    msg << "Dimension vector qmin (" << qmin.size()
      << ") or qmax () does not correspond to the number of jacobian "
      "columns";
    msg << "qmin size: " << qmin.size() << std::endl;
    throw(vpServoException(vpServoException::dimensionError, msg.str()));
  }
  if (q.size() != n || dq.size() != n) {
    vpERROR_TRACE("Dimension vector q or dq does not correspont to the "
      "number of jacobian columns");
    throw(vpServoException(vpServoException::dimensionError, "Dimension vector q or dq does not correspont to "
                           "the number of jacobian columns"));
  }

  double lambda_l = 0.0;

  vpColVector q2(n);

  vpColVector q_l0_min(n);
  vpColVector q_l0_max(n);
  vpColVector q_l1_min(n);
  vpColVector q_l1_max(n);

  // Computation of gi ([nx1] vector) and lambda_l ([nx1] vector)
  vpMatrix g(n, n);
  vpColVector q2_i(n);

  computeProjectionOperators(J1, I, I_WpW, error, P);

  for (unsigned int i = 0; i < n; i++) {
    q_l0_min[i] = qmin[i] + rho * (qmax[i] - qmin[i]);
    q_l0_max[i] = qmax[i] - rho * (qmax[i] - qmin[i]);

    q_l1_min[i] = q_l0_min[i] - rho * rho1 * (qmax[i] - qmin[i]);
    q_l1_max[i] = q_l0_max[i] + rho * rho1 * (qmax[i] - qmin[i]);

    if (q[i] < q_l0_min[i])
      g[i][i] = -1;
    else if (q[i] > q_l0_max[i])
      g[i][i] = 1;
    else
      g[i][i] = 0;
  }

  for (unsigned int i = 0; i < n; i++) {
    if (q[i] > q_l0_min[i] && q[i] < q_l0_max[i])
      q2_i = 0 * q2_i;

    else {
      vpColVector Pg_i(n);
      Pg_i = (P * g.getCol(i));
      double b = (vpMath::abs(dq[i])) / (vpMath::abs(Pg_i[i]));

      if (b < 1.) // If the ratio b is big we don't activate the joint
        // avoidance limit for the joint.
      {
        if (q[i] < q_l1_min[i] || q[i] > q_l1_max[i])
          q2_i = -(1 + lambda_tune) * b * Pg_i;

        else {
          if (q[i] >= q_l0_max[i] && q[i] <= q_l1_max[i])
            lambda_l = 1 / (1 + exp(-12 * ((q[i] - q_l0_max[i]) / (q_l1_max[i] - q_l0_max[i])) + 6));

          else if (q[i] >= q_l1_min[i] && q[i] <= q_l0_min[i])
            lambda_l = 1 / (1 + exp(-12 * ((q[i] - q_l0_min[i]) / (q_l1_min[i] - q_l0_min[i])) + 6));

          q2_i = -lambda_l * (1 + lambda_tune) * b * Pg_i;
        }
      }
    }
    q2 = q2 + q2_i;
  }
  return q2;
}
