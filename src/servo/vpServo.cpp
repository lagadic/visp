/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Visual servoing control law.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard
 * Fabien Spindler
 *
 *****************************************************************************/


#include <visp/vpServo.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

/*!
  \file vpServo.cpp
  \brief  Class required to compute the visual servoing control law
*/


/*!
  Default constructor that initializes the following settings:
  - No control law is specified. The user has to call setServo() to specify the control law.
  - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is computed with the desired
    features \f${\bf s}^*\f$. Using setInteractionMatrixType() you can also compute the
    interaction matrix with the current visual features, or from the mean
    \f$\left({\widehat {\bf L}}_s + {\widehat {\bf L}}_{s^*}\right)/2\f$.
  - In the control law the pseudo inverse will be used. The method setInteractionMatrixType()
    allows to use the transpose instead.

*/
vpServo::vpServo() 
  : L(), error(), J1(), J1p(), s(), sStar(), e1(), e(), q_dot(), v(), servoType(vpServo::NONE),
    rankJ1(0), featureList(), desiredFeatureList(), featureSelectionList(), lambda(), signInteractionMatrix(1),
    interactionMatrixType(DESIRED), inversionType(PSEUDO_INVERSE), cVe(), init_cVe(false),
    cVf(), init_cVf(false), fVe(), init_fVe(false), eJe(), init_eJe(false), fJe(), init_fJe(false),
    errorComputed(false), interactionMatrixComputed(false), dim_task(0), taskWasKilled(false),
    forceInteractionMatrixComputation(false), WpW(), I_WpW(), sv(), mu(4.)
{
}
/*!
  Constructor that allows to choose the visual servoing control law.
  \param servo_type : Visual servoing control law.

  The other settings are the following:
  - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is computed with the desired
    features \f${\bf s}^*\f$. Using setInteractionMatrixType() you can also compute the
    interaction matrix with the current visual features, or from the mean
    \f$\left({\widehat {\bf L}}_s + {\widehat {\bf L}}_{s^*}\right)/2\f$.
  - In the control law the pseudo inverse will be used. The method setInteractionMatrixType()
    allows to use the transpose instead.

 */
vpServo::vpServo(vpServoType servo_type)
  : L(), error(), J1(), J1p(), s(), sStar(), e1(), e(), q_dot(), v(), servoType(servo_type),
    rankJ1(0), featureList(), desiredFeatureList(), featureSelectionList(), lambda(), signInteractionMatrix(1),
    interactionMatrixType(DESIRED), inversionType(PSEUDO_INVERSE), cVe(), init_cVe(false),
    cVf(), init_cVf(false), fVe(), init_fVe(false), eJe(), init_eJe(false), fJe(), init_fJe(false),
    errorComputed(false), interactionMatrixComputed(false), dim_task(0), taskWasKilled(false),
    forceInteractionMatrixComputation(false), WpW(), I_WpW(), sv(), mu(4)
{
}

/*!
  Destructor.

  In fact, it does nothing. You have to call kill() to destroy the
  current and desired feature lists.

  \sa kill()
*/
//  \exception vpServoException::notKilledProperly : Task was not killed
//  properly. That means that you should explitly call kill().

vpServo::~vpServo()
{
  if (taskWasKilled == false) {
    vpTRACE("--- Begin Warning Warning Warning Warning Warning ---");
    vpTRACE("--- You should explicitly call vpServo.kill()...  ---");
    vpTRACE("--- End Warning Warning Warning Warning Warning   ---");
    //     throw(vpServoException(vpServoException::notKilledProperly,
    // 			   "Task was not killed properly"));
  }
}


/*!
  Initialize the servo with the following settings:

  - No control law is specified. The user has to call setServo() to specify the control law.
  - In the control law, the interaction matrix \f${\widehat {\bf L}}_e \f$ is computed with the desired
    features \f${\bf s}^*\f$. Using setInteractionMatrixType() you can also compute the
    interaction matrix with the current visual features, or from the mean
    \f$\left({\widehat {\bf L}}_s + {\widehat {\bf L}}_{s^*}\right)/2\f$.
  - In the control law the pseudo inverse will be used. The method setInteractionMatrixType()
    allows to use the transpose instead.


*/
void vpServo::init()
{
  // type of visual servoing
  servoType = vpServo::NONE ;

  // Twist transformation matrix
  init_cVe = false ;
  init_cVf = false ;
  init_fVe = false ;
  // Jacobians
  init_eJe = false ;
  init_fJe = false ;

  dim_task = 0 ;

  featureList.clear() ;
  desiredFeatureList.clear() ;
  featureSelectionList.clear();

  signInteractionMatrix = 1 ;

  interactionMatrixType = DESIRED ;
  inversionType=PSEUDO_INVERSE ;

  interactionMatrixComputed = false ;
  errorComputed = false ;

  taskWasKilled = false;

  forceInteractionMatrixComputation = false;

  rankJ1 = 0;
}

/*!
  Task destruction. Kill the current and desired visual feature lists.

  It is mendatory to call explicitly this function to avoid potential
  memory leaks.

  \code
  vpServo task ;
  vpFeatureThetaU s;
  ...
  task.addFeature(s); // Add current ThetaU feature

  task.kill(); // A call to kill() is requested here
  \endcode

*/
void vpServo::kill()
{
  if (taskWasKilled == false) {
    // kill the current and desired feature lists

    // current list
    for(std::list<vpBasicFeature *>::iterator it = featureList.begin(); it != featureList.end(); ++it) {
      if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
        delete (*it) ;
        (*it) = NULL ;
      }
    }
      //desired list
    for(std::list<vpBasicFeature *>::iterator it = desiredFeatureList.begin(); it != desiredFeatureList.end(); ++it) {
      if ((*it)->getDeallocate() == vpBasicFeature::vpServo) {
        delete (*it) ;
        (*it) = NULL ;
      }
    }

    featureList.clear() ;
    desiredFeatureList.clear() ;
    taskWasKilled = true;
  }
}

/*!
  Set the visual servoing control law.
  \param servo_type : Control law that will be considered.
  See vpServo::vpServoType to see the possible values.
 */
void vpServo::setServo(const vpServoType &servo_type)
{
  this->servoType = servo_type ;

  if ((servoType==EYEINHAND_CAMERA) ||(servoType==EYEINHAND_L_cVe_eJe))
    signInteractionMatrix = 1 ;
  else
    signInteractionMatrix = -1 ;

  // when the control is directly compute in the camera frame
  // we relieve the end-user to initialize cVa and aJe
  if (servoType==EYEINHAND_CAMERA)
  {
    vpVelocityTwistMatrix _cVe ; set_cVe(_cVe) ;

    vpMatrix _eJe ;
    _eJe.eye(6) ;
    set_eJe(_eJe) ;
  };
}


/*!

  Prints on \e os stream information about the task:

  \param displayLevel : Indicates which are the task informations to print. See vpServo::vpServoPrintType for more details.

  \param os : Output stream.
*/
void
    vpServo::print(const vpServo::vpServoPrintType displayLevel, std::ostream &os)
{
  switch (displayLevel)
  {
  case vpServo::ALL:
    {
      os << "Visual servoing task: " <<std::endl ;

      os << "Type of control law " <<std::endl ;
      switch( servoType )
      {
      case NONE :
        os << "Type of task have not been chosen yet ! " << std::endl ;
        break ;
      case EYEINHAND_CAMERA :
        os << "Eye-in-hand configuration " << std::endl ;
        os << "Control in the camera frame " << std::endl ;
        break ;
      case EYEINHAND_L_cVe_eJe :
        os << "Eye-in-hand configuration " << std::endl ;
        os << "Control in the articular frame " << std::endl ;
        break ;
      case EYETOHAND_L_cVe_eJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVe_eJe q_dot " << std::endl ;
        break ;
      case EYETOHAND_L_cVf_fVe_eJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVe_fVe_eJe q_dot " << std::endl ;
        break ;
      case EYETOHAND_L_cVf_fJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVf_fJe q_dot " << std::endl ;
        break ;
      }

      os << "List of visual features : s" <<std::endl ;
      std::list<vpBasicFeature *>::const_iterator it_s;
      std::list<vpBasicFeature *>::const_iterator it_s_star;
      std::list<unsigned int>::const_iterator it_select;

      for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end(); ++it_s, ++it_select)
      {
        os << "" ;
        (*it_s)->print( (*it_select) ) ;
      }

      os << "List of desired visual features : s*" <<std::endl ;
      for (it_s_star = desiredFeatureList.begin(), it_select = featureSelectionList.begin(); it_s_star != desiredFeatureList.end(); ++it_s_star, ++it_select)
      {
        os << "" ;
        (*it_s_star)->print( (*it_select) ) ;
      }

      os <<"Interaction Matrix Ls "<<std::endl  ;
      if (interactionMatrixComputed)
      {
        os << L << std::endl;
      }
      else
      {
        os << "not yet computed "<<std::endl ;
      }

      os <<"Error vector (s-s*) "<<std::endl  ;
      if (errorComputed)
      {
        os << error.t() << std::endl;
      }
      else
      {
        os << "not yet computed "<<std::endl ;
      }

      os << "Gain : " << lambda <<std::endl ;

      break;
    }

  case vpServo::CONTROLLER:
    {
      os << "Type of control law " <<std::endl ;
      switch( servoType )
      {
      case NONE :
        os << "Type of task have not been chosen yet ! " << std::endl ;
        break ;
      case EYEINHAND_CAMERA :
        os << "Eye-in-hand configuration " << std::endl ;
        os << "Control in the camera frame " << std::endl ;
        break ;
      case EYEINHAND_L_cVe_eJe :
        os << "Eye-in-hand configuration " << std::endl ;
        os << "Control in the articular frame " << std::endl ;
        break ;
      case EYETOHAND_L_cVe_eJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVe_eJe q_dot " << std::endl ;
        break ;
      case EYETOHAND_L_cVf_fVe_eJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVe_fVe_eJe q_dot " << std::endl ;
        break ;
      case EYETOHAND_L_cVf_fJe :
        os << "Eye-to-hand configuration " << std::endl ;
        os << "s_dot = _L_cVf_fJe q_dot " << std::endl ;
        break ;
      }
      break;
    }

  case vpServo::FEATURE_CURRENT:
    {
      os << "List of visual features : s" <<std::endl ;

      std::list<vpBasicFeature *>::const_iterator it_s;
      std::list<unsigned int>::const_iterator it_select;

      for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end(); ++it_s, ++it_select)
      {
        os << "" ;
        (*it_s)->print( (*it_select) ) ;
      }
      break;
    }
  case vpServo::FEATURE_DESIRED:
    {
      os << "List of desired visual features : s*" <<std::endl ;

      std::list<vpBasicFeature *>::const_iterator it_s_star;
      std::list<unsigned int>::const_iterator it_select;

      for (it_s_star = desiredFeatureList.begin(), it_select = featureSelectionList.begin(); it_s_star != desiredFeatureList.end(); ++it_s_star, ++it_select)
      {
        os << "" ;
        (*it_s_star)->print( (*it_select) ) ;
      }
      break;
    }
  case vpServo::GAIN:
    {
      os << "Gain : " << lambda <<std::endl ;
      break;
    }
  case vpServo::INTERACTION_MATRIX:
    {
      os <<"Interaction Matrix Ls "<<std::endl  ;
      if (interactionMatrixComputed) {
        os << L << std::endl;
      }
      else {
        os << "not yet computed "<<std::endl ;
      }
      break;
    }

  case vpServo::ERROR_VECTOR:
  case vpServo::MINIMUM:

    {
      os <<"Error vector (s-s*) "<<std::endl  ;
      if (errorComputed) {
        os << error.t() << std::endl;
      }
      else {
        os << "not yet computed "<<std::endl ;
      }

      break;
    }
  }
}

/*!
  Add a new set of 2 features \f$\bf s\f$ and \f${\bf s}^*\f$ in the task.

  \param s_cur : Current visual feature denoted \f$\bf s\f$.
  \param s_star : Desired visual feature denoted \f${\bf s}^*\f$.
  \param select : Feature selector. By default all the features in \e s and \e s_star are used,
  but is is possible to specify which one is used in case of multiple features.

  The following sample code explain how to use this method to add a visual feature point \f$(x,y)\f$:
  \code
  vpFeaturePoint s, s_star;
  ...
  vpServo task;
  task.addFeature(s, s_star);
  \endcode

  For example to use only the \f$x\f$ visual feature, the previous code becomes:
  \code
  vpFeaturePoint s, s_star;
  ...
  vpServo task;
  task.addFeature(s, s_star, vpFeaturePoint::selectX());
  \endcode

  */
void vpServo::addFeature(vpBasicFeature& s_cur, vpBasicFeature &s_star, unsigned int select)
{
  featureList.push_back( &s_cur );
  desiredFeatureList.push_back( &s_star );
  featureSelectionList.push_back( select );
}

/*!
  Add a new features \f$\bf s\f$ in the task. The desired visual feature denoted \f${\bf s}^*\f$ is equal to zero.

  \param s_cur : Current visual feature denoted \f$\bf s\f$.
  \param select : Feature selector. By default all the features in \e s are used,
  but is is possible to specify which one is used in case of multiple features.

  The following sample code explain how to use this method to add a \f$\theta {\bf u} =(\theta u_x, \theta u_y, \theta u_z)\f$ feature:
  \code
  vpFeatureThetaU s(vpFeatureThetaU::cRcd);
  ...
  vpServo task;
  task.addFeature(s);
  \endcode

  For example to use only the \f$\theta u_x\f$ feature, the previous code becomes:
  \code
  vpFeatureThetaU s(vpFeatureThetaU::cRcd);
  ...
  vpServo task;
  task.addFeature(s, vpFeatureThetaU::selectTUx);
  \endcode
  */
void vpServo::addFeature(vpBasicFeature& s_cur, unsigned int select)
{
  featureList.push_back( &s_cur );

  // in fact we have a problem with s_star that is not defined
  // by the end user.

  // If the same user want to compute the interaction at the desired position
  // this "virtual feature" must exist

  // a solution is then to duplicate the current feature (s* = s)
  // and reinitialized s* to 0

  // it remains the deallocation issue therefore a flag that stipulates that
  // the feature has been allocated in vpServo is set

  // vpServo must deallocate the memory (see ~vpServo and kill() )

  vpBasicFeature *s_star ;
  s_star = s_cur.duplicate()  ;

  s_star->init() ;
  s_star->setDeallocate(vpBasicFeature::vpServo)  ;

  desiredFeatureList.push_back( s_star );
  featureSelectionList.push_back( select );
}

//! Return the task dimension.
unsigned int vpServo::getDimension() const
{
  unsigned int dim = 0 ;
  std::list<vpBasicFeature *>::const_iterator it_s;
  std::list<unsigned int>::const_iterator it_select;

  for (it_s = featureList.begin(), it_select = featureSelectionList.begin(); it_s != featureList.end(); ++it_s, ++it_select)
  {
    dim += (*it_s)->getDimension(*it_select) ;
  }

  return dim;
}

void vpServo::setInteractionMatrixType(const vpServoIteractionMatrixType &interactionMatrix_type,
                                       const vpServoInversionType &interactionMatrixInversion)
{
  this->interactionMatrixType = interactionMatrix_type ;
  this->inversionType = interactionMatrixInversion ;
}


static void computeInteractionMatrixFromList  (const std::list<vpBasicFeature *> & featureList,
                                               const std::list<unsigned int> & featureSelectionList,
                                               vpMatrix & L)
{
  if (featureList.empty())
  {
    vpERROR_TRACE("feature list empty, cannot compute Ls") ;
    throw(vpServoException(vpServoException::noFeatureError,
                           "feature list empty, cannot compute Ls")) ;
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
    L .resize(rowL, colL);
  }

  /* vectTmp is used to store the return values of functions get_s() and
   * error(). */
  vpMatrix matrixTmp;
  unsigned int rowMatrixTmp, colMatrixTmp;

  /* The cursor are the number of the next case of the vector array to
   * be affected. A memory reallocation should be done when cursor
   * is out of the vector-array range.*/
  unsigned int cursorL = 0;

  std::list<vpBasicFeature *>::const_iterator it;
  std::list<unsigned int>::const_iterator it_select;

  for (it = featureList.begin(), it_select = featureSelectionList.begin(); it != featureList.end(); ++it, ++it_select)
  {
    /* Get s. */
    matrixTmp = (*it)->interaction( *it_select );
    rowMatrixTmp = matrixTmp .getRows();
    colMatrixTmp = matrixTmp .getCols();

    /* Check the matrix L size, and realloc if needed. */
    while (rowMatrixTmp + cursorL > rowL) {
      rowL *= 2;
      L.resize (rowL,colL,false);
      vpDEBUG_TRACE(15,"Realloc!");
    }

    /* Copy the temporarily matrix into L. */
    for (unsigned int k = 0; k < rowMatrixTmp; ++k, ++cursorL) {
      for (unsigned int j = 0; j <  colMatrixTmp; ++j) {
        L[cursorL][j] = matrixTmp[k][j];
      }
    }
  }

  L.resize (cursorL,colL,false);

  return ;
}


/*!

  Compute and return the interaction matrix related to the set of visual features.

  \return The interaction matrix \f${\widehat {\bf L}}_e\f$ used in the control law specified using setServo().
*/
vpMatrix vpServo::computeInteractionMatrix()
{
  try {

    switch (interactionMatrixType)
    {
    case CURRENT:
      {
        try
        {
          computeInteractionMatrixFromList(this ->featureList,
                                           this ->featureSelectionList,
                                           L);
          dim_task = L.getRows() ;
          interactionMatrixComputed = true ;
        }

        catch(vpException me)
        {
          vpERROR_TRACE("Error caught") ;
          throw ;
        }
      }
      break ;
    case DESIRED:
      {
        try
        {
          if (interactionMatrixComputed == false || forceInteractionMatrixComputation == true)
          {
            computeInteractionMatrixFromList(this ->desiredFeatureList,
                                             this ->featureSelectionList, L);

            dim_task = L.getRows() ;
            interactionMatrixComputed = true ;
          }

        }
        catch(vpException me)
        {
          vpERROR_TRACE("Error caught") ;
          throw ;
        }
      }
      break ;
    case MEAN:
      {
        vpMatrix Lstar (L.getRows(), L.getCols());
        try
        {
          computeInteractionMatrixFromList(this ->featureList,
                                           this ->featureSelectionList, L);
          computeInteractionMatrixFromList(this ->desiredFeatureList,
                                           this ->featureSelectionList, Lstar);
        }
        catch(vpException me)
        {
          vpERROR_TRACE("Error caught") ;
          throw ;
        }
        L = (L+Lstar)/2;

        dim_task = L.getRows() ;
        interactionMatrixComputed = true ;
      }
      break ;
    case USER_DEFINED:
      // dim_task = L.getRows() ;
      interactionMatrixComputed = false ;
      break;
    }

  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  return L ;
}

/*! 

  Compute the error \f$\bf e =(s - s^*)\f$ between the current set of visual
  features \f$\bf s\f$ and the desired set of visual features \f$\bf s^*\f$.

  \return The error vector \f$\bf e\f$.

*/
vpColVector vpServo::computeError()
{
  if (featureList.empty())
  {
    vpERROR_TRACE("feature list empty, cannot compute Ls") ;
    throw(vpServoException(vpServoException::noFeatureError,
                           "feature list empty, cannot compute Ls")) ;
  }
  if (desiredFeatureList.empty())
  {
    vpERROR_TRACE("feature list empty, cannot compute Ls") ;
    throw(vpServoException(vpServoException::noFeatureError,
                           "feature list empty, cannot compute Ls")) ;
  }

  try {
    vpBasicFeature *current_s ;
    vpBasicFeature *desired_s ;

    /* The vector dimensions are not known before the affectation loop.
     * They thus should be allocated on the flight, in the loop.
     * The first assumption is that the size has not changed. A double
     * reallocation (realloc(dim*2)) is done if necessary. In particular,
     * [log_2(dim)+1] reallocations are done for the first error computation.
     * If the allocated size is too large, a correction is done after the loop.
     * The algorithmic cost is linear in affectation, logarithmic in allocation
     * numbers and linear in allocation size.
     * No assumptions are made concerning size of each vector: they are
     * not said equal, and could be different.
     */

    /* First assumption: vector dimensions have not changed. If 0, they are
     * initialized to dim 1.*/
    unsigned int dimError = error .getRows();
    unsigned int dimS = s .getRows();
    unsigned int dimSStar = sStar .getRows();
    if (0 == dimError) { dimError = 1; error .resize(dimError);}
    if (0 == dimS) { dimS = 1; s .resize(dimS);}
    if (0 == dimSStar) { dimSStar = 1; sStar .resize(dimSStar);}

    /* vectTmp is used to store the return values of functions get_s() and
     * error(). */
    vpColVector vectTmp;
    unsigned int dimVectTmp;

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
         it_s != featureList.end();
         ++it_s, ++it_s_star, ++it_select)
    {
      current_s  = (*it_s);
      desired_s  = (*it_s_star);
      unsigned int select = (*it_select);

      /* Get s, and store it in the s vector. */
      vectTmp = current_s->get_s(select);
      dimVectTmp = vectTmp .getRows();
      while (dimVectTmp + cursorS > dimS)
      { dimS *= 2; s .resize (dimS,false); vpDEBUG_TRACE(15,"Realloc!"); }
      for (unsigned int k = 0; k <  dimVectTmp; ++k) { s[cursorS++] = vectTmp[k]; }

      /* Get s_star, and store it in the s vector. */
      vectTmp = desired_s->get_s(select);
      dimVectTmp = vectTmp .getRows();
      while (dimVectTmp + cursorSStar > dimSStar) {
        dimSStar *= 2;
        sStar.resize (dimSStar,false);
      }
      for (unsigned int k = 0; k <  dimVectTmp; ++k) {
        sStar[cursorSStar++] = vectTmp[k];
      }

      /* Get error, and store it in the s vector. */
      vectTmp = current_s->error(*desired_s, select) ;
      dimVectTmp = vectTmp .getRows();
      while (dimVectTmp + cursorError > dimError) {
        dimError *= 2;
        error.resize (dimError,false);
      }
      for (unsigned int k = 0; k <  dimVectTmp; ++k) {
        error[cursorError++] = vectTmp[k];
      }
    }

    /* If too much memory has been allocated, realloc. */
    s .resize(cursorS,false);
    sStar .resize(cursorSStar,false);
    error .resize(cursorError,false);

    /* Final modifications. */
    dim_task = error.getRows() ;
    errorComputed = true ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  catch(...)
  {
    throw ;
  }
  return error ;
}

bool vpServo::testInitialization()
{
  switch (servoType)
  {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined") ;
    throw(vpServoException(vpServoException::servoError,
                           "No control law have been yet defined")) ;
    break ;
  case EYEINHAND_CAMERA:
    return true ;
    break ;
  case EYEINHAND_L_cVe_eJe:
  case  EYETOHAND_L_cVe_eJe:
    if (!init_cVe) vpERROR_TRACE("cVe not initialized") ;
    if (!init_eJe) vpERROR_TRACE("eJe not initialized") ;
    return (init_cVe && init_eJe) ;
    break ;
  case  EYETOHAND_L_cVf_fVe_eJe:
    if (!init_cVf) vpERROR_TRACE("cVf not initialized") ;
    if (!init_fJe) vpERROR_TRACE("fVe not initialized") ;
    if (!init_eJe) vpERROR_TRACE("eJe not initialized") ;
    return (init_cVf && init_fVe && init_eJe) ;
    break ;

  case EYETOHAND_L_cVf_fJe    :
    if (!init_cVf) vpERROR_TRACE("cVf not initialized") ;
    if (!init_fJe) vpERROR_TRACE("fJe not initialized") ;
    return (init_cVf && init_fJe) ;
    break ;
  }

  return false ;
}
bool vpServo::testUpdated()
{
  switch (servoType)
  {
  case NONE:
    vpERROR_TRACE("No control law have been yet defined") ;
    throw(vpServoException(vpServoException::servoError,
                           "No control law have been yet defined")) ;
    break ;
  case EYEINHAND_CAMERA:
    return true ;
  case EYEINHAND_L_cVe_eJe:
    if (!init_eJe) vpERROR_TRACE("eJe not updated") ;
    return (init_eJe) ;
    break ;
  case  EYETOHAND_L_cVe_eJe:
    if (!init_cVe) vpERROR_TRACE("cVe not updated") ;
    if (!init_eJe) vpERROR_TRACE("eJe not updated") ;
    return (init_cVe && init_eJe) ;
    break ;
  case  EYETOHAND_L_cVf_fVe_eJe:
    if (!init_fVe) vpERROR_TRACE("fVe not updated") ;
    if (!init_eJe) vpERROR_TRACE("eJe not updated") ;
    return (init_fVe && init_eJe) ;
    break ;

  case EYETOHAND_L_cVf_fJe    :
    if (!init_fJe) vpERROR_TRACE("fJe not updated") ;
    return (init_fJe) ;
    break ;
  }

  return false ;
}
/*!

  Compute the control law specified using setServo(). See vpServo::vpServoType for more
  details concerning the control laws that are available. The \ref tutorial-ibvs and \ref tutorial-boost-vs
  are also useful to illustrate the usage of this function.

  The general form of the control law is the following:

  \f[
  {\bf \dot q}  = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e}
  \f]

  where :
  - \f${\bf \dot q}\f$ is the resulting velocity command to apply to the robot.
  - the sign of the control law depends on the eye in hand or eye to hand configuration.
  - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction matrix and of the robot
  Jacobian.
  - \f$\bf e = (s-s^*)\f$ is the error to regulate.

  To ensure continuous sequencing the computeControlLaw(double) function can be used. It will ensure that
  the velocities that are computed are continuous.
*/
vpColVector vpServo::computeControlLaw()
{
  static int iteration =0;

  try
  {
    vpVelocityTwistMatrix cVa ; // Twist transformation matrix
    vpMatrix aJe ;      // Jacobian

    if (iteration==0)
    {
      if (testInitialization() == false) {
        vpERROR_TRACE("All the matrices are not correctly initialized") ;
        throw(vpServoException(vpServoException::servoError,
                               "Cannot compute control law "
                               "All the matrices are not correctly"
                               "initialized")) ;
      }
    }
    if (testUpdated() == false) {
      vpERROR_TRACE("All the matrices are not correctly updated") ;
    }

    // test if all the required initialization have been done
    switch (servoType)
    {
    case NONE :
      vpERROR_TRACE("No control law have been yet defined") ;
      throw(vpServoException(vpServoException::servoError,
                             "No control law have been yet defined")) ;
      break ;
    case EYEINHAND_CAMERA:
    case EYEINHAND_L_cVe_eJe:
    case EYETOHAND_L_cVe_eJe:

      cVa = cVe ;
      aJe = eJe ;

      init_cVe = false ;
      init_eJe = false ;
      break ;
    case  EYETOHAND_L_cVf_fVe_eJe:
      cVa = cVf*fVe ;
      aJe = eJe ;
      init_fVe = false ;
      init_eJe = false ;
      break ;
    case EYETOHAND_L_cVf_fJe    :
      cVa = cVf ;
      aJe = fJe ;
      init_fJe = false ;
      break ;
    }

    computeInteractionMatrix() ;
    computeError() ;

    // compute  task Jacobian
    J1 = L*cVa*aJe ;

    // handle the eye-in-hand eye-to-hand case
    J1 *= signInteractionMatrix ;

    // pseudo inverse of the task Jacobian
    // and rank of the task Jacobian
    // the image of J1 is also computed to allows the computation
    // of the projection operator
    vpMatrix imJ1t, imJ1 ;
    bool imageComputed = false ;

    if (inversionType==PSEUDO_INVERSE)
    {
      rankJ1 = J1.pseudoInverse(J1p, sv, 1e-6, imJ1, imJ1t) ;

      imageComputed = true ;
    }
    else
      J1p = J1.t() ;

    if (rankJ1 == L.getCols())
    {
      /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
      e1 = J1p*error ;// primary task

      WpW.resize(J1.getCols(), J1.getCols()) ;
      WpW.setIdentity() ;
    }
    else
    {
      if (imageComputed!=true)
      {
        vpMatrix Jtmp ;
        // image of J1 is computed to allows the computation
        // of the projection operator
        rankJ1 = J1.pseudoInverse(Jtmp, sv, 1e-6, imJ1, imJ1t) ;
        imageComputed = true ;
      }
      WpW = imJ1t*imJ1t.t() ;

#ifdef DEBUG
      std::cout << "rank J1 " << rankJ1 <<std::endl ;
      std::cout << "imJ1t"<<std::endl  << imJ1t ;
      std::cout << "imJ1"<<std::endl  << imJ1 ;

      std::cout << "WpW" <<std::endl <<WpW  ;
      std::cout << "J1" <<std::endl <<J1  ;
      std::cout << "J1p" <<std::endl <<J1p  ;
#endif
      e1 = WpW*J1p*error ;
    }
    e = - lambda(e1) * e1 ;

    vpMatrix I ;

    I.resize(J1.getCols(),J1.getCols()) ;
    I.setIdentity() ;

    I_WpW = (I - WpW) ;
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("Caught a matrix related error") ;
    std::cout << me << std::endl ;
    throw me;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw me ;
  }

  iteration++ ;
  return e ;
}

/*!
  Compute the control law specified using setServo(). See vpServo::vpServoType for more
  details concerning the control laws that are available. The \ref tutorial-boost-vs is also useful to
  illustrate the usage of this function.

  To the general form of the control law given in computeControlLaw(), we add here an additional term that comes from
  the task sequencing approach described in \cite Mansard07e equation (17). This additional term allows to compute
  continuous velocities by avoiding abrupt changes in the command.

  The form of the control law considered here is the following:

  \f[
  {\bf \dot q} = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e} \mp \lambda {{\bf \widehat J}_{e(0)}}^+ {{\bf e}(0)} \exp(-\mu t)
  \f]

  where :
  - \f${\bf \dot q}\f$ is the resulting continuous velocity command to apply to the robot.
  - the sign of the control law depends on the eye in hand or eye to hand configuration.
  - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction matrix and of the robot
  Jacobian.
  - \f$\bf e = (s-s^*)\f$ is the error to regulate.
  - \f$t\f$ is the time given as parameter of this method.
  - \f$\mu\f$ is a gain that is set by default to 4 and that could be modified using setMu().
  - \f${\bf \widehat J}_{e(0)}^+ {\bf e}(0)\f$ is the value of \f${\bf \widehat J}_e^+ {\bf e}\f$ when \f$t=0\f$.
  This value is internally stored either at the first call of this method, or when \e t parameter is set to 0.

  \param t : Time in second. When set to zero, \f${{\bf \widehat J}_{e(0)}}^+ {{\bf e}(0)}\f$ is refreshed internally.
*/
vpColVector vpServo::computeControlLaw(double t)
{
  static int iteration =0;
  static vpColVector e1_initial;

  try
  {
    vpVelocityTwistMatrix cVa ; // Twist transformation matrix
    vpMatrix aJe ;      // Jacobian

    if (iteration==0)
    {
      if (testInitialization() == false) {
        vpERROR_TRACE("All the matrices are not correctly initialized") ;
        throw(vpServoException(vpServoException::servoError,
                               "Cannot compute control law "
                               "All the matrices are not correctly"
                               "initialized")) ;
      }
    }
    if (testUpdated() == false) {
      vpERROR_TRACE("All the matrices are not correctly updated") ;
    }

    // test if all the required initialization have been done
    switch (servoType)
    {
    case NONE :
      vpERROR_TRACE("No control law have been yet defined") ;
      throw(vpServoException(vpServoException::servoError,
                             "No control law have been yet defined")) ;
      break ;
    case EYEINHAND_CAMERA:
    case EYEINHAND_L_cVe_eJe:
    case EYETOHAND_L_cVe_eJe:

      cVa = cVe ;
      aJe = eJe ;

      init_cVe = false ;
      init_eJe = false ;
      break ;
    case  EYETOHAND_L_cVf_fVe_eJe:
      cVa = cVf*fVe ;
      aJe = eJe ;
      init_fVe = false ;
      init_eJe = false ;
      break ;
    case EYETOHAND_L_cVf_fJe    :
      cVa = cVf ;
      aJe = fJe ;
      init_fJe = false ;
      break ;
    }

    computeInteractionMatrix() ;
    computeError() ;

    // compute  task Jacobian
    J1 = L*cVa*aJe ;

    // handle the eye-in-hand eye-to-hand case
    J1 *= signInteractionMatrix ;

    // pseudo inverse of the task Jacobian
    // and rank of the task Jacobian
    // the image of J1 is also computed to allows the computation
    // of the projection operator
    vpMatrix imJ1t, imJ1 ;
    bool imageComputed = false ;

    if (inversionType==PSEUDO_INVERSE)
    {
      rankJ1 = J1.pseudoInverse(J1p, sv, 1e-6, imJ1, imJ1t) ;

      imageComputed = true ;
    }
    else
      J1p = J1.t() ;

    if (rankJ1 == L.getCols())
    {
      /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
      e1 = J1p*error ;// primary task

      WpW.resize(J1.getCols(), J1.getCols()) ;
      WpW.setIdentity() ;
    }
    else
    {
      if (imageComputed!=true)
      {
        vpMatrix Jtmp ;
        // image of J1 is computed to allows the computation
        // of the projection operator
        rankJ1 = J1.pseudoInverse(Jtmp, sv, 1e-6, imJ1, imJ1t) ;
        imageComputed = true ;
      }
      WpW = imJ1t*imJ1t.t() ;

#ifdef DEBUG
      std::cout << "rank J1 " << rankJ1 <<std::endl ;
      std::cout << "imJ1t"<<std::endl  << imJ1t ;
      std::cout << "imJ1"<<std::endl  << imJ1 ;

      std::cout << "WpW" <<std::endl <<WpW  ;
      std::cout << "J1" <<std::endl <<J1  ;
      std::cout << "J1p" <<std::endl <<J1p  ;
#endif
      e1 = WpW*J1p*error ;
    }

    // memorize the initial e1 value if the function is called the first time or if the time given as parameter is equal to 0.
    if (iteration==0 || std::fabs(t) < std::numeric_limits<double>::epsilon()) {
      e1_initial = e1;
    }

    e = - lambda(e1) * e1 + lambda(e1) * e1_initial*exp(-mu*t);

    vpMatrix I ;

    I.resize(J1.getCols(), J1.getCols()) ;
    I.setIdentity() ;

    I_WpW = (I - WpW) ;
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("Caught a matrix related error") ;
    std::cout << me << std::endl ;
    throw me;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw me ;
  }

  iteration++ ;
  return e ;
}

/*!
  Compute the control law specified using setServo(). See vpServo::vpServoType for more
  details concerning the control laws that are available.

  To the general form of the control law given in computeControlLaw(), we add here an additional term that comes from
  the task sequencing approach described in \cite Mansard07e equation (17). This additional term allows to compute
  continuous velocities by avoiding abrupt changes in the command.

  The form of the control law considered here is the following:

  \f[
  {\bf \dot q} = \pm \lambda {{\bf \widehat J}_e}^+ {\bf e} + \left({\bf \dot e}(0) \mp \lambda {{\bf \widehat J}_{e(0)}}^+ {{\bf e}(0)}\right) \exp(-\mu t)
  \f]

  where :
  - \f${\bf \dot q}\f$ is the resulting continuous velocity command to apply to the robot.
  - the sign of the control law depends on the eye in hand or eye to hand configuration.
  - \f$\bf J\f$ is the Jacobian of the task. It is function of the interaction matrix and of the robot
  Jacobian.
  - \f$\bf e = (s-s^*)\f$ is the error to regulate.
  - \f$t\f$ is the time given as parameter of this method.
  - \f$\mu\f$ is a gain that is set by default to 4 and that could be modified using setMu().
  - \f${\bf \widehat J}_{e(0)}^+ {\bf e}(0)\f$ is the value of \f${\bf \widehat J}_e^+ {\bf e}\f$ when \f$t=0\f$.
  This value is internally stored either at the first call of this method, or when \e t parameter is set to 0.

  \param t : Time in second. When set to zero, \f${{\bf \widehat J}_{e(0)}}^+ {{\bf e}(0)}\f$ is refreshed internally.
  \param e_dot_init : Initial value of \f${\bf \dot e}(0)\f$.
*/
vpColVector vpServo::computeControlLaw(double t, const vpColVector &e_dot_init)
{
  static int iteration =0;
  static vpColVector e1_initial;

  try
  {
    vpVelocityTwistMatrix cVa ; // Twist transformation matrix
    vpMatrix aJe ;      // Jacobian

    if (iteration==0)
    {
      if (testInitialization() == false) {
        vpERROR_TRACE("All the matrices are not correctly initialized") ;
        throw(vpServoException(vpServoException::servoError,
                               "Cannot compute control law "
                               "All the matrices are not correctly"
                               "initialized")) ;
      }
    }
    if (testUpdated() == false) {
      vpERROR_TRACE("All the matrices are not correctly updated") ;
    }

    // test if all the required initialization have been done
    switch (servoType)
    {
    case NONE :
      vpERROR_TRACE("No control law have been yet defined") ;
      throw(vpServoException(vpServoException::servoError,
                             "No control law have been yet defined")) ;
      break ;
    case EYEINHAND_CAMERA:
    case EYEINHAND_L_cVe_eJe:
    case EYETOHAND_L_cVe_eJe:

      cVa = cVe ;
      aJe = eJe ;

      init_cVe = false ;
      init_eJe = false ;
      break ;
    case  EYETOHAND_L_cVf_fVe_eJe:
      cVa = cVf*fVe ;
      aJe = eJe ;
      init_fVe = false ;
      init_eJe = false ;
      break ;
    case EYETOHAND_L_cVf_fJe    :
      cVa = cVf ;
      aJe = fJe ;
      init_fJe = false ;
      break ;
    }

    computeInteractionMatrix() ;
    computeError() ;

    // compute  task Jacobian
    J1 = L*cVa*aJe ;

    // handle the eye-in-hand eye-to-hand case
    J1 *= signInteractionMatrix ;

    // pseudo inverse of the task Jacobian
    // and rank of the task Jacobian
    // the image of J1 is also computed to allows the computation
    // of the projection operator
    vpMatrix imJ1t, imJ1 ;
    bool imageComputed = false ;

    if (inversionType==PSEUDO_INVERSE)
    {
      rankJ1 = J1.pseudoInverse(J1p, sv, 1e-6, imJ1, imJ1t) ;

      imageComputed = true ;
    }
    else
      J1p = J1.t() ;

    if (rankJ1 == L.getCols())
    {
      /* if no degrees of freedom remains (rank J1 = ndof)
       WpW = I, multiply by WpW is useless
    */
      e1 = J1p*error ;// primary task

      WpW.resize(J1.getCols(), J1.getCols()) ;
      WpW.setIdentity() ;
    }
    else
    {
      if (imageComputed!=true)
      {
        vpMatrix Jtmp ;
        // image of J1 is computed to allows the computation
        // of the projection operator
        rankJ1 = J1.pseudoInverse(Jtmp, sv, 1e-6, imJ1, imJ1t) ;
        imageComputed = true ;
      }
      WpW = imJ1t*imJ1t.t() ;

#ifdef DEBUG
      std::cout << "rank J1 " << rankJ1 <<std::endl ;
      std::cout << "imJ1t"<<std::endl  << imJ1t ;
      std::cout << "imJ1"<<std::endl  << imJ1 ;

      std::cout << "WpW" <<std::endl <<WpW  ;
      std::cout << "J1" <<std::endl <<J1  ;
      std::cout << "J1p" <<std::endl <<J1p  ;
#endif
      e1 = WpW*J1p*error ;
    }

    // memorize the initial e1 value if the function is called the first time or if the time given as parameter is equal to 0.
    if (iteration==0 || std::fabs(t) < std::numeric_limits<double>::epsilon()) {
      e1_initial = e1;
    }

    e = - lambda(e1) * e1 + (e_dot_init + lambda(e1) * e1_initial)*exp(-mu*t);

    vpMatrix I ;

    I.resize(J1.getCols(), J1.getCols()) ;
    I.setIdentity() ;

    I_WpW = (I - WpW) ;
  }
  catch(vpMatrixException me)
  {
    vpERROR_TRACE("Caught a matrix related error") ;
    std::cout << me << std::endl ;
    throw me;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw me ;
  }

  iteration++ ;
  return e ;
}


/*!
  Compute and return the secondary task vector according to the projection operator \f${\bf I-W^+W}\f$. For more details,
  see equation(7) in the paper \cite Marchand05b.

  \param de2dt : Value of \f$\frac{\partial {\bf e_2}}{\partial t}\f$ the derivative of the secondary task \f${\bf e}_2\f$.

  \return The secondary task vector: \f[
  ({\bf I-W^+W})\frac{\partial {\bf e_2}}{\partial t}
  \f]

  Note that the secondary task vector need than to be added to the primary task which can be in the general case written as:
  \f[
  -\lambda {\bf W^+W {\widehat {\bf J}}_e^+({\bf s-s^*})}
  \f]

  \warning computeControlLaw() must be call prior to this function since it updates the projection operator \f$ \bf W^+W \f$.

  The following sample code shows how to use this method:
  \code
  vpColVector v;     // Velocity applied to the robot
  vpColVector de2dt;
  vpServo task;
  ...
  v  = task.computeControlLaw(); // Compute the primary task
  v += task.secondaryTask(de2dt) // Compute and add the secondary task
  \endcode

  \sa computeControlLaw()
*/
vpColVector vpServo::secondaryTask(const vpColVector &de2dt)
{
  if (rankJ1 == L.getCols())
  {
    vpERROR_TRACE("no degree of freedom is free, cannot use secondary task") ;
    throw(vpServoException(vpServoException::noDofFree,
                           "no degree of freedom is free, cannot use secondary task")) ;
  }
  else
  {
    vpColVector sec ;
#if 0
    // computed in computeControlLaw()
    vpMatrix I ;

    I.resize(J1.getCols(),J1.getCols()) ;
    I.setIdentity() ;
    I_WpW = (I - WpW) ;
#endif
    //    std::cout << "I-WpW" << std::endl << I_WpW <<std::endl ;
    sec = I_WpW*de2dt ;

    return sec ;
  }
}

/*!
  Compute and return the secondary task vector according to the projection operator \f${\bf I-W^+W}\f$. For more details,
  see equation(7) in the paper \cite Marchand05b.

  \param e2 : Value of the secondary task \f${\bf e}_2\f$.
  \param de2dt : Value of \f$\frac{\partial {\bf e_2}}{\partial t}\f$ the derivative of the secondary task \f${\bf e}_2\f$.

  \return The secondary task vector:
  \f[
  -\lambda ({\bf I-W^+W}) {\bf e_2} +  ({\bf I-W^+W})\frac{\partial {\bf e_2}}{\partial t}
  \f]

  Note that the secondary task vector need than to be added to the primary task which can be in the general case written as:
  \f[
  -\lambda {\bf W^+W {\widehat {\bf J}}_e^+({\bf s-s^*})}
  \f]

  \warning computeControlLaw() must be call prior to this function since it updates the projection operator \f$ \bf W^+W \f$.

  The following sample code shows how to use this method:
  \code
  vpColVector v;     // Velocity applied to the robot
  vpColVector e2;
  vpColVector de2dt;
  vpServo task;
  ...
  v  = task.computeControlLaw();     // Compute the primary task
  v += task.secondaryTask(e2, de2dt) // Compute and add the secondary task
  \endcode

  \sa computeControlLaw()
*/
vpColVector vpServo::secondaryTask(const vpColVector &e2, const vpColVector &de2dt)
{
  if (rankJ1 == L.getCols())
  {
    vpERROR_TRACE("no degree of freedom is free, cannot use secondary task") ;
    throw(vpServoException(vpServoException::noDofFree,
                           "no degree of freedom is free, cannot use secondary task")) ;
  }
  else
  {
    vpColVector sec ;
#if 0
    // computed in computeControlLaw()
    vpMatrix I ;

    I.resize(J1.getCols(),J1.getCols()) ;
    I.setIdentity() ;

    I_WpW = (I - WpW) ;
#endif

    // To be coherent with the primary task the gain must be the same between
    // primary and secondary task.
    sec = -lambda(e1) *I_WpW*e2 + I_WpW *de2dt ;

    return sec ;
  }
}

/*!
   Return the projection operator \f${\bf I}-{\bf W}^+{\bf W}\f$. This operator is updated
   after a call of computeControlLaw().

\code
 vpServo task;
 ...
 vpColVector  v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
 vpMatrix I_WpW = task.getI_WpW();          // Get the projection operator
\endcode
   \sa getWpW()
 */
vpMatrix vpServo::getI_WpW() const
{
  return I_WpW;
}

/*!
   Return the task jacobian \f$J\f$. The task jacobian is updated after a call of computeControlLaw().

   In the general case, the task jacobian is given by \f${\bf J} = {\widehat {\bf L}} {^c}{\bf V}_a {^a}{\bf J}_e\f$.
\code
 vpServo task;
 ...
 vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
 vpMatrix    J = task.getTaskJacobian();   // Get the task jacobian used to compute v
\endcode
   \sa getTaskJacobianPseudoInverse(), getInteractionMatrix()
 */
vpMatrix vpServo::getTaskJacobian() const
{
  return J1;
}
/*!
   Return the pseudo inverse of the task jacobian \f$J\f$.

   In the general case, the task jacobian is given by \f${\bf J} = {\widehat {\bf L}} {^c}{\bf V}_a {^a}{\bf J}_e\f$.

   The task jacobian and its pseudo inverse are updated after a call of computeControlLaw().

   \return Pseudo inverse \f${J}^{+}\f$ of the task jacobian.
\code
 vpServo task;
 ...
 vpColVector v = task.computeControlLaw();            // Compute the velocity corresponding to the visual servoing
 vpMatrix   Jp = task.getTaskJacobianPseudoInverse(); // Get the pseudo inverse of task jacobian used to compute v
\endcode

 \sa getTaskJacobian()
 */
vpMatrix vpServo::getTaskJacobianPseudoInverse() const
{
  return J1p;
}
/*!
   Return the rank of the task jacobian. The rank is updated after a call of computeControlLaw().

\code
 vpServo task;
 ...
 vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
 unsigned int rank = task.getTaskRank();   // Get the rank of the task jacobian
\endcode
 */
unsigned int vpServo::getTaskRank() const
{
  return rankJ1;
}

/*!
   Return the projection operator \f${\bf W}^+{\bf W}\f$. This operator is updated
   after a call of computeControlLaw().

   When the dimension of the task is equal to the number of degrees of freedom available
   \f${\bf W^+W = I}\f$.

\code
 vpServo task;
 ...
 vpColVector v = task.computeControlLaw(); // Compute the velocity corresponding to the visual servoing
 vpMatrix  WpW = task.getWpW();            // Get the projection operator
\endcode
   \sa getI_WpW()
 */
vpMatrix vpServo::getWpW() const
{
  return WpW;
}
