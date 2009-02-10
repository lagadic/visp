/****************************************************************************
 *
 * $Id: vpServo.cpp,v 1.24 2009-01-04 10:11:42 fspindle Exp $
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
  Default constructor.

  By default:

  - the interaction matrix \f$ L \f$ is computed with the desired
    features \f$ s^*\f$ (\f$ L=L_{s^*}\f$). With
    setInteractionMatrixType() you can also use the interaction matrix
    with the current visual features \f$L_s\f$, or the mean \f$(L_s /
    2 + L_{s^*} / 2)\f$.

  - the control law is build from the pseudo inverse of the
    interaction matrix \f$ v = - \lambda \; L^+ \; e\f$. With
    setInteractionMatrixType() can also set the usage of the transpose
    \f$ v = \lambda \; L^t \; e\f$ .

*/
vpServo::vpServo() 
{
  init() ;
}

vpServo::vpServo(vpServoType _servoType)
{
  setServo(_servoType);
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
  Initialize the servo.

  By default:

  - the interaction matrix \f$ L \f$ is computed with the desired
    features \f$ s^*\f$ (\f$ L=L_{s^*}\f$). With
    setInteractionMatrixType() you can also use the interaction matrix
    with the current visual features \f$L_s\f$, or the mean \f$(L_s /
    2 + L_{s^*} / 2)\f$.

  - the control law is build from the pseudo inverse of the
    interaction matrix \f$ v = - \lambda \; L^+ \; e\f$. With
    setInteractionMatrixType() can also set the usage of the transpose
    \f$ v = \lambda \; L^t \; e\f$ .

*/
void
vpServo::init()
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

  featureList.kill() ;
  desiredFeatureList.kill() ;

  signInteractionMatrix = 1 ;

  interactionMatrixType = DESIRED ;
  inversionType=PSEUDO_INVERSE ;

  interactionMatrixComputed = false ;
  errorComputed = false ;

  taskWasKilled = false;
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
void
vpServo::kill()
{
  if (taskWasKilled == false) {
    // kill the current and desired feature lists
    featureList.front() ;
    desiredFeatureList.front() ;

    while (!featureList.outside()) {
	vpBasicFeature *s_ptr = NULL;

	// current list
	s_ptr=  featureList.value() ;
	if (s_ptr->getDeallocate() == vpBasicFeature::vpServo)
	  {
	    delete s_ptr ;
	    s_ptr = NULL ;
	  }

	//desired list
	s_ptr=  desiredFeatureList.value() ;
	if (s_ptr->getDeallocate() == vpBasicFeature::vpServo)
	  {
	    //	    s_ptr->print() ;
	    delete s_ptr ;
	    s_ptr = NULL ;
	  }

	desiredFeatureList.next() ;
	featureList.next() ;
      }
    featureList.kill() ;
    desiredFeatureList.kill() ;
    taskWasKilled = true;
  }
}

void
vpServo::setServo(vpServoType _servoType)
{

  servoType = _servoType ;

  if ((servoType==EYEINHAND_CAMERA) ||(servoType==EYEINHAND_L_cVe_eJe))
    signInteractionMatrix = 1 ;
  else
    signInteractionMatrix = -1 ;



  // when the control is directly compute in the camera frame
  // we relieve the end-user to initialize cVa and aJe
  if (servoType==EYEINHAND_CAMERA)
    {
      vpTwistMatrix _cVe ; set_cVe(_cVe) ;

      vpMatrix _eJe ;
      _eJe.eye(6) ;
      set_eJe(_eJe) ;
    };

}


/*!

  Prints on \e os stream informations about the task:

  \param displayLevel : If vpServo::ALL prints
  - Type of control law (eye-in-hand, eye-to-hand)
  - Control frame (camera frame, joint space)
  - List of current visual features : s
  - List of desired visual features : s*
  - Interaction Matrix Ls
  - Error vector (s-s*)
  - Gain of the controller

  \param displayLevel : If vpServo::MINIMUM prints only the error vector (s-s*).

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
	for (featureList.front(),
	       featureSelectionList.front() ;
	     !featureList.outside() ;
	     featureList.next(),
	       featureSelectionList.next()  )
	  {
	    vpBasicFeature *s ;
	    s = featureList.value();
	    os << "" ;
	    s->print(featureSelectionList.value()) ;
	  }


	os << "List of desired visual features : s*" <<std::endl ;
	for (desiredFeatureList.front(),
	       featureSelectionList.front() ;
	     !desiredFeatureList.outside() ;
	     desiredFeatureList.next(),
	       featureSelectionList.next()  )
	  {
	    vpBasicFeature *s ;
	    s = desiredFeatureList.value();
	    os << "" ;
	    s->print(featureSelectionList.value()) ;
	  }

	os <<"Interaction Matrix Ls "<<std::endl  ;
	if (interactionMatrixComputed)
	  {
	    os << L;
	  }
	else
	  {os << "not yet computed "<<std::endl ;}

	os <<"Error vector (s-s*) "<<std::endl  ;
	if (errorComputed)
	  {
	    os << error.t() ;
	  }
	else
	  {os << "not yet computed "<<std::endl ;}

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
	for (featureList.front(),
	       featureSelectionList.front() ;
	     !featureList.outside() ;
	     featureList.next(),
	       featureSelectionList.next()  )
	  {
	    vpBasicFeature *s ;
	    s = featureList.value();
	    os << "" ;
	    s->print(featureSelectionList.value()) ;
	  }

	break;
      }
    case vpServo::FEATURE_DESIRED: 
      {
	os << "List of desired visual features : s*" <<std::endl ;
	for (desiredFeatureList.front(),
	       featureSelectionList.front() ;
	     !desiredFeatureList.outside() ;
	     desiredFeatureList.next(),
	       featureSelectionList.next()  )
	  {
	    vpBasicFeature *s ;
	    s = desiredFeatureList.value();
	    os << "" ;
	    s->print(featureSelectionList.value()) ;
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
	if (interactionMatrixComputed)
	  {
	    os << L;
	  }
	else
	  {os << "not yet computed "<<std::endl ;}
	break;
      }

    case vpServo::ERROR_VECTOR:
    case vpServo::MINIMUM:

      {
	os <<"Error vector (s-s*) "<<std::endl  ;
	if (errorComputed)
	  {  os << error.t() ;  }
	else
	  {os << "not yet computed "<<std::endl ;}

	break;
      }
    }
}

//! create a new set of 2 features in the task
void
vpServo::addFeature(vpBasicFeature& s, vpBasicFeature &s_star, int select)
{
  featureList += &s ;
  desiredFeatureList += &s_star ;
  featureSelectionList+= select ;
}

/*!
  \brief create a new visual  features in the task
  (the desired feature is then null)
*/
void
vpServo::addFeature(vpBasicFeature& s, int select)
{
  featureList += &s ;

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
  s_star = s.duplicate()  ;

  s_star->init() ;
  s_star->setDeallocate(vpBasicFeature::vpServo)  ;

  desiredFeatureList += s_star ;
  featureSelectionList+= select ;
}

//! get the task dimension
int
vpServo::getDimension()
{

  featureList.front() ;
  featureSelectionList.front() ;

  dim_task  =0 ;
  while (!featureList.outside())
    {
      vpBasicFeature *s_ptr = NULL;
      s_ptr=  featureList.value() ;
      int select = featureSelectionList.value() ;

      dim_task += s_ptr->getDimension(select) ;

      featureSelectionList.next() ;
      featureList.next() ;
    }

  return dim_task ;
}

void
vpServo::setInteractionMatrixType(const vpServoIteractionMatrixType &_interactionMatrixType, const vpServoInversionType &_interactionMatrixInversion)
{
  interactionMatrixType = _interactionMatrixType ;
  inversionType = _interactionMatrixInversion ;
}


static void
computeInteractionMatrixFromList  (/*const*/ vpList<vpBasicFeature *> & featureList,
				   /*const*/ vpList<int> & featureSelectionList,
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
   * reallocation (realloc(dim*2)) is done if necessary. In particulary,
   * [log_2(dim)+1] reallocations are done for the first matrix computation.
   * If the allocated size is too large, a correction is done after the loop.
   * The algotithmic cost is linear in affectation, logarthmic in allocation
   * numbers and linear in allocation size.
   */

  /* First assumption: matrix dimensions have not changed. If 0, they are
   * initialized to dim 1.*/
  int rowL = L .getRows();
  const int colL = 6;
  if (0 == rowL) { rowL = 1; L .resize(rowL, colL);}

  /* vectTmp is used to store the return values of functions get_s() and
   * error(). */
  vpMatrix matrixTmp;
  int rowMatrixTmp, colMatrixTmp;

  /* The cursor are the number of the next case of the vector array to
   * be affected. A memory reallocation should be done when cursor
   * is out of the vector-array range.*/
  int cursorL = 0;

  for (featureList.front() ,featureSelectionList.front() ;
       !featureList.outside();
       featureSelectionList.next(),featureList.next() )
    {
      const vpBasicFeature * sPTR = featureList.value() ;
      const int select = featureSelectionList.value() ;

      /* Get s. */
      matrixTmp = sPTR->interaction(select);
      rowMatrixTmp = matrixTmp .getRows();
      colMatrixTmp = matrixTmp .getCols();

      /* Check the matrix L size, and realloc if needed. */
      while (rowMatrixTmp + cursorL > rowL)
	{ rowL *= 2; L .resize (rowL,colL,false); vpDEBUG_TRACE(15,"Realloc!"); }

      /* Copy the temporarily matrix into L. */
      for (int k = 0; k < rowMatrixTmp; ++k, ++cursorL)
	for (int j = 0; j <  colMatrixTmp; ++j)
	  { L[cursorL][j] = matrixTmp[k][j]; }

    }

  L.resize (cursorL,colL,false);

  return ;
}


/*!

  Compute the interaction matrix related to the set of visual features.

  \return Ls
*/
vpMatrix
vpServo::computeInteractionMatrix()
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
	      computeInteractionMatrixFromList(this ->desiredFeatureList,
					      this ->featureSelectionList, L);

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

  Compute the error \f$(s - s^*)\f$ between the current set of visual
  features \f$s\f$ and the desired set of visual features \f$s^*\f$.

  \return The error \f$(s - s^*)\f$. To access to the computed error
  you can also use the public vpServo::error class member.

*/
vpColVector
vpServo::computeError()
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
     * reallocation (realloc(dim*2)) is done if necessary. In particulary,
     * [log_2(dim)+1] reallocations are done for the first error computation.
     * If the allocated size is too large, a correction is done after the loop.
     * The algotithmic cost is linear in affectation, logarthmic in allocation
     * numbers and linear in allocation size.
     * No assumptions are made concerning size of each vector: they are
     * not said equal, and could be different.
     */

    /* First assumption: vector dimensions have not changed. If 0, they are
     * initialized to dim 1.*/
    int dimError = error .getRows();
    int dimS = s .getRows();
    int dimSStar = sStar .getRows();
    if (0 == dimError) { dimError = 1; error .resize(dimError);}
    if (0 == dimS) { dimS = 1; s .resize(dimS);}
    if (0 == dimSStar) { dimSStar = 1; sStar .resize(dimSStar);}

    /* vectTmp is used to store the return values of functions get_s() and
     * error(). */
    vpColVector vectTmp;
    int dimVectTmp;

    /* The cursor are the number of the next case of the vector array to
     * be affected. A memory reallocation should be done when cursor
     * is out of the vector-array range.*/
    int cursorS = 0;
    int cursorSStar = 0;
    int cursorError = 0;

    /* For each cell of the list, recopy value of s, s_star and error. */
    for (featureList.front(),
	   desiredFeatureList.front(),
	   featureSelectionList.front() ;

	 !featureList.outside() ;

	 featureList.next(),
	   desiredFeatureList.next(),
	   featureSelectionList.next() )
      {
	current_s  = featureList.value() ;
	desired_s  = desiredFeatureList.value() ;
	int select = featureSelectionList.value() ;

	/* Get s, and store it in the s vector. */
	vectTmp = current_s->get_s();
	dimVectTmp = vectTmp .getRows();
	while (dimVectTmp + cursorS > dimS)
	  { dimS *= 2; s .resize (dimS,false); vpDEBUG_TRACE(15,"Realloc!"); }
	for (int k = 0; k <  dimVectTmp; ++k) { s[cursorS++] = vectTmp[k]; }

	/* Get s_star, and store it in the s vector. */
	vectTmp = desired_s->get_s();
	dimVectTmp = vectTmp .getRows();
	while (dimVectTmp + cursorSStar > dimSStar)
	  { dimSStar *= 2; sStar .resize (dimSStar,false);  }
	for (int k = 0; k <  dimVectTmp; ++k)
	  { sStar[cursorSStar++] = vectTmp[k]; }

	/* Get error, and store it in the s vector. */
	vectTmp = current_s->error(*desired_s, select) ;
	dimVectTmp = vectTmp .getRows();
	while (dimVectTmp + cursorError > dimError)
	  { dimError *= 2; error .resize (dimError,false);  }
	for (int k = 0; k <  dimVectTmp; ++k)
	  { error[cursorError++] = vectTmp[k]; }
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

bool
vpServo::testInitialization()
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
bool
vpServo::testUpdated()
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
  \brief compute the desired control law

  Compute the control law according to the equation:
  \f[
  -\lambda {\bf W^+W\widehat J_s^+(s-s^*)}
  \f]
  or
  \f[
  -\lambda {\bf\widehat J_s^+(s-s^*)}
  \f]
  if the dimension of the task is equal to number of available degrees of freedom (in that case \f${\bf W^+W = I}\f$)

  in this equation Js is function of the interaction matrix and of the robot
  Jacobian. It is also built according to the chosen configuration eye-in-hand
  or eye-to-hand (see vpServo::setServo method)
*/
vpColVector
vpServo::computeControlLaw()
{
  static int iteration =0;

  try
    {
      vpTwistMatrix cVa ; // Twist transformation matrix
      vpMatrix aJe ;      // Jacobian

      if (iteration==0)
	{
	  if (testInitialization() == true)
	    {
	    }
	  else
	    {
	      vpERROR_TRACE("All the matrices are not correctly initialized") ;
	      throw(vpServoException(vpServoException::servoError,
				     "Cannot compute control law "
				     "All the matrices are not correctly"
				     "initialized")) ;
	    }
	}
      if (testUpdated() == true)
	{
	  //  os << " Init OK " << std::endl ;
	}
      else
	{
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
	  vpColVector sv ;
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
	}
      else
	{
	  if (imageComputed!=true)
	    {
	      vpMatrix Jtmp ;
	      vpColVector sv ;
	      // image of J1 is computed to allows the computation
	      // of the projection operator
	      rankJ1 = J1.pseudoInverse(Jtmp,sv, 1e-6, imJ1, imJ1t) ;
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
  compute the secondary task according to the projection operator

  see equation (7) of the IEEE RA magazine, dec 2005 paper.

  compute the vector
  \f[
  + ({\bf I-W^+W})\frac{\partial {\bf e_2}}{\partial t}
  \f]
  to be added to the primary task
  \f[
  -\lambda {\bf W^+W\widehat J_s^+(s-s^*)}
  \f]
  which is computed using the vpServo::computeControlLaw method

  \warning the projection operator \f$ \bf W^+W \f$ is computed in
  vpServo::computeControlLaw which must be called prior to this function.

  \sa vpServo::computeControlLaw
*/
vpColVector
vpServo::secondaryTask(vpColVector &de2dt)
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
      vpMatrix I ;

      I.resize(J1.getCols(),J1.getCols()) ;
      I.setIdentity() ;
      I_WpW = (I - WpW) ;

      //    std::cout << "I-WpW" << std::endl << I_WpW <<std::endl ;
      sec = I_WpW*de2dt ;

      return sec ;
    }
}

/*!
  compute the secondary task according to the projection operator.
  see equation (7) of the IEEE RA magazine, dec 2005 paper.

  compute the vector
  \f[
  -\lambda ({\bf I-W^+W}) {\bf e_2} +  ({\bf I-W^+W})\frac{\partial {\bf e_2}}{\partial t}
  \f]
  to be added to the primary task
  \f[
  -\lambda {\bf W^+W\widehat J_s^+(s-s^*)}
  \f]
  which is computed using the vpServo::computeControlLaw method

  \warning the projection operator \f$ \bf W^+W \f$ is computed in
  vpServo::computeControlLaw which must be called prior to this function.

  \sa vpServo::computeControlLaw
*/
vpColVector
vpServo::secondaryTask(vpColVector &e2, vpColVector &de2dt)
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
      vpMatrix I ;

      I.resize(J1.getCols(),J1.getCols()) ;
      I.setIdentity() ;

      I_WpW = (I - WpW) ;

      sec = -lambda(e2) *I_WpW*e2 + I_WpW *de2dt ;

      return sec ;
    }
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
