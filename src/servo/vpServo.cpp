
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
  \class vpServo

  \brief Class required to compute the visual servoing control law

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
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


}

//! destruction (memory deallocation if required)
void
vpServo::kill()
{ featureList.front() ;
  desiredFeatureList.front() ;


  while (!featureList.outside())
  {
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
      cout << "Deallocate " << endl ;
      s_ptr->print() ;
      delete s_ptr ;
      s_ptr = NULL ;
    }

    desiredFeatureList.next() ;
    featureList.next() ;
  }

}

void
vpServo::setServo(servoEnum _servoType)
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
//! destructor
vpServo::~vpServo()
{
  kill() ;
}

vpServo::vpServo(servoEnum _servoType)
{
  setServo(_servoType);
}

void
vpServo::print(const vpServo::printEnum display_level)
{
  cout << "Visual servoing task: " <<endl ;

  cout << "Type of control law " <<endl ;
  switch( servoType )
  {
  case NONE :
    cout << "Type of task have not been chosen yet ! " << endl ;
    break ;
  case EYEINHAND_CAMERA :
    cout << "Eye-in-hand configuration " << endl ;
    cout << "Control in the camera frame " << endl ;
    break ;
  case EYEINHAND_L_cVe_eJe :
    cout << "Eye-in-hand configuration " << endl ;
    cout << "Control in the articular frame " << endl ;
    break ;
  case EYETOHAND_L_cVe_eJe :
    cout << "Eye-to-hand configuration " << endl ;
    cout << "s_dot = _L_cVe_eJe q_dot " << endl ;
    break ;
  case EYETOHAND_L_cVf_fVe_eJe :
    cout << "Eye-to-hand configuration " << endl ;
    cout << "s_dot = _L_cVe_fVe_eJe q_dot " << endl ;
    break ;
  case EYETOHAND_L_cVf_fJe :
    cout << "Eye-to-hand configuration " << endl ;
    cout << "s_dot = _L_cVf_fJe q_dot " << endl ;
    break ;
  }

  cout << "List of visual features : s" <<endl ;
  for (featureList.front(),
	 featureSelectionList.front() ;
       !featureList.outside() ;
       featureList.next(),
	 featureSelectionList.next()  )
  {
    vpBasicFeature *s ;
    s = featureList.value();
    cout << "" ;
    s->print(featureSelectionList.value()) ;
  }


  cout << "List of desired visual features : s*" <<endl ;
  for (desiredFeatureList.front(),
	 featureSelectionList.front() ;
       !desiredFeatureList.outside() ;
       desiredFeatureList.next(),
	 featureSelectionList.next()  )
  {
    vpBasicFeature *s ;
    s = desiredFeatureList.value();
    cout << "" ;
    s->print(featureSelectionList.value()) ;
  }

  cout <<"Interaction Matrix Ls "<<endl  ;
  if (interactionMatrixComputed)
  {
    cout << L;
  }
  else
    cout << "not yet computed "<<endl ;

  cout <<"Error vector (s-s*) "<<endl  ;
  if (errorComputed)
  {
    cout << error.t() ;
  }
  else
    cout << "not yet computed "<<endl ;

  cout << "Gain : " << lambda <<endl ;
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
vpServo::setInteractionMatrixType(const int _interactionMatrixType, const int _interactionMatrixInversion)
{
  interactionMatrixType = _interactionMatrixType ;
  inversionType = _interactionMatrixInversion ;
}

/*! \brief compute the interaction matrix related to the set of visual features
   \param return Ls
*/
vpMatrix
vpServo::computeInteractionMatrix()
{

  try {
    L.resize(0,6) ;


    switch (interactionMatrixType)
    {
    case CURRENT:
      {
	if (featureList.empty())
	{
	  ERROR_TRACE("feature list empty, cannot compute Ls") ;
	  throw(vpServoException(vpServoException::noFeatureError,
				 "feature list empty, cannot compute Ls")) ;
	}
	vpMatrix         M ;
	featureList.front() ;
	featureSelectionList.front() ;
	while (!featureList.outside())
	{
	  vpBasicFeature *s_ptr = NULL;
	  s_ptr=  featureList.value() ;

	  int select = featureSelectionList.value() ;

	  M = s_ptr->interaction(select) ;

	  L = vpMatrix::stackMatrices(L,M) ;

	  featureSelectionList.next() ;
	  featureList.next() ;
	}
      }
      break ;
    case DESIRED:
      {
	if (desiredFeatureList.empty())
	{
	  ERROR_TRACE("feature list empty, cannot compute Ls") ;
	  throw(vpServoException(vpServoException::noFeatureError,
				 "feature list empty, cannot compute Ls")) ;
	}
	vpMatrix         M ;
	desiredFeatureList.front() ;
	featureSelectionList.front() ;
	while (!desiredFeatureList.outside())
	{
	  vpBasicFeature *s_ptr = NULL;
	  s_ptr=  desiredFeatureList.value() ;

	  int select = featureSelectionList.value() ;

	  M = s_ptr->interaction(select) ;

	  L = vpMatrix::stackMatrices(L,M) ;

	  featureSelectionList.next() ;
	  desiredFeatureList.next() ;
	}
      }
      break ;

    case MEAN:
      {
	if (featureList.empty())
	{
	  ERROR_TRACE("feature list empty, cannot compute Ls") ;
	  throw(vpServoException(vpServoException::noFeatureError,
				 "feature list empty, cannot compute Ls")) ;
	}
	if (desiredFeatureList.empty())
	{
	  ERROR_TRACE("feature list empty, cannot compute Ls") ;
	  throw(vpServoException(vpServoException::noFeatureError,
				 "feature list empty, cannot compute Ls")) ;
	}
	vpMatrix         M ;
	vpMatrix         Ms ;
	vpMatrix         Ms_star ;
	featureList.front() ;
	desiredFeatureList.front() ;
	featureSelectionList.front() ;
	while (!desiredFeatureList.outside())
	{
	  vpBasicFeature *s_ptr = NULL;
	  s_ptr=  featureList.value() ;
	  vpBasicFeature *s_star_ptr = NULL;
	  s_star_ptr=  desiredFeatureList.value() ;

	  int select = featureSelectionList.value() ;

	  Ms = s_ptr->interaction(select) ;
	  Ms_star = s_star_ptr->interaction(select) ;

	  M = 0.5*(Ms+Ms_star) ;
	  L = vpMatrix::stackMatrices(L,M) ;

	  featureSelectionList.next() ;
	  featureList.next() ;
	  desiredFeatureList.next() ;
	}
      }
      break ;
    }

    dim_task = L.getRows() ;

    interactionMatrixComputed = true ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    throw ;
  }
  return L ;
}

/*! \brief compute the error between the current set of visual features and
    the desired set of visual features

   \note vpServo::error is modified

   \return  error (vpColVector)

*/
vpColVector
vpServo::computeError()
{
  if (featureList.empty())
  {
    ERROR_TRACE("feature list empty, cannot compute Ls") ;
    throw(vpServoException(vpServoException::noFeatureError,
			   "feature list empty, cannot compute Ls")) ;
  }
  if (desiredFeatureList.empty())
  {
    ERROR_TRACE("feature list empty, cannot compute Ls") ;
    throw(vpServoException(vpServoException::noFeatureError,
			   "feature list empty, cannot compute Ls")) ;
  }

  try {
    vpBasicFeature *current_s ;
    vpBasicFeature *desired_s ;

    error.resize(0) ;

    for (featureList.front(),
	   desiredFeatureList.front(),
	   featureSelectionList.front() ;
	 !featureList.outside() ;
	 featureList.next(),
	   desiredFeatureList.next(),
	   featureSelectionList.next() )
    {
      current_s      = featureList.value() ;
      desired_s = desiredFeatureList.value() ;
      int select = featureSelectionList.value() ;

      s = vpColVector::stackMatrices(s,current_s->get_s()) ;
      sStar = vpColVector::stackMatrices(sStar,desired_s->get_s()) ;
      vpColVector err ;
      err = current_s->error(*desired_s, select) ;

      error = vpMatrix::stackMatrices(error,err) ;
    }

    dim_task = error.getRows() ;
    errorComputed = true ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
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
  bool test = false ;
  switch (servoType)
  {
  case NONE:
    ERROR_TRACE("No control law have been yet defined") ;
    throw(vpServoException(vpServoException::servoError,
			   "No control law have been yet defined")) ;
    test = true ;
    break ;
  case EYEINHAND_CAMERA:
    return true ;
    break ;
  case EYEINHAND_L_cVe_eJe:
  case  EYETOHAND_L_cVe_eJe:
    if (!init_cVe) ERROR_TRACE("cVe not initialized") ;
    if (!init_eJe) ERROR_TRACE("eJe not initialized") ;
    return (init_cVe && init_eJe) ;
    break ;
  case  EYETOHAND_L_cVf_fVe_eJe:
    if (!init_cVf) ERROR_TRACE("cVf not initialized") ;
    if (!init_fJe) ERROR_TRACE("fVe not initialized") ;
    if (!init_eJe) ERROR_TRACE("eJe not initialized") ;
    return (init_cVf && init_fVe && init_eJe) ;
    break ;

  case EYETOHAND_L_cVf_fJe    :
    if (!init_cVf) ERROR_TRACE("cVf not initialized") ;
    if (!init_fJe) ERROR_TRACE("fJe not initialized") ;
    return (init_cVf && init_fJe) ;
    break ;
  }

  return false ;
}
bool
vpServo::testUpdated()
{
  bool test = false ;
  switch (servoType)
  {
  case NONE:
    ERROR_TRACE("No control law have been yet defined") ;
    throw(vpServoException(vpServoException::servoError,
			   "No control law have been yet defined")) ;
    test = true ;
    break ;
  case EYEINHAND_CAMERA:
    return true ;
  case EYEINHAND_L_cVe_eJe:
    if (!init_eJe) ERROR_TRACE("eJe not updated") ;
    return (init_eJe) ;
    break ;
  case  EYETOHAND_L_cVe_eJe:
    if (!init_cVe) ERROR_TRACE("cVe not updated") ;
    if (!init_eJe) ERROR_TRACE("eJe not updated") ;
    return (init_cVe && init_eJe) ;
    break ;
  case  EYETOHAND_L_cVf_fVe_eJe:
    if (!init_fVe) ERROR_TRACE("fVe not updated") ;
    if (!init_eJe) ERROR_TRACE("eJe not updated") ;
    return (init_fVe && init_eJe) ;
    break ;

  case EYETOHAND_L_cVf_fJe    :
    if (!init_fJe) ERROR_TRACE("fJe not updated") ;
    return (init_fJe) ;
    break ;
  }

  return false ;
}
/*!
  \brief compute the desired control law

  \note
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
	ERROR_TRACE("All the matrices are not correctly initialized") ;
	throw(vpServoException(vpServoException::servoError,
			       "Cannot compute control law "
			       "All the matrices are not correctly"
			       "initialized")) ;
      }
    }
    if (testUpdated() == true)
    {
      //  cout << " Init OK " << endl ;
    }
    else
    {
      ERROR_TRACE("All the matrices are not correctly updated") ;
    }

    // test if all the required initialization have been done
    switch (servoType)
    {
    case NONE :
      ERROR_TRACE("No control law have been yet defined") ;
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
    vpMatrix J1p ;
    if (inversionType==PSEUDO_INVERSE)
      rankJ1 = J1.pseudoInverse(J1p,1e-6) ; // rank of the task Jacobian
    else
      J1p = J1.t() ;

    e1 = J1p*error ;// primary task

    e = -lambda* e1 ;

  }
  catch(vpMatrixException me)
  {
    ERROR_TRACE("Caught a matrix related error") ;
    cout << me << endl ;
    throw me;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw me ;
  }

  iteration++ ;
  return e ;
}
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
