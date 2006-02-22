
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project:   ViSP 2.0
 *
 * Version control
 * ===============
 *
 *  $Id: vpExponentialMap.h,v 1.2 2006-02-22 09:51:39 fspindle Exp $
 *
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpExponentialMap.h
  \brief Provides exponential map computation
*/



#ifndef vpExponentialMap_HH
#define vpExponentialMap_HH

#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpColVector.h>

/*
  \class vpExponentialMap

  \brief Direct or inverse exponential map computation.

*/
class vpExponentialMap
{

public:
  static vpHomogeneousMatrix direct(const vpColVector &v);
  static vpColVector inverse(const vpHomogeneousMatrix &M);

};
#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
