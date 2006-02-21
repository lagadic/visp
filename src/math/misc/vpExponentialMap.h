
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMath.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      simple-math, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpExponentialMap.h,v 1.1 2006-02-21 11:09:46 fspindle Exp $
 *
 * Description
 * ============
 *  provide some simple mathematical function that are not available in
 *  the C mathematics library (math.h)
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpMath.h
  \brief Provides simple Math computation that are not available in
  the C mathematics library (math.h)
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
