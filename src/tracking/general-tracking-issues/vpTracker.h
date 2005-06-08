
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTracker.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTracker.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a generic tracker
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpTracker_H
#define vpTracker_H

/*!
  \file vpTracker.h
  \brief  class that defines what is a generic tracker
*/

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpTracker
  \brief  class that defines what is a generic tracker
*/
class vpTracker
{


public:
  //! 2D feature coordinates
  vpColVector p ;
  //! feature coordinates  expressed in
  //! camera frame
  vpColVector cP ;

  //!
  bool cPAvailable ;


public:
  //! basic construction
  void init() ;
  //! constructor
  vpTracker() ;

  //! destructor
  virtual ~vpTracker() { ; }

public:

  //  virtual void track(vpHomogeneousMatrix &cMo) ;
  //  virtual void track(vpImage<unsigned char>, vpCameraParameters &c) ;
  //  virtual void pixelToMeter(vpCameraParameters &c) ;

} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
