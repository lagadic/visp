
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpForwardProjection.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpForwardProjection.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a generic geoemtric feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpForwardProjection_H
#define vpForwardProjection_H

/*!
  \file vpForwardProjection.h
  \brief  class that defines what is a generic geoemtric feature
*/

#include <visp/vpMatrix.h>
#include <visp/vpTracker.h>
#include <visp/vpColor.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpForwardProjection
  \brief  class that defines what is a generic geoemtric feature
*/
class vpForwardProjection : public vpTracker
{
public:
  //! feature coordinates  expressed in
  //! world frame
  vpColVector oP ;

public:
  //! basic construction
  virtual void init() = 0;

  //! destructor
  virtual ~vpForwardProjection() { ; }

public:
  //! set the point world coordinates
  virtual void setWorldCoordinates(const vpColVector &_oP) = 0;



  virtual void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP)=0;
  virtual void projection(const vpColVector &_cP, vpColVector &_p) =0 ;

  void project()  ;
  void changeFrame(const vpHomogeneousMatrix &cMo)  ;
  void project(const vpHomogeneousMatrix &cMo) ;
  void track(const vpHomogeneousMatrix &cMo) ;

  virtual void display(vpImage<unsigned char> &I,
		       const vpCameraParameters &cam,
		       const int color=vpColor::green) =0;
  virtual void display(vpImage<unsigned char> &I,
		       const vpHomogeneousMatrix &cMo,
		       const vpCameraParameters &cam,
		       const int color=vpColor::green) =0;

  virtual void print() const ;

  virtual vpForwardProjection *duplicate() const = 0 ;

  // memory issue (used by the vpServo class)
public:
  enum whoShouldDeallocateEnum
    {
      user,
      vpDisplayForwardProjection
    } ;

private:
  int deallocate ;
public:
  void setDeallocate(int d) { deallocate = d ; }
  int getDeallocate() { return deallocate ; }
} ;


#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
