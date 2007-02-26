/****************************************************************************
 *
 * $Id: vpForwardProjection.h,v 1.3 2007-02-26 16:42:18 fspindle Exp $
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
 * Forward projection.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpForwardProjection_H
#define vpForwardProjection_H

/*!
  \file vpForwardProjection.h
  \brief  class that defines what is a generic geoemtric feature
*/

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpTracker.h>
#include <visp/vpColor.h>

#include <visp/vpHomogeneousMatrix.h>


/*!
  \class vpForwardProjection
  \brief  class that defines what is a generic geoemtric feature
*/
class VISP_EXPORT vpForwardProjection : public vpTracker
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
		       const vpColor::vpColorType color=vpColor::green) =0;
  virtual void display(vpImage<unsigned char> &I,
		       const vpHomogeneousMatrix &cMo,
		       const vpCameraParameters &cam,
		       const vpColor::vpColorType color=vpColor::green) =0;

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
