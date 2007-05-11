/****************************************************************************
 *
 * $Id: vpMeTracker.h,v 1.6 2007-05-11 16:53:35 fspindle Exp $
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
 * Moving edges.
 *
 * Authors:
 * Andrew Comport
 *
 *****************************************************************************/

/*!
  \file vpMeTracker.h
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

// ===================================================================
/*!
 *\class vpMeTracker
 *\brief 2D state = list of points, 3D state = feature
 *\n Contains abstract elements for a Distance to Feature type feature.
 */
// ===================================================================

#ifndef vpMeTracker_HH
#define vpMeTracker_HH


#include <math.h>
#include <iostream>

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpMeSite.h>
#include <visp/vpMe.h>
#include <visp/vpList.h>
#include <visp/vpTracker.h>


class VISP_EXPORT vpMeTracker : public vpTracker
{

public:

  //! Tracking dependent variables/functions =====================
  //! \todo create list of points and move tracking dependent
  //! functionality into a seperate class i.e DistanceEcm

  //! List of tracked points
  vpList<vpMeSite> list ;
  //! Ecm initialisation parameters
  vpMe *me ;
  //! Used for backwards compatibility...could be removed
  int nGoodElement;
  int query_range;
  double seuil;
  bool display_point;// if 1 (TRUE) displays the line that is being tracked

  //! Distance variables/functions ==========================================

  //! Constructor/Destructor
  vpMeTracker() ;
  virtual ~vpMeTracker() ;
  void init() ;

  vpMeTracker& operator =(vpMeTracker& f);

  //! Displays the number of elements in the list
  void displayNumberOfElements() ;
  void setMe(vpMe *me1) { me = me1 ; }
  int outOfImage( int i , int j , int half , int rows , int cols) ;

  int numberOfSignal() ;
  int totalNumberOfSignal() ;

  //! Virtual functions for vpMeTracker
  //! Feature dependent functions

  //!display contour
  virtual void display(vpImage<unsigned char> &I, vpColor::vpColorType col)=0;
  //!Sample pixels at a given interval
  virtual void sample(vpImage<unsigned char> &image)=0;

  void initTracking(vpImage<unsigned char>& I);
  //!Track sampled pixels
  void track(vpImage<unsigned char>& I);
  //!Displays the status of me site
  void display(vpImage<unsigned char>& I);
  //!Displays the status of me sites
  void display(vpImage<unsigned char>& I,vpColVector &w,int &index_w);

protected:
  int selectDisplay ;
public:
  enum displayEnum
    {
      NONE,
      RANGE,
      RESULT,
      RANGE_RESULT
    } ;
  void setDisplay(int select)  { selectDisplay = select ; }

};


#endif


