/****************************************************************************
 *
 * $Id: vpMeTracker.cpp,v 1.7 2007-01-31 15:26:00 asaunier Exp $
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
  \file vpMeTracker.cpp
  \brief Contains abstract elements for a Distance to Feature type feature.
*/

#include <visp/vpMeTracker.h>
#include <visp/vpDisplay.h>
#include <visp/vpColor.h>

#include <visp/vpTrackingException.h>
#include <visp/vpDebug.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0




void
vpMeTracker::init()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpMeTracker::init() " <<  endl ;

  vpTracker::init()  ;
  p.resize(2) ;
  selectDisplay = NONE ;

  if (DEBUG_LEVEL1)
    cout << "end vpMeTracker::init() " <<  endl ;
}

vpMeTracker::vpMeTracker()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpMeTracker::vpMeTracker() " <<  endl ;

  init();
  me = NULL ;
  display_point = false ;
  nGoodElement = 0;

  if (DEBUG_LEVEL1)
    cout << "end vpMeTracker::vpMeTracker() " << endl ;
}

vpMeTracker::~vpMeTracker()
{
  if (DEBUG_LEVEL1) cout << "begin vpMeTracker::~vpMeTracker() " << endl ;

  if(!(list.empty()))
    list.kill();

  if (DEBUG_LEVEL1) cout << "end vpMeTracker::~vpMeTracker() " << endl ;
}

vpMeTracker&
vpMeTracker::operator = (vpMeTracker& p)
{
  if (DEBUG_LEVEL1) cout << "begin vpMeTracker::operator=" << endl ;

  list = p.list;
  me = p.me;
  selectDisplay = p.selectDisplay ;

  if (DEBUG_LEVEL1) cout << "end vpMeTracker::operator=" << endl ;
  return *this;
}

int
vpMeTracker::numberOfSignal()
{
  int number_signal=0;

  // Loop through all the points tracked from the contour
  list.front();
  while(!list.outside())
  {
    vpMeSite P = list.value();
    if(P.suppress == 0) number_signal++;
    list.next();
  }

  return number_signal;
}

int
vpMeTracker::totalNumberOfSignal()
{
  return list.nbElement();

}

int
vpMeTracker::outOfImage(int i, int j, int half, int rows, int cols)
{
  return (! ((i> half+2) &&
	     (i< rows -(half+2)) &&
	     (j>half+2) &&
	     (j<cols-(half+2)))
	  ) ;
}


//! Virtual function that is called by lower classes vpMeTrackerLine/Circle/Cylinder
void
vpMeTracker::initTracking(vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
    cout << "begin vpMeTracker::initTracking() " << endl ;


  // Must set range to 0
  int range_tmp = me->range;
  me->range=1;

  nGoodElement=0;

  int d = 0;
  // Loop through list of sites to track
  list.front();
  while(!list.outside())
  {
    vpMeSite refp = list.value() ;//current reference pixel

    d++ ;
    // If element hasn't been suppressed
    if(refp.suppress==0)
    {
      try {
	refp.track(I,me,false);
      }
      catch(...)
      {
	// EM verifier quel signal est de sortie !!!
	vpERROR_TRACE(" ") ;
	throw ;
      }
      if(refp.suppress==0) nGoodElement++;
    }


    if(DEBUG_LEVEL2)
    {
      double a,b ;
      a = refp.i_1 - refp.i ;
      b = refp.j_1 - refp.j ;
      if(refp.suppress==0)
	vpDisplay::displayArrow(I,
				refp.i,refp.j,
				refp.i+(int)(a),refp.j+(int)(b),
				vpColor::green) ;
    }

    list.modify(refp) ;
    list.next() ;
  }

  /*
  if (res != OK)
  {
    cout<< "In vpMeTracker::initTracking(): " ;
    switch (res)
    {
    case  ERR_TRACKING:
      cout << "vpMeTracker::initTracking:Track return ERR_TRACKING " << endl ;
      break ;
    case fatalError:
      cout << "vpMeTracker::initTracking:Track return fatalError" << endl ;
      break ;
    default:
      cout << "vpMeTracker::initTracking:Track return error " << res << endl ;
    }
    return res ;
  }
  */

  me->range=range_tmp;


  if (DEBUG_LEVEL1)
  cout << "end vpMeTracker::initTracking() " << endl ;

}


void
vpMeTracker::track(vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
    cout << "begin  vpMeTracker::Track():" << endl ;

  if (list.nbElement()==0)
  {

    vpERROR_TRACE("Error Tracking: only %d "
		 "pixels when entered the function ",list.nbElement()) ;
    throw(vpTrackingException(vpTrackingException::notEnoughPointError,
			      "too few pixel to track")) ;

  }

  nGoodElement=0;
  //  int d =0;
  // Loop through list of sites to track
  list.front();
  while(!list.outside())
  {
    vpMeSite s = list.value() ;//current reference pixel

    //    d++ ;
    // If element hasn't been suppressed
    if(s.suppress==0)
    {

      try{
	//	vpERROR_TRACE("%d",d ) ;
	//	vpERROR_TRACE("range %d",me->range) ;
	 s.track(I,me,true);
      }
      catch(vpTrackingException)
      {
	vpERROR_TRACE("catch exception ") ;
	s.suppress=2 ;
      }

      if(s.suppress != 2)
      {
	nGoodElement++;

	if(DEBUG_LEVEL2)
	{
	  double a,b ;
	  a = s.i_1 - s.i ;
	  b = s.j_1 - s.j ;
	  if(s.suppress==0)
	    vpDisplay::displayArrow(I,
				    s.i,s.j,
				    s.i+(int)(a*5),s.j+(int)(b*5),
				    vpColor::red) ;
	}

      }
      list.modify(s) ;
    }
    list.next() ;

  }

  if (DEBUG_LEVEL1)
    cout << "end  vpMeTracker::Track()" <<nGoodElement << endl ;

}


void
vpMeTracker::display(vpImage<unsigned char>& I)
{
  if (DEBUG_LEVEL1)
  {
    cout <<"begin vpMeTracker::displayList() " << endl ;
    cout<<" There are "<<list.nbElement()<< " sites in the list " << endl ;
  }
  list.front();

  while (!list.outside())
  {
    vpMeSite p = list.value() ;

    if(p.suppress == 1)
      vpDisplay::displayCross(I,p.i, p.j, 2, vpColor::white) ; // Contrast
    else if(p.suppress == 2)
      vpDisplay::displayCross(I,p.i, p.j, 2,vpColor::blue) ; // Threshold
    else if(p.suppress == 3)
      vpDisplay::displayCross(I,p.i, p.j, 3, vpColor::green) ; // M-estimator
    else if(p.suppress == 0)
      vpDisplay::displayCross(I,p.i, p.j, 2, vpColor::red) ; // OK

    list.next() ;
  }

  list.front() ;

  if (DEBUG_LEVEL1)
  {
    cout <<"end vpMeTracker::displayList() " << endl ;
  }
}


void
vpMeTracker::display(vpImage<unsigned char>& I,vpColVector &w,int &index_w)
{

  list.front();
  while(!list.outside())
  {
    vpMeSite P = list.value();

    if(P.suppress == 0)
    {
      P.weight = w[index_w];
      index_w++;
    }

    list.modify(P) ;
    list.next();

  }
  display(I);
}

#undef DEBUG_LEVEL1
#undef DEBUG_LEVEL2
#undef DEBUG_LEVEL3

