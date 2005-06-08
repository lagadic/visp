

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_point.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: test_point.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *   performs various tests on the the point class
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file test_point.cpp
  \brief   performs various tests on the the point class
*/


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPoint.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureException.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>
int
main()
{

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpPoint "  <<endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  vpHomogeneousMatrix cMo ;
  cMo[0][3] = 0.1 ;
  cMo[1][3] = 0.2 ;
  cMo[2][3] = 2 ;

  vpPoint point ;
  TRACE("set point coordinates in the world  frame ") ;
  point.setWorldCoordinates(0,0,0) ;


  cout <<"------------------------------------------------------"<<endl ;
  TRACE("test the projection ") ;
  point.track(cMo) ;

  TRACE("coordinates in the world frame ") ;
  cout << point.oP.t() << endl ;
  TRACE("coordinates in the camera frame  ") ;
  cout << point.cP.t() << endl ;

  TRACE("2D coordinates ") ;
  cout<< point.get_x() << "  " << point.get_y() << endl ;

  cout <<"------------------------------------------------------"<<endl ;
  TRACE("test the interaction matrix ") ;

  vpFeaturePoint p ;
  vpFeatureBuilder::create(p,point) ;

  vpMatrix L ;
  L = p.interaction() ;
  cout << L << endl ;

  TRACE("test the interaction matrix select") ;
  TRACE("\t only X") ;
  L = p.interaction(vpFeaturePoint::selectX()) ;
  cout << L << endl ;

  TRACE("\t only Y") ;
  L = p.interaction(vpFeaturePoint::selectY()) ;
  cout << L << endl ;

  TRACE("\t X & Y") ;
  L = p.interaction(vpFeaturePoint::selectX() |
		    vpFeaturePoint::selectY()) ;
  cout << L << endl ;

  TRACE("\t selectAll") ;
  L = p.interaction(vpFeaturePoint::selectAll() ) ;
  cout << L << endl ;

  cout <<"------------------------------------------------------"<<endl ;
  TRACE("test the error ") ;

  try{
  vpFeaturePoint pd ;
  pd.set_x(0) ;
  pd.set_y(0) ;

  pd.print() ; cout << endl ;
  vpColVector e ;
  e = p.error(pd) ;
  cout << e << endl ;

  TRACE("test the interaction matrix select") ;
  TRACE("\t only X") ;
  e = p.error(pd,vpFeaturePoint::selectX()) ;
  cout << e << endl ;

  TRACE("\t only Y") ;
  e = p.error(pd,vpFeaturePoint::selectY()) ;
  cout << e << endl ;

  TRACE("\t X & Y") ;
  e = p.error(pd,vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  cout << e << endl ;

  TRACE("\t selectAll") ;
  e = p.error(pd,vpFeaturePoint::selectAll() ) ;
  cout << e << endl ;
  }
  catch(vpFeatureException me){ cout << me << endl ; }
  catch(vpException me){ cout << me << endl ; }
  cout <<"------------------------------------------------------"<<endl ;
  TRACE("test the  dimension") ;
  int dim ;
  dim = p.getDimension() ;
  cout << "Dimension = " << dim << endl ;

  TRACE("test the dimension with  select") ;
  TRACE("\t only X") ;
  dim = p.getDimension(vpFeaturePoint::selectX()) ;
  cout << "Dimension = " << dim << endl ;

  TRACE("\t only Y") ;
  dim = p.getDimension(vpFeaturePoint::selectY()) ;
  cout << "Dimension = " << dim << endl ;

  TRACE("\t X & Y") ;
  dim = p.getDimension(vpFeaturePoint::selectX() | vpFeaturePoint::selectY()) ;
  cout << "Dimension = " << dim << endl ;

  TRACE("\t selectAll") ;
  dim = p.getDimension(vpFeaturePoint::selectAll() ) ;
  cout << "Dimension = " << dim << endl ;

}
