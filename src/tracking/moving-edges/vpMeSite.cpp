/****************************************************************************
 *
 * $Id: vpMeSite.cpp,v 1.6 2006-10-30 12:34:16 marchand Exp $
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
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
  \file vpMeSite.cpp
  \name Element de Contour en Mouvement
*/

// ====================================================================
/*!
 * \class vpMeSite
 * \brief Performs search in a given direction(normal) for a given distance(pixels)
 * \n for a given 'site'. Gives the most likely site given the probablility
 * \n from an ME mask
 * \n
 * \n - Bug fix rewrote application of masks to use the temporal information
 * \n instead of applying both temporal masks to the same image.
 * \n ie: spacial -> spatio/temporal
 * \n - Added new tracking function to choose the most similar edge amongst
 * \n - all edges found.
 * \n sample step.
 */
// ====================================================================


#include <visp/vpMeSite.h>
#include <visp/vpMe.h>
#include <visp/vpTrackingException.h>

#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 0


#ifndef DOXYGEN_SHOULD_SKIP_THIS
static
bool horsImage( int i , int j , int half , int rows , int cols)
{
  return((i < half + 1) || ( i > (rows - half - 3) )||(j < half + 1) || (j > (cols - half - 3) )) ;
}
#endif

void
vpMeSite::init()
{
  // Site components
  alpha =  0.0 ;
  convlt = 0.0 ;
  suppress = 0;
  weight=-1;

  selectDisplay = NONE ;

  // Pixel components
  i = 0 ;
  j = 0 ;
  v = 0 ;
  ifloat =i ;
  jfloat = j ;
  i_1 = 0 ;
  j_1 = 0 ;

  mask_sign =1 ;

}

vpMeSite::vpMeSite()
{
  init() ;
}

vpMeSite::vpMeSite(double ip, double jp)
{
  init() ;

  selectDisplay = NONE ;
  i = vpMath::round(ip) ;
  j = vpMath::round(jp) ;
  ifloat = ip ;
  jfloat = jp ;
}


// More an Update than init
// For points in meter form (to avoid approximations)
void
vpMeSite::init(double ip, double jp, double alphap)
{
  // Note: keep old value of convlt, suppress and contrast
  selectDisplay = NONE ;

  ifloat = ip;
  i= vpMath::round(ip);
  jfloat = jp ;
  j = vpMath::round(jp);
  alpha = alphap ;
  mask_sign =1 ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;

}

// initialise with convolution
void
vpMeSite::init(double ip, double jp, double alphap, double convltp)
{
  selectDisplay = NONE ;
  ifloat = ip ;
  i= (int)ip ;
  jfloat = jp ;
  j =(int)jp  ;
  alpha = alphap ;
  convlt = convltp;
  mask_sign =1 ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;
}
// initialise with convolution
void
vpMeSite::init(double ip, double jp, double alphap, double convltp, int sign)
{
  selectDisplay = NONE ;
  ifloat = ip ;
  i= (int)ip ;
  jfloat = jp ;
  j =(int)jp  ;
  alpha = alphap ;
  convlt = convltp;
  mask_sign = sign ;

  v = 0 ;
  i_1 = 0 ;
  j_1 = 0 ;
}

vpMeSite &vpMeSite::operator=(const vpMeSite &m)
{
  i = m.i ;
  j = m.j ;
  i_1 = m.i_1 ;
  j_1 = m.j_1 ;
  alpha = m.alpha ;
  v = m.v ;
  ifloat = m.ifloat ;
  jfloat = m.jfloat ;
  convlt = m.convlt ;
  suppress = m.suppress;
  weight = m.weight;

  selectDisplay = m.selectDisplay ;
  mask_sign = m.mask_sign ;

  return *this ;
}



// get query sites along the normal to the contour
vpMeSite*
vpMeSite::getQueryList(vpImage<unsigned char> &I, const int range)
{

  int   k ;

  int n;
  double ii , jj ;
  vpMeSite *list_query_pixels ;
  list_query_pixels =  NULL ;

  // Size of query list includes point on line
  list_query_pixels = new vpMeSite[2 * range + 1] ;

  n = 0 ;
  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);

  for(k = -range ; k <= range ; k++)
  {
    ii = (ifloat+k*salpha);
    jj = (jfloat+k*calpha);

    //   if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE))
      vpDisplay::displayCross(I,vpMath::round(ii),vpMath::round(jj),1,vpColor::blue) ;

    // Copy parent's convolution
    vpMeSite pel ;
    pel.init(ii, jj, alpha, convlt,mask_sign) ;
    pel.setDisplay(selectDisplay) ;

    list_query_pixels[n] = pel ;
    n++ ;
  }

  return(list_query_pixels) ;
}

// get query sites along the normal to the contour
void
vpMeSite::getSign(vpImage<unsigned char> &I, const int range)
{

  int   k ;

  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);


  k = -range ;
  int i1 = vpMath::round(ifloat+k*salpha);
  int j1 = vpMath::round(jfloat+k*calpha);

  k = range ;
  int i2 = vpMath::round(ifloat+k*salpha);
  int j2 = vpMath::round(jfloat+k*calpha);

  if (I[i1][j1] > I[i2][j2]) mask_sign = 1 ; else mask_sign = -1 ;
}


// Specific function for ME
double
vpMeSite::convolution(vpImage<unsigned char>&ima, const  vpMe *me)
{

  int half, index_mask ;
  double conv = 0.0 ;
  half = (me->mask_size - 1) >> 1 ;

  if(horsImage( i , j , half + me->strip , ima.getRows(), ima.getCols()))
  {
    conv = 0.0 ;
    i =0 ; j = 0 ;
  }
  else
  {
    // Calculate tangent angle from normal
    double theta  = alpha+M_PI/2;
    // Move tangent angle to within 0->M_PI for a positive
    // mask index
    while (theta<0) theta += M_PI;
    while (theta>M_PI) theta -= M_PI;

    // Convert radians to degrees
    int thetadeg = vpMath::round(theta * 180 / M_PI) ;

    if(abs(thetadeg) == 180 )
    {
      thetadeg= 0 ;
    }

    index_mask = (int)(thetadeg/(double)me->anglestep);

    for(int a = 0 ; a < me->mask_size ; a++ )
    {
      for(int b = 0 ; b < me->mask_size ; b++ )
      {
	// Removed mask_sign !!NO (EM)
	conv += mask_sign* me->mask[index_mask][a][b] *
	  ima(i-half+a,j-half+b) ;

      }
    }

  }

  return(conv) ;
}


// Specific function for ME
void
vpMeSite::track(vpImage<unsigned char>& I,
		 const vpMe *me,
		 const bool test_contraste)
{
  vpMeSite  *list_query_pixels ;
  int  max_rank =0 ;
  int max_rank1=0 ;
  int max_rank2 = 0;
  double  convolution = 0 ;
  double  max_convolution = 0 ;
  double max = 0 ;
  double contraste = 0;
  //  vpDisplay::display(I) ;
  //  vpERROR_TRACE("getclcik %d",me->range) ;
  //  vpDisplay::getClick(I) ;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  int range  = me->range ;

  //  cout << i << "  " << j<<"  " << range << "  " << suppress  << endl ;
  list_query_pixels = getQueryList(I, range) ;

  double  contraste_max = 1 + me->mu2 ;
  double  contraste_min = 1 - me->mu1 ;

  // array in which likelihood ratios will be stored
  double  *likelyhood= new double[ 2 * range + 1 ] ;

  int ii_1 = i ;
  int jj_1 = j ;
  i_1 = i ;
  j_1 = j ;
  double threshold;
  threshold = me->threshold ;

  //    cout <<"---------------------"<<endl ;
  for(int n = 0 ; n < 2 * range + 1 ; n++)
  {

    //   convolution results
    convolution = list_query_pixels[n].convolution(I ,me) ;

    // luminance ratio of reference pixel to potential correspondent pixel
    // the luminance must be similar, hence the ratio value should
    // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
    if( test_contraste )
    {
      // Include this to eliminate temporal calulation
      if (convlt==0)
      {
	cout << "vpMeSite::track : Division by zero  convlt = 0" << endl ;
	delete []list_query_pixels ;
	delete []likelyhood;
	throw(vpTrackingException(vpTrackingException::initializationError,
				  "Division by zero")) ;
      }

      contraste = fabs(convolution / convlt) ;
      // likelihood ratios
      if((contraste > contraste_min) && (contraste < contraste_max))
	likelyhood[n] = fabs(convolution + convlt ) ;
      else
	likelyhood[n] = 0 ;
    }
    else
      likelyhood[n] = fabs(2*convolution) ;

    // establishment of the maximal likelihood ratios's  rank
    // in the array, the value of the likelihood ratio can now be
    // referenced by its rank in the array
    if (likelyhood[n] > max)
    {
      max_convolution= convolution;
      max = likelyhood[n] ;
      max_rank = n ;
      max_rank2 = max_rank1;
      max_rank1 = max_rank;
    }

  }
  
  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected

  //  if (test_contrast)
  if(max > threshold)
  {
    if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
    {
      vpDisplay::displayPoint(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, vpColor::red);
      vpDisplay::flush(I) ;
    }

    *this = list_query_pixels[max_rank] ;

    convlt = max_convolution;
    i_1 = ii_1; //list_query_pixels[max_rank].i ;
    j_1 = jj_1; //list_query_pixels[max_rank].j ;
    delete []list_query_pixels ;
    delete []likelyhood;
  }
  else
  {
       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
    {
      vpDisplay::displayPoint(I, list_query_pixels[max_rank].i,list_query_pixels[max_rank].j, vpColor::green);
      vpDisplay::flush(I) ;
    }

    if(max == 0)
      suppress = 1; // contrast suppression
    else
      suppress = 2; // threshold suppression

    delete []list_query_pixels ;
    delete []likelyhood; // modif portage
  }
}

int vpMeSite::operator!=(const vpMeSite &m)
{
  return((m.i != i) || (m.j != j)) ;

}

ostream& operator<<(ostream& os, vpMeSite& vpMeS)
    {
      return (os<<"Alpha: "<<vpMeS.alpha<<"  Convolution: "<<vpMeS.convlt<<"  Flag: "<<vpMeS.suppress<<"  Weight: "<<vpMeS.weight );
    }
