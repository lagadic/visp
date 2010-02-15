/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
  \brief Moving edges
*/

#include <stdlib.h>

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

/*!
  Copy constructor.
*/
vpMeSite::vpMeSite (const vpMeSite &mesite)
{
  *this = mesite;
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
// initialise with convolution and sign
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
  i = m.i;
  j = m.j;
  i_1 = m.i_1;
  j_1 = m.j_1;
  ifloat = m.ifloat;
  jfloat = m.jfloat;
  v = m.v;
  mask_sign = m.mask_sign;
  alpha = m.alpha;
  convlt = m.convlt;
  normGradient = m.normGradient;
  suppress = m.suppress;
  weight = m.weight;
  selectDisplay = m.selectDisplay;

  return *this ;
}


// ===================================================================
/*!
 * Construct and return the list of vpMeSite along the normal to the contour, in the given range.
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set. 
 * \param I : Image in which the display is performed.
 * \param range :  +/- the range within which the pixel's correspondent will be sought
 * \return Pointer to the list of query sites
 */
// ===================================================================

vpMeSite*
vpMeSite::getQueryList(vpImage<unsigned char> &I, const int range)
{

  int   k ;

  int n;
  double ii , jj ;
  vpMeSite *list_query_pixels ;
  list_query_pixels =  NULL ;

  // Size of query list includes the point on the line
  list_query_pixels = new vpMeSite[2 * range + 1] ;

  // range : +/- the range within which the pixel's
  //correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);
  n = 0 ;
  vpImagePoint ip;
	
  for(k = -range ; k <= range ; k++)
  {
    ii = (ifloat+k*salpha);
    jj = (jfloat+k*calpha);

	// Display
    if    ((selectDisplay==RANGE_RESULT)||(selectDisplay==RANGE)) {
      ip.set_i( ii );
      ip.set_j( jj );
      vpDisplay::displayCross(I, ip, 1, vpColor::yellow) ;
    }

    // Copy parent's convolution
    vpMeSite pel ;
    pel.init(ii, jj, alpha, convlt,mask_sign) ;
    pel.setDisplay(selectDisplay) ;// Display

	// Add site to the query list
    list_query_pixels[n] = pel ;
    n++ ;
  }

  return(list_query_pixels) ;
}

// ===================================================================
/*!
 * get the sign (according to the difference of values of the intensities of the extremities).
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set. 
 * \param I : Image in which the sign is computed.
 * \param range :  +/- the range within which the pixel's correspondent is sought
 * \post : mask_sign is computed
 */
// ===================================================================
void
vpMeSite::getSign(vpImage<unsigned char> &I, const int range)
{

  int   k ;

  double salpha = sin(alpha);
  double calpha = cos(alpha);

	//First extremity
  k = -range ;
  int i1 = vpMath::round(ifloat+k*salpha);
  int j1 = vpMath::round(jfloat+k*calpha);

	//Second extremity
  k = range ;
  int i2 = vpMath::round(ifloat+k*salpha);
  int j2 = vpMath::round(jfloat+k*calpha);

  if (I[i1][j1] > I[i2][j2]) mask_sign = 1 ; else mask_sign = -1 ;
}


// Specific function for ME
double
vpMeSite::convolution(vpImage<unsigned char>&I, const  vpMe *me)
{

  int half, index_mask ;
  double conv = 0.0 ;
  half = (me->mask_size - 1) >> 1 ;

  if(horsImage( i , j , half + me->strip , I.getHeight(), I.getWidth()))
  {
    conv = 0.0 ;
    i = 0 ; j = 0 ;
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

    int ihalf = i-half ;
    int jhalf = j-half ;
    int ihalfa ;
    int a ;
    int b ;
    for( a = 0 ; a < me->mask_size ; a++ )
    {
      ihalfa = ihalf+a ;
      for( b = 0 ; b < me->mask_size ; b++ )
      {
	conv += mask_sign* me->mask[index_mask][a][b] *
	  //	  I(i-half+a,j-half+b) ;
	  I(ihalfa,jhalf+b) ;
      }
    }

  }

  return(conv) ;
}


/*!

  Specific function for ME.

  \warning To display the moving edges graphics a call to vpDisplay::flush()
  is needed.

*/
void
vpMeSite::track(vpImage<unsigned char>& I,
		const vpMe *me,
		const bool test_contraste)
{
//   vpMeSite  *list_query_pixels ;
//   int  max_rank =0 ;
//   int max_rank1=0 ;
//   int max_rank2 = 0;
//   double  convolution = 0 ;
//   double  max_convolution = 0 ;
//   double max = 0 ;
//   double contraste = 0;
//   //  vpDisplay::display(I) ;
//   //  vpERROR_TRACE("getclcik %d",me->range) ;
//   //  vpDisplay::getClick(I) ;
// 
//   // range = +/- range of pixels within which the correspondent
//   // of the current pixel will be sought
//   int range  = me->range ;
// 
//   //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
//   list_query_pixels = getQueryList(I, range) ;
// 
//   double  contraste_max = 1 + me->mu2 ;
//   double  contraste_min = 1 - me->mu1 ;
// 
//   // array in which likelihood ratios will be stored
//   double  *likelihood= new double[ 2 * range + 1 ] ;
// 
//   int ii_1 = i ;
//   int jj_1 = j ;
//   i_1 = i ;
//   j_1 = j ;
//   double threshold;
//   threshold = me->threshold ;
// 
//   //    std::cout <<"---------------------"<<std::endl ;
//   for(int n = 0 ; n < 2 * range + 1 ; n++)
//     {
// 
//       //   convolution results
//       convolution = list_query_pixels[n].convolution(I, me) ;
// 
//       // luminance ratio of reference pixel to potential correspondent pixel
//       // the luminance must be similar, hence the ratio value should
//       // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
//       if( test_contraste )
// 	{
// 	  // Include this to eliminate temporal calculation
// 	  if (convlt==0)
// 	    {
// 	      std::cout << "vpMeSite::track : Division by zero  convlt = 0" << std::endl ;
// 	      delete []list_query_pixels ;
// 	      delete []likelihood;
// 	      throw(vpTrackingException(vpTrackingException::initializationError,
// 					"Division by zero")) ;
// 	    }
// 	
// // 	  contraste = fabs(convolution / convlt) ;
// 	  contraste = convolution / convlt ;
// 	  // likelihood ratios
// 	  if((contraste > contraste_min) && (contraste < contraste_max))
// 	    likelihood[n] = fabs(convolution + convlt ) ;
// 	  else
// 	    likelihood[n] = 0 ;
// 	}
//       else
// 	likelihood[n] = fabs(2*convolution) ;
//   
// 
//       // establishment of the maximal likelihood ratios's  rank
//       // in the array, the value of the likelihood ratio can now be
//     // referenced by its rank in the array
//     if (likelihood[n] > max)
//     {
//       max_convolution= convolution;
//       max = likelihood[n] ;
//       max_rank = n ;
//       max_rank2 = max_rank1;
//       max_rank1 = max_rank;
//     }
// 
//   }
//   
//   // test on the likelihood threshold if threshold==-1 then
//   // the me->threshold is  selected
// 
//   vpImagePoint ip;
// 
//   //  if (test_contrast)
//   if(max > threshold)
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::red);
// 	}
// 		
//       *this = list_query_pixels[max_rank] ;//The vpMeSite is replaced by the vpMeSite of max likelihood
//       normGradient =  vpMath::sqr(max_convolution);
// 	
//       convlt = max_convolution;
//       i_1 = ii_1; //list_query_pixels[max_rank].i ;
//       j_1 = jj_1; //list_query_pixels[max_rank].j ;
//       delete []list_query_pixels ;
//       delete []likelihood;
//     }
//   else //none of the query sites is better than the threshold
//     {
//       if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
// 	{
// 	  ip.set_i( list_query_pixels[max_rank].i );
// 	  ip.set_j( list_query_pixels[max_rank].j );
// 	  vpDisplay::displayPoint(I, ip, vpColor::green);
// 	}
//       normGradient = 0 ;
//       if(max == 0)
// 	suppress = 1; // contrast suppression
//       else
// 	suppress = 2; // threshold suppression
// 	
//       delete []list_query_pixels ;
//       delete []likelihood; // modif portage
//     }

  vpMeSite  *list_query_pixels ;
  int  max_rank =-1 ;
  int max_rank1=-1 ;
  int max_rank2 = -1;
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

  //  std::cout << i << "  " << j<<"  " << range << "  " << suppress  << std::endl ;
  list_query_pixels = getQueryList(I, range) ;

  double  contraste_max = 1 + me->mu2 ;
  double  contraste_min = 1 - me->mu1 ;

  // array in which likelihood ratios will be stored
  double  *likelihood= new double[ 2 * range + 1 ] ;

  int ii_1 = i ;
  int jj_1 = j ;
  i_1 = i ;
  j_1 = j ;
  double threshold;
  threshold = me->threshold ;
  double diff = 1e6;

  //    std::cout <<"---------------------"<<std::endl ;
  for(int n = 0 ; n < 2 * range + 1 ; n++)
  {
      //   convolution results
      convolution = list_query_pixels[n].convolution(I, me) ;

      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      if( test_contraste )
      {
	likelihood[n] = fabs(convolution + convlt );
	if (likelihood[n]> threshold)
	{
	  contraste = convolution / convlt;
	  if((contraste > contraste_min) && (contraste < contraste_max) && fabs(1-contraste) < diff)
	  {
	    diff = fabs(1-contraste);
	    max_convolution= convolution;
	    max = likelihood[n] ;
	    max_rank = n ;
	    max_rank2 = max_rank1;
	    max_rank1 = max_rank;
	  }
	}
      }
      
      else
      {
	likelihood[n] = fabs(2*convolution) ;
	if (likelihood[n] > max  && likelihood[n] > threshold)
	{
	  max_convolution= convolution;
          max = likelihood[n] ;
          max_rank = n ;
          max_rank2 = max_rank1;
          max_rank1 = max_rank;
        }
      }
  }
  
  // test on the likelihood threshold if threshold==-1 then
  // the me->threshold is  selected

  vpImagePoint ip;

  //  if (test_contrast)
  if(max_rank >= 0)
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[max_rank].i );
	  ip.set_j( list_query_pixels[max_rank].j );
	  vpDisplay::displayPoint(I, ip, vpColor::red);
	}
		
      *this = list_query_pixels[max_rank] ;//The vpMeSite2 is replaced by the vpMeSite2 of max likelihood
      normGradient =  vpMath::sqr(max_convolution);
	
      convlt = max_convolution;
      i_1 = ii_1; //list_query_pixels[max_rank].i ;
      j_1 = jj_1; //list_query_pixels[max_rank].j ;
      delete []list_query_pixels ;
      delete []likelihood;
    }
  else //none of the query sites is better than the threshold
    {
      if ((selectDisplay==RANGE_RESULT)||(selectDisplay==RESULT))
	{
	  ip.set_i( list_query_pixels[0].i );
	  ip.set_j( list_query_pixels[0].j );
	  vpDisplay::displayPoint(I, ip, vpColor::green);
	}
      normGradient = 0 ;
      if(contraste != 0)
	suppress = 1; // contrast suppression
      else
	suppress = 2; // threshold suppression
	
      delete []list_query_pixels ;
      delete []likelihood; // modif portage
    }
}

int vpMeSite::operator!=(const vpMeSite &m)
{
  return((m.i != i) || (m.j != j)) ;

}

std::ostream& operator<<(std::ostream& os, vpMeSite& vpMeS)
    {
      return (os << "Alpha: " << vpMeS.alpha
	      << "  Convolution: " << vpMeS.convlt 
	      << "  Flag: " << vpMeS.suppress
	      << "  Weight: " << vpMeS.weight );
    }
