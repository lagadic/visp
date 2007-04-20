/****************************************************************************
 *
 * $Id: vpScale.cpp,v 1.4 2007-04-20 14:22:16 asaunier Exp $
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
 * This file is part of the ViSP toolkit
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
 * Median Absolute Deviation (MAD), MPDE, Mean shift kernel density estimation.
 *
 * Authors:
 * Andrew Comport
 *
 *****************************************************************************/

/*!
  \file vpScale.cpp
*/

// ===================================================================
/*!
 * \class vpScale
 * \brief Contains different methods for estimating the robust scale
 * 				of an error distribution.
 * \n Methods of Median Absolute Deviation and Density Gradient
 * 		estimation using the Mean Shift method.
 * \author Andrew Comport
 * \date 24/10/03
 */
// ===================================================================

#include <stdlib.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>


#include "visp/vpScale.h"
//#include <robust/vpScale.h>


#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0
#define DEBUG_LEVEL3 1




//! Constructor
vpScale::vpScale()
{
  if(DEBUG_LEVEL2)
    std::cout << "vpScale constructor reached" << std::endl;

  bandwidth = 0.02;
  dimension = 1;
  kernel_type = EPANECHNIKOV;

  if(DEBUG_LEVEL2)
    std::cout << "vpScale constructor finished" << std::endl;

}

//! Constructor
vpScale::vpScale(double kernel_bandwidth,
		 int dimension=1, int kernel_type=EPANECHNIKOV)
{
  if(DEBUG_LEVEL2)
    std::cout << "vpScale constructor reached" << std::endl;

  bandwidth = kernel_bandwidth;
  dimension = dimension;
  kernel_type = kernel_type;

  if(DEBUG_LEVEL2)
    std::cout << "vpScale constructor finished" << std::endl;

}

//! Destructor
vpScale::~vpScale()
{

}



// Calculate the modes of the density for the distribution
// and their associated errors
double
vpScale::MeanShift(vpColVector &error)
{

  int n = error.getRows()/dimension;
  vpColVector density(n);
  vpColVector density_gradient(n);
  vpColVector mean_shift(n);

  int increment=1;

  // choose smallest error as start point
  int i=0;
  while(error[i]<0 && error[i]<error[i+1])
    i++;

  // Do mean shift until no shift
  while(increment >= 1 && i<n)
  {
    increment=0;
    density[i] = KernelDensity(error, i);
    density_gradient[i] = KernelDensityGradient(error, i);
    mean_shift[i]=vpMath::sqr(bandwidth)*density_gradient[i]/((dimension+2)*density[i]);

    double tmp_shift = mean_shift[i];

    // Do mean shift
    while(tmp_shift>0 && tmp_shift>error[i]-error[i+1])
    {
      i++;
      increment++;
      tmp_shift-=(error[i]-error[i-1]);
    }
  }

  return error[i];

}

// Calculate the density of each point in the error vector
// Requires ordered set of errors
double
vpScale::KernelDensity(vpColVector &error, int position)
{

  int n = error.getRows()/dimension;
  double density=0;
  double Ke = 1;
  int j=position;

  vpColVector X(dimension);


  // Use each error in the bandwidth to calculate
  // the local density of error i
  // First treat larger errors
  while(Ke !=0 && j<=n)
  {
    //Create vector of errors corresponding to each dimension of a feature
    for(int i=0; i<dimension; i++)
    {
      X[i]=(error[position]-error[j])/bandwidth;
      position++;
      j++;
    }
    position-=dimension; // reset position

    Ke = KernelDensity_EPANECHNIKOV(X);
    density+=Ke;
  }

  Ke = 1;
  j=position;
  // Then treat smaller errors
  while(Ke !=0 && j>=dimension)
  {
    //Create vector of errors corresponding to each dimension of a feature
    for(int i=0; i<dimension; i++)
    {
      X[i]=(error[position]-error[j])/bandwidth;
      position++;
      j--;
    }
    position-=dimension; // reset position

    Ke = KernelDensity_EPANECHNIKOV(X);
    density+=Ke;
  }

  density*=1/(n*bandwidth);

  return density;

}

double
vpScale::KernelDensityGradient(vpColVector &error, int position)
{

  int n = error.getRows()/dimension;
  double density_gradient=0;
  double sum_delta=0;
  double delta=0;
  int nx=0;

  double inside_kernel = 1;
  int j=position;
  // Use each error in the bandwidth to calculate
  // the local density gradient
  // First treat larger errors than current
  while(inside_kernel !=0 && j<=n)
  {
    delta = error[position]-error[j];
    if(vpMath::sqr(delta/bandwidth)<1)
    {
      inside_kernel = 1;
      sum_delta+=error[j]-error[position];
      j++;
      nx++;
    }
    else
      inside_kernel = 0;
  }

  inside_kernel = 1;
  j=position;
  // Then treat smaller errors than current
  while(inside_kernel !=0 && j>=dimension)
  {
    delta = error[position]-error[j];
    if(vpMath::sqr(delta/bandwidth)<1)
    {
      inside_kernel = 1;
      sum_delta+=error[j]-error[position];
      j--;
      nx++;
    }
    else
      inside_kernel = 0;
  }

  density_gradient = KernelDensityGradient_EPANECHNIKOV(sum_delta, n);


  return density_gradient;

}


//Epanechnikov_kernel for an d dimensional Euclidian space R^d
double
vpScale::KernelDensity_EPANECHNIKOV(vpColVector &X)
{

  double XtX= X*X;
  double c;  // Volume of an n dimensional unit sphere

  switch (dimension)
  {
  case 1:
    c = 2;
    break;
  case 2:
    c = M_PI;
    break;
  case 3:
    c = 4*M_PI/3;
    break;
  default:
    std::cout << "ERROR in vpScale::Kernel_EPANECHNIKOV : wrong dimension" << std::endl;
    exit(1);
  }

  if(XtX < 1)
    return 1/(2*c)*(dimension+2)*(1-XtX);
  else
    return 0;

}


//Epanechnikov_kernel for an d dimensional Euclidian space R^d
double
vpScale::KernelDensityGradient_EPANECHNIKOV(double sumX, int n)
{

  double c;  // Volume of an n dimensional unit sphere

  switch (dimension)
  {
  case 1:
    c = 2;
    break;
  case 2:
    c = M_PI;
    break;
  case 3:
    c = 4*M_PI/3;
    break;
  default:
    std::cout << "ERROR in vpScale::Kernel_EPANECHNIKOV : wrong dimension" << std::endl;
    exit(1);
  }

  //return sumX*(dimension+2)/(n*pow(bandwidth, (double)dimension)*c*vpMath::sqr(bandwidth));
  return sumX*(dimension+2)/(n*bandwidth*c*vpMath::sqr(bandwidth));

}

