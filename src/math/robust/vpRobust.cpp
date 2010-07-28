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
 * M-Estimator and various influence function.
 *
 * Authors:
 * Andrew Comport
 * Jean Laneurit
 *
 *****************************************************************************/

/*!
  \file vpRobust.cpp
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <visp/vpDebug.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>

#include <visp/vpRobust.h>


#define vpITMAX 100
#define vpEPS 3.0e-7
#define vpCST 1


// ===================================================================
/*!
  \brief Constructor.
  \param n_data : Size of the data vector.

*/
vpRobust::vpRobust(int n_data)
{
  vpCDEBUG(2) << "vpRobust constructor reached" << std::endl;

  size=n_data;
  normres.resize(n_data); 
  sorted_normres.resize(n_data); 
  sorted_residues.resize(n_data);
  it=0;
  NoiseThreshold=0.0017; //Can not be more accurate than 1 pixel

}

/*!
  \brief Resize containers.
  \param n_data : size of input data vector.

*/
void vpRobust::resize(int n_data){

  if(n_data!=size){
  normres.resize(n_data); 
  sorted_normres.resize(n_data); 
  sorted_residues.resize(n_data);
  size=n_data;
  }
  
}

// ===================================================================
/*!

  \brief Calculate an Mestimate given a particular loss function using MAD
  (Median Absolute Deviation) as a scale estimate at each iteration.

  \pre Requires a column vector of residues.

  \post Keeps a copy of the weights so that rejected points are kept at zero
  weight.

  \param method : Type of M-Estimator \f$\rho(r_i)\f$:

  - TUKEY : \f$ \rho(r_i, C) = \left\{
  \begin{array}{ll}
  \frac{r_i^6}{6} - \frac{C^2r_i^4}{2} +\frac{C^4r_i^2}{2} & \mbox{if} |r_i| < C \\
  \frac{1}{6} C^6 & \mbox{else} \end{array}
  \right.
  \f$
  with influence function \f$ \psi(r_i, C) = \left\{
  \begin{array}{ll} r_i(r_i^2-C^2)^2 & \mbox{if} |r_i| < C \\ 0 & \mbox{else} \end{array} \right. \f$ where \f$C=4.7 \hat{\sigma} \f$ and with \f$ \hat{\sigma} = 1.48{Med}_i(|r_i - {Med}_j(r_j)|) \f$

  - CAUCHY :

  - HUBER :

  \param residues : Residues \f$ r_i \f$ used in the previous formula.

  \param weights : Vector of weights \f$w_i =
  \frac{\psi(r_i)}{r_i}\f$. Values are in [0, 1]. A value near zero
  means that the data is an outlier. This vector must have the same size
  residue vector.

  \return Returns a Column Vector of weights associated to each residue.
 */

// ===================================================================
void vpRobust::MEstimator(const vpRobustEstimatorType method,
		     const vpColVector &residues,
		     vpColVector &weights)
{

  double med=0;	// median
  double normmedian=0; 	// Normalized median
  double sigma=0;// Standard Deviation

  // resize vector only if the size of residue vector has changed
  int n_data = residues.getRows();
  resize(n_data); 
  
  sorted_residues = residues;
  
  int ind_med = (int)(ceil(n_data/2.0))-1;

  // Calculate median
   med = select(sorted_residues, 0, n_data-1, ind_med/*(int)n_data/2*/);
   //residualMedian = med ;

  // Normalize residues
  for(int i=0; i<n_data; i++)
  {
    normres[i] = (fabs(residues[i]- med));
    sorted_normres[i] = (fabs(sorted_residues[i]- med));

  }

  // Calculate MAD
  normmedian = select(sorted_normres, 0, n_data-1, ind_med/*(int)n_data/2*/);
  //normalizedResidualMedian = normmedian ;
  // 1.48 keeps scale estimate consistent for a normal probability dist.
  sigma = 1.4826*normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold)
  {
    sigma= NoiseThreshold;
  }

  switch (method)
  {
  case TUKEY :
    {
      psiTukey(sigma, normres,weights);

      vpCDEBUG(2) << "Tukey's function computed" << std::endl;
      break ;

    }
  case CAUCHY :
    {
      psiCauchy(sigma, normres,weights);
      break ;
    }
    /*  case MCLURE :
    {
      psiMcLure(sigma, normres);
      break ;
      }*/
  case HUBER :
    {
      psiHuber(sigma, normres,weights);
      break ;
    }
  }
}



void vpRobust::MEstimator(const vpRobustEstimatorType method,
		     const vpColVector &residues,
		     const vpColVector& all_residues,
		     vpColVector &weights)
{


  double normmedian=0; 	// Normalized median
  double sigma=0;// Standard Deviation

  int n_all_data = all_residues.getRows();
  vpColVector all_normres(n_all_data); 

  // compute median with the residues vector, return all_normres which are the normalized all_residues vector.
  normmedian = computeNormalizedMedian(all_normres,residues,all_residues,weights);


  // 1.48 keeps scale estimate consistent for a normal probability dist.
  sigma = 1.4826*normmedian; // Median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold)
  {
    sigma= NoiseThreshold;
  }


  switch (method)
  {
  case TUKEY :
    {
      psiTukey(sigma, all_normres,weights);

      vpCDEBUG(2) << "Tukey's function computed" << std::endl;
      break ;

    }
  case CAUCHY :
    {
      psiCauchy(sigma, all_normres,weights);
      break ;
    }
    /*  case MCLURE :
    {
      psiMcLure(sigma, all_normres);
      break ;
      }*/
  case HUBER :
    {
      psiHuber(sigma, all_normres,weights);
      break ;
    }


  };
}



double vpRobust::computeNormalizedMedian(vpColVector &all_normres,
					 const vpColVector &residues,
					 const vpColVector &all_residues,
					 const vpColVector & weights
					 )
{
  double med=0;
  double normmedian=0;

  int n_all_data = all_residues.getRows();
  int n_data = residues.getRows();
  
  // resize vector only if the size of residue vector has changed
  resize(n_data);
    
  sorted_residues = residues;
  vpColVector no_null_weight_residues;
  no_null_weight_residues.resize(n_data);
  
  //all_normres.resize(n_all_data); // Normalized Residue
  //vpColVector sorted_normres(n_data); // Normalized Residue
  //vpColVector sorted_residues = residues;
  //vpColVector sorted_residues;
  

  int index =0;
  for(int j=0;j<n_data;j++)
  {
    if(weights[j]!=0)
    {
      no_null_weight_residues[index]=residues[j];
      index++;
    }
  }
  sorted_residues.resize(index);
  memcpy(sorted_residues.data,no_null_weight_residues.data,index*sizeof(double));
  n_data=index;

  vpCDEBUG(2) << "vpRobust MEstimator reached. No. data = " << n_data
	      << std::endl;

  // Calculate Median
  // Be careful to not use the rejected residues for the
  // calculation.
  
  int ind_med = (int)(ceil(n_data/2.0))-1;
  med = select(sorted_residues, 0, n_data-1, ind_med/*(int)n_data/2*/);

  int i;
  // Normalize residues
  for(i=0; i<n_all_data; i++)
  {
    all_normres[i] = (fabs(all_residues[i]- med));
  }

  for(i=0; i<n_data; i++)
  {
    sorted_normres[i] = (fabs(sorted_residues[i]- med));
  }
  // MAD calculated only on first iteration

  //normmedian = Median(normres, weights);
  //normmedian = Median(normres);
  normmedian = select(sorted_normres, 0, n_data-1, ind_med/*(int)n_data/2*/);

  return normmedian;
}

// ===================================================================
/*!
 * \brief Calculate an Mestimate with a simultaneous scale estimate
 *				using HUBER's influence function
 * \pre Requires a column vector of residues
 * \post None
 * \return Returns a Column Vector of weights associated to each residue
 */
// ===================================================================
vpColVector
vpRobust::simultMEstimator(vpColVector &residues)
{
 
  double med=0;					// Median
  double normmedian=0; 	// Normalized Median
  double sigma=0;				// Standard Deviation

  int n_data = residues.getRows();
  vpColVector normres(n_data); // Normalized Residue
  vpColVector w(n_data);
  
  vpCDEBUG(2) << "vpRobust MEstimator reached. No. data = " << n_data
	      << std::endl;

  // Calculate Median
  int ind_med = (int)(ceil(n_data/2.0))-1;
  med = select(residues, 0, n_data-1, ind_med/*(int)n_data/2*/);

  // Normalize residues
  for(int i=0; i<n_data; i++)
    normres[i] = (fabs(residues[i]- med));

  // Check for various methods.
  // For Huber compute Simultaneous scale estimate
  // For Others use MAD calculated on first iteration
  if(it==0)
  {
    normmedian = select(normres, 0, n_data-1, ind_med/*(int)n_data/2*/);
    // 1.48 keeps scale estimate consistent for a normal probability dist.
    sigma = 1.4826*normmedian; // Median Absolute Deviation
  }
  else
  {
    // compute simultaneous scale estimate
    sigma = simultscale(residues);
  }

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if(sigma < NoiseThreshold)
  {
    sigma= NoiseThreshold;
  }


  vpCDEBUG(2) << "MAD and C computed" << std::endl;

  psiHuber(sigma, normres,w);

  sig_prev = sigma;

  return w;
}

double
vpRobust::scale(vpRobustEstimatorType method, vpColVector &x)
{
  int p = 6; //Number of parameters to be estimated.
  int n = x.getRows();
  double sigma2=0;
  long double Expectation=0;
  long double Sum_chi=0;
  long double chiTmp =0;


  for(int i=0; i<n; i++)
  {

    chiTmp = constrainedChi(method, x[i]);
    Expectation += chiTmp*(1-erf(chiTmp));
    Sum_chi += chiTmp;

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
    {
      std::cout << "erf = " << 1-erf(chiTmp) << std::endl;
      std::cout << "x[i] = " << x[i] <<std::endl;
      std::cout << "chi = " << chiTmp << std::endl;
      std::cout << "Sum chi = " << chiTmp*vpMath::sqr(sig_prev) << std::endl;
      std::cout << "Expectation = " << chiTmp*(1-erf(chiTmp)) << std::endl;
      //getchar();
    }
#endif
#endif
  }


  sigma2 = Sum_chi*vpMath::sqr(sig_prev)/((n-p)*Expectation);

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  {
    std::cout << "Expectation = " << Expectation << std::endl;
    std::cout << "Sum chi = " << Sum_chi << std::endl;
    std::cout << "sig_prev" << sig_prev << std::endl;
    std::cout << "sig_out" << sqrt(fabs(sigma2)) << std::endl;
  }
#endif
#endif

  return sqrt(fabs(sigma2));

}


double
vpRobust::simultscale(vpColVector &x)
{
  int p = 6; //Number of parameters to be estimated.
  int n = x.getRows();
  double sigma2=0;
  long double Expectation=0;
  long double Sum_chi=0;
  long double chiTmp =0;


  for(int i=0; i<n; i++)
  {

    chiTmp = simult_chi_huber(x[i]);
    Expectation += chiTmp*(1-erf(chiTmp));
    Sum_chi += chiTmp;

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
    {
      std::cout << "erf = " << 1-erf(chiTmp) << std::endl;
      std::cout << "x[i] = " << x[i] <<std::endl;
      std::cout << "chi = " << chiTmp << std::endl;
      std::cout << "Sum chi = " << chiTmp*vpMath::sqr(sig_prev) << std::endl;
      std::cout << "Expectation = " << chiTmp*(1-erf(chiTmp)) << std::endl;
      //getchar();
    }
#endif
#endif
  }


  sigma2 = Sum_chi*vpMath::sqr(sig_prev)/((n-p)*Expectation);

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  {
    std::cout << "Expectation = " << Expectation << std::endl;
    std::cout << "Sum chi = " << Sum_chi << std::endl;
    std::cout << "sig_prev" << sig_prev << std::endl;
    std::cout << "sig_out" << sqrt(fabs(sigma2)) << std::endl;
  }
#endif
#endif

  return sqrt(fabs(sigma2));

}

double
vpRobust::constrainedChi(vpRobustEstimatorType method, double x)
{
  switch (method)
  {
  case TUKEY :
    return constrainedChiTukey(x);
  case CAUCHY :
    return constrainedChiCauchy(x);
  case HUBER :
    return constrainedChiHuber(x);
  };

  return -1;

}


double
vpRobust::constrainedChiTukey(double x)
{
  double sct=0;
  double a=4.7;
  double s=sig_prev;
  //double epsillon=0.5;

  if(fabs(x) <= 4.7*sig_prev)
  {
    //sct = (vpMath::sqr(s*a-x)*vpMath::sqr(s*a+x)*vpMath::sqr(x))/(s*vpMath::sqr(vpMath::sqr(a*vpMath::sqr(s))));
    sct = (vpMath::sqr(s*a)*x-s*vpMath::sqr(s*a)-x*vpMath::sqr(x))*(vpMath::sqr(s*a)*x+s*vpMath::sqr(s*a)-x*vpMath::sqr(x))/s*vpMath::sqr(vpMath::sqr(vpMath::sqr(s)))/vpMath::sqr(vpMath::sqr(a));
  }
  else
    sct = -1/s;

  return sct;
}


double
vpRobust::constrainedChiCauchy(double x)
{
  double sct = 0;
  //double u = x/sig_prev;
  double s = sig_prev;
  double b = 2.3849;

  sct = -1*(vpMath::sqr(x)*b)/(s*(vpMath::sqr(s*b)+vpMath::sqr(x)));

  return sct;
}

double
vpRobust::constrainedChiHuber(double x)
{
  double sct=0;
  double u = x/sig_prev;
  double c = 1.2107; //1.345;

  if(fabs(u) <= c)
    sct = vpMath::sqr(u);
  else
    sct = vpMath::sqr(c);

  return sct;
}

double
vpRobust::simult_chi_huber(double x)
{
  double sct=0;
  double u = x/sig_prev;
  double c = 1.2107; //1.345;

  if(fabs(u) <= c)
  {
    //sct = 0.5*vpMath::sqr(u);
    sct = vpMath::sqr(u);
  }
  else
  {
    //sct = 0.5*vpMath::sqr(c);
    sct = vpMath::sqr(c);
  }

  return sct;
}

/*!
  \brief calculation of Tukey's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/

void vpRobust::psiTukey(double sig, vpColVector &x, vpColVector & weights)
{

  int n_data = x.getRows();
  double cst_const = vpCST*4.6851;

  for(int i=0; i<n_data; i++)
  {
    if(sig==0 && weights[i]!=0)
    {
      weights[i]=1;
      continue;
    }

    double xi_sig = x[i]/sig;

    if((fabs(xi_sig)<=(cst_const)) && weights[i]!=0)
    {
      weights[i] = vpMath::sqr(1-vpMath::sqr(xi_sig/cst_const));
      //w[i] = vpMath::sqr(1-vpMath::sqr(x[i]/sig/4.7));
    }
    else
    {
      //Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}

/*!
  \brief calculation of Tukey's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/
void vpRobust::psiHuber(double sig, vpColVector &x, vpColVector &weights)
{
  double c = 1.2107; //1.345;
  int n_data = x.getRows();

  for(int i=0; i<n_data; i++)
  {
    if(weights[i]!=0)
    {
      double xi_sig = x[i]/sig;
      if(fabs(xi_sig)<=c)
	weights[i] = 1;
      else
	weights[i] = c/fabs(xi_sig);
    }
  }
}

/*!
  \brief calculation of Cauchy's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/

void vpRobust::psiCauchy(double sig, vpColVector &x, vpColVector &weights)
{
  int n_data = x.getRows();
  double const_sig = 2.3849*sig;

  //Calculate Cauchy's equation
  for(int i=0; i<n_data; i++)
  {
    weights[i] = 1/(1+vpMath::sqr(x[i]/(const_sig)));

    // If one coordinate is an outlier the other is too!
    // w[i] < 0.01 is a threshold to be set
    /*if(w[i] < 0.01)
      {
      if(i%2 == 0)
      {
      w[i+1] = w[i];
      i++;
      }
      else
      w[i-1] = w[i];
      }*/
  }
}


/*!
  \brief calculation of McLure's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/
void vpRobust::psiMcLure(double sig, vpColVector &r,vpColVector &weights)
{
  int n_data = r.getRows();

  //McLure's function
  for(int i=0; i<n_data; i++)
  {
    weights[i] = 1/(vpMath::sqr(1+vpMath::sqr(r[i]/sig)));
    //w[i] = 2*mad/vpMath::sqr((mad+r[i]*r[i]));//odobez

    // If one coordinate is an outlier the other is too!
    // w[i] < 0.01 is a threshold to be set
    /*if(w[i] < 0.01)
      {
      if(i%2 == 0)
      {
      w[i+1] = w[i];
      i++;
      }
      else
      w[i-1] = w[i];
      }*/
  }
}


/*!
  \brief partition function
  \param a : vector to be sorted
  \param l : first value to be considered
  \param r : last value to be considered
*/
int vpRobust::partition(vpColVector &a, int l, int r)
{
  int i = l-1;
  int j = r;
  double v = a[r];

  for (;;)
  {
    while (a[++i] < v) ;
    while (v < a[--j]) if (j == l) break;
    if (i >= j) break;
    exch(a[i], a[j]);
  }
  exch(a[i], a[r]);
  return i;
}

/*!
  \brief sort a part of a vector and select a value of this new vector
  \param a : vector to be sorted
  \param l : first value to be considered
  \param r : last value to be considered
  \param k : value to be selected
*/
double vpRobust::select(vpColVector &a, int l, int r, int k)
{
  while (r > l)
  {
    int i = partition(a, l, r);
    if (i >= k) r = i-1;
    if (i <= k) l = i+1;
  }
  return a[k];
}


double
vpRobust::erf(double x)
{
  return x < 0.0 ? -gammp(0.5,x*x) : gammp(0.5,x*x);
}

double
vpRobust::gammp(double a, double x)
{
  double gamser,gammcf,gln;

  if (x < 0.0 || a <= 0.0)
    std::cout << "Invalid arguments in routine GAMMP";
  if (x < (a+1.0))
  {
    gser(&gamser,a,x,&gln);
    return gamser;
  }
  else
  {
    gcf(&gammcf,a,x,&gln);
    return 1.0-gammcf;
  }
}

void
vpRobust::gser(double *gamser, double a, double x, double *gln)
{
  double sum,del,ap;

  *gln=gammln(a);
  if (x <= 0.0)
  {
    if (x < 0.0)
      std::cout << "x less than 0 in routine GSER";
    *gamser=0.0;
    return;
  }
  else
  {
    ap=a;
    del=sum=1.0/a;
    for (int n=1; n<=vpITMAX; n++)
    {
      ap += 1.0;
      del *= x/ap;
      sum += del;
      if (fabs(del) < fabs(sum)*vpEPS)
      {
	*gamser=sum*exp(-x+a*log(x)-(*gln));
	return;
      }
    }
    std::cout << "a too large, vpITMAX too small in routine GSER";
    return;
  }
}

void
vpRobust::gcf(double *gammcf, double a, double x, double *gln)
{
  double gold=0.0,g,fac=1.0,b1=1.0;
  double  b0=0.0,anf,ana,an,a1,a0=1.0;

  *gln=gammln(a);
  a1=x;
  for (int n=1; n<=vpITMAX; n++)
  {
    an=(double) n;
    ana=an-a;
    a0=(a1+a0*ana)*fac;
    b0=(b1+b0*ana)*fac;
    anf=an*fac;
    a1=x*a0+anf*a1;
    b1=x*b0+anf*b1;
    if (a1)
    {
      fac=1.0/a1;
      g=b1*fac;
      if (fabs((g-gold)/g) < vpEPS)
      {
	*gammcf=exp(-x+a*log(x)-(*gln))*g;
	return;
      }
      gold=g;
    }
  }
  std::cout << "a too large, vpITMAX too small in routine GCF";
}

double
vpRobust::gammln(double xx)
{
  double x,tmp,ser;
  static double cof[6]={76.18009173,-86.50532033,24.01409822,
			-1.231739516,0.120858003e-2,-0.536382e-5};

  x=xx-1.0;
  tmp=x+5.5;
  tmp -= (x+0.5)*log(tmp);
  ser=1.0;
  for (int j=0; j<=5; j++)
  {
    x += 1.0;
    ser += cof[j]/x;
  }
  return -tmp+log(2.50662827465*ser);
}



// double
// vpRobust::median(const vpColVector &v)
// {
//   int i,j;
//   int inf, sup;
//   int n = v.getRows() ;
//   vpColVector infsup(n) ;
//   vpColVector eq(n) ;
// 
//   for (i=0;i<n;i++)
//   {
//     // We compute the number of elements superior to the current value (sup)
//     // the number of elements inferior (inf) to the current value and
//     // the number of elements equal to the current value (eq)
//     inf = sup = 0;
//     for (j=0;j<n;j++)
//     {
//       if (i != j)
//       {
// 	if (v[i] <= v[j]) inf++;
// 	if (v[i] >= v[j]) sup++;
// 	if (v[i] == v[j]) eq[i]++;
//       }
//     }
//     // We compute then difference between inf and sup
//     // the median should be for |inf-sup| = 0 (1 if an even number of element)
//     // which means that there are the same number of element in the array
//     // that are greater and smaller that this value.
//     infsup[i] = abs(inf-sup);
//   }
// 
//   // seek for the smaller value of |inf-sup| (should be 0 or 1)
//   int imin = 0 ; // index of the median in the array
//   //double eqmax = 0 ; // count of equal values
//   // min cannot be greater than the number of element
//   double min = n;
// 
//   // number of medians
//   int mediancount = 0;
//   // array of medians
//   int *medianindex = new int[n];
// 
//   for (i=0; i<n; i++)
//   {
//     if(infsup[i] < min)
//     {
//       min = infsup[i];
//       imin = i ;
// 
//       //reset count of median values
//       mediancount=0;
//       medianindex[mediancount]=i;
//     }
//     else if(infsup[i]==min) //If there is another median
//     {
//       mediancount++;
//       medianindex[mediancount]=i;
//     }
//   }
// 
//   // Choose smalest data to be the median
//   /*for(i=0; i<mediancount+1; i++)
//     {
//     //Choose the value with the greatest count
//     if(eq[medianindex[i]] > eqmax)
//     {
//     eqmax = eq[medianindex[i]];
//     imin = medianindex[i];
//     }
//     //If we have identical counts
//     // Choose smalest data to be the median
//     //if(v[medianindex[i]] < v[imin])
//     //	imin = medianindex[i];
//     }*/
// 
//   // return the median
//   delete []medianindex;
//   return(v[imin]);
// }
// 
// // Calculate median only for the residues which have
// // not be rejected. i.e. weight=0
// double
// vpRobust::median(const vpColVector &v, vpColVector &weights)
// {
//   int i,j;
//   int inf, sup;
//   int n = v.getRows() ;
//   vpColVector infsup(n) ;
//   vpColVector eq(n) ;
// 
//   for (i=0;i<n;i++)
//   {
//     if(weights[i]!=0)
//     {
//       // We compute the number of elements superior to the current value (sup)
//       // the number of elements inferior (inf) to the current value and
//       // the number of elements equal to the current value (eq)
//       inf = sup = 0;
//       for (j=0;j<n;j++)
//       {
// 	if (weights[j]!=0 && i!=j)
// 	{
// 	  if (v[i] <= v[j]) inf++;
// 	  if (v[i] >= v[j]) sup++;
// 	  if (v[i] == v[j]) eq[i]++;
// 	}
//       }
//       // We compute then difference between inf and sup
//       // the median should be for |inf-sup| = 0 (1 if an even number of element)
//       // which means that there are the same number of element in the array
//       // that are greater and smaller that this value.
//       infsup[i] = abs(inf-sup);
//     }
//   }
// 
//   // seek for the smaller value of |inf-sup| (should be 0 or 1)
//   int imin = 0 ; // index of the median in the array
//   //double eqmax = 0 ; // count of equal values
//   // min cannot be greater than the number of element
//   double min = n;
// 
//   for (i=0; i<n; i++)
//   {
//     if(weights[i]!=0)
//     {
//       if(infsup[i] < min)
//       {
// 	min = infsup[i];
// 	imin = i ;
//       }
//     }
//   }
// 
//   // return the median
//   return(v[imin]);
// }


#undef vpITMAX
#undef vpEPS
#undef vpCST
