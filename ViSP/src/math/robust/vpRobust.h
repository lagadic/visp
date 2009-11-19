/****************************************************************************
 *
 * $Id: vpRobust.h,v 1.8 2008-11-07 15:31:31 marchand Exp $
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
 * M-Estimator and various influence function.
 *
 * Authors:
 * Andrew Comport
 * Jean Laneurit
 *
 *****************************************************************************/

/*!
 \file vpRobust.h
*/



#ifndef CROBUST_HH
#define CROBUST_HH

#include <visp/vpConfig.h>
#include <visp/vpColVector.h>
#include <visp/vpMath.h>


/*!
  \class vpRobust
  \ingroup Robust
  \brief Contains an M-Estimator and various influence function.

  Supported methods: M-estimation, Tukey, Cauchy and Huber
*/
class VISP_EXPORT vpRobust
{
  
public:
  //! Enumeration of influence functions
  typedef enum
  {
    TUKEY,
    CAUCHY,
    //    MCLURE,
    HUBER
  } vpRobustEstimatorType;
  
private:

  //!Normalized residue
  vpColVector normres; 
  //!Sorted normalized Residues
  vpColVector sorted_normres;
  //!Sorted residues
  vpColVector sorted_residues;

  //!Noise threshold
  double NoiseThreshold;
  //!
  double sig_prev;
  //!
  int it;
  //! Vairiable used in swap method
  double swap;
  //! Size of the containers
  int size;

public:

  //!Default Constructor
  vpRobust(int n_data);
  
  //!Destructor
  virtual ~vpRobust(){};

  //!Resize containers for sort methods
  void resize(int n_data);
  
  //! Compute the weights according a residue vector and a PsiFunction
  void MEstimator(const vpRobustEstimatorType method,
		 const vpColVector &residues,
		 vpColVector &weights);

  //! Compute the weights according a residue vector and a PsiFunction
  void MEstimator(const vpRobustEstimatorType method,
		 const vpColVector &residues,
		 const vpColVector& all_residues,
		 vpColVector &weights);

  //! Simult Mestimator 
  vpColVector simultMEstimator(vpColVector &residues);

  //! Set iteration 
  void setIteration(const int iter){it=iter;}
  
  //! Set maximal noise threshold
  void setThreshold(const double x);//{NoiseThreshold=x;}

//public :
//double residualMedian ;
//double normalizedResidualMedian ;
//  private:
//   double median(const vpColVector &x);
//   double median(const vpColVector &x, vpColVector &weights);

 private:
  //!Compute normalized median
  double computeNormalizedMedian(vpColVector &all_normres,
				 const vpColVector &residues,
				 const vpColVector &all_residues,
				 const vpColVector &weights				 
				 );


  //! Calculate various scale estimates
  double scale(vpRobustEstimatorType method, vpColVector &x);
  double simultscale(vpColVector &x);


  //---------------------------------
  //  Partial derivative of loss function with respect to the residue
  //---------------------------------
  /** @name PsiFunctions  */
  //@{
  //! Tuckey influence function 
  void psiTukey(double sigma, vpColVector &x, vpColVector &w);
  //! Caucht influence function 
  void psiCauchy(double sigma, vpColVector &x, vpColVector &w);
  //! McLure influence function 
  void psiMcLure(double sigma, vpColVector &x, vpColVector &w);
  //! Huber influence function 
  void psiHuber(double sigma, vpColVector &x, vpColVector &w);
  //@}

  //! Partial derivative of loss function
  //! with respect to the scale
  double simult_chi_huber(double x);

  //---------------------------------
  // Constrained Partial derivative of loss function with respect to the scale
  //--------------------------------- 
   /** @name Constrained Chi Functions  */
  //@{
  //! Constrained Chi Function
  double constrainedChi(vpRobustEstimatorType method, double x);
  //! Constrained Chi Tukey Function
  double constrainedChiTukey(double x);
  //! Constrained Chi Cauchy Function
  double constrainedChiCauchy(double x);
   //! Constrained Chi Huber Function
  double constrainedChiHuber(double x);
  //@}
  
  //---------------------------------
  // Mathematic functions used to calculate the Expectation
  //---------------------------------
  /** @name Some math function  */
  //@{
  double erf(double x);
  double gammp(double a, double x);
  void gser(double *gamser, double a, double x, double *gln);
  void gcf(double *gammcf, double a, double x, double *gln);
  double gammln(double xx);
  //@}
  
  /** @name Sort function  */
  //@{
  //! Swap two value
  void exch(double &A, double &B){swap = A; A = B;  B = swap;}
  //! Sort function using partition method
  int partition(vpColVector &a, int l, int r);
  //! Sort the vector and select a value in the sorted vector
  double select(vpColVector &a, int l, int r, int k);
  //@}
};

#endif
