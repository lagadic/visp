

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImage.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpRansac.t.cpp,v 1.1 2005-06-28 08:37:15 marchand Exp $
 *
 * Description
 * ============
 *   template class for ransac robust algorithm
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



/*
  Creation     : june, 15 2005
  Modification
*/

#include <visp/vpRansac.h>

// random number generation
#include <visp/vpNoise.h>

// debug and trace
#include <visp/vpDebug.h>

// vector
#include <visp/vpColVector.h>

/*!
  \brief
  RANSAC - Robustly fits a model to data with the RANSAC algorithm

  \param  x
  Data sets to which we are seeking to fit a model M
  It is assumed that x is of size [d x Npts]
  where d is the dimensionality of the data and Npts is
  the number of data points.

  \param  s
  The minimum number of samples from x required by
  fittingfn to fit a model.
  \param   t
  The distance threshold between data point and the model
  used to decide whether a point is an inlier or not.

  \param M
  The model having the greatest number of inliers.

  \param    inliers
  An array of indices of the elements of x that were
  the inliers for the best model.

  References:
  M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
  for model fitting with applications to image analysis and automated
  cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981

  Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
  Computer Vision". pp 101-113. Cambridge University Press, 2001

  this code is inspired by :
  Peter Kovesi
  School of Computer Science & Software Engineering
  The University of Western Australia
  pk at csse uwa edu au
  http://www.csse.uwa.edu.au/~pk

*/

template <class vpTransformation>
void
vpRansac<vpTransformation>::ransac(int npts, vpColVector &x,
				   int s, double t,
				   vpColVector &M,
				   vpColVector &inliers,
				   int consensus)
{

  double eps = 1e-6 ;
  //  [rows, npts] = size(x);

  double p = 0.99;    // Desired probability of choosing at least one sample
  // free from outliers

  int maxTrials = 1000;    // Maximum number of trials before we give up.
  int  maxDataTrials = 100; // Max number of attempts to select a non-degenerate
  // data set.

  // Sentinel value allowing detection of solution failure.
  bool solutionFind = false ;
  vpColVector bestM ;
  int trialcount = 0;
  int  bestscore =  -1;
  double   N = 1;            // Dummy initialisation for number of trials.

  vpUniRand random ;

  // index of the detected inliers
  vpColVector bestinliers ;
  int ind[s] ;
  while(( N > trialcount) && (consensus > bestscore))
  {
    // Select at random s datapoints to form a trial model, M.
    // In selecting these points we have to check that they are not in
    // a degenerate configuration.
    bool degenerate = true;
    int count = 1;
    while ( degenerate == true)
    {
      // Generate s random indicies in the range 1..npts
      for  (int i=0 ; i < s ; i++)
      {
	ind[i] = (int)ceil(random()*npts);
      }

      // Test that these points are not a degenerate configuration.
      degenerate = vpTransformation::degenerateConfiguration(x,ind) ;
      //   degenerate = feval(degenfn, x(:,ind));

      // Safeguard against being stuck in this loop forever
      count = count + 1;
      if (count > maxDataTrials)
      {
	ERROR_TRACE("Unable to select a nondegenerate data set");
	throw ;
      }

    }


    //   for (int i=0 ; i < 5 ; i++) ind[i] = 11*i ;


    // Fit model to this random selection of data points.
    vpTransformation::computeTransformation(x,ind, M) ;

    vpColVector d ;
    // Evaluate distances between points and model.
    vpTransformation::computeResidual(x, M, d) ;

    // Find the indices of points that are inliers to this model.

    int ninliers =0 ;
    for (int i=0 ; i < npts ; i++)
      if (fabs(d[i])<t) { inliers[i] = 1 ; ninliers++ ; }
      else inliers[i] = 0 ;

    if (ninliers > bestscore)    // Largest set of inliers so far...
    {
      bestscore = ninliers;  // Record data for this model
      bestinliers = inliers;
      bestM = M;
      solutionFind = true ;
      // Update estimate of N, the number of trials to ensure we pick,
      // with probability p, a data set with no outliers.
      double fracinliers =  (double)ninliers/(double)npts;
      double pNoOutliers = 1 -  pow(fracinliers,s);

      pNoOutliers = vpMath::max(eps, pNoOutliers);  // Avoid division by -Inf
      pNoOutliers = vpMath::min(1-eps, pNoOutliers);// Avoid division by 0.
      N = (log(1-p)/log(pNoOutliers));

    }

    trialcount = trialcount+1;
    //    if (trialcount%20==0) printf("trial %d out of %d         \n",trialcount, (int)ceil((double)N));

    // Safeguard against being stuck in this loop forever
    if (trialcount > maxTrials)
    {
      //      TRACE("ransac reached the maximum number of %d trials",   maxTrials);
    }
  }

  if (solutionFind==true)   // We got a solution
  {
    M = bestM;
    inliers = bestinliers;
  }
  else
  {
    TRACE("ransac was unable to find a useful solution");
    M = 0;
  }
}

