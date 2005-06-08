/*!
 \file CRobust.h
*/

// ===================================================================
/*!
 * \brief Contains an M-Estimator and various
 * influence function.
 * \n Methods : M-estimation, Tukey, Cauchy, Mclure and Huber
 * \author Andrew Comport
 * \date 29/1/02
*/
// ===================================================================


#ifndef CROBUST_HH
#define CROBUST_HH

#include <visp/vpColVector.h>
#include <visp/vpMath.h>



class vpRobust
{
public:
  enum estimatorTypeEnum
  {
    TUKEY,
    CAUCHY,
    MCLURE,
    HUBER
  } ;
private:

  double sig_prev;
  int it;

public:
  vpColVector w;

  double NoiseThreshold;

  //! Constructor
  vpRobust(int);
  //! Destructor
  ~vpRobust(void);

  int MEstimator(const int method,
		 const vpColVector &residues,
		 vpColVector &weights);

  int MEstimator(const int method,
		 const  vpColVector &residues,
		 const vpColVector& all_residues,
		 vpColVector &weights);

  vpColVector SimultMEstimator(vpColVector &residues);
  vpColVector CMEstimator(int method, vpColVector &residues);

  void setIteration(const int x);
  void setThreshold(const double x);

private:

  double median(vpColVector &x);
  double median(vpColVector &x, vpColVector &weights);

  double computeNormalizedMedian(vpColVector &all_normres,
				 const vpColVector &residues,
				 const vpColVector &all_residues);


  //! Calculate various scale estimates
  double scale(int method, vpColVector &x);
  double simultscale(vpColVector &x);

  //! Partial derivative of loss function
  //! with respect to the residue
  int psiTukey(double sigma, vpColVector &x);
  int psiCauchy(double sigma, vpColVector &x);
  int psiMcLure(double sigma, vpColVector &x);
  int psiHuber(double sigma, vpColVector &x);

  //! Partial derivative of loss function
  //! with respect to the scale
  double simult_chi_huber(double x);

  //! Constrained Partial derivative of loss function
  //! with respect to the scale
  double constrainedChi(int method, double x);
  double constrainedChiTukey(double x);
  double constrainedChiCauchy(double x);
  double constrainedChiHuber(double x);

  //! Used to calculate the Expectation
  double erf(double x);
  double gammp(double a, double x);
  void gser(double *gamser, double a, double x, double *gln);
  void gcf(double *gammcf, double a, double x, double *gln);
  double gammln(double xx);

  // pour le calcul de la mediane
  void exch(double &A, double &B);
  int partition(vpColVector &a, int l, int r);
  double select(vpColVector &a, int l, int r, int k);
};

#endif
