/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/me/vpMeEllipse.h>

#include <visp3/me/vpMe.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpDebug.h>

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <vector>

void computeTheta(double &theta, vpColVector &K, vpImagePoint iP);

/*!
  Computes the \f$ \theta \f$ angle which represents the angle between the tangente to the curve and the i axis.

  \param theta : The computed value.
  \param K : The parameters of the ellipse.
  \param iP : the point belonging th the ellipse where the angle is computed.
*/
void
computeTheta(double &theta, vpColVector &K, vpImagePoint iP)
{
  double i = iP.get_i();
  double j = iP.get_j();

  double A = 2*i+2*K[1]*j + 2*K[2];
  double B = 2*K[0]*j + 2*K[1]*i + 2*K[3];

  theta = atan2(A,B); //Angle between the tangente and the i axis.

  while (theta > M_PI) { theta -= M_PI; }
  while (theta < 0) { theta += M_PI; }
}


/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMeEllipse::vpMeEllipse()
  : K(), iPc(), a(0.), b(0.), e(0.), iP1(), iP2(), alpha1(0), alpha2(2*M_PI),
    ce(0.), se(0.), angle(), m00(0.), mu11(0.), mu20(0.), mu02(0.),
    m10(0.), m01(0.), m11(0.), m02(0.), m20(0.),
    thresholdWeight(0.2), expecteddensity(0.)
{
  // redimensionnement du vecteur de parametre
  // i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4

  K.resize(5);

  //j1 = j2 = i1 = i2 = 0;
  iP1.set_i(0);
  iP1.set_j(0);
  iP2.set_i(0);
  iP2.set_j(0);
}

/*!
  Copy constructor.
*/
vpMeEllipse::vpMeEllipse(const vpMeEllipse &meellipse)
  : vpMeTracker(meellipse), K(), iPc(), a(0.), b(0.), e(0.), iP1(), iP2(), alpha1(0), alpha2(2*M_PI),
    ce(0.), se(0.), angle(), m00(0.), mu11(0.), mu20(0.), mu02(0.),
    m10(0.), m01(0.), m11(0.), m02(0.), m20(0.),
    thresholdWeight(0.2), expecteddensity(0.)
{
  K = meellipse.K;
  iPc = meellipse.iPc;
  a = meellipse.a;
  b = meellipse.b;
  e = meellipse.e;

  iP1 = meellipse.iP1;
  iP2 = meellipse.iP2;
  alpha1 = meellipse.alpha1;
  alpha2 = meellipse.alpha2;
  ce = meellipse.ce;
  se = meellipse.se;

  angle = meellipse.angle;

  m00 = meellipse.m00;
  mu11 = meellipse.mu11;
  mu20 = meellipse.mu20;
  mu02 = meellipse.mu02;
  m10 = meellipse.m10;
  m01 = meellipse.m01;
  m11 = meellipse.m11;
  m02 = meellipse.m02;
  m20 = meellipse.m20;
  thresholdWeight = meellipse.thresholdWeight;

  expecteddensity = meellipse.expecteddensity;
}

/*!
  Basic destructor.
*/
vpMeEllipse::~vpMeEllipse()
{
  list.clear();
  angle.clear();
}

/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities. The two extremities are defined by
  the points with the smallest and the biggest \f$ alpha \f$ angle.

  \param I : Image in which the ellipse appears.

  \exception vpTrackingException::initializationError : Moving edges not initialized.

*/
void
vpMeEllipse::sample(const vpImage<unsigned char> & I)
{
  if (!me) {
    throw(vpException(vpException::fatalError,
                      "Moving edges on ellipse tracking not initialized"));
  }

  int height = (int)I.getHeight();
  int width = (int)I.getWidth();

  //if (me->getSampleStep()==0)
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon())
  {
    std::cout << "In vpMeEllipse::sample: ";
    std::cout << "function called with sample step = 0";
    //return fatalError;
  }

  vpImagePoint iP11;

  double incr = vpMath::rad(me->getSampleStep()); // angle increment en degree
  vpColor col = vpColor::red;
  getParameters();

  // Delete old list
  list.clear();

  angle.clear();

  // sample positions
  double k = alpha1;
  while (k<alpha2)
  {
    double j = a *sin(k); // equation of an ellipse
    double i = b *cos(k); // equation of an ellipse

    // (i,j) are the coordinates on the origin centered ellipse;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    iP11.set_j( iPc.get_j() + ce *j + se *i );
    iP11.set_i( iPc.get_i() - se *j + ce *i );

    vpDisplay::displayCross(I, iP11,  5, col);

    double theta;
    computeTheta(theta, K, iP11) ;

    // If point is in the image, add to the sample list
    if(!outOfImage(vpMath::round(iP11.get_i()), vpMath::round(iP11.get_j()), 0, height, width))
    {
      vpMeSite pix;
      pix.init((int)iP11.get_i(), (int)iP11.get_j(), theta);
      pix.setDisplay(selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);

      if(vpDEBUG_ENABLE(3))
      {
        vpDisplay::displayCross(I,iP11, 5, vpColor::blue);
      }
      list.push_back(pix);
      angle.push_back(k);
    }
    k += incr;

  }
  vpMeTracker::initTracking(I);
}


/*!

  Resample the ellipse if the number of sample is less than 90% of the
  expected value.

  \note The expected value is computed thanks to the difference between the smallest and the biggest \f$ \alpha \f$ angles
  and the parameter which indicates the number of degrees between
  two points (vpMe::sample_step).

  \param I : Image in which the ellipse appears.

  \exception vpTrackingException::initializationError : Moving edges not initialized.

*/
void
vpMeEllipse::reSample(const vpImage<unsigned char>  &I)
{
  if (!me) {
    throw(vpException(vpException::fatalError,
                      "Moving edges on ellipse tracking not initialized"));
  }

  unsigned int n = numberOfSignal();
  expecteddensity = (alpha2-alpha1) / vpMath::rad((double)me->getSampleStep());
  if ((double)n < 0.5*expecteddensity){
    sample(I);
    vpMeTracker::track(I);
  }
}


/*!
  Computes the coordinates of the ellipse center, length of the semiminor axis \f$ a \f$, the length of the semimajor axis \f$ b \f$ and,
  \f$ e \f$ which is the angle made by the major axis and the i axis of the image frame \f$ (i,j) \f$.

  All those computations are made thanks to the parameters \f$ K = {K_0, ..., K_4} \f$.
*/
void
vpMeEllipse::getParameters()
{
  double k[6];
  for (unsigned int i=0; i < 5; i++)
    k[i+1] = K[i];
  k[0] = 1;

  double d = k[2]*k[2] - k[0]*k[1];

  iPc.set_i( (k[1] * k[3] - k[2] * k[4]) / d );
  iPc.set_j( (k[0] * k[4] - k[2] * k[3]) / d );

  double sq =  sqrt(vpMath::sqr(k[1]-k[0]) + 4.0*vpMath::sqr(k[2]));

  if (std::fabs(k[2]) <= std::numeric_limits<double>::epsilon()) {
    e = 0;
  }
  else {
    e = (k[1] - k[0] + sq) / (2.0*k[2]);
    e = (-1/e);
  }

  e = atan(e);

  if(e < 0.0)  e += M_PI;

  ce = cos(e);
  se = sin(e);

  double num = 2.0*(k[0]*iPc.get_i()*iPc.get_i() + 2.0*k[2]*iPc.get_j()*iPc.get_i() + k[1]*iPc.get_j()*iPc.get_j() - k[5]);
  double a2 = num / (k[0] + k[1] + sq );
  double b2 = num / (k[0] + k[1] - sq );

  a = sqrt( a2 );
  b = sqrt( b2 );
}

/*!
  Print the parameters \f$ K = {K_0, ..., K_4} \f$ and the coordinates of the ellipse center.
*/
void
vpMeEllipse::printParameters()
{
  std::cout << "K" << std::endl;
  std::cout << K.t();
  std::cout << iPc << std::endl;
}

/*!
  Computes the \f$ alpha \f$ counter clockwise angle of the two points and store them into alpha1 for the
  smallest and alpha2 for the biggest.

  \note This function is useful only during the initialization.

  \param pt1 : First point whose \f$ alpha \f$ angle is computed.
  \param pt2 : Second point whose \f$ alpha \f$ angle is computed.
*/
void
vpMeEllipse::computeAngle(vpImagePoint pt1, vpImagePoint pt2)
{
  getParameters();

  int number_of_points = 2000;
  double incr = 2 * M_PI / number_of_points; // angle increment

  double dmin1 = 1e6 ;
  double dmin2 = 1e6 ;

  double k =  0;
  while(k < 2*M_PI) {

    double j1 = a *sin(k); // equation of an ellipse
    double i1 = b *cos(k); // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse

    double j11 = iPc.get_j() + ce *j1 + se *i1;
    double i11 = iPc.get_i() - se *j1 + ce *i1;

    double  d = vpMath::sqr(pt1.get_i()-i11) + vpMath::sqr(pt1.get_j()-j11);
    if (d < dmin1)
    {
      dmin1 = d;
      alpha1 = k;
    }
    d = vpMath::sqr(pt2.get_i()-i11) + vpMath::sqr(pt2.get_j()-j11);
    if (d < dmin2)
    {
      dmin2 = d;
      alpha2 = k;
    }
    k += incr;
  }

  if (alpha2 < alpha1)
    alpha2 += 2 * M_PI;
  //else if (alpha2 == alpha1)
  else if (std::fabs(alpha2 - alpha1) < std::fabs(alpha1) * std::numeric_limits<double>::epsilon())
    alpha2 += 2 * M_PI;
}


/*!
  Compute the \f$ theta \f$ angle for each vpMeSite.

  \note The \f$ theta \f$ angle is useful during the tracking part.
*/
void
vpMeEllipse::updateTheta()
{
  vpMeSite p_me;
  double theta;
  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ++it){
    p_me = *it;
    vpImagePoint iP;
    iP.set_i(p_me.ifloat);
    iP.set_j(p_me.jfloat);
    computeTheta(theta, K, iP);
    p_me.alpha = theta;
    *it = p_me;
  }
}

/*!
  Suppress the vpMeSite which are no more detected as point which belongs to the ellipse edge.
*/
void
vpMeEllipse::suppressPoints()
{
  // Loop through list of sites to track
  std::list<vpMeSite>::iterator itList = list.begin();
  for(std::list<double>::iterator it=angle.begin(); it!=angle.end(); ){
    vpMeSite s = *itList;//current reference pixel
    if (s.getState() != vpMeSite::NO_SUPPRESSION)
    {
      itList = list.erase(itList);
      it = angle.erase(it);
    }
    else
    {
      ++itList;
      ++it;
    }
  }
}


/*!
  Seek along the ellipse edge defined by its equation, the two extremities of
  the ellipse (ie the two points with the smallest and the biggest \f$ \alpha \f$ angle.

  \param I : Image in which the ellipse appears.

  \exception vpTrackingException::initializationError : Moving edges not initialized.

*/
void
vpMeEllipse::seekExtremities(const vpImage<unsigned char>  &I)
{
  if (!me) {
    throw(vpException(vpException::fatalError,
                      "Moving edges on ellipse tracking not initialized"));
  }

  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();

  vpImagePoint ip;

  unsigned int  memory_range = me->getRange();
  me->setRange(2);

  double  memory_mu1 = me->getMu1();
  me->setMu1(0.5);

  double  memory_mu2 = me->getMu2();
  me->setMu2(0.5);

  double incr = vpMath::rad(2.0);

  if (alpha2-alpha1 < 2*M_PI-vpMath::rad(6.0))
  {
    vpMeSite P;
    double k = alpha1;
    double i1,j1;

    for (unsigned int i=0; i < 3; i++)
    {
      k -= incr;
      //while ( k < -M_PI ) { k+=2*M_PI; }

      i1 = b *cos(k); // equation of an ellipse
      j1 = a *sin(k); // equation of an ellipse
      P.ifloat = iPc.get_i() - se *j1 + ce *i1; P.i = (int)P.ifloat;
      P.jfloat = iPc.get_j() + ce *j1 + se *i1; P.j = (int)P.jfloat;

      if(!outOfImage(P.i, P.j, 5, rows, cols))
      {
        P.track(I,me,false);

        if (P.getState() == vpMeSite::NO_SUPPRESSION)
        {
          list.push_back(P);
          angle.push_back(k);
          if (vpDEBUG_ENABLE(3)) {
            ip.set_i( P.i );
            ip.set_j( P.j );

            vpDisplay::displayCross(I, ip, 5, vpColor::green);
          }
        }
        else {
          if (vpDEBUG_ENABLE(3)) {
            ip.set_i( P.i );
            ip.set_j( P.j );
            vpDisplay::displayCross(I, ip, 10, vpColor::blue);
          }
        }
      }
    }

    k = alpha2;

    for (unsigned int i=0; i < 3; i++)
    {
      k += incr;
      //while ( k > M_PI ) { k-=2*M_PI; }

      i1 = b *cos(k); // equation of an ellipse
      j1 = a *sin(k); // equation of an ellipse
      P.ifloat = iPc.get_i() - se *j1 + ce *i1; P.i = (int)P.ifloat;
      P.jfloat = iPc.get_j() + ce *j1 + se *i1; P.j = (int)P.jfloat;

      if(!outOfImage(P.i, P.j, 5, rows, cols))
      {
        P.track(I,me,false);

        if (P.getState() == vpMeSite::NO_SUPPRESSION)
        {
          list.push_back(P);
          angle.push_back(k);
          if (vpDEBUG_ENABLE(3)) {
            ip.set_i( P.i );
            ip.set_j( P.j );

            vpDisplay::displayCross(I, ip, 5, vpColor::green);
          }
        }
        else {
          if (vpDEBUG_ENABLE(3)) {
            ip.set_i( P.i );
            ip.set_j( P.j );
            vpDisplay::displayCross(I, ip, 10, vpColor::blue);
          }
        }
      }
    }
  }

  suppressPoints();

  me->setRange(memory_range);
  me->setMu1(memory_mu1);
  me->setMu2(memory_mu2);
}


/*!
  Finds in the list of vpMeSite the two points with the smallest and the biggest \f$ \alpha \f$ angle value, and stores them.
*/
void
vpMeEllipse::setExtremities()
{
  double alphamin = +1e6;
  double alphamax = -1e6;
  double imin = 0;
  double jmin = 0;
  double imax = 0;
  double jmax = 0;

  // Loop through list of sites to track
  std::list<double>::const_iterator itAngle = angle.begin();

  for(std::list<vpMeSite>::const_iterator itList=list.begin(); itList!=list.end(); ++itList){
    vpMeSite s = *itList;//current reference pixel
    double alpha = *itAngle;
    if (alpha < alphamin)
    {
      alphamin = alpha;
      imin = s.ifloat;
      jmin = s.jfloat;
    }

    if (alpha > alphamax)
    {
      alphamax = alpha;
      imax = s.ifloat;
      jmax = s.jfloat;
    }
    ++itAngle;
  }

  alpha1 = alphamin;
  alpha2 = alphamax;
  iP1.set_ij(imin,jmin);
  iP2.set_ij(imax,jmax);
}


/*!
  Least squares method used to make the tracking more robust. It
  ensures that the points taken into account to compute the right
  equation belong to the ellipse.
*/
void
vpMeEllipse::leastSquare()
{
  // Construction du systeme Ax=b
  // i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )
  unsigned int i;

  vpMeSite p_me;

  unsigned int iter =0;
  vpColVector b_(numberOfSignal());
  vpRobust r(numberOfSignal());
  r.setThreshold(2);
  r.setIteration(0);
  vpMatrix D(numberOfSignal(),numberOfSignal());
  D.eye();
  vpMatrix DA, DAmemory;
  vpColVector DAx;
  vpColVector w(numberOfSignal());
  w =1;
  unsigned int nos_1 = numberOfSignal();

  if (list.size() < 3)
  {
    throw(vpException(vpException::dimensionError,
                      "Not enought moving edges to track the ellipse"));
  }

  vpMatrix A(numberOfSignal(),5);
  vpColVector x(5);

  unsigned int k =0;
  for(std::list<vpMeSite>::const_iterator it=list.begin(); it!=list.end(); ++it){
    p_me = *it;
    if (p_me.getState() == vpMeSite::NO_SUPPRESSION)
    {
      A[k][0] = vpMath::sqr(p_me.jfloat);
      A[k][1] = 2 * p_me.ifloat * p_me.jfloat;
      A[k][2] = 2 * p_me.ifloat;
      A[k][3] = 2 * p_me.jfloat;
      A[k][4] = 1;

      b_[k] = - vpMath::sqr(p_me.ifloat);
      k++;
    }
  }

  while (iter < 4 )
  {
    DA = D*A;
    vpMatrix DAp;

    x = DA.pseudoInverse(1e-26) *D*b_;

    vpColVector residu(nos_1);
    residu = b_ - A*x;
    r.setIteration(iter);
    r.MEstimator(vpRobust::TUKEY,residu,w);

    k = 0;
    for (i=0; i < nos_1; i++)
    {
      D[k][k] =w[k] ;
      k++;
    }
    iter++;
  }

  k =0;
  for(std::list<vpMeSite>::iterator it=list.begin(); it!=list.end(); ++it){
    p_me = *it;
    if (p_me.getState() == vpMeSite::NO_SUPPRESSION)
    {
      if (w[k] < thresholdWeight)
      {
        p_me.setState(vpMeSite::M_ESTIMATOR);

        *it = p_me;
      }
      k++;
    }
  }
  for(i = 0; i < 5; i ++)
    K[i] = x[i];

  getParameters();
}


/*!
  Display the ellipse.

  \warning To effectively display the ellipse a call to
  vpDisplay::flush() is needed.

  \param I : Image in which the ellipse appears.
  \param col : Color of the displayed ellipse.
 */
void
vpMeEllipse::display(const vpImage<unsigned char> &I, vpColor col)
{
  vpMeEllipse::display(I,iPc,a,b,e,alpha1,alpha2,col);
}


/*!
  Initilization of the tracking. Ask the user to click counter clockwise on five points
  located on the ellipse edge to track.

  \warning The points should be selected counter clockwise.

  \param I : Image in which the ellipse appears.
*/
void
vpMeEllipse::initTracking(const vpImage<unsigned char> &I)
{
  const unsigned int n=5;
  vpImagePoint iP[n];

  for (unsigned int k =0; k < n; k++)
  {
    std::cout << "Click points "<< k+1 <<"/" << n;
    std::cout << " on the ellipse in the trigonometric order" <<std::endl;
    vpDisplay::getClick(I, iP[k], true);
    vpDisplay::displayCross(I, iP[k], 7, vpColor::red);
    vpDisplay::flush(I);
    std::cout << iP[k] << std::endl;
  }

  iP1 = iP[0];
  iP2 = iP[n-1];

  initTracking(I, n, iP);
}


/*!
  Initialization of the tracking. The ellipse is defined thanks to the
  coordinates of n points.

  \warning It is better to use at least five points to well estimate the ellipse parameters.
  \warning The n points should be selected counter clockwise.

  \param I : Image in which the ellipse appears.
  \param n : The number of points in the list.
  \param iP : A pointer to a list of points belonging to the ellipse edge.
*/
void
vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const unsigned int n,
                          vpImagePoint *iP)
{
  vpMatrix A(n,5);
  vpColVector b_(n);
  vpColVector x(5);

  // Construction du systeme Ax=b
  // i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )

  for (unsigned int k =0; k < n; k++)
  {
    A[k][0] = vpMath::sqr(iP[k].get_j());
    A[k][1] = 2* iP[k].get_i() * iP[k].get_j();
    A[k][2] = 2* iP[k].get_i();
    A[k][3] = 2* iP[k].get_j();
    A[k][4] = 1;

    b_[k] = - vpMath::sqr(iP[k].get_i());
  }

  K = A.pseudoInverse(1e-26)*b_;

  iP1 = iP[0];
  iP2 = iP[n-1];

  getParameters();

  computeAngle(iP1, iP2);

  expecteddensity = (alpha2-alpha1) / vpMath::rad((double)me->getSampleStep());

  display(I, vpColor::green);
  sample(I);

  vpMeTracker::initTracking(I);

  track(I);

  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

/*!
  Initialization of the tracking. The ellipse is defined thanks to a vector of
  n points.

  \warning It is better to use at least five points to well estimate the ellipse parameters.
  \warning The n points should be selected counter clockwise.

  \param I : Image in which the ellipse appears.
  \param iP : A vector of points belonging to the ellipse edge used to initialize the tracking.
*/
void
vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP)
{
  unsigned int n = (unsigned int)(iP.size());
  vpMatrix A(n,5);
  vpColVector b_(n);
  vpColVector x(5);

  // Construction du systeme Ax=b
  // i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )

  for (unsigned int k =0; k < n; k++)
  {
    A[k][0] = vpMath::sqr(iP[k].get_j());
    A[k][1] = 2* iP[k].get_i() * iP[k].get_j();
    A[k][2] = 2* iP[k].get_i();
    A[k][3] = 2* iP[k].get_j();
    A[k][4] = 1;

    b_[k] = - vpMath::sqr(iP[k].get_i());
  }

  K = A.pseudoInverse(1e-26)*b_;

  iP1 = iP[0];
  iP2 = iP[n-1];

  getParameters();

  computeAngle(iP1, iP2);

  expecteddensity = (alpha2-alpha1) / vpMath::rad((double)me->getSampleStep());

  display(I, vpColor::green);
  sample(I);

  vpMeTracker::initTracking(I);

  track(I);

  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

void
vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ic, double a_p, double b_p, double e_p,
                          double low_alpha, double high_alpha)
{
  iPc = ic;
  a = a_p;
  b = b_p;
  e = e_p;
  alpha1 = low_alpha;
  alpha2 = high_alpha;

  if (alpha2 <alpha1)
    alpha2 += 2 * M_PI;

  ce = cos(e);
  se = sin(e);

  display(I, vpColor::green);
  sample(I);

  vpMeTracker::initTracking(I);

  track(I);

  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

/*!
  Track the ellipse in the image I.

  \param I : Image in which the ellipse appears.
*/
void
vpMeEllipse::track(const vpImage<unsigned char> &I)
{
  vpMeTracker::track(I);

  // Estimation des parametres de la droite aux moindres carre
  suppressPoints();
  setExtremities();

  leastSquare();
  seekExtremities(I);
  setExtremities();
  leastSquare();

  // suppression des points rejetes par la regression robuste
  suppressPoints();
  setExtremities();

  //reechantillonage si necessaire
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste

  updateTheta();

  computeMoments();

  // Remise a jour de delta dans la liste de site me
  if (vpDEBUG_ENABLE(2))
  {
    display(I,vpColor::red);
    vpMeTracker::display(I);
    vpDisplay::flush(I);
  }
}

/*!
  Computes the 0 order moment \f$ m_{00} \f$ which represents the area of the ellipse.

  Computes the second central moments \f$ \mu_{20} \f$, \f$ \mu_{02} \f$ and \f$ \mu_{11} \f$
*/
void
vpMeEllipse::computeMoments()
{
  double tane = tan(-1/e);
  m00 = M_PI*a*b;
  m10 = m00*iPc.get_i();
  m01 = m00*iPc.get_j();
  m20 = m00*(a*a+b*b*tane*tane)/(4*(1+tane*tane))+m00*iPc.get_i()*iPc.get_i();
  m02 = m00*(a*a*tane*tane+b*b)/(4*(1+tane*tane))+m00*iPc.get_j()*iPc.get_j();
  m11 = m00*tane*(a*a-b*b)/(4*(1+tane*tane))+m00*iPc.get_i()*iPc.get_j();
  mu11 = m11 - iPc.get_j()*m10;
  mu02 = m02 - iPc.get_j()*m01;
  mu20 = m20 - iPc.get_i()*m10;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
 * \brief computeAngle
 */
void
vpMeEllipse::computeAngle(int ip1, int jp1, double &_alpha1,
                          int ip2, int jp2, double &_alpha2)
{
  getParameters();

  int number_of_points = 2000;
  double incr = 2 * M_PI / number_of_points; // angle increment

  double dmin1 = 1e6 ;
  double dmin2 = 1e6 ;

  double k =  -M_PI;
  while(k < M_PI) {

    double j1 = a *cos(k); // equation of an ellipse
    double i1 = b *sin(k); // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    double j11 = iPc.get_j() + ce *j1 - se *i1;
    double i11 = iPc.get_i() -( se *j1 + ce *i1);

    double  d = vpMath::sqr(ip1-i11) + vpMath::sqr(jp1-j11);
    if (d < dmin1)
    {
      dmin1 = d;
      alpha1 = k;
      _alpha1 = k;
    }
    d = vpMath::sqr(ip2-i11) + vpMath::sqr(jp2-j11);
    if (d < dmin2)
    {
      dmin2 = d;
      alpha2 = k;
      _alpha2 = k;
    }
    k += incr;
  }

  if (alpha2 <alpha1) alpha2 += 2*M_PI;

  vpCDEBUG(1) << "end vpMeEllipse::computeAngle(..)" << alpha1 << "  " << alpha2 << std::endl;
}


/*!
 * \brief computeAngle
 */
void
vpMeEllipse::computeAngle(int ip1, int jp1, int ip2, int jp2)
{

  double a1, a2;
  computeAngle(ip1,jp1,a1, ip2, jp2,a2);
}


void
vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const unsigned int n,
        unsigned *i, unsigned *j)
{
  vpMatrix A(n,5);
  vpColVector b_(n);
  vpColVector x(5);

  // Construction du systeme Ax=b
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  // A = (j^2 2ij 2i 2j 1)   x = (K0 K1 K2 K3 K4)^T  b = (-i^2 )

  for (unsigned int k =0; k < n; k++)
  {
    A[k][0] = vpMath::sqr(j[k]);
    A[k][1] = 2* i[k] * j[k];
    A[k][2] = 2* i[k];
    A[k][3] = 2* j[k];
    A[k][4] = 1;

    b_[k] = - vpMath::sqr(i[k]);
  }

  K = A.pseudoInverse(1e-26)*b_;
  //std::cout << K << std::endl;

  iP1.set_i( i[0] );
  iP1.set_j( j[0] );
  iP2.set_i( i[n-1] );
  iP2.set_j( j[n-1] );

  getParameters();
  computeAngle(iP1, iP2);
  display(I, vpColor::green);

  sample(I);

  //  2. On appelle ce qui n'est pas specifique
  {
    vpMeTracker::initTracking(I);
  }

  track(I);

  vpMeTracker::display(I);
  vpDisplay::flush(I);
}
#endif // Deprecated

/*!

  Display of the ellipse thanks to the equation parameters.

  \param I : The image used as background.

  \param center : Center of the ellipse

  \param A : Semiminor axis of the ellipse.

  \param B : Semimajor axis of the ellipse.

  \param E : Angle made by the major axis and the i axis of the image frame \f$ (i,j) \f$

  \param smallalpha : Smallest \f$ alpha \f$ angle in rad.

  \param highalpha : Highest \f$ alpha \f$ angle in rad.

  \param color : Color used to display th lines.

  \param thickness : Thickness of the drawings.
*/
void vpMeEllipse::display(const vpImage<unsigned char>& I, const vpImagePoint &center,
                          const double &A, const double &B, const double &E,
                          const double & smallalpha, const double &highalpha,
                          const vpColor &color, unsigned int thickness)
{
  double j1, i1;
  vpImagePoint iP11;
  double j2, i2;
  vpImagePoint iP22;
  j1 = j2 = i1 = i2 = 0;

  double incr = vpMath::rad(2); // angle increment

  vpDisplay::displayCross(I,center,20, vpColor::red, thickness);

  double k = smallalpha;
  while (k+incr<highalpha)
  {
    j1 = A *cos(k); // equation of an ellipse
    i1 = B *sin(k); // equation of an ellipse

    j2 = A *cos(k+incr); // equation of an ellipse
    i2 = B *sin(k+incr); // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    iP11.set_j ( center.get_j() + cos(E) *j1 - sin(E) *i1 );
    iP11.set_i ( center.get_i() -( sin(E) *j1 + cos(E) *i1) );
    // to get the coordinates of the point on the shifted ellipse
    iP22.set_j ( center.get_j() + cos(E) *j2 - sin(E) *i2 );
    iP22.set_i ( center.get_i() -( sin(E) *j2 + cos(E) *i2) );

    vpDisplay::displayLine(I, iP11, iP22, color, thickness);

    k += incr;
  }

  j1 = A *cos(smallalpha); // equation of an ellipse
  i1 = B *sin(smallalpha); // equation of an ellipse

  j2 = A *cos(highalpha); // equation of an ellipse
  i2 = B *sin(highalpha); // equation of an ellipse

  // (i1,j1) are the coordinates on the origin centered ellipse;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  iP11.set_j ( center.get_j() + cos(E) *j1 - sin(E) *i1 );
  iP11.set_i ( center.get_i() -( sin(E) *j1 + cos(E) *i1) );
  // to get the coordinates of the point on the shifted ellipse
  iP22.set_j ( center.get_j() + cos(E) *j2 - sin(E) *i2 );
  iP22.set_i ( center.get_i() -( sin(E) *j2 + cos(E) *i2) );

  vpDisplay::displayLine(I, center, iP11, vpColor::red, thickness);
  vpDisplay::displayLine(I, center, iP22, vpColor::blue, thickness);
}

/*!

  Display of the ellipse thanks to the equation parameters.

  \param I : The image used as background.

  \param center : Center of the ellipse

  \param A : Semiminor axis of the ellipse.

  \param B : Semimajor axis of the ellipse.

  \param E : Angle made by the major axis and the i axis of the image frame \f$ (i,j) \f$

  \param smallalpha : Smallest \f$ alpha \f$ angle in rad.

  \param highalpha : Highest \f$ alpha \f$ angle in rad.

  \param color : Color used to display th lines.

  \param thickness : Thickness of the drawings.
*/
void vpMeEllipse::display(const vpImage<vpRGBa>& I, const vpImagePoint &center,
                          const double &A, const double &B, const double &E,
                          const double & smallalpha, const double &highalpha,
                          const vpColor &color, unsigned int thickness)
{
  double j1, i1;
  vpImagePoint iP11;
  double j2, i2;
  vpImagePoint iP22;
  j1 = j2 = i1 = i2 = 0;

  double incr = vpMath::rad(2); // angle increment

  vpDisplay::displayCross(I,center,20, vpColor::red, thickness);

  double k = smallalpha;
  while (k+incr<highalpha)
  {
    j1 = A *cos(k); // equation of an ellipse
    i1 = B *sin(k); // equation of an ellipse

    j2 = A *cos(k+incr); // equation of an ellipse
    i2 = B *sin(k+incr); // equation of an ellipse

    // (i1,j1) are the coordinates on the origin centered ellipse;
    // a rotation by "e" and a translation by (xci,jc) are done
    // to get the coordinates of the point on the shifted ellipse
    iP11.set_j ( center.get_j() + cos(E) *j1 - sin(E) *i1 );
    iP11.set_i ( center.get_i() -( sin(E) *j1 + cos(E) *i1) );
    // to get the coordinates of the point on the shifted ellipse
    iP22.set_j ( center.get_j() + cos(E) *j2 - sin(E) *i2 );
    iP22.set_i ( center.get_i() -( sin(E) *j2 + cos(E) *i2) );

    vpDisplay::displayLine(I, iP11, iP22, color, thickness);

    k += incr;
  }

  j1 = A *cos(smallalpha); // equation of an ellipse
  i1 = B *sin(smallalpha); // equation of an ellipse

  j2 = A *cos(highalpha); // equation of an ellipse
  i2 = B *sin(highalpha); // equation of an ellipse

  // (i1,j1) are the coordinates on the origin centered ellipse;
  // a rotation by "e" and a translation by (xci,jc) are done
  // to get the coordinates of the point on the shifted ellipse
  iP11.set_j ( center.get_j() + cos(E) *j1 - sin(E) *i1 );
  iP11.set_i ( center.get_i() -( sin(E) *j1 + cos(E) *i1) );
  // to get the coordinates of the point on the shifted ellipse
  iP22.set_j ( center.get_j() + cos(E) *j2 - sin(E) *i2 );
  iP22.set_i ( center.get_i() -( sin(E) *j2 + cos(E) *i2) );

  vpDisplay::displayLine(I, center, iP11, vpColor::red, thickness);
  vpDisplay::displayLine(I, center, iP22, vpColor::blue, thickness);
}
