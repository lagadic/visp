/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2009 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire deBeaulieu
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
 * Kalman filtering.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file vpKalman.cpp
  \brief Generic kalman filtering implementation.
*/

#include <math.h>
#include <stdlib.h>

#include <visp/vpKalman.h>

/*!
  Initialize the Kalman filter.
  
  \param size_state : Size of the state vector \f${\bf x}_{(k)}\f$ for one signal.

  \param size_measure : Size of the measure vector \f${\bf z}_{(k)}\f$
  for one signal.

  \param nsignal : Number of signal to filter.
*/
void
vpKalman::init(int size_state, int size_measure, int nsignal)
{
  this->size_state = size_state;
  this->size_measure = size_measure ;
  this->nsignal = nsignal ;
  F.resize(size_state*nsignal, size_state*nsignal) ;
  H.resize(size_measure*nsignal,  size_state*nsignal) ;

  R.resize(size_measure*nsignal, size_measure*nsignal) ;
  Q.resize(size_state*nsignal, size_state*nsignal) ;

  Xest.resize(size_state*nsignal) ; Xest = 0;
  Xpre.resize(size_state*nsignal) ; Xpre = 0 ;

  Pest.resize(size_state*nsignal, size_state*nsignal) ; Pest = 0 ;

  I.resize(size_state*nsignal, size_state*nsignal) ;
  init_done = false ;
  iter = 0 ;
  dt = -1 ;
}

/*!
  Construct a default Kalman filter.

  The verbose mode is by default desactivated.
  
*/
vpKalman::vpKalman()
{
  verbose(false);
  init_done = false ;
  this->size_state = 0;
  this->size_measure = 0 ;
  this->nsignal = 0 ;
  iter = 0 ;
  dt = -1 ;
}

/*!
  Construct a default Kalman filter by setting the number of signal to filter.

  The verbose mode is by default desactivated.
  
  \param nsignal : Number of signal to filter.
*/
vpKalman::vpKalman(int nsignal)
{
  verbose(false);
  init_done = false;
  this->size_state = 0;
  this->size_measure = 0 ;
  this->nsignal = nsignal;
  iter = 0 ;
  dt = -1 ;
}

/*!
  Construct a Kalman filter.

  The verbose mode is by default desactivated.
  
  \param size_state : Size of the state vector \f${\bf x}_{(k)}\f$ for one signal.

  \param size_measure : Size of the measure vector \f${\bf z}_{(k)}\f$
  for one signal.

  \param nsignal : Number of signal to filter.
*/
vpKalman::vpKalman(int size_state, int size_measure, int nsignal)
{
  verbose(false);
  init( size_state, size_measure, nsignal) ;
}

/*!
  \brief Mise a jour du filtre de Kalman : Etape de prediction

  �quations de pr�diction
  \f[
  {{\bf x}}_{k|k-1}   =  {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}
  \f]
  \f[
  {\bf P}_{k \mid k-1}  = {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf F}^T_{k-1}
  + {\bf Q}_k
  \f]

*/

void
vpKalman::prediction()
{
  if (Xest.getRows() != size_state*nsignal) {
    std::cout << " in vpKalman::prediction()" << Xest.getRows()
	      <<" " << size_state*nsignal<<  std::endl ;
    std::cout << " Error : Filter non initialized " << std::endl;
    exit(1) ;
  }

//   if (!init_done) {
//     std::cout << " in vpKalman::prediction()" << Xest.getRows()<<" " << size_state<<  std::endl ;
//     std::cout << " Error : Filter non initialized " << std::endl;
//     exit(1) ;
//     return;
//   }
  
  if (verbose_mode) {
    std::cout << "F = " << std::endl <<  F << std::endl ;
    std::cout << "Xest = "<< std::endl  << Xest << std::endl  ;  
  }
  // Prediction
  // Bar-Shalom  5.2.3.2
  Xpre = F*Xest  ;
  if (verbose_mode) {
    std::cout << "Xpre = "<< std::endl  << Xpre << std::endl  ;
    std::cout << "Q = "<< std::endl  << Q << std::endl  ;  
    std::cout << "Pest " << std::endl << Pest << std::endl ;
  }
  // Bar-Shalom  5.2.3.5
  Ppre = F*Pest*F.t() + Q ;

  // Matrice de covariance de l'erreur de prediction
  if (verbose_mode) 
    std::cout << "Ppre " << std::endl << Ppre << std::endl ;
}

/*!
  \brief  Mise a jour du filtre de Kalman : Etape de Filtrage

  \param Z : nouvelle mesure

  �quations de filtrage
  \f[
  {\bf x}_{k \mid k} = {\bf x}_{k \mid k-1} + {\bf W}_k  \left[ {\bf Z}_k -
  {\bf H x}_{k \mid k-1} \right]
  \f]
  gain du filtre de Kalman
  \f[
  {\bf W_k} = {\bf P}_{k \mid k-1} {\bf H}^T
  \left[  {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf R}_k \right]^{-1}
  \f]
  \f[
  {\bf P}_{k \mid k} = \left({\bf I - K}_k {\bf H} \right)  {\bf P}_{k \mid k-1}
  \f]

*/
void
vpKalman::filtering(vpColVector &Z)
{
  if (verbose_mode)
    std::cout << "Z " << std::endl << Z << std::endl ;
  // Bar-Shalom  5.2.3.11
  vpMatrix S =  H*Ppre*H.t() + R ;
  if (verbose_mode)
    std::cout << "S " << std::endl << S << std::endl ;

  W = (Ppre * H.t())* (S).inverseByLU() ;
  if (verbose_mode)
    std::cout << "W " << std::endl << W << std::endl ;
  // Bar-Shalom  5.2.3.15
  Pest = Ppre - W*S*W.t() ;
  if (verbose_mode)
    std::cout << "Pest " << std::endl << Pest << std::endl ;

  if (0) {
    // Bar-Shalom  5.2.3.16
    // numeriquement plus stable que  5.2.3.15
    vpMatrix  Pestinv  ;
    Pestinv = Ppre.inverseByLU()  + H.t() * R.inverseByLU()*H ;
    Pest =   Pestinv.inverseByLU() ;
  }
  // Bar-Shalom  5.2.3.12 5.2.3.13 5.2.3.7
  Xest = Xpre + (W*(Z - (H*Xpre))) ;
  if (verbose_mode)
    std::cout << "Xest " << std::endl << Xest << std::endl ;
  
  iter++ ;
}


#if 0
/*!
  \brief Filter initialization for a constant velocity model

  \param dt : time between two measures
  \param Z0 : Measure at iteration 0.
  \param Z1 : Measure at iteration 1.
  \param Vn : Variance of measure noise
  \param Vw : Variance of the state noise

  mod�le d'�tat
  \f[ {\bf x} = \left[\; y  \quad \frac{\partial y}{\partial t}\;\right]^T = \left[\;  y  \quad \dot{ y}\;\right]^T
  \f]

  Mod�lisation des filtres
  \f[
  \begin{array}{rcl} \\
  {\bf x}(t+1) &= &  {\bf Fx} (t) + {\bf Q} \\
  {\bf z}(t) &=& {\bf H x}(t) + {\bf R}
  \end{array}
  \f]


  La matrice F d�crit le mod�le d'�volution de l'�tat. Dans le cas pr�sent elle
  est donn�e par :
  \f[
 {\bf F}= \left( \begin{array}{cc} 1 & \Delta t  \\ 0 & 1 \end{array} \right)
  \f]

  Le bruit \f${\bf Q} = \left( \begin{array}{c} \;q_1 \quad q_2\;  \end{array} \right)^T\f$
  vient mod�liser les variations sur le mod�le � vitesse constante (dues aux acc�l�rations)

  En effet on a:
  \f[
  \left\{
  \begin{array}{rcl}
  y(t+1)& =& y(t) + \Delta(t) \dot y(t) + \underbrace{\frac{\Delta t^2}{2} \ddot y(t)}_{q_1} \\
  \dot y(t+1) &=& \dot y(t) + \underbrace{\Delta(t) \ddot y(t)}_{q_2}
  \end{array}
  \right.
  \f]
  et donc
  \f[
  \left\{
  \begin{array}{rcccccc}
  y(t+1)& =& y(t) &+& \Delta(t) \dot y(t) &+& {q_1} \\
  \dot y(t+1) &=& & &\dot y(t) &+& {q_2}
  \end{array}
  \right. \qquad \Leftrightarrow  \qquad {\bf x}(t+1) = {\bf F x}(t) + Q
  \f]
*/
void
vpKalman::initFilterCteVelocity(double dt, 
				vpColVector &Z0, 
				vpColVector &Z1, 
				vpColVector  &sigma_noise, 
				vpColVector &sigma_state )
{
  init_done = true ;

  this->dt = dt ;

  double dt2 = dt*dt ;
  double dt3 = dt2*dt ;

  Pest =0 ;
  // initialise les matrices decrivant les modeles
  for (int i=0;  i < size_measure ;  i++ )
  {
    // modele sur l'etat

    //         | 1  dt |
    //     F = |       |
    //         | 0   1 |

    F[2*i][2*i] = 1 ;
    F[2*i][2*i+1] = dt ;
    F[2*i+1][2*i+1] = 1 ;


    // modele sur la mesure
    H[i][2*i] = 1 ;
    H[i][2*i+1] = 0 ;

    double sR = sigma_noise[i] ;
    double sQ = sigma_state[i] ;

    // bruit de mesure
    R[i][i] = sR ;
    // bruit d'etat 6.2.2.12

    Q[2*i][2*i] = sQ *dt3/3 ;
    Q[2*i][2*i+1] = sQ*dt2/2;
    Q[2*i+1][2*i] =sQ *dt2/2 ;
    Q[2*i+1][2*i+1] =sQ*dt  ;

    // T[1|1]
    Pest[2*i][2*i]     = sR ;
    Pest[2*i][2*i+1]     =  sR/(2*dt) ;
    Pest[2*i+1][2*i]     =    sR/(2*dt) ;
    Pest[2*i+1][2*i+1] =  sQ*2*dt/3.0+ sR/(2*dt2) ;


    Xest[2*i] = Z1[i] ;
    Xest[2*i+1] = (Z1[i] - Z0[i])/dt ; ;

  }
}


/*!
  \brief Filter initialization for a constant velocity model

  \param dt : time between two measures
  \param Vn : Variance of measure noise
  \param Vw : Variance of the state noise

  mod�le d'�tat
  \f[ S = \left[\; y  \quad \frac{\partial y}{\partial t}\;\right]^T = \left[\;  y  \quad \dot{ y}\;\right]^T
  \f]

  Mod�lisation des filtres
  \f[
  \begin{array}{rclll} \\
  S(t+1) &= & F S(t) + W(t)&~~~~~~~~~~~& S(t+1) \mbox{ est un vecteur} \left[\;
  y\quad \dot y\;\right]^T \\
  X(t) &=& C S(t) + N(t)&& X(t) \mbox{ est un scalaire}
  \end{array}
  \f]


  La matrice F d�crit le mod�le d'�volution de l'�tat. Dans le cas pr�sent elle
  est donn�e par :
  \f[
  F= \left( \begin{array}{cc} 1 & \Delta t  \\ 0 & 1 \end{array} \right)
  \f]

  Le bruit \f$W = \left( \begin{array}{c} \;W_1 \quad W_2\;  \end{array} \right)^T\f$
  vient mod�liser les variations sur le mod�le � vitesse constante (dues aux acc�l�rations)

  En effet on a:
  \f[
  \left\{
  \begin{array}{rcl}
  y(t+1)& =& y(t) + \Delta(t) \dot y(t) + \underbrace{\frac{\Delta t^2}{2} \ddot y(t)}_{W_1} \\
  \dot y(t+1) &=& \dot y(t) + \underbrace{\Delta(t) \ddot y(t)}_{W_2}
  \end{array}
  \right.
  \f]
  et donc
  \f[
  \left\{
  \begin{array}{rcccccc}
  y(t+1)& =& y(t) &+& \Delta(t) \dot y(t) &+& {W_1} \\
  \dot y(t+1) &=& & &\dot y(t) &+& {W_2}
  \end{array}
  \right. \qquad \Leftrightarrow  \qquad S(t+1) = F S(t) + W
  \f]
*/
void
vpKalman::initFilterCteAcceleration(double dt,
				    vpColVector &Z0,
				    vpColVector &Z1,
				    vpColVector &Z2,
				    vpColVector  &sigma_noise,
				    vpColVector &sigma_state )
{
  this->dt = dt ;

  double dt2 = dt*dt ;
  double dt3 = dt2*dt ;
  double dt4 = dt3*dt ;
  double dt5 = dt4*dt ;

  init_done = true ;

  Pest =0 ;
  // initialise les matrices decrivant les modeles
  for (int i=0;  i < size_measure ;  i++ )
  {
    // modele sur l'etat

    //         | 1  dt  dt2/2 |
    //     F = | 0   1   dt   |
    //         | 0   0    1   |

    F[3*i][3*i] = 1 ;
    F[3*i][3*i+1] = dt ;
    F[3*i][3*i+2] = dt*dt/2 ;
    F[3*i+1][3*i+1] = 1 ;
    F[3*i+1][3*i+2] = dt ;
    F[3*i+2][3*i+2] = 1 ;


    // modele sur la mesure
    H[i][3*i] = 1 ;
    H[i][3*i+1] = 0 ;
    H[i][3*i+2] = 0 ;

    double sR = sigma_noise[i] ;
    double sQ = sigma_state[i] ;

    // bruit de mesure
    R[i][i] = (sR) ;

    // bruit d'etat 6.2.3.9
    Q[3*i  ][3*i  ] =  sQ * dt5/20;
    Q[3*i  ][3*i+1] =  sQ * dt4/8;
    Q[3*i  ][3*i+2] =  sQ * dt3/6 ;

    Q[3*i+1][3*i  ] = sQ * dt4/8 ;
    Q[3*i+1][3*i+1] = sQ * dt3/3 ;
    Q[3*i+1][3*i+2] = sQ * dt2/2 ;

    Q[3*i+2][3*i  ] = sQ * dt3/6 ;
    Q[3*i+2][3*i+1] = sQ * dt2/2.0 ;
    Q[3*i+2][3*i+2] = sQ * dt ;


    // Initialisation pour la matrice de covariance sur l'etat

    Pest[3*i  ][3*i  ] = sR ;
    Pest[3*i  ][3*i+1] = 1.5/dt*sR ;
    Pest[3*i  ][3*i+2] = sR/(dt2) ;

    Pest[3*i+1][3*i  ] = 1.5/dt*sR ;
    Pest[3*i+1][3*i+1] = dt3/3*sQ + 13/(2*dt2)*sR ;
    Pest[3*i+1][3*i+2] = 9*dt2*sQ/40.0 +6/dt3*sR ;

    Pest[3*i+2][3*i  ] = sR/(dt2) ;
    Pest[3*i+2][3*i+1] = 9*dt2*sQ/40.0 +6/dt3*sR ;
    Pest[3*i+2][3*i+2] = 23*dt/30.0*sQ+6.0/dt4*sR ;


    // Initialisation pour l'etat

    Xest[3*i] = Z2[i] ;
    Xest[3*i+1] = ( 1.5 *Z2[i] - Z1[i] -0.5*Z0[i] ) /( 2*dt ) ;
    Xest[3*i+2] = ( Z2[i] - 2*Z1[i] + Z0[i] ) /( dt*dt ) ;

  }
}

void
vpKalman::initFilterSinger(double dt,
			  double a,
			  vpColVector &Z0,
			  vpColVector &Z1,
			  vpColVector  &sigma_noise,
			  vpColVector &sigma_state )
{
  this->dt = dt ;

  double dt2 = dt*dt ;
  double dt3 = dt2*dt ;

  double a2 = a*a ;
  double a3 = a2*a ;
  double a4 = a3*a ;

  init_done = true ;

  Pest =0 ;
  // initialise les matrices decrivant les modeles
  for (int i=0;  i < size_measure ;  i++ )
  {
    // modele sur l'etat

    //         | 1  dt  dt2/2 |
    //     F = | 0   1   dt   |
    //         | 0   0    1   |

    F[3*i][3*i] = 1 ;
    F[3*i][3*i+1] = dt ;
    F[3*i][3*i+2] = 1/a2*(1+a*dt+exp(-a*dt)) ;
    F[3*i+1][3*i+1] = 1 ;
    F[3*i+1][3*i+2] = 1/a*(1-exp(-a*dt)) ;
    F[3*i+2][3*i+2] = exp(-a*dt) ;


    // modele sur la mesure
    H[i][3*i] = 1 ;
    H[i][3*i+1] = 0 ;
    H[i][3*i+2] = 0 ;

    double sR = sigma_noise[i] ;
    double sQ = sigma_state[i] ;

    R[i][i] = (sR) ; // bruit de mesure 1.5mm

    Q[3*i  ][3*i  ] =  sQ/a4*(1-exp(-2*a*dt)+2*a*dt+2*a3/3*dt3-2*a2*dt2-4*a*dt*exp(-a*dt) ) ;
    Q[3*i  ][3*i+1] =  sQ/a3*(1+exp(-2*a*dt)-2*exp(-a*dt)+2*a*dt*exp(-a*dt)-2*a*dt+a2*dt2 ) ;
    Q[3*i  ][3*i+2] =  sQ/a2*(1-exp(-2*a*dt)-2*a*dt*exp(-a*dt) ) ;

    Q[3*i+1][3*i  ] =  Q[3*i  ][3*i+1] ;
    Q[3*i+1][3*i+1] = sQ/a2*(4*exp(-a*dt)-3-exp(-2*a*dt)+2*a*dt ) ;
    Q[3*i+1][3*i+2] = sQ/a*(exp(-2*a*dt)+1- 2*exp(-a*dt)) ;

    Q[3*i+2][3*i  ] = Q[3*i  ][3*i+2] ;
    Q[3*i+2][3*i+1] = Q[3*i+1][3*i+2] ;
    Q[3*i+2][3*i+2] = sQ*(1-exp(-2*a*dt) ) ;


    // Initialisation pour la matrice de covariance sur l'etat
    Pest[3*i  ][3*i  ] = sR ;
    Pest[3*i  ][3*i+1] = 1/dt*sR ;
    Pest[3*i  ][3*i+2] = 0 ;

    Pest[3*i+1][3*i  ] = 1/dt*sR ;
    Pest[3*i+1][3*i+1] = 2*sR/dt2 + sQ/(a4*dt2)*(2-a2*dt2+2*a3*dt3/3.0 -2*exp(-a*dt)-2*a*dt*exp(-a*dt));
    Pest[3*i+1][3*i+2] = sQ/(a2*dt)*(exp(-a*dt)+a*dt-1)  ;

    Pest[3*i+2][3*i  ] = 0  ;
    Pest[3*i+2][3*i+1] =  Pest[3*i+1][3*i+2] ;
    Pest[3*i+2][3*i+2] = 0  ;


    // Initialisation pour l'etat

    Xest[3*i]   = Z1[i] ;
    Xest[3*i+1] = ( Z1[i] - Z0[i] ) /(dt ) ;
    Xest[3*i+2] = 0 ;

  }
}

#endif
