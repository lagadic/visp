/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2009 Inria. All rights reserved.
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
 * Kalman filtering.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpKalman_h
#define vpKalman_h

#include <math.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

/*!
  \file vpKalman.h
  \brief Generic kalman filtering implementation
*/

/*!
  \class vpKalman
  \brief Generic kalman filtering implementation.

  The CKalman provide of a generic kalman filtering algorithm along with some
  specific state model (constant velocity, constant acceleration)

  Équation d'etat
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]
  où
  <ul>
  <li>  \f${\bf z}{k}\f$ est la mesure à l'itération \f$k\f$
  <li> \f${\bf x}{k}\f$ est la valeur inconnue de l'état à l'itération \f$k\f$
  </ul>
  Equation de mesure
  \f[
  {\bf z}_k = {\bf H} {\bf x}_k + {\bf r}_k
  \f]
  Équations de prédiction
  \f[
  {{\bf x}}_{k|k-1}  =  {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}
  \f]
  \f[
  {\bf P}_{k \mid k-1} = {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf F}^T_{k-1}
  + {\bf Q}_k \f]
  où
  <ul>
  <li> \f$ {{\bf x}}_{k|k-1}\f$ est la valeur prédite de l'état
  <li> \f$ {\bf P}_{k \mid k-1}\f$ est la matrice de covariance de l'erreur de
  prediction
  </ul>
  Équations de filtrage
  \f[
  {\bf W}_k = {\bf P}_{k \mid k-1} {\bf H}^T
  \left[  {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf R}_k \right]^{-1}
  \f]
  \f[
  {\bf x}_{k \mid k} =  {\bf x}_{k \mid k-1} + {\bf W}_k  \left[ {\bf z}_k -
  {\bf H x}_{k \mid k-1} \right]
  \f]
  \f[
  {\bf P}_{k \mid k} = \left({\bf I - W}_k {\bf H} \right)  {\bf P}_{k \mid
  k-1} \f]

  \f$ {\bf W}_k \f$ est le gain du filtre de Kalman

  Notons que \f$ {\bf P}_{k \mid k}\f$  peut aussi s'écrire :
  \f[
  {\bf P}_{k \mid k}^{-1}=  {\bf P}_{k \mid k-1}^{-1} + {\bf H}^T {\bf
  R}^{-1} {\bf H}
  \f]
  ou \f${\bf P}_{k \mid k}^{-1}\f$ désigne l'inverse de la matrice de covariance.


  La classe implémente différent modèle d'évolution
  <ul>
  <li> Modèle à vitesse constante
  <li>  Modèle à accélération constante
  <li> Modèle de Singer
  </ul>
*/
class vpKalman
{
protected :
  bool init_done ;
  long iter ;

  int size_state ;
  int size_measure ;
  int nsignal ;

  bool verbose_mode;

public:
  /*!
    Set the number of signal to filter.
  */
  void setNumberOfSignal(int nsignal)
  {
    this->nsignal = nsignal;
  }
  /*!
    Return the size of the state vector \f${\bf x}_{(k)}\f$ for one signal.
  */
  int getStateSize() { return size_state; };
  /*!
    Return the size of the measure vector \f${\bf z}_{(k)}\f$ for one signal.
  */
  int getMeasureSize() { return size_measure; };
  /*!
    Return the number of signal to filter.
  */
  int getNumberOfSignal() { return nsignal; };
  /*!
    Return the iteration number.
  */
  int getIteration() { return iter ; }
  /*!
    Sets the verbose mode.
    \param on : If true, activates the verbose mode by printing the Kalman 
    filter internal values.
  */
  void verbose(bool on) { verbose_mode = on;};

public:
  //!  \f${\bf x}_{k \mid k} \f$ valeur estime de l'etat   \f${\bf x}_{k \mid k} =  {\bf x}_{k \mid k-1} + {\bf W}_k  \left[ {\bf z}_k -  {\bf H x}_{k \mid k-1} \right]\f$
  vpColVector Xest ;
  //! \f${\bf x}_{k \mid k-1} \f$ : valeur predite   de l'etat \f$ {{\bf x}}_{k|k-1}  =  {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}\f$
  vpColVector Xpre ;

protected:
  //! \f${\bf P}_{k \mid k-1} \f$ : matrice de covariance de l'erreur de prediction
  //! \f$ {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf F}^T_{k-1}   + {\bf Q}_k\f$
  vpMatrix Ppre ;

  //!  \f${\bf P}_{k \mid k}\f$ matrice de covariance de l'erreur d'estimation
  //!  \f${\bf P}_{k \mid k} = \left({\bf I - W}_k {\bf H} \right)  {\bf P}_{k \mid  k-1}\f$
  vpMatrix Pest ;

  //! \f${\bf W}_k\f$ : Gain du filtre de Kalman
  //! \f$ {\bf W}_k = {\bf P}_{k \mid k-1} {\bf H}^T   \left[  {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf R}_k \right]^{-1}\f$
  vpMatrix W ;

  //! \f$ \bf I\f$ : matrice identite
  vpMatrix I ;

public:
  //! matrice décrivant le modele d'evolution de l'etat
  vpMatrix F ;

  //! matrice décrivant le modele d'evolution de la mesure
  vpMatrix H ;

  //! Variance du bruits sur le modele de mesure
  vpMatrix R ;
  //! Variance du bruits sur le modele d'etat
  vpMatrix Q ;

public:
  vpKalman() ;
  vpKalman(int nsignal) ;
  vpKalman(int size_state, int size_measure, int nsignal) ;

  int init() { return init_done ; }
  void init(int size_state, int size_measure, int nsignal) ;
  /*
  void initFilterCteAcceleration(double dt,
				vpColVector &Z0,
				vpColVector &Z1,
				vpColVector &Z2,
				vpColVector  &sigma_noise,
				vpColVector &sigma_state ) ;
  void initFilterCteVelocity(double dt,
			    vpColVector &Z0,
			    vpColVector &Z1,
			    vpColVector  &sigma_noise,
			    vpColVector &sigma_state) ;

  void initFilterSinger(double dt,
		       double a,
		       vpColVector &Z0,
		       vpColVector &Z1,
		       vpColVector  &sigma_noise,
		       vpColVector &sigma_state) ;
  */

  void prediction() ;
  void filtering(vpColVector &Xmes) ;
  double dt ;
} ;



#endif
