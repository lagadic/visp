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
  \brief This class provides a generic Kalman filtering algorithm along with
  some specific state model (constant velocity, constant acceleration)
  which are implemented in the vpKalmanFilter class.

  The state evolution equation is given by:
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]
  where \f${\bf x}_{k}\f$ is the unknown state at iteration \f$k\f$.
  
  The measurement equation is given by:
  \f[
  {\bf z}_k = {\bf H} {\bf x}_k + {\bf r}_k
  \f]
  where \f${\bf z}_{k}\f$ is the measure (also named observation) at iteration \f$k\f$.
 
  The predicted state is obtained by:
  \f[
  {\bf x}_{k|k-1}  =  {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}
  \f]
  \f[
  {\bf P}_{k \mid k-1} = {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf F}^T_{k-1}
  + {\bf Q}_k \f]
  where
  <ul>
  <li> \f$ {\bf x}_{k|k-1}\f$ is the prediction of the state,
  <li> \f$ {\bf P}_{k \mid k-1}\f$ is the state prediction covariance matrix.
  </ul>
  Filtering equation are:
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

  where \f$ {\bf W}_k \f$ is the filter gain.

  Notice that there is a recursion for the inverse covariance
  \f[
  {\bf P}_{k \mid k}^{-1}=  {\bf P}_{k \mid k-1}^{-1} + {\bf H}^T {\bf
  R}^{-1} {\bf H}
  \f]
  where \f${\bf P}_{k \mid k}^{-1}\f$ is the inverse of the covariance matrix.

  ViSP provides different state evolution models implemented in the vpKalmanFilter class.
*/
class VISP_EXPORT vpKalman
{
protected :
  //bool init_done ;

  //! Filter step or iteration. When set to zero, initialize the filter. 
  long iter ;

  //! Size of the state vector \f${\bf x}_k\f$.
  int size_state ;
  //! Size of the measure vector \f${\bf z}_k\f$.
  int size_measure ;
  //! Number of signal to filter.
  int nsignal ;

  //! When set to true, print the content of internal variables during filtering() and prediction().
  bool verbose_mode;

public:
  vpKalman() ;
  vpKalman(int nsignal) ;
  vpKalman(int size_state, int size_measure, int nsignal) ;
  /*!
    Set the number of signal to filter.
  */
  void setNumberOfSignal(int nsignal)
  {
    this->nsignal = nsignal;
  }

  // int init() { return init_done ; }
  void init(int size_state, int size_measure, int nsignal) ;
  void prediction() ;
  void filtering(vpColVector &z) ;
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
    \param on : If true, activates the verbose mode which consists in printing the Kalman 
    filter internal values.
  */
  void verbose(bool on) { verbose_mode = on;};

public:
  /*!
    The updated state estimate \f${\bf x}_{k \mid k} \f$ where 
    \f${\bf x}_{k \mid k} = {\bf x}_{k \mid k-1} + {\bf W}_k  
    \left[ {\bf z}_k -  {\bf H x}_{k \mid k-1} \right]\f$.
  */
  vpColVector Xest ;
  /*!
    The predicted state \f${\bf x}_{k \mid k-1} \f$ where 
    \f$ {\bf x}_{k|k-1} = {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}\f$.
  */
  vpColVector Xpre ;
  //! Transition matrix \f${\bf F}\f$ that describes the evolution of the state.
  vpMatrix F ;

  //! Matrix \f${\bf H}\f$ that describes the evolution of the measurements. 
  vpMatrix H ;

  //! Measurement noise covariance matrix \f${\bf R}\f$.
  vpMatrix R ;
  //! Process noise covariance matrix \f${\bf Q}\f$.
  vpMatrix Q ;
  /*! Sampling time \f$\Delta t\f$ in second between two succesive
      iterations. Only used in some specific state models implemented
      in vpKalmanFilter.*/
  double dt ;

protected:
  /*!
    The state prediction covariance \f${\bf P}_{k \mid k-1} \f$ where 
    \f$ {\bf P}_{k \mid k-1} = {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf F}^T_{k-1} 
    + {\bf Q}_k\f$.
  */
  vpMatrix Ppre ;

  /*!  
    The updated covariance of the state \f${\bf P}_{k \mid k}\f$
    where \f${\bf P}_{k \mid k} = \left({\bf I - W}_k {\bf H}
    \right) {\bf P}_{k \mid k-1}\f$. 
  */
  vpMatrix Pest ;

  /*!  
    Filter gain \f${\bf W}_k\f$ where \f$ {\bf W}_k = {\bf P}_{k
    \mid k-1} {\bf H}^T \left[ {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf
    R}_k \right]^{-1}\f$.
  */
  vpMatrix W ;

  //! Identity matrix \f$ \bf I\f$.
  vpMatrix I ;
} ;



#endif
