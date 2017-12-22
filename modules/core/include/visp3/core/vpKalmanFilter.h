/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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
 * Kalman filtering.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpKalmanFilter_h
#define vpKalmanFilter_h

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#include <math.h>

/*!
  \file vpKalmanFilter.h
  \brief Generic kalman filtering implementation
*/

/*!
  \class vpKalmanFilter
  \ingroup group_core_kalman
  \brief This class provides a generic Kalman filtering algorithm along with
  some specific state model (constant velocity, constant acceleration)
  which are implemented in the vpLinearKalmanFilterInstantiation class.

  The state evolution equation is given by:
  \f[
  {\bf x}_k= {\bf F}_{k-1} {\bf x}_{k-1} + {\bf w}_{k-1} \\
  \f]
  where \f${\bf x}_{k}\f$ is the unknown state at iteration \f$k\f$.

  The measurement equation is given by:
  \f[
  {\bf z}_k = {\bf H} {\bf x}_k + {\bf r}_k
  \f]
  where \f${\bf z}_{k}\f$ is the measure (also named observation) at iteration
  \f$k\f$.

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

  ViSP provides different state evolution models implemented in the
  vpLinearKalmanFilterInstantiation class.
*/
class VISP_EXPORT vpKalmanFilter
{
protected:
  // bool init_done ;

  //! Filter step or iteration. When set to zero, initialize the filter.
  long iter;

  //! Size of the state vector \f${\bf x}_k\f$.
  unsigned int size_state;
  //! Size of the measure vector \f${\bf z}_k\f$.
  unsigned int size_measure;
  //! Number of signal to filter.
  unsigned int nsignal;

  //! When set to true, print the content of internal variables during
  //! filtering() and prediction().
  bool verbose_mode;

public:
  vpKalmanFilter();
  explicit vpKalmanFilter(unsigned int n_signal);
  vpKalmanFilter(unsigned int size_state, unsigned int size_measure, unsigned int n_signal);
  /*! Destructor that does noting. */
  virtual ~vpKalmanFilter(){};
  /*!
    Set the number of signal to filter.
  */
  void setNumberOfSignal(unsigned int n_signal) { this->nsignal = n_signal; }

  // int init() { return init_done ; }
  void init(unsigned int size_state, unsigned int size_measure, unsigned int n_signal);
  void prediction();
  void filtering(const vpColVector &z);
  /*!
    Return the size of the state vector \f${\bf x}_{(k)}\f$ for one signal.
  */
  unsigned int getStateSize() { return size_state; };
  /*!
    Return the size of the measure vector \f${\bf z}_{(k)}\f$ for one signal.
  */
  unsigned int getMeasureSize() { return size_measure; };
  /*!
    Return the number of signal to filter.
  */
  unsigned int getNumberOfSignal() { return nsignal; };
  /*!
    Return the iteration number.
  */
  long getIteration() { return iter; }
  /*!
    Sets the verbose mode.
    \param on : If true, activates the verbose mode which consists in printing
    the Kalman filter internal values.
  */
  void verbose(bool on) { verbose_mode = on; };

public:
  /*!
    The updated state estimate \f${\bf x}_{k \mid k} \f$ where
    \f${\bf x}_{k \mid k} = {\bf x}_{k \mid k-1} + {\bf W}_k
    \left[ {\bf z}_k -  {\bf H x}_{k \mid k-1} \right]\f$.
  */
  vpColVector Xest;
  /*!
    The predicted state \f${\bf x}_{k \mid k-1} \f$ where
    \f$ {\bf x}_{k|k-1} = {\bf F}_{k-1} {\bf x}_{k-1\mid k-1}\f$.
  */
  vpColVector Xpre;
  //! Transition matrix \f${\bf F}\f$ that describes the evolution of the
  //! state.
  vpMatrix F;

  //! Matrix \f${\bf H}\f$ that describes the evolution of the measurements.
  vpMatrix H;

  //! Measurement noise covariance matrix \f${\bf R}\f$.
  vpMatrix R;
  //! Process noise covariance matrix \f${\bf Q}\f$.
  vpMatrix Q;
  /*! Sampling time \f$\Delta t\f$ in second between two succesive
      iterations. Only used in some specific state models implemented
      in vpLinearKalmanFilterInstantiation.*/
  double dt;
  /*!
    The state prediction covariance \f${\bf P}_{k \mid k-1} \f$ where
    \f$ {\bf P}_{k \mid k-1} = {\bf F}_{k-1}  {\bf P}_{k-1 \mid k-1} {\bf
    F}^T_{k-1}
    + {\bf Q}_k\f$.
  */
  vpMatrix Ppre;

  /*!
    The updated covariance of the state \f${\bf P}_{k \mid k}\f$
    where \f${\bf P}_{k \mid k} = \left({\bf I - W}_k {\bf H}
    \right) {\bf P}_{k \mid k-1}\f$.
  */
  vpMatrix Pest;

protected:
  /*!
    Filter gain \f${\bf W}_k\f$ where \f$ {\bf W}_k = {\bf P}_{k
    \mid k-1} {\bf H}^T \left[ {\bf H P}_{k \mid k-1} {\bf H}^T + {\bf
    R}_k \right]^{-1}\f$.
  */
  vpMatrix W;

  //! Identity matrix \f$ \bf I\f$.
  vpMatrix I;
};

#endif
