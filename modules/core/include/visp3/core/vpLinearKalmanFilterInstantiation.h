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

#ifndef vpLinearKalmanFilterInstantiation_h
#define vpLinearKalmanFilterInstantiation_h

#include <visp3/core/vpKalmanFilter.h>

#include <math.h>

/*!
  \file vpLinearKalmanFilterInstantiation.h
  \brief Implementation of some specific linear Kalman filters.
*/

/*!
  \class vpLinearKalmanFilterInstantiation
  \ingroup group_core_kalman
  \brief This class provides an implementation of some specific linear Kalman
  filters.
*/
class VISP_EXPORT vpLinearKalmanFilterInstantiation : public vpKalmanFilter
{
public:
  /*!
    Selector used to set the Kalman filter state model.
  */
  typedef enum {
    /*! Consider the state as a constant velocity model with white
        noise. Measures available are the succesive positions of the
        target. To know more about this state model, see
        initStateConstVel_MeasurePos(). */
    stateConstVel_MeasurePos,
    /*! Consider the state as a constant velocity model with colored noise
        measurements as acceleration terms. Measured available are the
        velocities of the target. To know more about this state model,
        see initStateConstVelWithColoredNoise_MeasureVel(). */
    stateConstVelWithColoredNoise_MeasureVel,
    /*! Consider the state as a constant acceleration model with colored noise
        measurements as acceleration terms. Measured available are the
        velocities of the target. To know more about this state model,
        see initStateConstAccWithColoredNoise_MeasureVel(). */
    stateConstAccWithColoredNoise_MeasureVel,
    /*! Used to indicate that the state model is not initialized. */
    unknown
  } vpStateModel;

  /*!
    Default linear Kalman filter.

    By default the state model is unknown and set to
    vpLinearKalmanFilterInstantiation::unknown.
  */
  vpLinearKalmanFilterInstantiation() : model(unknown){};

  /*! Destructor that does nothng. */
  virtual ~vpLinearKalmanFilterInstantiation(){};
  /*!
    Return the current state model.
   */
  inline vpStateModel getStateModel() { return model; }
  void filter(vpColVector &z);

  /*! @name Generic linear filter initializer */
  //@{
  inline void setStateModel(vpStateModel model);

  void initFilter(unsigned int nsignal, vpColVector &sigma_state, vpColVector &sigma_measure, double rho, double dt);
  //@}

  /*! @name Linear filter initializer with constant velocity models */
  //@{
  void initStateConstVel_MeasurePos(unsigned int nsignal, vpColVector &sigma_state, vpColVector &sigma_measure,
                                    double dt);
  void initStateConstVelWithColoredNoise_MeasureVel(unsigned int nsignal, vpColVector &sigma_state,
                                                    vpColVector &sigma_measure, double rho);
  //@}

  /*! @name Linear filter initializer with constant acceleration models */
  //@{
  void initStateConstAccWithColoredNoise_MeasureVel(unsigned int nsignal, vpColVector &sigma_state,
                                                    vpColVector &sigma_measure, double rho, double dt);
  //@}

protected:
  vpStateModel model;
};

/*!
  Set the Kalman state model. Depending on the state model, we set
  the state vector size and the measure vector size.

  The example below shows how to use this method and then to get the
  size of the state and measure vectors.

  \code
#include <visp3/core/vpLinearKalmanFilterInstantiation.h>

int main()
{
  vpLinearKalmanFilterInstantiation kalman;

  kalman.setStateModel(vpLinearKalmanFilterInstantiation::stateConstVelWithColoredNoise_MeasureVel);
  // Value is 2
  std::cout << "State vector size: " << kalman.getStateSize() << std::endl;
  // Value is 1
  std::cout << "Measure vector size: " << kalman.getMeasureSize() << std::endl;
}
  \endcode
*/
void vpLinearKalmanFilterInstantiation::setStateModel(vpStateModel mdl)
{
  this->model = mdl;
  switch (model) {
  case stateConstVel_MeasurePos:
  case stateConstVelWithColoredNoise_MeasureVel:
    size_state = 2;
    size_measure = 1;
    break;
  case stateConstAccWithColoredNoise_MeasureVel:
    size_state = 3;
    size_measure = 1;
    break;
  case unknown:
    size_state = 0;
    size_measure = 0;
    break;
  }
}

#endif
