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
 * Adaptive gain.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/
/*!
\file vpAdaptiveGain.h
\brief Adaptive gain
*/

#ifndef __VP_ADAPTIVE_GAIN_H
#define __VP_ADAPTIVE_GAIN_H

#include <visp3/core/vpConfig.h>
#include <iostream>

class vpColVector;
/*!
  \class vpAdaptiveGain
  
  \ingroup group_task
  
  \brief Adaptive gain computation.
  
  The formula used to compute the gain is the following :
  
  \f[ \lambda (x) = a * exp (-b*x) + c \f]
  
  where \f$ a \f$, \f$ b \f$ and \f$ c \f$ are constant parameters and \f$ x \f$ is the entry to consider.
  
  The parameters \f$a,b,c\f$ are not set directly. They are computed from three other parameters
  \f$\lambda(0), \lambda(\infty), {\dot \lambda}(0)\f$ that are more intuitive to tune:
  \f[ a = \lambda(0) - \lambda(\infty) \f]
  \f[ b = {\dot \lambda}(0) / a \f]
  \f[ c = \lambda(\infty) \f]
  
  where \f$ \lambda(0)\f$ represents the gain when \f$x=0\f$, \f$ \lambda(\infty)\f$ represents the gain when \f$x=\infty\f$
  and \f$ {\dot \lambda}(0)\f$ represents the slope of \f$\lambda(x)\f$ when \f$x=0\f$.
  
*/

class VISP_EXPORT vpAdaptiveGain
{

public: /* constantes */

    static const double DEFAULT_LAMBDA_ZERO;
    static const double DEFAULT_LAMBDA_INFINITY;
    static const double DEFAULT_LAMBDA_SLOPE;


private: /* Attributs*/
    /* Coefficient de la fonction de calcul de lambda.
     * lambda (x) = a * exp (-b*x) + c. */
    double                      coeff_a;
    double                      coeff_b;
    double                      coeff_c;

    /* Derniere valeur calculee.  */
    mutable double              lambda;



public:  /* Methodes*/

    /* --- CONSTRUCTOR -------------------------------------------------------- */

    vpAdaptiveGain ();
    vpAdaptiveGain (double c);
    vpAdaptiveGain (double gain_at_zero,
                    double gain_at_infinity,
                    double slope_at_zero);


    /* --- INIT --------------------------------------------------------------- */
    void                        initFromConstant (double c);
    void                        initFromVoid (void);
    void                        initStandard (double gain_at_zero,
                                              double gain_at_infinity,
                                              double slope_at_zero);


    /* --- MODIFIORS ---------------------------------------------------------- */
    double                      setConstant (void);


    /* --- COMPUTE ------------------------------------------------------------ */
    /* \brief Calcule la valeur de lambda au point courrant.
     *
     * Determine la valeur du lambda adaptatif en fonction de la valeur
     * de la norme de la fonction de tache e par extrapolation exponentielle.
     * La fonction est : (en_infini - en_zero) * exp (-pente * ||e|| ) + en_infini.
     * On a bien :
     *    - lambda(10^5) = en_infini ;
     *    - lambda(0) = en_zero ;
     *    - lambda(x ~ 0) ~ - pente * x + en_zero.
     * \param val_e: valeur de la norme de l'erreur.
     * \return: valeur de gain au point courrant.
     */
    double                      value_const (double x) const;

    /* \brief Calcule la valeur de lambda au point courrant et stockage du
     * resultat.
     *
     * La fonction calcule la valeur de lambda d'apres la valeur de la norme
     * de l'erreur, comme le fait la fonction valeur_const.
     * La fonction non constante stocke de plus le resultat dans this ->lambda.
     * \param val_e: valeur de la norme de l'erreur.
     * \return: valeur de gain au point courrant.
     */
    double                      value (double x) const;

    double                      limitValue_const (void) const;

    double                      limitValue (void) const;

    /* --- ACCESSORS ---------------------------------------------------------- */

    /*!
      Gets the last adaptive gain value which was stored in the class.
  
      \return It returns the last adaptive gain value which was stored in the class.
    */
    inline double               getLastValue (void) const {return this ->lambda;}
   
    double                      operator() (double x) const;

    /* \brief Lance la fonction valeur avec la norme INFINIE du vecteur. */
    double                      operator()  (const vpColVector & x) const;

    /* \brief Idem function limitValue. */
    double                      operator() (void) const;


    /* --- IOSTREAM ----------------------------------------------------------- */

    friend VISP_EXPORT std::ostream& operator<< (std::ostream &os, const vpAdaptiveGain& lambda);
};

#endif /*  __VP_ADAPTIVE_GAIN_H	*/
