/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Adaptative gain.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/
/*!
  \file vpAdaptativeGain.h

  \brief Adaptative gain. The content of this file is deprecated. You
  should better use vpAdaptiveGain.
*/

#ifndef __VP_ADAPTATIVE_GAIN_H
#define __VP_ADAPTATIVE_GAIN_H


#include <iostream>

#include <visp/vpConfig.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

class vpColVector;
/*!
  \class vpAdaptativeGain
  
  \ingroup VsTask
  
  \brief Adaptative gain computation.

  \deprecated This class is deprecated. You shoud use
  vpAdaptive class instead.

  The formula used to compute the gain is the following :
  
  \f[ lambda (x) = a * exp (-b*x) + c \f]
  
  where \f$ a \f$, \f$ b \f$ and \f$ c \f$ are parameters which must be set 
  and \f$ x \f$ is the vector error of the task.
  
  By default, the parameters are set with default values:
  \f[ a = lambda(0) - lambda(inf) \f]
  \f[ b = lambda'(0) / a \f]
  \f[ c = lambda(inf) \f]
  
  with \f$ lambda(0) = 1.666 \f$, \f$ lambda(inf) = 0.1666 \f$ and \f$ lambda'(0) = 1.666 \f$.
  
  \f$ lambda(0)\f$ represents the gain in 0, \f$ lambda(inf)\f$ represents the gain to infinity and \f$ lambda'(0)\f$ represents the slope in 0.
  
*/

class VISP_EXPORT vpAdaptativeGain
{

public: /* constantes */

    static const double DEFAULT_LAMBDA_ZERO;
    static const double DEFAULT_LAMBDA_INFINI;
    static const double DEFAULT_LAMBDA_PENTE;


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

    vp_deprecated vpAdaptativeGain (void);

    /* --- INIT --------------------------------------------------------------- */
    void                        initFromConstant (double lambda);
    void                        initFromVoid (void);
    void                        initStandard (double en_zero,
					      double en_infini,
					      double pente_en_zero);


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
    double                      value_const (double val_e) const;

    /* \brief Calcule la valeur de lambda au point courrant et stockage du
     * resultat.
     *
     * La fonction calcule la valeur de lambda d'apres la valeur de la norme
     * de l'erreur, comme le fait la fonction valeur_const.
     * La fonction non constante stocke de plus le resultat dans this ->lambda.
     * \param val_e: valeur de la norme de l'erreur.
     * \return: valeur de gain au point courrant.
     */
    double                      value (double val_e) const;

    double                      limitValue_const (void) const;

    double                      limitValue (void) const;

    /* --- ACCESSORS ---------------------------------------------------------- */

    /*!
      Gets the last adaptative gain value which was stored in the class.
  
      \return It returns the last adaptative gain value which was stored in the class.
    */
    inline double                      getLastValue (void) const {return this ->lambda;}
   
    double                      operator() (double val_e) const;

    /* \brief Lance la fonction valeur avec la norme INFINIE du vecteur. */
    double                      operator()  (const vpColVector & e) const;

    /* \brief Idem function limitValue. */
    double                      operator() (void) const;


    /* --- IOSTREAM ----------------------------------------------------------- */

    friend std::ostream&        operator<< (std::ostream &os,
					    const vpAdaptativeGain& lambda);
};

#endif

#endif /*  __VP_ADAPTATIVE_GAIN_H	*/




/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
