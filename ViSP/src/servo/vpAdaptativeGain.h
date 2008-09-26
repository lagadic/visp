/****************************************************************************
 *
 * $Id: vpAdaptativeGain.h,v 1.6 2008-09-26 15:20:58 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Adaptative gain.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/
/*!
\file vpAdaptativeGain.h
\brief Adaptative gain
*/

#ifndef __VP_ADAPTATIVE_GAIN_H
#define __VP_ADAPTATIVE_GAIN_H


#include <iostream>

#include <visp/vpConfig.h>

class vpColVector;
/*!
  \class vpAdaptativeGain
  \ingroup VsTask
  \brief Adaptative gain computation.
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

    /** \brief Construction et initialisation aux valeurs par default. */
    vpAdaptativeGain (void);

    /* --- INIT --------------------------------------------------------------- */
    /** \brief Initialisation pour un gain constant (l0 = linf).
     */
    void                        initFromConstant (double lambda);

    /** \brief  Initialisation par default (faite apres construction).
     */
    void                        initFromVoid (void);

    /** \brief Initialisation a partir des trois valeurs, en zero, derivee en
     * zero et limite a l'infini.
     *
     * Calcule les trois parametres a, b et c a partir de la valeur lambda
     * a l'infini (c'est a dire la valeur theorique dans les equations), de
     * la valeur en 0, et de la pente en 0.
     */
    void                        initStandard (double en_zero,
					      double en_infini,
					      double pente_en_zero);


    /* --- MODIFIORS ---------------------------------------------------------- */
    /** \brief Passe le gain adaptatif en gain constant, egal a la valeur du gain
     * adaptatif en 0.
     * \return renvoie la valeur du gain constant.
     */
    double                      setConstant (void);


    /* --- COMPUTE ------------------------------------------------------------ */
    /** \brief Calcule la valeur de lambda au point courrant.
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

    /** \brief Calcule la valeur de lambda au point courrant et stockage du
     * resultat.
     *
     * La fonction calcule la valeur de lambda d'apres la valeur de la norme
     * de l'erreur, comme le fait la fonction valeur_const.
     * La fonction non constante stocke de plus le resultat dans this ->lambda.
     * \param val_e: valeur de la norme de l'erreur.
     * \return: valeur de gain au point courrant.
     */
    double                      value (double val_e) const;

    /** \brief Renvoye la valeur en +inf.
     */
    double                      limitValue_const (void) const;

    /** \brief Renvoye la valeur en +inf.
     * La fonction non constante stocke de plus le resultat dans this ->lambda.
     */
    double                      limitValue (void) const;

    /* --- ACCESSORS ---------------------------------------------------------- */

    /** \brief Renvoie la derniere valeur stockee dans this ->lambda. */
    double                      getLastValue (void) const;

    /** \brief Idem fonction valeur. */
    double                      operator() (double val_e) const;

    /** \brief Lance la fonction valeur avec la norme INFINIE du vecteur. */
    double                      operator()  (const vpColVector & e) const;

    /** \brief Idem function limitValue. */
    double                      operator() (void) const;


    /* --- IOSTREAM ----------------------------------------------------------- */

    friend std::ostream&        operator<< (std::ostream &os,
					    const vpAdaptativeGain& lambda);
};

#endif /*  __VP_ADAPTATIVE_GAIN_H	*/




/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
