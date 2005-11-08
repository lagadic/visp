/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet LAGADIC / IRISA, 2005
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      $Source: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/servo/vpAdaptativeGain.h,v $
 * Author:    Anne-Sophie Tranchant
 * Corrections: Nicolas Mansard 30-10-5
 *
 * Version control
 * ===============
 *
 * 30-1-4 : initialisation of coefficients a, b et c from f(inf),
 *          f(0) and f'(0). (N.Mansard)
 * 3-3-4: Conversion into c++. (N.Mansard)
 * 18-10-4: Review toward inclusion in VisP (dir servo). In particular 
 *        conversion to double (N.Mansard)
 *
 *  $Id: vpAdaptativeGain.h,v 1.1 2005-11-08 10:39:29 nmansard Exp $
 *
 * Description
 * ============
 *
 * Implementation of class vpAdaptativeGain, defined in vpAdaptativeGain.h
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef __VP_ADAPTATIVE_GAIN_H
#define __VP_ADAPTATIVE_GAIN_H


#include <iostream>                /* Class ostream.                          */

class vpColVector;

class vpAdaptativeGain
{

public: /* constantes */

    static const double DEFAULT_LAMBDA_ZERO = 1.666;
    static const double DEFAULT_LAMBDA_INFINI = 0.1666;
    static const double DEFAULT_LAMBDA_PENTE  = 1.666;


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
     * de l'erreur, comme le fait la fonction #valeur_const.
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




/** \file $Source: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/servo/vpAdaptativeGain.h,v $ */
/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
