/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Levenberg Marquartd.
 */

#ifndef VP_LEVENBERG_MARQUARTD_H
#define VP_LEVENBERG_MARQUARTD_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMath.h>

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

BEGIN_VISP_NAMESPACE

/*!
 * ENTREE  :
 * n     Ordre de la matrice "r".
 * r    Matrice de taille "n" x "n". En entree, la partie triangulaire
 *    complete de "r" doit contenir la partie triangulaire
 * superieure complete de "r".
 * ldr    Taille maximale de la matrice "r". "ldr" >= n.
 * ipvt    Vecteur de taille "n" definissant la matrice de permutation
 * "p" La jeme colonne de de "p" est la colonne ipvt[j] de la matrice identite.
 * diag    Vecteur de taille "n" contenant les elements diagonaux de la
 *    matrice "d".
 * qtb    Vecteur de taille "n" contenant les "n" premiers elements du
 *    vecteur (q transpose) * b.
 * wa    Vecteur de travail de taille "n".
 *
 * DESCRIPTION  :
 * La procedure complete la solution du probleme, si on fournit les
 * information necessaires de  la factorisation qr, avec pivotage des colonnes.
 * Soit une matrice "a" de taille "m" x "n" donnee, une matrice diagonale "d"
 * de taille "n" x "n" et un vecteur "b" de taille "m", le probleme est la
 * determination un vecteur "x" qui est solution du systeme
 *
 *               a*x = b ,     d*x = 0 ,
 *
 * Au sens des moindres carres.
 *
 * Soit a * p = q * r, ou p est une matrice de permutation, les colonnes de
 * "q" sont orthogonales et "r" est une matrice traingulaire superieure dont
 * les elements diagonaux sont classes de l'ordre decroissant de leur valeur.
 * Cette procedure attend donc la matrice triangulaire superieure remplie "r",
 * la matrice de permutaion "p" et les "n" premiers elements de (q transpose)
 * * b. Le systeme
 *
 *         a * x = b, d * x = 0, est alors equivalent a
 *
 *                     t         t
 *            r * z = q * b  ,  p * d * p * z = 0 ,
 *
 * Ou x = p * z. Si ce systeme ne possede pas de rangee pleine, alors une
 * solution au moindre carre est obtenue. En sortie, la procedure fournit
 * aussi une matrice triangulaire superieure "s" tel que
 *
 *            t    t                    t
 *           p * (a * a + d * d) * p = s * s .
 *
 * "s" est calculee a l'interieure de qrsolv et peut etre hors interet.
 *
 * SORTIE  :
 * r    En sortie, le triangle superieur n'est pas altere, et la
 * partie triangulaire inferieure contient la partie triangulaire superieure
 *    (transpose) de la matrice triangulaire "s".
 * x    Vecteur de taille "n" contenant les solutions au moindres
 * carres du systeme a * x = b, d * x = 0. sdiag  Vecteur de taille "n"
 * contenant les elements diagonaux de la matrice triangulaire superieure "s".
 */
  int VISP_EXPORT qrsolv(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *x, double *sdiag,
                         double *wa);

  /*
   * PROCEDURE  : enorm
   *
   * ENTREE  :
   *
   * x    Vecteur de taille "n"
   * n    Taille du vecteur "x"
   *
   * DESCRIPTION  :
   * La procedure calcule la norme euclidienne d'un vecteur "x" de taille "n"
   * La norme euclidienne est calculee par accumulation de la somme  des carres
   * dans les trois directions. Les sommes des carres pour les petits et grands
   * elements sont mis a echelle afin d'eviter les overflows. Des underflows non
   * destructifs sont autorisee. Les underflows et overflows sont evites dans le
   * calcul des sommes des carres non encore mis a echelle par les elements
   * intermediaires. La definition des elements petit, intermediaire et grand
   * depend de deux constantes : rdwarf et rdiant. Les restrictions principales
   * sur ces constantes sont rdwarf^2 n'est pas en underflow et rdgiant^2 n'est
   * pas en overflow. Les constantes donnees ici conviennent pour la plupart des
   * pc connus.
   *
   * RETOUR  :
   * En cas de succes,  la valeur retournee est la norme euclidienne du vecteur
   * Sinon, la valeur -1 est retournee et la variable globale "errno" est
   * initialisee pour indiquee le type de l'erreur.
   */
double VISP_EXPORT enorm(const double *x, int n);

/* PROCEDURE  : lmpar
 *
 * ENTREE  :
 * n    Ordre de la matrice "r".
 * r    Matrice de taille "n" x "n". En entree, la toute la partie
 *    triangulaire superieure doit contenir toute la partie
 * triangulaire superieure de "r".
 *
 * ldr    Taille maximum de la matrice "r". "ldr" >= "n".
 *
 * ipvt    Vecteur de taille "n" qui definit la matrice de permutation
 * "p" tel que : a * p = q * r. La jeme colonne de p la colonne ipvt[j] de la
 * matrice d'identite.
 *
 * diag    Vecteur de taille "n" contenant les elements diagonaux de la
 *    matrice "d".
 *
 * qtb    Vecteur de taille "n" contenant les "n" premiers elements du
 *    vecteur (q transpose)*b.
 *
 * delta  Limite superieure de la norme euclidienne de d * x.
 *
 * par    Estimee initiale du parametre de Levenberg-Marquardt.
 * wa1, wa2  Vecteurs de taille "n" de travail.
 *
 * DESCRIPTION  :
 * La procedure determine le parametre de Levenberg-Marquardt. Soit une
 * matrice "a" de taille "m" x "n", une matrice diagonale "d" non singuliere de
 * taille "n" x "n", un vecteur "b" de taille "m" et un nombre positf delta,
 * le probleme est le calcul du parametre "par" de telle facon que si "x"
 * resoud le systeme
 *
 *             a * x = b ,     sqrt(par) * d * x = 0 ,
 *
 * au sens des moindre carre, et dxnorm est la norme euclidienne de d * x
 * alors "par" vaut 0.0 et (dxnorm - delta) <= 0.1 * delta ,
 * ou "par" est positif et abs(dxnorm-delta) <= 0.1 * delta.
 * Cette procedure complete la solution du probleme si on lui fourni les infos
 * nessaires de la factorisation qr, avec pivotage de colonnes de a.
 * Donc, si a * p = q * r, ou "p" est une matrice de permutation, les colonnes
 * de "q" sont orthogonales, et "r" est une matrice triangulaire superieure
 * avec les elements diagonaux classes par ordre decroissant de leur valeur,
 * lmpar attend une matrice triangulaire superieure complete, la matrice de
 * permutation "p" et les "n" premiers elements de  (q transpose) * b. En
 * sortie, la procedure lmpar fournit aussi une matrice triangulaire
 * superieure "s" telle que
 *
 *            t     t                          t
 *           p  * (a * a + par * d * d )* p = s * s .
 *
 * "s" est utilise a l'interieure de lmpar et peut etre d'un interet separe.
 *
 * Seulement quelques iterations sont necessaire pour la convergence de
 * l'algorithme. Si neanmoins la limite de 10 iterations est atteinte, la
 * valeur de sortie "par" aura la derniere meilleure valeur.
 *
 * SORTIE  :
 * r    En sortie, tout le triangle superieur est inchange, et le
 *    le triangle inferieur contient les elements de la partie
 *    triangulaire superieure (transpose) de la matrice triangulaire
 *    superieure de "s".
 * par    Estimee finale du parametre de Levenberg-Marquardt.
 * x    Vecteur de taille "n" contenant la solution au sens des
 *    moindres carres du systeme a * x = b, sqrt(par) * d * x = 0, pour le
 *    parametre en sortie "par"
 * sdiag  Vecteur de taille "n" contenant les elements diagonaux de la
 *    matrice triangulaire "s".
 *
 * RETOUR  :
 * En cas de succes, la valeur 0.0 est retournee.
 */
int VISP_EXPORT lmpar(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *delta, double *par,
                      double *x, double *sdiag, double *wa1, double *wa2);

/*
 * PROCEDURE  : pythag
 *
 * ENTREES  :
 * a, b    Variables dont on veut la racine carre de leur somme de carre
 *
 * DESCRIPTION  :
 * La procedure calcule la racine carre de la somme des carres de deux nombres
 * en evitant l'overflow ou l'underflow destructif.
 *
 * RETOUR  :
 * La procedure retourne la racine carre de a^2 + b^2.
 */
double VISP_EXPORT pythag(double a, double b);

/*
 * PROCEDURE  : qrfac
 *
 * ENTREE  :
 * m    Nombre de lignes de la matrice "a".
 * n    Nombre de colonne de la matrice "a".
 * a    Matrice de taille "m" x "n". elle contient, en entree la
 *matrice dont on veut sa factorisation qr.
 * lda    Taille maximale de "a". lda >= m.
 * pivot  Booleen. Si pivot est TRUE, le pivotage de colonnes est
 *realise Si pivot = FALSE, pas de pivotage.
 * lipvt  Taille du vecteur "ipvt". Si pivot est FALSE, lipvt est de
 *    l'ordre de 1. Sinon lipvt est de l'ordre de "n".
 * wa    Vecteur de travail de taille "n". Si pivot = FALSE "wa"
 *    coincide avec rdiag.
 *
 * DESCRIPTION  :
 * La procedure effectue une decomposition de la matrice "a"par la methode qr.
 * Elle utilise les transformations de householders avec pivotage sur les
 * colonnes (option) pour calculer la factorisation qr de la matrice "a" de
 * taille "m" x "n". La procedure determine une matrice orthogonale "q", une
 * matrice de permutation "p" et une matrice trapesoidale superieure "r" dont
 * les elements diagonaux sont ordonnes dans l'ordre decroissant de leurs
 * valeurs,tel que a * p = q * r. La transformation de householder pour la
 * colonne k, k = 1,2,...,min(m,n), est de la forme
 *                             t
 *               i - (1 / u(k)) * u * u
 *
 * Ou u a des zeros dans les k-1 premieres positions.
 *
 * SORTIE  :
 * a    Matrice de taille "m" x "n" dont le trapeze superieur de "a"
 *    contient la partie trapezoidale superieure de "r" et la partie
 *    trapezoidale inferieure de "a" contient une forme factorisee
 *    de "q" (les elements non triviaux du vecteurs "u" sont decrits
 *    ci-dessus).
 * ipvt    Vecteur de taille "n". Il definit la matrice de permutation
 * "p" tel que a * p = q * r. La jeme colonne de p est la colonne ipvt[j] de la
 * matrice d'identite. Si pivot = FALSE, ipvt n'est pas referencee. rdiag
 * Vecteur de taille "n" contenant les elements diagonaux de la matrice
 * "r". acnorm  Vecteur de taille "n" contenant les normes des lignes
 *    correspondantes de la matrice "a". Si cette information n'est
 *    pas requise, acnorm coincide avec rdiag.
 */
int VISP_EXPORT qrfac(int m, int n, double *a, int lda, int *pivot, int *ipvt, int lipvt, double *rdiag, double *acnorm,
                      double *wa);

/*
 * PROCEDURE    : lmder
 *
 *
 * ENTREE  :
 * fcn    Fonction qui calcule la fonction et le jacobien de la
 * fonction. m    Nombre de fonctions.
 * n    Nombre de variables. n <= m
 * x    Vecteur de taille "n" contenant en entree une estimation
 *    initiale de la solution.
 * ldfjac  Taille dominante de la matrice "fjac". ldfjac >= "m".
 * ftol    Erreur relative desiree dans la somme des carre. La
 * terminaison survient quand les preductions estimee et vraie de la somme des
 *    carres sont toutes deux au moins egal a ftol.
 * xtol    Erreur relative desiree dans la solution approximee. La
 *    terminaison survient quand l'erreur relative entre deux
 *    iterations consecutives est au moins egal a xtol.
 * gtol    Mesure de l'orthogonalite entre le vecteur des fonctions et
 * les colonnes du jacobien. La terminaison survient quand le cosinus de
 * l'angle entre fvec et n'importe quelle colonne du jacobien est au moins
 * egal a gtol, en valeur absolue. maxfev  Nombre d'appel maximum. La
 * terminaison se produit lorsque le nombre d'appel a fcn avec iflag = 1 a
 * atteint "maxfev".
 * diag    Vecteur de taille "n". Si mode = 1 (voir ci-apres), diag est
 *    initialisee en interne. Si mode = 2, diag doit contenir les
 *    entree positives qui servent de facteurs d'echelle aux
 * variables.
 * mode    Si mode = 1, les variables seront mis a l'echelle en interne.
 *    Si mode = 2, la mise a l'echelle est specifie par l'entree
 * diag. Les autres valeurs de mode sont equivalents a mode = 1. factor
 * Definit la limite de l'etape initial. Cette limite est initialise au
 * produit de "factor" et de la norme euclidienne de "diag" * "x" sinon nul.
 * ou a "factor" lui meme. Dans la plupart des cas, "factor" doit se trouve
 * dans l'intervalle (1, 100); ou 100 est la valeur recommandee. nprint
 * Controle de l'impression des iterees (si valeur positive). Dans ce
 * cas, fcn est appelle avec iflag = 0 au debut de la premiere iteration et
 * apres chaque nprint iteration, x, fvec, et fjac sont disponible pour
 * impression, cela avant de quitter la procedure. Si "nprint" est negatif,
 * aucun appel special de fcn est faite. wa1, wa2, wa3 Vecteur de travail de
 * taille "n". wa4    Vecteur de travail de taille "m".
 *
 * SORTIE  :
 * x    Vecteur de taille "n" contenant en sortie l'estimee finale
 *    de la solution.
 * fvec    Vecteur de taille "m" contenant les fonctions evaluee en "x".
 * fjac    Matrice de taille "m" x "n". La sous matrice superieure de
 *    taille "n" x "n" de fjac contient une matrice triangulaire
 *    superieure r dont les elements diagonaux, classe dans le sens
 *    decroissant de leur valeur, sont de la forme :
 *
 *                         T      T              T
 *        p * (jac * jac) * p = r * r
 *
 *    Ou p est une matrice de permutation et jac est le jacobien
 *    final calcule.
 *    La colonne j de p est la colonne ipvt (j) (voir ci apres) de
 *    la matrice identite. La partie trapesoidale inferieure de fjac
 *    contient les information genere durant le calcul de r.
 * info    Information de l'execution de la procedure. Lorsque la
 * procedure a termine son execution, "info" est inialisee a la valeur
 *    (negative) de iflag. sinon elle prend les valeurs suivantes :
 *    info = 0 : parametres en entree non valides.
 *    info = 1 : les reductions relatives reelle et estimee de la
 *         somme des carres sont au moins egales a ftol.
 *    info = 2 : erreur relative entre deux iteres consecutives sont
 *         egaux a xtol.
 *    info = 3 : conditions info = 1 et info = 2 tous deux requis.
 *    info = 4 : le cosinus de l'angle entre fvec et n'importe
 * quelle colonne du jacobien est au moins egal a gtol, en valeur absolue. info
 * = 5 : nombre d'appels a fcn avec iflag = 1 a atteint maxfev. info = 6 :
 * ftol est trop petit. Plus moyen de reduire de la somme des carres. info =
 * 7 : xtol est trop petit. Plus d'amelioration possible pour approximer la
 * solution x. info = 8 : gtol est trop petit. "fvec" est orthogonal aux
 *         colonnes du jacobien a la precision machine pres.
 * nfev    Nombre d'appel a "fcn" avec iflag = 1.
 * njev    Nombre d'appel a "fcn" avec iflag = 2.
 * ipvt    Vecteur de taille "n". Il definit une matrice de permutation p
 *    tel que jac * p = q * p, ou jac est le jacbien final calcule,
 *    q est orthogonal (non socke) et r est triangulaire superieur,
 *    avec les elements diagonaux classes en ordre decroissant de
 *    leur valeur. La colonne j de p est ipvt[j] de la matrice
 * identite. qtf    Vecteur de taille n contenant les n premiers elements
 * du vecteur qT * fvec.
 *
 * DESCRIPTION  :
 * La procedure minimize la somme de carre de m equation non lineaire a n
 * variables par une modification de l'algorithme de Levenberg - Marquardt.
 *
 * REMARQUE  :
 * L'utilisateur doit fournir une procedure "fcn" qui calcule la fonction et
 * le jacobien.
 * "fcn" doit etre declare dans une instruction externe a la procedure et doit
 * etre appele comme suit :
 * fcn (int m, int n, int ldfjac, double *x, double *fvec, double *fjac, int
 * *iflag)
 *
 * si iflag = 1 calcul de la fonction en x et retour de ce vecteur dans fvec.
 *    fjac n'est pas modifie.
 * si iflag = 2 calcul du jacobien en x et retour de cette matrice dans fjac.
 *    fvec n'est pas modifie.
 *
 * RETOUR  :
 * En cas de succes, la valeur zero est retournee.
 * Sinon la valeur -1 est retournee.
 */
int VISP_EXPORT lmder(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag),
                      int m, int n, double *x, double *fvec, double *fjac, int ldfjac, double ftol, double xtol,
                      double gtol, unsigned int maxfev, double *diag, int mode, const double factor, int nprint,
                      int *info, unsigned int *nfev, int *njev, int *ipvt, double *qtf, double *wa1, double *wa2,
                      double *wa3, double *wa4);

/*
 * PROCEDURE    : lmder1
 *
 * ENTREE  :
 * fcn    Fonction qui calcule la fonction et le jacobien de la
 * fonction. m    Nombre de fonctions. n    Nombre de variables
 * (parametres). n <= m x    Vecteur de taille "n" contenant en
 * entree une estimation initiale de la solution.
 * ldfjac  Taille maximale de la matrice "fjac". ldfjac >= "m".
 * tol    Tolerance. La terminaison de la procedure survient quand
 *    l'algorithme estime que l'erreur relative dans la somme des
 *    carres est au moins egal a tol ou bien que l'erreur relative
 *    entre x et la solution est au moins egal atol.
 * wa    Vecteur de travail de taille "lwa".
 * lwa    Taille du vecteur "wa". wa >= 5 * n + m.
 *
 *
 * SORTIE  :
 * x    Vecteur de taille "n" contenant en sortie l'estimee finale
 *    de la solution.
 * fvec    Vecteur de taille "m" contenant les fonctions evaluee en "x".
 * fjac    Matrice de taille "m" x "n". La sous matrice superieure de
 *    taille "n" x "n" de fjac contient une matrice triangulaire
 *    superieure r dont les elements diagonaux, classe dans le sens
 *    decroissant de leur valeur, sont de la forme :
 *
 *                         T      T              T
 *        p * (jac * jac) * p = r * r
 *
 *    Ou p est une matrice de permutation et jac est le jacobien
 *         final calcule.
 *    La colonne j de p est la colonne ipvt (j) (voir ci apres) de
 *    la matrice identite. La partie trapesoidale inferieure de fjac
 *    contient les information genere durant le calcul de r.
 * info    Information de l'executionde la procedure. Lorsque la
 * procedure a termine son execution, "info" est inialisee a la valeur
 *    (negative) de iflag. sinon elle prend les valeurs suivantes :
 *    info = 0 : parametres en entre non valides.
 *    info = 1 : estimation par l'algorithme que l'erreur relative
 *         de la somme des carre est egal a tol.
 *    info = 2 : estimation par l'algorithme que l'erreur relative
 *         entre x et la solution est egal a tol.
 *    info = 3 : conditions info = 1 et info = 2 tous deux requis.
 *    info = 4 : fvec est orthogonal aux colonnes du jacobien.
 *    info = 5 : nombre d'appels a fcn avec iflag = 1 a atteint
 *         100 * (n + 1).
 *    info = 6 : tol est trop petit. Plus moyen de reduire de la
 *         somme des carres.
 *    info = 7 : tol est trop petit. Plus d'amelioration possible
 *         d' approximer la solution x.
 * ipvt    Vecteur de taille "n". Il definit une matrice de permutation p
 *    tel que jac * p = q * p, ou jac est le jacbien final calcule,
 *    q est orthogonal (non socke) et r est triangulaire superieur,
 *    avec les elements diagonaux classes en ordre decroissant de
 *    leur valeur. La colonne j de p est ipvt[j] de la matrice
 *    identite.
 *
 * DESCRIPTION  :
 * La procedure minimize la somme de carre de m equation non lineaire a n
 * variables par une modification de l'algorithme de Levenberg - Marquardt.
 * Cette procedure appele la procedure generale au moindre carre lmder.
 *
 * REMARQUE  :
 * L'utilisateur doit fournir une procedure "fcn" qui calcule la fonction et
 * le jacobien.
 * "fcn" doit etre declare dans une instruction externe a la procedure et doit
 * etre appele comme suit :
 * fcn (int m, int n, int ldfjac, double *x, double *fvec, double *fjac, int
 * *iflag)
 *
 * si iflag = 1 calcul de la fonction en x et retour de ce vecteur dans fvec.
 *    fjac n'est pas modifie.
 * si iflag = 2 calcul du jacobien en x et retour de cette matrice dans fjac.
 *    fvec n'est pas modifie.
 *
 * RETOUR  :
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1.
 */
int VISP_EXPORT lmder1(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag),
                       int m, int n, double *x, double *fvec, double *fjac, int ldfjac, double tol, int *info,
                       int *ipvt, int lwa, double *wa);

END_VISP_NAMESPACE

#endif
