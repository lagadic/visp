/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Levenberg Marquartd.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/

#include <algorithm> // (std::min)
#include <cmath>     // std::fabs
#include <iostream>
#include <limits> // numeric_limits

#include <visp3/core/vpMath.h>
#include <visp3/vision/vpLevenbergMarquartd.h>

#define SIGN(x) ((x) < 0 ? -1 : 1)
#define SWAP(a, b, c)                                                                                                  \
  {                                                                                                                    \
    (c) = (a);                                                                                                         \
    (a) = (b);                                                                                                         \
    (b) = (c);                                                                                                         \
  }
#define MIJ(m, i, j, s) ((m) + ((long)(i) * (long)(s)) + (long)(j))
#define TRUE 1
#define FALSE 0

/*
 * PROCEDURE	: enorm
 *
 * ENTREE	:
 *
 * x		Vecteur de taille "n"
 * n		Taille du vecteur "x"
 *
 * DESCRIPTION	:
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
 * RETOUR	:
 * En cas de succes,  la valeur retournee est la norme euclidienne du vecteur
 * Sinon, la valeur -1 est retournee et la variable globale "errno" est
 * initialisee pour indiquee le type de l'erreur.
 *
 */
double enorm(const double *x, int n)
{
  const double rdwarf = 3.834e-20;
  const double rgiant = 1.304e19;

  int i;
  double agiant, floatn;
  double norm_eucl = 0.0;
  double s1 = 0.0, s2 = 0.0, s3 = 0.0;
  double x1max = 0.0, x3max = 0.0;

  floatn = (double)n;
  agiant = rgiant / floatn;

  for (i = 0; i < n; i++) {
    double xabs = std::fabs(x[i]);
    if ((xabs > rdwarf) && (xabs < agiant)) {
      /*
       *	somme pour elements intemediaires.
       */
      s2 += xabs * xabs;
    }

    else if (xabs <= rdwarf) {
      /*
       *	somme pour elements petits.
       */
      if (xabs <= x3max) {
        // if (xabs != 0.0)
        if (xabs > std::numeric_limits<double>::epsilon())
          s3 += (xabs / x3max) * (xabs / x3max);
      } else {
        s3 = 1.0 + s3 * (x3max / xabs) * (x3max / xabs);
        x3max = xabs;
      }
    }

    else {
      /*
       *	somme pour elements grand.
       */
      if (xabs <= x1max) {
        s1 += (xabs / x1max) * (xabs / x1max);
      } else {
        s1 = 1.0 + s1 * (x1max / xabs) * (x1max / xabs);
        x1max = xabs;
      }
    }
  }

  /*
   *	calcul de la norme.
   */
  // if (s1 == 0.0)
  if (std::fabs(s1) <= std::numeric_limits<double>::epsilon()) {
    // if (s2 == 0.0)
    if (std::fabs(s2) <= std::numeric_limits<double>::epsilon())
      norm_eucl = x3max * sqrt(s3);
    else if (s2 >= x3max)
      norm_eucl = sqrt(s2 * (1.0 + (x3max / s2) * (x3max * s3)));
    else /*if (s2 < x3max)*/
      norm_eucl = sqrt(x3max * ((s2 / x3max) + (x3max * s3)));
  } else
    norm_eucl = x1max * sqrt(s1 + (s2 / x1max) / x1max);

  return (norm_eucl);
}

/* PROCEDURE	: lmpar
 *
 * ENTREE	:
 * n		Ordre de la matrice "r".
 * r		Matrice de taille "n" x "n". En entree, la toute la partie
 *		triangulaire superieure doit contenir toute la partie
 *triangulaire superieure de "r".
 *
 * ldr		Taille maximum de la matrice "r". "ldr" >= "n".
 *
 * ipvt		Vecteur de taille "n" qui definit la matrice de permutation
 *"p" tel que : a * p = q * r. La jeme colonne de p la colonne ipvt[j] de la
 *matrice d'identite.
 *
 * diag		Vecteur de taille "n" contenant les elements diagonaux de la
 *		matrice "d".
 *
 * qtb		Vecteur de taille "n" contenant les "n" premiers elements du
 *		vecteur (q transpose)*b.
 *
 * delta	Limite superieure de la norme euclidienne de d * x.
 *
 * par		Estimee initiale du parametre de Levenberg-Marquardt.
 * wa1, wa2	Vecteurs de taille "n" de travail.
 *
 * DESCRIPTION	:
 * La procedure determine le parametre de Levenberg-Marquardt. Soit une
 *matrice "a" de taille "m" x "n", une matrice diagonale "d" non singuliere de
 *taille "n" x "n", un vecteur "b" de taille "m" et un nombre positf delta,
 *le probleme est le calcul du parametre "par" de telle facon que si "x"
 *resoud le systeme
 *
 *	           a * x = b ,     sqrt(par) * d * x = 0 ,
 *
 * au sens des moindre carre, et dxnorm est la norme euclidienne de d * x
 * alors "par" vaut 0.0 et (dxnorm - delta) <= 0.1 * delta ,
 * ou "par" est positif et abs(dxnorm-delta) <= 0.1 * delta.
 * Cette procedure complete la solution du probleme si on lui fourni les infos
 * nessaires de la factorisation qr, avec pivotage de colonnes de a.
 * Donc, si a * p = q * r, ou "p" est une matrice de permutation, les colonnes
 * de "q" sont orthogonales, et "r" est une matrice triangulaire superieure
 * avec les elements diagonaux classes par ordre decroissant de leur valeur,
 *lmpar attend une matrice triangulaire superieure complete, la matrice de
 *permutation "p" et les "n" premiers elements de  (q transpose) * b. En
 *sortie, la procedure lmpar fournit aussi une matrice triangulaire
 *superieure "s" telle que
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
 * SORTIE	:
 * r		En sortie, tout le triangle superieur est inchange, et le
 *		le triangle inferieur contient les elements de la partie
 *		triangulaire superieure (transpose) de la matrice triangulaire
 *		superieure de "s".
 * par		Estimee finale du parametre de Levenberg-Marquardt.
 * x		Vecteur de taille "n" contenant la solution au sens des
 *moindres carres du systeme a * x = b, sqrt(par) * d * x = 0, pour le
 *		parametre en sortie "par"
 * sdiag	Vecteur de taille "n" contenant les elements diagonaux de la
 *		matrice triangulaire "s".
 *
 * RETOUR	:
 * En cas de succes, la valeur 0.0 est retournee.
 *
 */
int lmpar(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *delta, double *par, double *x,
          double *sdiag, double *wa1, double *wa2)
{
  const double tol1 = 0.1; /* tolerance a 0.1	*/

  int l;
  unsigned int iter; /* compteur d'iteration */
  int nsing;         /* nombre de singularite de la matrice */
  double dxnorm, fp;
  double temp;
  double dwarf = DBL_MIN; /* plus petite amplitude positive	*/

  /*
   *	calcul et stockage dans "x" de la direction de Gauss-Newton. Si
   *	le jacobien a une rangee de moins, on a une solution au moindre
   *	carres.
   */
  nsing = n;

  for (int i = 0; i < n; i++) {
    wa1[i] = qtb[i];
    double *pt = MIJ(r, i, i, ldr);
    // if (*MIJ(r, i, i, ldr) == 0.0 && nsing == n)
    if (std::fabs(*pt) <= std::numeric_limits<double>::epsilon() && nsing == n)
      nsing = i - 1;

    if (nsing < n)
      wa1[i] = 0.0;
  }

  if (nsing >= 0) {
    for (int k = 0; k < nsing; k++) {
      int i = nsing - 1 - k;
      wa1[i] /= *MIJ(r, i, i, ldr);
      temp = wa1[i];
      int jm1 = i - 1;

      if (jm1 >= 0) {
        for (unsigned int j = 0; j <= (unsigned int)jm1; j++)
          wa1[j] -= *MIJ(r, i, j, ldr) * temp;
      }
    }
  }

  for (int j = 0; j < n; j++) {
    l = ipvt[j];
    x[l] = wa1[j];
  }

  /*
   *	initialisation du compteur d'iteration.
   *	evaluation de la fonction a l'origine, et test
   *	d'acceptation de la direction de Gauss-Newton.
   */
  iter = 0;

  for (int i = 0; i < n; i++)
    wa2[i] = diag[i] * x[i];

  dxnorm = enorm(wa2, n);

  fp = dxnorm - *delta;

  if (fp > tol1 * (*delta)) {
    /*
     *	Si le jacobien n'a pas de rangee deficiente,l'etape de
     *	Newton fournit une limite inferieure, parl pour le
     *	zero de la fonction. Sinon cette limite vaut 0.0.
     */
    double parl = 0.0;

    if (nsing >= n) {
      for (int i = 0; i < n; i++) {
        l = ipvt[i];
        wa1[i] = diag[l] * (wa2[l] / dxnorm);
      }

      for (int i = 0; i < n; i++) {
        long im1;
        double sum = 0.0;
        im1 = (i - 1L);

        if (im1 >= 0) {
          for (unsigned int j = 0; j <= (unsigned int)im1; j++)
            sum += (*MIJ(r, i, j, ldr) * wa1[j]);
        }
        wa1[i] = (wa1[i] - sum) / *MIJ(r, i, i, ldr);
      }

      temp = enorm(wa1, n);
      parl = ((fp / *delta) / temp) / temp;
    }

    /*
     *	calcul d'une limite superieure, paru, pour le zero de la
     *	fonction.
     */
    for (int i = 0; i < n; i++) {
      double sum = 0.0;

      for (int j = 0; j <= i; j++)
        sum += *MIJ(r, i, j, ldr) * qtb[j];

      l = ipvt[i];
      wa1[i] = sum / diag[l];
    }

    double gnorm = enorm(wa1, n);
    double paru = gnorm / *delta;

    // if (paru == 0.0)
    if (std::fabs(paru) <= std::numeric_limits<double>::epsilon())
      paru = dwarf / vpMath::minimum(*delta, tol1);

    /*
     *	Si "par" en entree tombe hors de l'intervalle (parl,paru),
     *	on le prend proche du point final.
     */

    *par = vpMath::maximum(*par, parl);
    *par = vpMath::maximum(*par, paru);

    // if (*par == 0.0)
    if (std::fabs(*par) <= std::numeric_limits<double>::epsilon())
      *par = gnorm / dxnorm;

    /*
     *	debut d'une iteration.
     */
    for (;;) // iter >= 0)
    {
      iter++;

      /*
       *	evaluation de la fonction a la valeur courant
       *	de "par".
       */
      // if (*par == 0.0)
      if (std::fabs(*par) <= std::numeric_limits<double>::epsilon()) {
        const double tol001 = 0.001; /* tolerance a 0.001	*/
        *par = vpMath::maximum(dwarf, (tol001 * paru));
      }

      temp = sqrt(*par);

      for (int i = 0; i < n; i++)
        wa1[i] = temp * diag[i];

      qrsolv(n, r, ldr, ipvt, wa1, qtb, x, sdiag, wa2);

      for (int i = 0; i < n; i++)
        wa2[i] = diag[i] * x[i];

      dxnorm = enorm(wa2, n);
      temp = fp;
      fp = dxnorm - *delta;

      /*
       *	si la fonction est assez petite, acceptation de
       *	la valeur courant de "par". de plus, test des cas
       *	ou parl est nul et ou le nombre d'iteration a
       *	atteint 10.
       */
      // if ((std::fabs(fp) <= tol1 * (*delta)) || ((parl == 0.0) && (fp <=
      // temp)
      //	  && (temp < 0.0)) || (iter == 10))
      if ((std::fabs(fp) <= tol1 * (*delta)) ||
          ((std::fabs(parl) <= std::numeric_limits<double>::epsilon()) && (fp <= temp) && (temp < 0.0)) ||
          (iter == 10)) {
        // terminaison.

        // Remove the two next lines since this is a dead code
        /* if (iter == 0)
         *par = 0.0; */

        return (0);
      }

      /*
       *        calcul de la correction de Newton.
       */

      for (int i = 0; i < n; i++) {
        l = ipvt[i];
        wa1[i] = diag[l] * (wa2[l] / dxnorm);
      }

      for (unsigned int i = 0; i < (unsigned int)n; i++) {
        wa1[i] = wa1[i] / sdiag[i];
        temp = wa1[i];
        unsigned int jp1 = i + 1;
        if ((unsigned int)n >= jp1) {
          for (unsigned int j = jp1; j < (unsigned int)n; j++)
            wa1[j] -= (*MIJ(r, i, j, ldr) * temp);
        }
      }

      temp = enorm(wa1, n);
      double parc = ((fp / *delta) / temp) / temp;

      /*
       *	selon le signe de la fonction, mise a jour
       *	de parl ou paru.
       */
      if (fp > 0.0)
        parl = vpMath::maximum(parl, *par);

      if (fp < 0.0)
        paru = vpMath::minimum(paru, *par);

      /*
       *	calcul d'une estimee ameliree de "par".
       */
      *par = vpMath::maximum(parl, (*par + parc));
    } /* fin boucle sur iter	*/
  }   /* fin fp > tol1 * delta	*/

  /*
   *	terminaison.
   */
  if (iter == 0)
    *par = 0.0;

  return (0);
}

/*
 * PROCEDURE	: pythag
 *
 * ENTREES	:
 * a, b		Variables dont on veut la racine carre de leur somme de carre
 *
 * DESCRIPTION	:
 * La procedure calcule la racine carre de la somme des carres de deux nombres
 * en evitant l'overflow ou l'underflow destructif.
 *
 * RETOUR	:
 * La procedure retourne la racine carre de a^2 + b^2.
 *
 */
double pythag(double a, double b)
{
  double pyth, p, r, t;

  p = vpMath::maximum(std::fabs(a), std::fabs(b));

  // if (p == 0.0)
  if (std::fabs(p) <= std::numeric_limits<double>::epsilon()) {
    pyth = p;
    return (pyth);
  }

  r = ((std::min)(std::fabs(a), std::fabs(b)) / p) * ((std::min)(std::fabs(a), std::fabs(b)) / p);
  t = 4.0 + r;

  // while (t != 4.0)
  while (std::fabs(t - 4.0) < std::fabs(vpMath::maximum(t, 4.0)) * std::numeric_limits<double>::epsilon()) {
    double s = r / t;
    double u = 1.0 + 2.0 * s;
    p *= u;
    r *= (s / u) * (s / u);
    t = 4.0 + r;
  }

  pyth = p;
  return (pyth);
}

/*
 * PROCEDURE	: qrfac
 *
 * ENTREE	:
 * m		Nombre de lignes de la matrice "a".
 * n		Nombre de colonne de la matrice "a".
 * a		Matrice de taille "m" x "n". elle contient, en entree la
 *matrice dont on veut sa factorisation qr.
 * lda		Taille maximale de "a". lda >= m.
 * pivot	Booleen. Si pivot est TRUE, le pivotage de colonnes est
 *realise Si pivot = FALSE, pas de pivotage.
 * lipvt	Taille du vecteur "ipvt". Si pivot est FALSE, lipvt est de
 *		l'ordre de 1. Sinon lipvt est de l'ordre de "n".
 * wa		Vecteur de travail de taille "n". Si pivot = FALSE "wa"
 *		coincide avec rdiag.
 *
 * DESCRIPTION	:
 * La procedure effectue une decomposition de la matrice "a"par la methode qr.
 * Elle utilise les transformations de householders avec pivotage sur les
 *colonnes (option) pour calculer la factorisation qr de la matrice "a" de
 *taille "m" x "n". La procedure determine une matrice orthogonale "q", une
 *matrice de permutation "p" et une matrice trapesoidale superieure "r" dont
 *les elements diagonaux sont ordonnes dans l'ordre decroissant de leurs
 *valeurs,tel que a * p = q * r. La transformation de householder pour la
 *colonne k, k = 1,2,...,min(m,n), est de la forme
 *                             t
 *        		   i - (1 / u(k)) * u * u
 *
 * Ou u a des zeros dans les k-1 premieres positions.
 *
 * SORTIE	:
 * a		Matrice de taille "m" x "n" dont le trapeze superieur de "a"
 *		contient la partie trapezoidale superieure de "r" et la partie
 *		trapezoidale inferieure de "a" contient une forme factorisee
 *		de "q" (les elements non triviaux du vecteurs "u" sont decrits
 *		ci-dessus).
 * ipvt		Vecteur de taille "n". Il definit la matrice de permutation
 *"p" tel que a * p = q * r. La jeme colonne de p est la colonne ipvt[j] de la
 *matrice d'identite. Si pivot = FALSE, ipvt n'est pas referencee. rdiag
 *Vecteur de taille "n" contenant les elements diagonaux de la matrice
 *"r". acnorm	Vecteur de taille "n" contenant les normes des lignes
 *		correspondantes de la matrice "a". Si cette information n'est
 *		pas requise, acnorm coincide avec rdiag.
 *
 */
int qrfac(int m, int n, double *a, int lda, int *pivot, int *ipvt, int /* lipvt */, double *rdiag, double *acnorm,
          double *wa)
{
  const double tolerance = 0.05;

  int i, j, ip1, k, kmax, minmn;
  double epsmch;
  double sum, temp, tmp;

  /*
   *	epsmch est la precision machine.
   */
  epsmch = std::numeric_limits<double>::epsilon();

  /*
   *	calcul des normes initiales des lignes et initialisation
   *	de plusieurs tableaux.
   */
  for (i = 0; i < m; i++) {
    acnorm[i] = enorm(MIJ(a, i, 0, lda), n);
    rdiag[i] = acnorm[i];
    wa[i] = rdiag[i];

    if (pivot)
      ipvt[i] = i;
  }
  /*
   *     reduction de "a" en "r" avec les tranformations de Householder.
   */
  minmn = vpMath::minimum(m, n);
  for (i = 0; i < minmn; i++) {
    if (pivot) {
      /*
       *	met la ligne de plus grande norme en position
       *	de pivot.
       */
      kmax = i;
      for (k = i; k < m; k++) {
        if (rdiag[k] > rdiag[kmax])
          kmax = k;
      }

      if (kmax != i) {
        for (j = 0; j < n; j++)
          SWAP(*MIJ(a, i, j, lda), *MIJ(a, kmax, j, lda), tmp);

        rdiag[kmax] = rdiag[i];
        wa[kmax] = wa[i];

        SWAP(ipvt[i], ipvt[kmax], k);
      }
    }

    /*
     *	calcul de al transformationde Householder afin de reduire
     *	la jeme ligne de "a" a un multiple du jeme vecteur unite.
     */
    double ajnorm = enorm(MIJ(a, i, i, lda), n - i);

    // if (ajnorm != 0.0)
    if (std::fabs(ajnorm) > std::numeric_limits<double>::epsilon()) {
      if (*MIJ(a, i, i, lda) < 0.0)
        ajnorm = -ajnorm;

      for (j = i; j < n; j++)
        *MIJ(a, i, j, lda) /= ajnorm;
      *MIJ(a, i, i, lda) += 1.0;

      /*
       *	application de la tranformation aux lignes
       *	restantes et mise a jour des normes.
       */
      ip1 = i + 1;

      if (m >= ip1) {
        for (k = ip1; k < m; k++) {
          sum = 0.0;
          for (j = i; j < n; j++)
            sum += *MIJ(a, i, j, lda) * *MIJ(a, k, j, lda);

          temp = sum / *MIJ(a, i, i, lda);

          for (j = i; j < n; j++)
            *MIJ(a, k, j, lda) -= temp * *MIJ(a, i, j, lda);

          // if (pivot && rdiag[k] != 0.0)
          if (pivot && (std::fabs(rdiag[k]) > std::numeric_limits<double>::epsilon())) {
            temp = *MIJ(a, k, i, lda) / rdiag[k];
            rdiag[k] *= sqrt(vpMath::maximum(0.0, (1.0 - temp * temp)));

            if (tolerance * (rdiag[k] / wa[k]) * (rdiag[k] / wa[k]) <= epsmch) {
              rdiag[k] = enorm(MIJ(a, k, ip1, lda), (n - 1 - (int)i));
              wa[k] = rdiag[k];
            }
          }
        } /* fin boucle for k	*/
      }

    } /* fin if (ajnorm) */

    rdiag[i] = -ajnorm;
  } /* fin for (i = 0; i < minmn; i++) */
  return (0);
}

/* PROCEDURE	: qrsolv
 *
 * ENTREE	:
 * n 		Ordre de la matrice "r".
 * r		Matrice de taille "n" x "n". En entree, la partie triangulaire
 *		complete de "r" doit contenir la partie triangulaire
 *superieure complete de "r".
 * ldr		Taille maximale de la matrice "r". "ldr" >= n.
 * ipvt		Vecteur de taille "n" definissant la matrice de permutation
 *"p" La jeme colonne de de "p" est la colonne ipvt[j] de la matrice identite.
 * diag		Vecteur de taille "n" contenant les elements diagonaux de la
 *		matrice "d".
 * qtb		Vecteur de taille "n" contenant les "n" premiers elements du
 *		vecteur (q transpose) * b.
 * wa		Vecteur de travail de taille "n".
 *
 * DESCRIPTION	:
 * La procedure complete la solution du probleme, si on fournit les
 *information necessaires de  la factorisation qr, avec pivotage des colonnes.
 * Soit une matrice "a" de taille "m" x "n" donnee, une matrice diagonale "d"
 *de taille "n" x "n" et un vecteur "b" de taille "m", le probleme est la
 *determination un vecteur "x" qui est solution du systeme
 *
 *		           a*x = b ,     d*x = 0 ,
 *
 * Au sens des moindres carres.
 *
 * Soit a * p = q * r, ou p est une matrice de permutation, les colonnes de
 *"q" sont orthogonales et "r" est une matrice traingulaire superieure dont
 *les elements diagonaux sont classes de l'ordre decroissant de leur valeur.
 *Cette procedure attend donc la matrice triangulaire superieure remplie "r",
 *la matrice de permutaion "p" et les "n" premiers elements de (q transpose)
 ** b. Le systeme
 *
 *		     a * x = b, d * x = 0, est alors equivalent a
 *
 *                     t         t
 *            r * z = q * b  ,  p * d * p * z = 0 ,
 *
 * Ou x = p * z. Si ce systeme ne possede pas de rangee pleine, alors une
 * solution au moindre carre est obtenue. En sortie, la procedure fournit
 *aussi une matrice triangulaire superieure "s" tel que
 *
 *            t    t                    t
 *           p * (a * a + d * d) * p = s * s .
 *
 * "s" est calculee a l'interieure de qrsolv et peut etre hors interet.
 *
 * SORTIE	:
 * r		En sortie, le triangle superieur n'est pas altere, et la
 *partie triangulaire inferieure contient la partie triangulaire superieure
 *		(transpose) de la matrice triangulaire "s".
 * x		Vecteur de taille "n" contenant les solutions au moindres
 *carres du systeme a * x = b, d * x = 0. sdiag	Vecteur de taille "n"
 *contenant les elements diagonaux de la matrice triangulaire superieure "s".
 *
 */
int qrsolv(int n, double *r, int ldr, int *ipvt, double *diag, double *qtb, double *x, double *sdiag, double *wa)
{
  int i, j, k, kp1, l; /* compteur de boucle	*/
  int nsing;
  double cosi, cotg, qtbpj, sinu, tg, temp;

  /*
   *	copie de r et (q transpose) * b afin de preserver l'entree
   *	et initialisation de "s". En particulier, sauvegarde des elements
   *	diagonaux de "r" dans "x".
   */
  for (i = 0; i < n; i++) {
    for (j = i; j < n; j++)
      *MIJ(r, i, j, ldr) = *MIJ(r, j, i, ldr);

    x[i] = *MIJ(r, i, i, ldr);
    wa[i] = qtb[i];
  }

  /*
   *	Elimination de la matrice diagonale "d" en utlisant une rotation
   *	connue.
   */

  for (i = 0; i < n; i++) {
    /*
     *	preparation de la colonne de d a eliminer, reperage de
     *	l'element diagonal par utilisation de p de la
     *	factorisation qr.
     */
    l = ipvt[i];

    // if (diag[l] != 0.0)
    if (std::fabs(diag[l]) > std::numeric_limits<double>::epsilon()) {
      for (k = i; k < n; k++)
        sdiag[k] = 0.0;

      sdiag[i] = diag[l];

      /*
       *	Les transformations qui eliminent la colonne de d
       *	modifient seulement qu'un seul element de
       *	(q transpose)*b avant les n premiers elements
       *	lesquels sont inialement nuls.
       */

      qtbpj = 0.0;

      for (k = i; k < n; k++) {
        /*
         *	determination d'une rotation qui elimine
         *	les elements appropriees dans la colonne
         *	courante de d.
         */

        // if (sdiag[k] != 0.0)
        if (std::fabs(sdiag[k]) > std::numeric_limits<double>::epsilon()) {
          if (std::fabs(*MIJ(r, k, k, ldr)) >= std::fabs(sdiag[k])) {
            tg = sdiag[k] / *MIJ(r, k, k, ldr);
            cosi = 0.5 / sqrt(0.25 + 0.25 * (tg * tg));
            sinu = cosi * tg;
          } else {
            cotg = *MIJ(r, k, k, ldr) / sdiag[k];
            sinu = 0.5 / sqrt(0.25 + 0.25 * (cotg * cotg));
            cosi = sinu * cotg;
          }

          /*
           *	calcul des elements de la diagonale modifiee
           *	de r et des elements modifies de
           *	((q transpose)*b,0).
           */
          *MIJ(r, k, k, ldr) = cosi * *MIJ(r, k, k, ldr) + sinu * sdiag[k];
          temp = cosi * wa[k] + sinu * qtbpj;
          qtbpj = -sinu * wa[k] + cosi * qtbpj;
          wa[k] = temp;

          /*
           *	accumulation des tranformations dans
           *	les lignes de s.
           */

          kp1 = k + 1;

          if (n >= kp1) {
            for (j = kp1; j < n; j++) {
              temp = cosi * *MIJ(r, k, j, ldr) + sinu * sdiag[j];
              sdiag[j] = -sinu * *MIJ(r, k, j, ldr) + cosi * sdiag[j];
              *MIJ(r, k, j, ldr) = temp;
            }
          }
        } /* fin if diag[] !=0	*/
      }   /* fin boucle for k -> n */
    }     /* fin if diag =0	*/

    /*
     *	stokage de l'element diagonal de s et restauration de
     *	l'element diagonal correspondant de r.
     */
    sdiag[i] = *MIJ(r, i, i, ldr);
    *MIJ(r, i, i, ldr) = x[i];
  } /* fin boucle for j -> n	*/

  /*
   *	resolution du systeme triangulaire pour z. Si le systeme est
   *	singulier, on obtient une solution au moindres carres.
   */
  nsing = n;

  for (i = 0; i < n; i++) {
    // if (sdiag[i] == 0.0 && nsing == n)
    if ((std::fabs(sdiag[i]) <= std::numeric_limits<double>::epsilon()) && nsing == n)
      nsing = i - 1;

    if (nsing < n)
      wa[i] = 0.0;
  }

  if (nsing >= 0) {
    for (k = 0; k < nsing; k++) {
      i = nsing - 1 - k;
      double sum = 0.0;
      int jp1 = i + 1;

      if (nsing >= jp1) {
        for (j = jp1; j < nsing; j++)
          sum += *MIJ(r, i, j, ldr) * wa[j];
      }
      wa[i] = (wa[i] - sum) / sdiag[i];
    }
  }
  /*
   *	permutation arriere des composants de z et des componants de x.
   */

  for (j = 0; j < n; j++) {
    l = ipvt[j];
    x[l] = wa[j];
  }
  return (0);
}

/*
 * PROCEDURE    : lmder
 *
 *
 * ENTREE	:
 * fcn		Fonction qui calcule la fonction et le jacobien de la
 *fonction. m		Nombre de fonctions.
 * n		Nombre de variables. n <= m
 * x		Vecteur de taille "n" contenant en entree une estimation
 *		initiale de la solution.
 * ldfjac	Taille dominante de la matrice "fjac". ldfjac >= "m".
 * ftol		Erreur relative desiree dans la somme des carre. La
 *terminaison survient quand les preductions estimee et vraie de la somme des
 *		carres sont toutes deux au moins egal a ftol.
 * xtol		Erreur relative desiree dans la solution approximee. La
 *		terminaison survient quand l'erreur relative entre deux
 *		iterations consecutives est au moins egal a xtol.
 * gtol		Mesure de l'orthogonalite entre le vecteur des fonctions et
 *les colonnes du jacobien. La terminaison survient quand le cosinus de
 *l'angle entre fvec et n'importe quelle colonne du jacobien est au moins
 *egal a gtol, en valeur absolue. maxfev	Nombre d'appel maximum. La
 *terminaison se produit lorsque le nombre d'appel a fcn avec iflag = 1 a
 *atteint "maxfev".
 * diag		Vecteur de taille "n". Si mode = 1 (voir ci-apres), diag est
 *		initialisee en interne. Si mode = 2, diag doit contenir les
 *		entree positives qui servent de facteurs d'echelle aux
 *variables.
 * mode		Si mode = 1, les variables seront mis a l'echelle en interne.
 *		Si mode = 2, la mise a l'echelle est specifie par l'entree
 *diag. Les autres valeurs de mode sont equivalents a mode = 1. factor
 *Definit la limite de l'etape initial. Cette limite est initialise au
 *produit de "factor" et de la norme euclidienne de "diag" * "x" sinon nul.
 *ou a "factor" lui meme. Dans la plupart des cas, "factor" doit se trouve
 *dans l'intervalle (1, 100); ou 100 est la valeur recommandee. nprint
 *Controle de l'impression des iterees (si valeur positive). Dans ce
 *cas, fcn est appelle avec iflag = 0 au debut de la premiere iteration et
 *apres chaque nprint iteration, x, fvec, et fjac sont disponible pour
 *impression, cela avant de quitter la procedure. Si "nprint" est negatif,
 *aucun appel special de fcn est faite. wa1, wa2, wa3 Vecteur de travail de
 *taille "n". wa4		Vecteur de travail de taille "m".
 *
 *
 * SORTIE	:
 * x		Vecteur de taille "n" contenant en sortie l'estimee finale
 *		de la solution.
 * fvec		Vecteur de taille "m" contenant les fonctions evaluee en "x".
 * fjac		Matrice de taille "m" x "n". La sous matrice superieure de
 *		taille "n" x "n" de fjac contient une matrice triangulaire
 *		superieure r dont les elements diagonaux, classe dans le sens
 *		decroissant de leur valeur, sont de la forme :
 *
 *      	                 T      T              T
 *				p * (jac * jac) * p = r * r
 *
 *		Ou p est une matrice de permutation et jac est le jacobien
 *		final calcule.
 *		La colonne j de p est la colonne ipvt (j) (voir ci apres) de
 *		la matrice identite. La partie trapesoidale inferieure de fjac
 *		contient les information genere durant le calcul de r.
 * info		Information de l'execution de la procedure. Lorsque la
 *procedure a termine son execution, "info" est inialisee a la valeur
 *		(negative) de iflag. sinon elle prend les valeurs suivantes :
 *		info = 0 : parametres en entree non valides.
 *		info = 1 : les reductions relatives reelle et estimee de la
 *			   somme des carres sont au moins egales a ftol.
 *		info = 2 : erreur relative entre deux iteres consecutives sont
 *			   egaux a xtol.
 *		info = 3 : conditions info = 1 et info = 2 tous deux requis.
 *		info = 4 : le cosinus de l'angle entre fvec et n'importe
 *quelle colonne du jacobien est au moins egal a gtol, en valeur absolue. info
 *= 5 : nombre d'appels a fcn avec iflag = 1 a atteint maxfev. info = 6 :
 *ftol est trop petit. Plus moyen de reduire de la somme des carres. info =
 *7 : xtol est trop petit. Plus d'amelioration possible pour approximer la
 *solution x. info = 8 : gtol est trop petit. "fvec" est orthogonal aux
 *			   colonnes du jacobien a la precision machine pres.
 * nfev		Nombre d'appel a "fcn" avec iflag = 1.
 * njev		Nombre d'appel a "fcn" avec iflag = 2.
 * ipvt		Vecteur de taille "n". Il definit une matrice de permutation p
 *		tel que jac * p = q * p, ou jac est le jacbien final calcule,
 *		q est orthogonal (non socke) et r est triangulaire superieur,
 *		avec les elements diagonaux classes en ordre decroissant de
 *		leur valeur. La colonne j de p est ipvt[j] de la matrice
 *identite. qtf		Vecteur de taille n contenant les n premiers elements
 *du vecteur qT * fvec.
 *
 * DESCRIPTION  :
 * La procedure minimize la somme de carre de m equation non lineaire a n
 * variables par une modification de l'algorithme de Levenberg - Marquardt.
 *
 * REMARQUE	:
 * L'utilisateur doit fournir une procedure "fcn" qui calcule la fonction et
 * le jacobien.
 * "fcn" doit etre declare dans une instruction externe a la procedure et doit
 * etre appele comme suit :
 * fcn (int m, int n, int ldfjac, double *x, double *fvec, double *fjac, int
 **iflag)
 *
 * si iflag = 1 calcul de la fonction en x et retour de ce vecteur dans fvec.
 *		fjac n'est pas modifie.
 * si iflag = 2 calcul du jacobien en x et retour de cette matrice dans fjac.
 *		fvec n'est pas modifie.
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon la valeur -1 est retournee.
 */
int lmder(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag), int m, int n,
          double *x, double *fvec, double *fjac, int ldfjac, double ftol, double xtol, double gtol, unsigned int maxfev,
          double *diag, int mode, const double factor, int nprint, int *info, unsigned int *nfev, int *njev, int *ipvt,
          double *qtf, double *wa1, double *wa2, double *wa3, double *wa4)
{
  const double tol1 = 0.1, tol5 = 0.5, tol25 = 0.25, tol75 = 0.75, tol0001 = 0.0001;
  int oncol = TRUE;
  int iflag, iter;
  int count = 0;
  int i, j, l;
  double actred, delta, dirder, epsmch, fnorm, fnorm1;
  double ratio = std::numeric_limits<double>::epsilon();
  double par, pnorm, prered;
  double sum, temp, temp1, temp2, xnorm = 0.0;

  /* epsmch est la precision machine.	*/
  epsmch = std::numeric_limits<double>::epsilon();

  *info = 0;
  iflag = 0;
  *nfev = 0;
  *njev = 0;

  /*	verification des parametres d'entree.	*/

  /*if (n <= 0)
    return 0;*/
  if (m < n)
    return 0;
  if (ldfjac < m)
    return 0;
  if (ftol < 0.0)
    return 0;
  if (xtol < 0.0)
    return 0;
  if (gtol < 0.0)
    return 0;
  if (maxfev == 0)
    return 0;
  if (factor <= 0.0)
    return 0;
  if ((n <= 0) || (m < n) || (ldfjac < m) || (ftol < 0.0) || (xtol < 0.0) || (gtol < 0.0) || (maxfev == 0) ||
      (factor <= 0.0)) {
    /*
     * termination, normal ou imposee par l'utilisateur.
     */
    if (iflag < 0)
      *info = iflag;

    iflag = 0;

    if (nprint > 0)
      (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

    return (count);
  }

  if (mode == 2) {
    for (j = 0; j < n; j++) {
      if (diag[j] <= 0.0) {
        /*
         * termination, normal ou imposee par l'utilisateur.
         */
        if (iflag < 0)
          *info = iflag;

        iflag = 0;

        if (nprint > 0)
          (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

        return (count);
      }
    }
  }

  /*
   *	evaluation de la fonction au point de depart
   *	et calcul de sa norme.
   */
  iflag = 1;

  (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

  *nfev = 1;

  if (iflag < 0) {
    /*
     * termination, normal ou imposee par l'utilisateur.
     */
    if (iflag < 0)
      *info = iflag;

    iflag = 0;

    if (nprint > 0)
      (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

    return (count);
  }

  fnorm = enorm(fvec, m);

  /*
   *	initialisation du parametre de Levenberg-Marquardt
   *	et du conteur d'iteration.
   */

  par = 0.0;
  iter = 1;

  /*
   *	debut de la boucle la plus externe.
   */
  while (count < (int)maxfev) {
    count++;
    /*
     *	calcul de la matrice jacobienne.
     */

    iflag = 2;

    (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

    (*njev)++;

    if (iflag < 0) {
      /*
       * termination, normal ou imposee par l'utilisateur.
       */
      if (iflag < 0)
        *info = iflag;

      iflag = 0;

      if (nprint > 0)
        (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

      return (count);
    }

    /*
     *	si demandee, appel de fcn pour impression des iterees.
     */
    if (nprint > 0) {
      iflag = 0;
      if ((iter - 1) % nprint == 0)
        (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

      if (iflag < 0) {
        /*
         * termination, normal ou imposee par l'utilisateur.
         */
        if (iflag < 0)
          *info = iflag;

        iflag = 0;

        if (nprint > 0)
          (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

        return (count);
      }
    }

    /*
     * calcul de la factorisation qr du jacobien.
     */
    qrfac(n, m, fjac, ldfjac, &oncol, ipvt, n, wa1, wa2, wa3);

    /*
     *	a la premiere iteration et si mode est 1, mise a l'echelle
     *	en accord avec les normes des colonnes du jacobien initial.
     */

    if (iter == 1) {
      if (mode != 2) {
        for (j = 0; j < n; j++) {
          diag[j] = wa2[j];
          // if (wa2[j] == 0.0)
          if (std::fabs(wa2[j]) <= std::numeric_limits<double>::epsilon())
            diag[j] = 1.0;
        }
      }

      /*
       *	a la premiere iteration, calcul de la norme de x mis
       *	a l'echelle et initialisation de la limite delta de
       *	l'etape.
       */

      for (j = 0; j < n; j++)
        wa3[j] = diag[j] * x[j];

      xnorm = enorm(wa3, n);
      delta = factor * xnorm;

      // if (delta == 0.0)
      if (std::fabs(delta) <= std::numeric_limits<double>::epsilon())
        delta = factor;
    }

    /*
     *	formation de (q transpose) * fvec et  stockage des n premiers
     *	composants dans qtf.
     */
    for (i = 0; i < m; i++)
      wa4[i] = fvec[i];

    for (i = 0; i < n; i++) {
      double *pt = MIJ(fjac, i, i, ldfjac);
      // if (*MIJ(fjac, i, i, ldfjac) != 0.0)
      if (std::fabs(*pt) > std::numeric_limits<double>::epsilon()) {
        sum = 0.0;

        for (j = i; j < m; j++)
          sum += *MIJ(fjac, i, j, ldfjac) * wa4[j];

        temp = -sum / *MIJ(fjac, i, i, ldfjac);

        for (j = i; j < m; j++)
          wa4[j] += *MIJ(fjac, i, j, ldfjac) * temp;
      }

      *MIJ(fjac, i, i, ldfjac) = wa1[i];
      qtf[i] = wa4[i];
    }

    /*
     *	calcul de la norme du gradient mis a l'echelle.
     */

    double gnorm = 0.0;

    // if (fnorm != 0.0)
    if (std::fabs(fnorm) > std::numeric_limits<double>::epsilon()) {
      for (i = 0; i < n; i++) {
        l = ipvt[i];
        // if (wa2[l] != 0.0)
        if (std::fabs(wa2[l]) > std::numeric_limits<double>::epsilon()) {
          sum = 0.0;
          for (j = 0; j <= i; j++)
            sum += *MIJ(fjac, i, j, ldfjac) * (qtf[j] / fnorm);

          gnorm = vpMath::maximum(gnorm, std::fabs(sum / wa2[l]));
        }
      }
    }

    /*
     *	test pour la  convergence de la norme du gradient .
     */

    if (gnorm <= gtol)
      *info = 4;

    if (*info != 0) {
      /*
       * termination, normal ou imposee par l'utilisateur.
       */
      if (iflag < 0)
        *info = iflag;

      iflag = 0;

      if (nprint > 0)
        (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

      return (count);
    }

    /*
     * remise a l'echelle si necessaire.
     */

    if (mode != 2) {
      for (j = 0; j < n; j++)
        diag[j] = vpMath::maximum(diag[j], wa2[j]);
    }

    /*
     *	debut de la boucle la plus interne.
     */
    ratio = 0.0;
    while (ratio < tol0001) {

      /*
       *	determination du parametre de Levenberg-Marquardt.
       */
      lmpar(n, fjac, ldfjac, ipvt, diag, qtf, &delta, &par, wa1, wa2, wa3, wa4);

      /*
       *	stockage de la direction p et x + p. calcul de la norme de p.
       */

      for (j = 0; j < n; j++) {
        wa1[j] = -wa1[j];
        wa2[j] = x[j] + wa1[j];
        wa3[j] = diag[j] * wa1[j];
      }

      pnorm = enorm(wa3, n);

      /*
       *	a la premiere iteration, ajustement de la premiere limite de
       *	l'etape.
       */

      if (iter == 1)
        delta = vpMath::minimum(delta, pnorm);

      /*
       *	evaluation de la fonction en x + p et calcul de leur norme.
       */

      iflag = 1;
      (*ptr_fcn)(m, n, wa2, wa4, fjac, ldfjac, iflag);

      (*nfev)++;

      if (iflag < 0) {
        // termination, normal ou imposee par l'utilisateur.
        if (iflag < 0)
          *info = iflag;

        iflag = 0;

        if (nprint > 0)
          (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

        return (count);
      }

      fnorm1 = enorm(wa4, m);

      /*
       *	calcul de la reduction reelle mise a l'echelle.
       */

      actred = -1.0;

      if ((tol1 * fnorm1) < fnorm)
        actred = 1.0 - ((fnorm1 / fnorm) * (fnorm1 / fnorm));

      /*
       *	calcul de la reduction predite mise a l'echelle et
       *	de la derivee directionnelle mise a l'echelle.
       */

      for (i = 0; i < n; i++) {
        wa3[i] = 0.0;
        l = ipvt[i];
        temp = wa1[l];
        for (j = 0; j <= i; j++)
          wa3[j] += *MIJ(fjac, i, j, ldfjac) * temp;
      }

      temp1 = enorm(wa3, n) / fnorm;
      temp2 = (sqrt(par) * pnorm) / fnorm;
      prered = (temp1 * temp1) + (temp2 * temp2) / tol5;
      dirder = -((temp1 * temp1) + (temp2 * temp2));

      /*
       *	calcul du rapport entre la reduction reel et predit.
       */

      ratio = 0.0;

      // if (prered != 0.0)
      if (std::fabs(prered) > std::numeric_limits<double>::epsilon())
        ratio = actred / prered;

      /*
       * mise a jour de la limite de l'etape.
       */

      if (ratio > tol25) {
        // if ((par == 0.0) || (ratio <= tol75))
        if ((std::fabs(par) <= std::numeric_limits<double>::epsilon()) || (ratio <= tol75)) {
          delta = pnorm / tol5;
          par *= tol5;
        }
      } else {
        if (actred >= 0.0)
          temp = tol5;

        else
          temp = tol5 * dirder / (dirder + tol5 * actred);

        if ((tol1 * fnorm1 >= fnorm) || (temp < tol1))
          temp = tol1;

        delta = temp * vpMath::minimum(delta, (pnorm / tol1));
        par /= temp;
      }

      /*
       *	test pour une iteration reussie.
       */
      if (ratio >= tol0001) {
        /*
         *	iteration reussie. mise a jour de x, de fvec, et  de
         *	leurs normes.
         */

        for (j = 0; j < n; j++) {
          x[j] = wa2[j];
          wa2[j] = diag[j] * x[j];
        }

        for (i = 0; i < m; i++)
          fvec[i] = wa4[i];

        xnorm = enorm(wa2, n);
        fnorm = fnorm1;
        iter++;
      }

      /*
       *	tests pour convergence.
       */

      if ((std::fabs(actred) <= ftol) && (prered <= ftol) && (tol5 * ratio <= 1.0))
        *info = 1;

      if (delta <= xtol * xnorm)
        *info = 2;

      if ((std::fabs(actred) <= ftol) && (prered <= ftol) && (tol5 * ratio <= 1.0) && *info == 2)
        *info = 3;

      if (*info != 0) {
        /*
         * termination, normal ou imposee par l'utilisateur.
         */
        if (iflag < 0)
          *info = iflag;

        iflag = 0;

        if (nprint > 0)
          (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

        return (count);
      }
      /*
       *	tests pour termination et
       *	verification des tolerances.
       */

      if (*nfev >= maxfev)
        *info = 5;

      if ((std::fabs(actred) <= epsmch) && (prered <= epsmch) && (tol5 * ratio <= 1.0))
        *info = 6;

      if (delta <= epsmch * xnorm)
        *info = 7;

      if (gnorm <= epsmch)
        *info = 8;

      if (*info != 0) {
        /*
         * termination, normal ou imposee par l'utilisateur.
         */
        if (iflag < 0)
          *info = iflag;

        iflag = 0;

        if (nprint > 0)
          (*ptr_fcn)(m, n, x, fvec, fjac, ldfjac, iflag);

        return (count);
      }
    } /* fin while ratio >=tol0001	*/
  }   /*fin while 1*/

  return 0;
}

/*
 * PROCEDURE    : lmder1
 *
 * ENTREE	:
 * fcn		Fonction qui calcule la fonction et le jacobien de la
 *fonction. m		Nombre de fonctions. n		Nombre de variables
 *(parametres). n <= m x		Vecteur de taille "n" contenant en
 *entree une estimation initiale de la solution.
 * ldfjac	Taille maximale de la matrice "fjac". ldfjac >= "m".
 * tol		Tolerance. La terminaison de la procedure survient quand
 *		l'algorithme estime que l'erreur relative dans la somme des
 *		carres est au moins egal a tol ou bien que l'erreur relative
 *		entre x et la solution est au moins egal atol.
 * wa		Vecteur de travail de taille "lwa".
 * lwa		Taille du vecteur "wa". wa >= 5 * n + m.
 *
 *
 * SORTIE	:
 * x		Vecteur de taille "n" contenant en sortie l'estimee finale
 *		de la solution.
 * fvec		Vecteur de taille "m" contenant les fonctions evaluee en "x".
 * fjac		Matrice de taille "m" x "n". La sous matrice superieure de
 *		taille "n" x "n" de fjac contient une matrice triangulaire
 *		superieure r dont les elements diagonaux, classe dans le sens
 *		decroissant de leur valeur, sont de la forme :
 *
 *      	                 T      T              T
 *				p * (jac * jac) * p = r * r
 *
 *		Ou p est une matrice de permutation et jac est le jacobien
 *		     final calcule.
 *		La colonne j de p est la colonne ipvt (j) (voir ci apres) de
 *		la matrice identite. La partie trapesoidale inferieure de fjac
 *		contient les information genere durant le calcul de r.
 * info		Information de l'executionde la procedure. Lorsque la
 *procedure a termine son execution, "info" est inialisee a la valeur
 *		(negative) de iflag. sinon elle prend les valeurs suivantes :
 *		info = 0 : parametres en entre non valides.
 *		info = 1 : estimation par l'algorithme que l'erreur relative
 *			   de la somme des carre est egal a tol.
 *		info = 2 : estimation par l'algorithme que l'erreur relative
 *			   entre x et la solution est egal a tol.
 *		info = 3 : conditions info = 1 et info = 2 tous deux requis.
 *		info = 4 : fvec est orthogonal aux colonnes du jacobien.
 *		info = 5 : nombre d'appels a fcn avec iflag = 1 a atteint
 *			   100 * (n + 1).
 *		info = 6 : tol est trop petit. Plus moyen de reduire de la
 *			   somme des carres.
 *		info = 7 : tol est trop petit. Plus d'amelioration possible
 *			   d' approximer la solution x.
 * ipvt		Vecteur de taille "n". Il definit une matrice de permutation p
 *		tel que jac * p = q * p, ou jac est le jacbien final calcule,
 *		q est orthogonal (non socke) et r est triangulaire superieur,
 *		avec les elements diagonaux classes en ordre decroissant de
 *		leur valeur. La colonne j de p est ipvt[j] de la matrice
 *identite.
 *
 * DESCRIPTION  :
 * La procedure minimize la somme de carre de m equation non lineaire a n
 * variables par une modification de l'algorithme de Levenberg - Marquardt.
 * Cette procedure appele la procedure generale au moindre carre lmder.
 *
 * REMARQUE	:
 * L'utilisateur doit fournir une procedure "fcn" qui calcule la fonction et
 * le jacobien.
 * "fcn" doit etre declare dans une instruction externe a la procedure et doit
 * etre appele comme suit :
 * fcn (int m, int n, int ldfjac, double *x, double *fvec, double *fjac, int
 **iflag)
 *
 * si iflag = 1 calcul de la fonction en x et retour de ce vecteur dans fvec.
 *		fjac n'est pas modifie.
 * si iflag = 2 calcul du jacobien en x et retour de cette matrice dans fjac.
 *		fvec n'est pas modifie.
 *
 * RETOUR	:
 * En cas de succes, la valeur zero est retournee.
 * Sinon, la valeur -1.
 *
 */
int lmder1(void (*ptr_fcn)(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag), int m, int n,
           double *x, double *fvec, double *fjac, int ldfjac, double tol, int *info, int *ipvt, int lwa, double *wa)
{
  const double factor = 100.0;
  unsigned int maxfev, nfev;
  int mode, njev, nprint;
  double ftol, gtol, xtol;

  *info = 0;

  /* verification des parametres en entree qui causent des erreurs */

  if (/*(n <= 0) ||*/ (m < n) || (ldfjac < m) || (tol < 0.0) || (lwa < (5 * n + m))) {
    printf("%d %d %d  %d \n", (m < n), (ldfjac < m), (tol < 0.0), (lwa < (5 * n + m)));
    return (-1);
  }

  /* appel a lmder	*/

  maxfev = (unsigned int)(100 * (n + 1));
  ftol = tol;
  xtol = tol;
  gtol = 0.0;
  mode = 1;
  nprint = 0;

  lmder(ptr_fcn, m, n, x, fvec, fjac, ldfjac, ftol, xtol, gtol, maxfev, wa, mode, factor, nprint, info, &nfev, &njev,
        ipvt, &wa[n], &wa[2 * n], &wa[3 * n], &wa[4 * n], &wa[5 * n]);

  if (*info == 8)
    *info = 4;

  return (0);
}

#undef TRUE
#undef FALSE
