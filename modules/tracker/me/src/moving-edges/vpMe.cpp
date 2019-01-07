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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
        \file vpMe.cpp
        \brief Moving edges
*/

#include <stdlib.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMe.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

struct point {
  double x;
  double y;
};

struct droite {
  double a;
  double b;
  double c;
};

template <class Type> inline void permute(Type &a, Type &b)
{
  Type t = a;
  a = b;
  b = t;
}

static droite droite_cartesienne(point P, point Q)
{
  droite PQ;

  PQ.a = P.y - Q.y;
  PQ.b = Q.x - P.x;
  PQ.c = Q.y * P.x - Q.x * P.y;

  return (PQ);
}

static point point_intersection(droite D1, droite D2)
{
  point I;
  double det; // determinant des 2 vect.normaux

  det = (D1.a * D2.b - D2.a * D1.b); // interdit D1,D2 paralleles
  I.x = (D2.c * D1.b - D1.c * D2.b) / det;
  I.y = (D1.c * D2.a - D2.c * D1.a) / det;

  return (I);
}

static void recale(point &P, double Xmin, double Ymin, double Xmax, double Ymax)
{
  if (vpMath::equal(P.x, Xmin))
    P.x = Xmin; // a peu pres => exactement !
  if (vpMath::equal(P.x, Xmax))
    P.x = Xmax;

  if (vpMath::equal(P.y, Ymin))
    P.y = Ymin;
  if (vpMath::equal(P.y, Ymax))
    P.y = Ymax;
}

static void permute(point &A, point &B)
{
  point C;

  if (A.x > B.x) // fonction sans doute a tester...
  {
    C = A;
    A = B;
    B = C;
  }
}

// vrai si partie visible
static bool clipping(point A, point B, double Xmin, double Ymin, double Xmax, double Ymax, point &Ac,
                     point &Bc) // resultat: A,B clippes
{
  droite AB, D[4];
  D[0].a = 1;
  D[0].b = 0;
  D[0].c = -Xmin;
  D[1].a = 1;
  D[1].b = 0;
  D[1].c = -Xmax;
  D[2].a = 0;
  D[2].b = 1;
  D[2].c = -Ymin;
  D[3].a = 0;
  D[3].b = 1;
  D[3].c = -Ymax;

  point P[2];
  P[0] = A;
  P[1] = B;
  int code_P[2], // codes de P[n]
      i, bit_i,  // i -> (0000100...)
      n;

  AB = droite_cartesienne(A, B);

  for (;;) // 2 sorties directes internes
  {
    // CALCULE CODE DE VISIBILITE (Sutherland & Sproul)
    // ================================================
    for (n = 0; n < 2; n++) {
      code_P[n] = 0000;

      if (P[n].x < Xmin)
        code_P[n] |= 1; // positionne bit0
      if (P[n].x > Xmax)
        code_P[n] |= 2; //    ..      bit1
      if (P[n].y < Ymin)
        code_P[n] |= 4; //    ..      bit2
      if (P[n].y > Ymax)
        code_P[n] |= 8; //    ..      bit3
    }

    // 2 CAS OU L'ON PEUT CONCLURE => sortie
    // =====================================
    if ((code_P[0] | code_P[1]) == 0000) // Aucun bit a 1
    /* NE TRIE PLUS LE RESULTAT ! S_relative() en tient compte
{ if(P[0].x < P[1].x) // Rend le couple de points
    { Ac=P[0];  Bc=P[1]; }  //  clippes (ordonnes selon
else  { Ac=P[1];  Bc=P[0]; }  //  leur abscisse x)
    */
    {
      Ac = P[0];
      Bc = P[1];
      if (vpMath::equal(Ac.x, Bc.x) && vpMath::equal(Ac.y, Bc.y))
        return (false); // AB = 1 point = invisible
      else
        return (true); // Partie de AB clippee visible!
    }

    if ((code_P[0] & code_P[1]) != 0000) // au moins 1 bit commun
    {
      return (false); // AB completement invisible!
    }

    // CAS GENERAL (on sait que code_P[0 ou 1] a au moins un bit a 1
    //   - clippe le point P[n] qui sort de la fenetre (coupe Droite i)
    //   - reboucle avec le nouveau couple de points
    // ================================================================
    if (code_P[0] != 0000) {
      n = 0; // c'est P[0] qu'on clippera
      for (i = 0, bit_i = 1; !(code_P[0] & bit_i); i++, bit_i <<= 1) {
        ;
      }
    } else {
      n = 1; // c'est P[1] qu'on clippera
      for (i = 0, bit_i = 1; !(code_P[1] & bit_i); i++, bit_i <<= 1) {
        ;
      }
    }

    P[n] = point_intersection(AB, D[i]); // clippe le point concerne

    // RECALE EXACTEMENT LE POINT (calcul flottant => arrondi)
    // AFIN QUE LE CALCUL DES CODES NE BOUCLE PAS INDEFINIMENT
    // =======================================================
    recale(P[n], Xmin, Ymin, Xmax, Ymax);
  }
}

// calcule la surface relative des 2 portions definies
// par le segment PQ sur le carre Xmin,Ymin,Xmax,Ymax
// Rem : P,Q tries sur x, et donc seulement 6 cas
static double S_relative(point P, point Q, double Xmin, double Ymin, double Xmax, double Ymax)
{

  if (Q.x < P.x)   // tri le couple de points
    permute(P, Q); //  selon leur abscisse x

  recale(P, Xmin, Ymin, Xmax, Ymax); // permet des calculs de S_relative
  recale(Q, Xmin, Ymin, Xmax, Ymax); //  moins approximatifs.

  // if(P.x==Xmin && Q.x==Xmax)
  if ((std::fabs(P.x - Xmin) <=
       vpMath::maximum(std::fabs(P.x), std::fabs(Xmin)) * std::numeric_limits<double>::epsilon()) &&
      (std::fabs(Q.x - Xmax) <=
       vpMath::maximum(std::fabs(Q.x), std::fabs(Xmax)) * std::numeric_limits<double>::epsilon()))
    return (fabs(Ymax + Ymin - P.y - Q.y));

  // if( (P.y==Ymin && Q.y==Ymax) ||
  //  (Q.y==Ymin && P.y==Ymax))
  if (((std::fabs(P.y - Ymin) <=
        vpMath::maximum(std::fabs(P.y), std::fabs(Ymin)) * std::numeric_limits<double>::epsilon()) &&
       (std::fabs(Q.y - Ymax) <=
        vpMath::maximum(std::fabs(Q.y), std::fabs(Ymax)) * std::numeric_limits<double>::epsilon())) ||
      ((std::fabs(Q.y - Ymin) <=
        vpMath::maximum(std::fabs(Q.y), std::fabs(Ymin)) * std::numeric_limits<double>::epsilon()) &&
       (std::fabs(P.y - Ymax) <=
        vpMath::maximum(std::fabs(P.y), std::fabs(Ymax)) * std::numeric_limits<double>::epsilon())))
    return (fabs(Xmax + Xmin - P.x - Q.x));

  // if( P.x==Xmin && Q.y==Ymax )
  if (std::fabs(P.x - Xmin) <=
          vpMath::maximum(std::fabs(P.x), std::fabs(Xmin)) * std::numeric_limits<double>::epsilon() &&
      std::fabs(Q.y - Ymax) <=
          vpMath::maximum(std::fabs(Q.y), std::fabs(Ymax)) * std::numeric_limits<double>::epsilon())
    return (1 - (Ymax - P.y) * (Q.x - Xmin));
  // if( P.x==Xmin && Q.y==Ymin )
  if (std::fabs(P.x - Xmin) <=
          vpMath::maximum(std::fabs(P.x), std::fabs(Xmin)) * std::numeric_limits<double>::epsilon() &&
      std::fabs(Q.y - Ymin) <=
          vpMath::maximum(std::fabs(Q.y), std::fabs(Ymin)) * std::numeric_limits<double>::epsilon())
    return (1 - (P.y - Ymin) * (Q.x - Xmin));
  // if( P.y==Ymin && Q.x==Xmax )
  if (std::fabs(P.y - Ymin) <=
          vpMath::maximum(std::fabs(P.y), std::fabs(Ymin)) * std::numeric_limits<double>::epsilon() &&
      std::fabs(Q.x - Xmax) <=
          vpMath::maximum(std::fabs(Q.x), std::fabs(Xmax)) * std::numeric_limits<double>::epsilon())
    return (1 - (Xmax - P.x) * (Q.y - Ymin));
  // if( P.y==Ymax && Q.x==Xmax )
  if (std::fabs(P.y - Ymax) <=
          vpMath::maximum(std::fabs(P.y), std::fabs(Ymax)) * std::numeric_limits<double>::epsilon() &&
      std::fabs(Q.x - Xmax) <=
          vpMath::maximum(std::fabs(Q.x), std::fabs(Xmax)) * std::numeric_limits<double>::epsilon())
    return (1 - (Xmax - P.x) * (Ymax - Q.y));

  printf("utils_ecm: ERREUR dans S_relative (%f,%f) (%f,%f) %f %f %f %f\n", P.x, P.y, Q.x, Q.y, Xmin, Ymin, Xmax, Ymax);
  exit(-1); // DEBUG Stoppe net l'execution
}

static void calcul_masques(vpColVector &angle, // definitions des angles theta
                           unsigned int n,     // taille masques (PAIRE ou IMPAIRE Ok)
                           vpMatrix *M)        // resultat M[theta](n,n)
{
  // Le coef |a| = |1/2n| n'est pas incorpore dans M(i,j) (=> que des int)

  unsigned int i_theta,           // indice (boucle sur les masques)
      i, j;                       // indices de boucle sur M(i,j)
  double X, Y,                    // point correspondant/centre du masque
      moitie = ((double)n) / 2.0; // moitie REELLE du masque
  point P1, Q1, P, Q;             // clippe Droite(theta) P1,Q1 -> P,Q
  int sgn;                        // signe de M(i,j)
  double v;                       // ponderation de M(i,j)

  unsigned int nb_theta = angle.getRows();

  for (i_theta = 0; i_theta < nb_theta; i_theta++) {
    double theta = M_PI / 180 * angle[i_theta]; // indice i -> theta(i) en radians
                                                //  angle[] dans [0,180[
    double cos_theta = cos(theta);              // vecteur directeur de l'ECM
    double sin_theta = sin(theta);              //  associe au masque

    // PRE-CALCULE 2 POINTS DE D(theta) BIEN EN DEHORS DU MASQUE
    // =========================================================
    // if( angle[i_theta]==90 )                     // => tan(theta) infinie !
    if (std::fabs(angle[i_theta] - 90) <= vpMath::maximum(std::fabs(angle[i_theta]), 90.) *
                                              std::numeric_limits<double>::epsilon()) // => tan(theta) infinie !
    {
      P1.x = 0;
      P1.y = -(int)n;
      Q1.x = 0;
      Q1.y = n;
    } else {
      double tan_theta = sin_theta / cos_theta; // pente de la droite D(theta)
      P1.x = -(int)n;
      P1.y = tan_theta * (-(int)n);
      Q1.x = n;
      Q1.y = tan_theta * n;
    }

    // CALCULE MASQUE M(theta)
    // ======================
    M[i_theta].resize(n, n); // allocation (si necessaire)

    for (i = 0, Y = -moitie + 0.5; i < n; i++, Y++) {
      for (j = 0, X = -moitie + 0.5; j < n; j++, X++) {
        // produit vectoriel dir_droite*(X,Y)
        sgn = vpMath::sign(cos_theta * Y - sin_theta * X);

        // Resultat = P,Q
        if (clipping(P1, Q1, X - 0.5, Y - 0.5, X + 0.5, Y + 0.5, P, Q)) {
          // v dans [0,1]
          v = S_relative(P, Q, X - 0.5, Y - 0.5, X + 0.5, Y + 0.5);
        } else
          v = 1; // PQ ne coupe pas le pixel(i,j)

        M[i_theta][i][j] = vpMath::round(100 * sgn * v);

        // 2 chiffres significatifs
        // M(i,j) sans incorporer le coef a
      }
    }
  }
}

#endif

/*!
  Initialise the array of matrices with the defined size and the number of
  matrices to create.

*/
void vpMe::initMask()
{

  if (mask != NULL)
    delete[] mask;

  mask = new vpMatrix[n_mask];

  vpColVector angle(n_mask);

  unsigned int angle_pas;
  angle_pas = 180 / n_mask;

  unsigned int k = 0;
  for (unsigned int i = 0; /* i < 180, */ k < n_mask; i += angle_pas)
    angle[k++] = i;

  calcul_masques(angle, mask_size, mask);
}

void vpMe::print()
{

  std::cout << std::endl;
  std::cout << "Moving edges settings " << std::endl;
  std::cout << std::endl;
  std::cout << " Size of the convolution masks...." << mask_size << "x" << mask_size << " pixels" << std::endl;
  std::cout << " Number of masks.................." << n_mask << "        " << std::endl;
  std::cout << " Query range +/- J................" << range << " pixels  " << std::endl;
  std::cout << " Likelihood test ratio............" << threshold << std::endl;
  std::cout << " Contrast tolerance +/-..........." << mu1 * 100 << "% and " << mu2 * 100 << "%     " << std::endl;
  std::cout << " Sample step......................" << sample_step << " pixels" << std::endl;
  std::cout << " Strip............................" << strip << " pixels  " << std::endl;
  std::cout << " Min_Samplestep..................." << min_samplestep << " pixels  " << std::endl;
}

vpMe::vpMe()
  : threshold(1500), mu1(0.5), mu2(0.5), min_samplestep(4), anglestep(1), mask_sign(0), range(4), sample_step(10),
    ntotal_sample(0), points_to_track(500), mask_size(5), n_mask(180), strip(2), mask(NULL)
{
  // ntotal_sample = 0; // not sure that it is used
  // points_to_track = 500; // not sure that it is used
  anglestep = (180 / n_mask);

  initMask();
}

vpMe::vpMe(const vpMe &me)
  : threshold(1500), mu1(0.5), mu2(0.5), min_samplestep(4), anglestep(1), mask_sign(0), range(4), sample_step(10),
    ntotal_sample(0), points_to_track(500), mask_size(5), n_mask(180), strip(2), mask(NULL)
{
  *this = me;
}

//! Copy operator.
vpMe &vpMe::operator=(const vpMe &me)
{
  if (mask != NULL) {
    delete[] mask;
    mask = NULL;
  }
  threshold = me.threshold;
  mu1 = me.mu1;
  mu2 = me.mu2;
  min_samplestep = me.min_samplestep;
  anglestep = me.anglestep;
  mask_size = me.mask_size;
  n_mask = me.n_mask;
  mask_sign = me.mask_sign;
  range = me.range;
  sample_step = me.sample_step;
  ntotal_sample = me.ntotal_sample;
  points_to_track = me.points_to_track;
  strip = me.strip;

  initMask();
  return *this;
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
//! Move operator.
vpMe &vpMe::operator=(const vpMe &&me)
{
  if (mask != NULL) {
    delete[] mask;
    mask = NULL;
  }
  threshold = std::move(me.threshold);
  mu1 = std::move(me.mu1);
  mu2 = std::move(me.mu2);
  min_samplestep = std::move(me.min_samplestep);
  anglestep = std::move(me.anglestep);
  mask_size = std::move(me.mask_size);
  n_mask = std::move(me.n_mask);
  mask_sign = std::move(me.mask_sign);
  range = std::move(me.range);
  sample_step = std::move(me.sample_step);
  ntotal_sample = std::move(me.ntotal_sample);
  points_to_track = std::move(me.points_to_track);
  strip = std::move(me.strip);

  initMask();
  return *this;
}
#endif

vpMe::~vpMe()
{
  if (mask != NULL) {
    delete[] mask;
    mask = NULL;
  }
}

void vpMe::setMaskNumber(const unsigned int &n)
{
  n_mask = n;
  anglestep = 180 / n_mask;
  initMask();
}

void vpMe::setMaskSize(const unsigned int &s)
{
  mask_size = s;
  initMask();
}
