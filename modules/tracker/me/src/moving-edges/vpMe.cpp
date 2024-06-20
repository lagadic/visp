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
 * Moving edges.
 */

/*!
  \file vpMe.cpp
  \brief Moving edges
*/

#include <stdlib.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/me/vpMe.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace
{
struct vpPoint2Dt
{
  double x;
  double y;
};

struct vpDroite2Dt
{
  double a;
  double b;
  double c;
};

struct vpBBoxt
{
  double Xmin;
  double Ymin;
  double Xmax;
  double Ymax;
};

template <class Type> inline void permute(Type &a, Type &b)
{
  Type t = a;
  a = b;
  b = t;
}

static vpDroite2Dt droiteCartesienne(vpPoint2Dt P, vpPoint2Dt Q)
{
  vpDroite2Dt PQ;

  PQ.a = P.y - Q.y;
  PQ.b = Q.x - P.x;
  PQ.c = (Q.y * P.x) - (Q.x * P.y);

  return PQ;
}

static vpPoint2Dt pointIntersection(vpDroite2Dt D1, vpDroite2Dt D2)
{
  vpPoint2Dt I;
  double det; // determinant des 2 vect.normaux

  det = ((D1.a * D2.b) - (D2.a * D1.b)); // interdit D1,D2 paralleles
  I.x = ((D2.c * D1.b) - (D1.c * D2.b)) / det;
  I.y = ((D1.c * D2.a) - (D2.c * D1.a)) / det;

  return I;
}

static void recale(vpPoint2Dt &P, const vpBBoxt &bbox)
{
  if (vpMath::equal(P.x, bbox.Xmin)) {
    P.x = bbox.Xmin; // a peu pres => exactement !
  }

  if (vpMath::equal(P.x, bbox.Xmax)) {
    P.x = bbox.Xmax;
  }

  if (vpMath::equal(P.y, bbox.Ymin)) {
    P.y = bbox.Ymin;
  }

  if (vpMath::equal(P.y, bbox.Ymax)) {
    P.y = bbox.Ymax;
  }
}

static void permute(vpPoint2Dt &A, vpPoint2Dt &B)
{
  vpPoint2Dt C;

  if (A.x > B.x) // fonction sans doute a tester...
  {
    C = A;
    A = B;
    B = C;
  }
}

// vrai si partie visible
static bool clipping(vpPoint2Dt A, vpPoint2Dt B, const vpBBoxt &bbox, vpPoint2Dt &Ac,
                     vpPoint2Dt &Bc) // resultat: A,B clippes
{
  vpDroite2Dt AB, D[4];
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int val_1 = 1;
  const unsigned int val_2 = 2;
  const unsigned int val_4 = 4;
  const unsigned int val_8 = 8;

  D[index_0].a = 1;
  D[index_0].b = 0;
  D[index_0].c = -bbox.Xmin;
  D[index_1].a = 1;
  D[index_1].b = 0;
  D[index_1].c = -bbox.Xmax;
  D[index_2].a = 0;
  D[index_2].b = 1;
  D[index_2].c = -bbox.Ymin;
  D[index_3].a = 0;
  D[index_3].b = 1;
  D[index_3].c = -bbox.Ymax;

  const int nbP = val_2;
  vpPoint2Dt P[nbP];
  P[index_0] = A;
  P[index_1] = B;
  unsigned int code_P[nbP], // codes de P[n]
    i, bit_i,  // i -> (0000100...)
    n;

  AB = droiteCartesienne(A, B);

  for (;;) // 2 sorties directes internes
  {
    // CALCULE CODE DE VISIBILITE (Sutherland & Sproul)
    // ================================================
    for (n = 0; n < nbP; ++n) {
      code_P[n] = 0;

      if (P[n].x < bbox.Xmin) {
        code_P[n] |= val_1; // positionne bit0
      }

      if (P[n].x > bbox.Xmax) {
        code_P[n] |= val_2; //    ..      bit1
      }

      if (P[n].y < bbox.Ymin) {
        code_P[n] |= val_4; //    ..      bit2
      }

      if (P[n].y > bbox.Ymax) {
        code_P[n] |= val_8; //    ..      bit3
      }
    }

    // 2 CAS OU L'ON PEUT CONCLURE => sortie
    // =====================================
    if ((code_P[0] | code_P[1]) == 0) {
      Ac = P[0];
      Bc = P[1];
      if (vpMath::equal(Ac.x, Bc.x) && vpMath::equal(Ac.y, Bc.y)) {
        return false; // AB = 1 point = invisible
      }
      else {
        return true; // Partie de AB clippee visible!
      }
    }

    if ((code_P[0] & code_P[1]) != 0) // au moins 1 bit commun
    {
      return false; // AB completement invisible!
    }

    // CAS GENERAL (on sait que code_P[0 ou 1] a au moins un bit a 1
    //   - clippe le point P[n] qui sort de la fenetre (coupe Droite i)
    //   - reboucle avec le nouveau couple de points
    // ================================================================
    if (code_P[0] != 0) {
      n = 0; // c'est P[0] qu'on clippera
      bit_i = 1;
      i = 0;
      while (!(code_P[0] & bit_i)) {
        bit_i <<= 1;
        ++i;
      }
    }
    else {
      n = 1; // c'est P[1] qu'on clippera
      bit_i = 1;
      i = 0;
      while (!(code_P[1] & bit_i)) {
        bit_i <<= 1;
        ++i;
      }
    }

    P[n] = pointIntersection(AB, D[i]); // clippe le point concerne

    // RECALE EXACTEMENT LE POINT (calcul flottant => arrondi)
    // AFIN QUE LE CALCUL DES CODES NE BOUCLE PAS INDEFINIMENT
    // =======================================================
    recale(P[n], bbox);
  }
}

// calcule la surface relative des 2 portions definies
// par le segment PQ sur le carre Xmin,Ymin,Xmax,Ymax
// Rem : P,Q tries sur x, et donc seulement 6 cas
static double surfaceRelative(vpPoint2Dt P, vpPoint2Dt Q, const vpBBoxt &bbox)
{

  if (Q.x < P.x) {   // tri le couple de points
    permute(P, Q); //  selon leur abscisse x
  }

  recale(P, bbox); // permet des calculs de S_relative
  recale(Q, bbox); //  moins approximatifs.

  // Case P.x=Xmin and Q.x=Xmax
  if ((std::fabs(P.x - bbox.Xmin) <=
       (vpMath::maximum(std::fabs(P.x), std::fabs(bbox.Xmin)) * std::numeric_limits<double>::epsilon())) &&
      (std::fabs(Q.x - bbox.Xmax) <=
       (vpMath::maximum(std::fabs(Q.x), std::fabs(bbox.Xmax)) * std::numeric_limits<double>::epsilon()))) {
    return fabs((bbox.Ymax + bbox.Ymin) - (P.y - Q.y));
  }

  // Case (P.y=Ymin and Q.y==Ymax) or (Q.y=Ymin and P.y==Ymax)
  if (((std::fabs(P.y - bbox.Ymin) <=
        (vpMath::maximum(std::fabs(P.y), std::fabs(bbox.Ymin)) * std::numeric_limits<double>::epsilon())) &&
       (std::fabs(Q.y - bbox.Ymax) <=
        (vpMath::maximum(std::fabs(Q.y), std::fabs(bbox.Ymax)) * std::numeric_limits<double>::epsilon()))) ||
      ((std::fabs(Q.y - bbox.Ymin) <=
        (vpMath::maximum(std::fabs(Q.y), std::fabs(bbox.Ymin)) * std::numeric_limits<double>::epsilon())) &&
       (std::fabs(P.y - bbox.Ymax) <=
        (vpMath::maximum(std::fabs(P.y), std::fabs(bbox.Ymax)) * std::numeric_limits<double>::epsilon())))) {
    return fabs((bbox.Xmax + bbox.Xmin) - (P.x - Q.x));
  }

  // Case P.x=Xmin and Q.y=Ymax
  if ((std::fabs(P.x - bbox.Xmin) <=
       (vpMath::maximum(std::fabs(P.x), std::fabs(bbox.Xmin)) * std::numeric_limits<double>::epsilon())) &&
      (std::fabs(Q.y - bbox.Ymax) <=
       (vpMath::maximum(std::fabs(Q.y), std::fabs(bbox.Ymax)) * std::numeric_limits<double>::epsilon()))) {
    return (1 - ((bbox.Ymax - P.y) * (Q.x - bbox.Xmin)));
  }

  // Case P.x=Xmin and Q.y=Ymin
  if ((std::fabs(P.x - bbox.Xmin) <=
       (vpMath::maximum(std::fabs(P.x), std::fabs(bbox.Xmin)) * std::numeric_limits<double>::epsilon())) &&
      (std::fabs(Q.y - bbox.Ymin) <=
       (vpMath::maximum(std::fabs(Q.y), std::fabs(bbox.Ymin)) * std::numeric_limits<double>::epsilon()))) {
    return (1 - ((P.y - bbox.Ymin) * (Q.x - bbox.Xmin)));
  }

  // Case P.y=Ymin and Q.x=Xmax
  if ((std::fabs(P.y - bbox.Ymin) <=
       (vpMath::maximum(std::fabs(P.y), std::fabs(bbox.Ymin)) * std::numeric_limits<double>::epsilon())) &&
      (std::fabs(Q.x - bbox.Xmax) <=
       (vpMath::maximum(std::fabs(Q.x), std::fabs(bbox.Xmax)) * std::numeric_limits<double>::epsilon()))) {
    return (1 - ((bbox.Xmax - P.x) * (Q.y - bbox.Ymin)));
  }

  // Case P.y=Ymax and Q.x=Xmax
  if ((std::fabs(P.y - bbox.Ymax) <=
       (vpMath::maximum(std::fabs(P.y), std::fabs(bbox.Ymax)) * std::numeric_limits<double>::epsilon())) &&
      (std::fabs(Q.x - bbox.Xmax) <=
       (vpMath::maximum(std::fabs(Q.x), std::fabs(bbox.Xmax)) * std::numeric_limits<double>::epsilon()))) {
    return (1 - ((bbox.Xmax - P.x) * (bbox.Ymax - Q.y)));
  }

  throw(vpException(vpException::fatalError, "utils_ecm: error in surfaceRelative (%f,%f) (%f,%f) %f %f %f %f",
                    P.x, P.y, Q.x, Q.y, bbox.Xmin, bbox.Ymin, bbox.Xmax, bbox.Ymax));
}

static void calculMasques(vpColVector &angle, // definitions des angles theta
                          unsigned int n,     // taille masques (PAIRE ou IMPAIRE Ok)
                          vpMatrix *M)        // resultat M[theta](n,n)
{
  unsigned int i, j;
  double X, Y, moitie = (static_cast<double>(n)) / 2.0; // moitie REELLE du masque
  vpPoint2Dt P1, Q1, P, Q;             // clippe Droite(theta) P1,Q1 -> P,Q
  double v;                       // ponderation de M(i,j)

  // For a mask of size nxn, normalization given by n*trunc(n/2.0)
  // Typically, norm = 1/10 for a mask of size 5x5
  double norm = 1.0 / (n * trunc(n / 2.0));

  unsigned int nb_theta = angle.getRows();

  for (unsigned int i_theta = 0; i_theta < nb_theta; ++i_theta) {
    double theta = (M_PI / 180) * angle[i_theta]; // indice i -> theta(i) en radians
    //  angle[] dans [0,180[
    double cos_theta = cos(theta);              // vecteur directeur de l'ECM
    double sin_theta = sin(theta);              //  associe au masque

    // PRE-CALCULE 2 POINTS DE D(theta) BIEN EN DEHORS DU MASQUE
    // =========================================================
    // if( angle[i_theta]==90 )                     // => tan(theta) infinie !
    const double thetaWhoseTanInfinite = 90.;
    if (std::fabs(angle[i_theta] - thetaWhoseTanInfinite) <= (vpMath::maximum(std::fabs(angle[i_theta]), thetaWhoseTanInfinite) *
                                                              std::numeric_limits<double>::epsilon())) // => tan(theta) infinie !
    {
      P1.x = 0;
      P1.y = -static_cast<int>(n);
      Q1.x = 0;
      Q1.y = n;
    }
    else {
      double tan_theta = sin_theta / cos_theta; // pente de la droite D(theta)
      P1.x = -static_cast<int>(n);
      P1.y = tan_theta * (-static_cast<int>(n));
      Q1.x = n;
      Q1.y = tan_theta * n;
    }

    // CALCULE MASQUE M(theta)
    // ======================
    M[i_theta].resize(n, n); // allocation (si necessaire)

    for (i = 0, Y = (-moitie + 0.5); i < n; ++i, ++Y) {
      for (j = 0, X = (-moitie + 0.5); j < n; ++j, ++X) {
        // produit vectoriel dir_droite*(X,Y)
        int sgn = vpMath::sign((cos_theta * Y) - (sin_theta * X));

        // Resultat = P,Q
        vpBBoxt bbox;
        bbox.Xmin = X - 0.5;
        bbox.Ymin = Y - 0.5;
        bbox.Xmax = X + 0.5;
        bbox.Ymax = Y + 0.5;
        if (clipping(P1, Q1, bbox, P, Q)) {
          // v dans [0,1]
          v = surfaceRelative(P, Q, bbox);
        }
        else {
          v = 1; // PQ ne coupe pas le pixel(i,j)
        }

        M[i_theta][i][j] = sgn * v * norm;
      }
    }
  }
}
}
#endif

void vpMe::initMask()
{
  if (m_mask != nullptr) {
    delete[] m_mask;
  }

  m_mask = new vpMatrix[m_mask_number];

  vpColVector angle(m_mask_number);

  unsigned int angle_pas = 180 / m_mask_number;

  unsigned int i = 0;
  for (unsigned int k = 0; k < m_mask_number; ++k) {
    angle[k] = i;
    i += angle_pas;
  }

  calculMasques(angle, m_mask_size, m_mask);
}

void vpMe::print()
{
  std::cout << std::endl;
  std::cout << "Moving edges settings " << std::endl;
  std::cout << std::endl;
  std::cout << " Size of the convolution masks...." << m_mask_size << "x" << m_mask_size << " pixels" << std::endl;
  std::cout << " Number of masks.................." << m_mask_number << std::endl;
  std::cout << " Query range +/- J................" << m_range << " pixels" << std::endl;
  std::cout << " Likelihood threshold type........" << (m_likelihood_threshold_type == NORMALIZED_THRESHOLD ? "normalized " : "old threshold (to be avoided)") << std::endl;

  if (m_useAutomaticThreshold) {
    std::cout << " Likelihood threshold............." << "unused" << std::endl;
    std::cout << " Likelihood margin ratio.........." << m_thresholdMarginRatio << std::endl;
    std::cout << " Minimum likelihood threshold....." << m_minThreshold << std::endl;
  }
  else {
    std::cout << " Likelihood threshold............." << m_threshold << std::endl;
    std::cout << " Likelihood margin ratio.........." << "unused" << std::endl;
    std::cout << " Minimum likelihood threshold....." << "unused" << std::endl;
  }

  const unsigned int val_100 = 100;
  std::cout << " Contrast tolerance +/-..........." << (m_mu1 * val_100) << "% and " << (m_mu2 * val_100) << "%     " << std::endl;
  std::cout << " Sample step......................" << m_sample_step << " pixels" << std::endl;
  std::cout << " Strip............................" << m_strip << " pixels  " << std::endl;
  std::cout << " Min sample step.................." << m_min_samplestep << " pixels  " << std::endl;
}

vpMe::vpMe()
  : m_likelihood_threshold_type(OLD_THRESHOLD), m_thresholdMarginRatio(-1), m_minThreshold(-1), m_useAutomaticThreshold(false),
  m_mu1(0.5), m_mu2(0.5), m_anglestep(1), m_mask_sign(0), m_ntotal_sample(0), m_mask(nullptr)
{
  const double threshold_default = 1000;
  const int points_to_track_default = 500;
  const unsigned int mask_number_default = 180;
  const unsigned int mask_size_default = 5;
  const int strip_default = 2;
  const double min_samplestep_default = 4;
  const unsigned int flatAngle = 180;
  const unsigned int range_default = 4;
  const double sample_step_default = 10;

  m_threshold = threshold_default;
  m_points_to_track = points_to_track_default;
  m_mask_number = mask_number_default;
  m_mask_size = mask_size_default;
  m_strip = strip_default;
  m_min_samplestep = min_samplestep_default;
  m_range = range_default;
  m_sample_step = sample_step_default;

  m_anglestep = (flatAngle / m_mask_number);

  initMask();
}

vpMe::vpMe(const vpMe &me)
  : m_likelihood_threshold_type(OLD_THRESHOLD), m_thresholdMarginRatio(-1), m_minThreshold(-1), m_useAutomaticThreshold(false),
  m_mu1(0.5), m_mu2(0.5), m_anglestep(1), m_mask_sign(0), m_ntotal_sample(0), m_mask(nullptr)
{
  const double threshold_default = 1000;
  const int points_to_track_default = 500;
  const unsigned int mask_number_default = 180;
  const unsigned int mask_size_default = 5;
  const int strip_default = 2;
  const double min_samplestep_default = 4;
  const unsigned int range_default = 4;
  const double sample_step_default = 10;

  m_threshold = threshold_default;
  m_points_to_track = points_to_track_default;
  m_mask_number = mask_number_default;
  m_mask_size = mask_size_default;
  m_strip = strip_default;
  m_min_samplestep = min_samplestep_default;
  m_range = range_default;
  m_sample_step = sample_step_default;

  *this = me;
}

vpMe &vpMe::operator=(const vpMe &me)
{
  if (m_mask != nullptr) {
    delete[] m_mask;
    m_mask = nullptr;
  }

  m_likelihood_threshold_type = me.m_likelihood_threshold_type;
  m_threshold = me.m_threshold;
  m_thresholdMarginRatio = me.m_thresholdMarginRatio;
  m_minThreshold = me.m_minThreshold;
  m_useAutomaticThreshold = me.m_useAutomaticThreshold;
  m_mu1 = me.m_mu1;
  m_mu2 = me.m_mu2;
  m_min_samplestep = me.m_min_samplestep;
  m_anglestep = me.m_anglestep;
  m_mask_size = me.m_mask_size;
  m_mask_number = me.m_mask_number;
  m_mask_sign = me.m_mask_sign;
  m_range = me.m_range;
  m_sample_step = me.m_sample_step;
  m_ntotal_sample = me.m_ntotal_sample;
  m_points_to_track = me.m_points_to_track;
  m_strip = me.m_strip;

  initMask();
  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMe &vpMe::operator=(const vpMe &&me)
{
  if (m_mask != nullptr) {
    delete[] m_mask;
    m_mask = nullptr;
  }
  m_likelihood_threshold_type = std::move(me.m_likelihood_threshold_type);
  m_threshold = std::move(me.m_threshold);
  m_thresholdMarginRatio = std::move(me.m_thresholdMarginRatio);
  m_minThreshold = std::move(me.m_minThreshold);
  m_useAutomaticThreshold = std::move(me.m_useAutomaticThreshold);
  m_mu1 = std::move(me.m_mu1);
  m_mu2 = std::move(me.m_mu2);
  m_min_samplestep = std::move(me.m_min_samplestep);
  m_anglestep = std::move(me.m_anglestep);
  m_mask_size = std::move(me.m_mask_size);
  m_mask_number = std::move(me.m_mask_number);
  m_mask_sign = std::move(me.m_mask_sign);
  m_range = std::move(me.m_range);
  m_sample_step = std::move(me.m_sample_step);
  m_ntotal_sample = std::move(me.m_ntotal_sample);
  m_points_to_track = std::move(me.m_points_to_track);
  m_strip = std::move(me.m_strip);

  initMask();
  return *this;
}
#endif

vpMe::~vpMe()
{
  if (m_mask != nullptr) {
    delete[] m_mask;
    m_mask = nullptr;
  }
}

void vpMe::setMaskNumber(const unsigned int &mask_number)
{
  const unsigned int flatAngle = 180;
  m_mask_number = mask_number;
  m_anglestep = flatAngle / m_mask_number;
  initMask();
}

void vpMe::setMaskSize(const unsigned int &mask_size)
{
  m_mask_size = mask_size;
  initMask();
}

END_VISP_NAMESPACE
