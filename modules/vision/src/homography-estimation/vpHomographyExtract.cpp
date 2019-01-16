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
 * Homography transformation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <math.h>
#include <visp3/core/vpMath.h>
#include <visp3/vision/vpHomography.h>

//#define DEBUG_Homographie 0

/* ---------------------------------------------------------------------- */
/* --- STATIC ------------------------------------------------------------ */
/* ------------------------------------------------------------------------ */
const double vpHomography::sing_threshold = 0.0001;

/*!
  Compute the camera displacement between two images from the homography \f$
  {^a}{\bf H}_b \f$ which is here an implicit parameter (*this).

  \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.

  \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.

  \param n : Normal vector to the plane as an output.

*/
void vpHomography::computeDisplacement(vpRotationMatrix &aRb, vpTranslationVector &atb, vpColVector &n)
{

  vpColVector nd(3);
  nd[0] = 0;
  nd[1] = 0;
  nd[2] = 1;

  computeDisplacement(*this, aRb, atb, n);
}

/*!

  Compute the camera displacement between two images from the homography \f$
  {^a}{\bf H}_b \f$ which is here an implicit parameter (*this).

  Camera displacement between \f$ {^a}{\bf p} \f$ and \f$ {^a}{\bf p} \f$ is
  represented as a rotation matrix \f$ {^a}{\bf R}_b \f$ and a translation
  vector \f$ ^a{\bf t}_b \f$ from which an homogeneous matrix can be build
  (vpHomogeneousMatrix).

  \param nd : Input normal vector to the plane used to compar with the normal
  vector \e n extracted from the homography.

  \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.

  \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.

  \param n : Normal vector to the plane as an output.

*/
void vpHomography::computeDisplacement(const vpColVector &nd, vpRotationMatrix &aRb, vpTranslationVector &atb,
                                       vpColVector &n)
{
  computeDisplacement(*this, nd, aRb, atb, n);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!

  Compute the camera displacement between two images from the homography \f$
  {^a}{\bf H}_b \f$.

  Camera displacement between \f$ {^a}{\bf p} \f$ and \f$ {^b}{\bf p} \f$ is
  represented as a rotation matrix \f$ {^a}{\bf R}_b \f$ and a translation
  vector \f$ ^a{\bf t}_b \f$ from which an homogeneous matrix can be build
  (vpHomogeneousMatrix).

  \param aHb : Input homography \f$ {^a}{\bf H}_b \f$.

  \param nd : Input normal vector to the plane used to compar with the normal
  vector \e n extracted from the homography.

  \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.

  \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.

  \param n : Normal vector to the plane as an output.
*/

void vpHomography::computeDisplacement(const vpHomography &aHb, const vpColVector &nd, vpRotationMatrix &aRb,
                                       vpTranslationVector &atb, vpColVector &n)
{
  /**** Declarations des variables ****/

  vpMatrix aRbint(3, 3);
  vpColVector svTemp(3), sv(3);
  vpMatrix mX(3, 2);
  vpColVector aTbp(3), normaleEstimee(3);
  double distanceFictive;
  double sinusTheta, cosinusTheta, signeSinus = 1;
  double s, determinantU, determinantV;
  unsigned int vOrdre[3];

  // vpColVector normaleDesiree(3) ;
  // normaleDesiree[0]=0;normaleDesiree[1]=0;normaleDesiree[2]=1;
  vpColVector normaleDesiree(nd);

/**** Corps de la focntion ****/
#ifdef DEBUG_Homographie
  printf("debut : Homographie_EstimationDeplacementCamera\n");
#endif

  /* Allocation des matrices */
  vpMatrix mTempU(3, 3);
  vpMatrix mTempV(3, 3);
  vpMatrix mU(3, 3);
  vpMatrix mV(3, 3);
  vpMatrix aRbp(3, 3);

  vpMatrix mH(3, 3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      mH[i][j] = aHb[i][j];

  /* Preparation au calcul de la SVD */
  mTempU = mH;

  /*****
        Remarque : mTempU, svTemp et mTempV sont modifies par svd
        Il est necessaire apres de les trier dans l'ordre decroissant
        des valeurs singulieres
  *****/
  mTempU.svd(svTemp, mTempV);

  /* On va mettre les valeurs singulieres en ordre decroissant : */

  /* Determination de l'ordre des valeurs */
  if (svTemp[0] >= svTemp[1]) {
    if (svTemp[0] >= svTemp[2]) {
      if (svTemp[1] > svTemp[2]) {
        vOrdre[0] = 0;
        vOrdre[1] = 1;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 0;
        vOrdre[1] = 2;
        vOrdre[2] = 1;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 0;
      vOrdre[2] = 1;
    }
  } else {
    if (svTemp[1] >= svTemp[2]) {
      if (svTemp[0] > svTemp[2]) {
        vOrdre[0] = 1;
        vOrdre[1] = 0;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 1;
        vOrdre[1] = 2;
        vOrdre[2] = 0;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 1;
      vOrdre[2] = 0;
    }
  }
  /*****
        Tri decroissant des matrices U, V, sv
        en fonction des valeurs singulieres car
        hypothese : sv[0]>=sv[1]>=sv[2]>=0
  *****/

  for (unsigned int i = 0; i < 3; i++) {
    sv[i] = svTemp[vOrdre[i]];
    for (unsigned int j = 0; j < 3; j++) {
      mU[i][j] = mTempU[i][vOrdre[j]];
      mV[i][j] = mTempV[i][vOrdre[j]];
    }
  }

#ifdef DEBUG_Homographie
  printf("U : \n");
  std::cout << mU << std::endl;
  printf("V : \n");
  std::cout << mV << std::endl;
  printf("Valeurs singulieres : ");
  std::cout << sv.t();
#endif

  /* A verifier si necessaire!!! */
  determinantV = mV.det();
  determinantU = mU.det();

  s = determinantU * determinantV;

#ifdef DEBUG_Homographie
  printf("s = det(U) * det(V) = %f * %f = %f\n", determinantU, determinantV, s);
#endif
  if (s < 0)
    mV *= -1;

  /* d' = d2 */
  distanceFictive = sv[1];
#ifdef DEBUG_Homographie
  printf("d = %f\n", distanceFictive);
#endif
  n.resize(3);

  if (((sv[0] - sv[1]) < sing_threshold) && (sv[0] - sv[2]) < sing_threshold) {
    //#ifdef DEBUG_Homographie
    //   printf ("\nPure  rotation\n");
    //#endif
    /*****
          Cas ou le deplacement est une rotation pure
          et la normale reste indefini
          sv[0] = sv[1]= sv[2]
    *****/
    aTbp[0] = 0;
    aTbp[1] = 0;
    aTbp[2] = 0;

    n[0] = normaleDesiree[0];
    n[1] = normaleDesiree[1];
    n[2] = normaleDesiree[2];
  } else {
#ifdef DEBUG_Homographie
    printf("\nCas general\n");
#endif
    /* Cas general */

    /*****
          test pour determiner quelle est la bonne solution on teste
          d'abord quelle solution est plus proche de la perpendiculaire
          au plan de la cible constuction de la normale n
    *****/

    /*****
          Calcul de la normale au plan : n' = [ esp1*x1 , x2=0 , esp3*x3 ]
          dans l'ordre : cas (esp1=+1, esp3=+1) et (esp1=-1, esp3=+1)
    *****/
    mX[0][0] = sqrt((sv[0] * sv[0] - sv[1] * sv[1]) / (sv[0] * sv[0] - sv[2] * sv[2]));
    mX[1][0] = 0.0;
    mX[2][0] = sqrt((sv[1] * sv[1] - sv[2] * sv[2]) / (sv[0] * sv[0] - sv[2] * sv[2]));

    mX[0][1] = -mX[0][0];
    mX[1][1] = mX[1][0];
    mX[2][1] = mX[2][0];

    /* Il y a 4 solutions pour n : 2 par cas => n1, -n1, n2, -n2 */
    double cosinusAncien = 0.0;
    for (unsigned int w = 0; w < 2; w++) {   /* Pour les 2 cas */
      for (unsigned int k = 0; k < 2; k++) { /* Pour le signe */

        /* Calcul de la normale estimee : n = V.n' */
        for (unsigned int i = 0; i < 3; i++) {
          normaleEstimee[i] = 0.0;
          for (unsigned int j = 0; j < 3; j++) {
            normaleEstimee[i] += (2.0 * k - 1.0) * mV[i][j] * mX[j][w];
          }
        }

        /* Calcul du cosinus de l'angle entre la normale reelle et desire */
        double cosinusDesireeEstimee = 0.0;
        for (unsigned int i = 0; i < 3; i++)
          cosinusDesireeEstimee += normaleEstimee[i] * normaleDesiree[i];

        /*****
              Si la solution est meilleur
              Remarque : On ne teste pas le cas oppose (cos<0)
        *****/
        if (cosinusDesireeEstimee > cosinusAncien) {
          cosinusAncien = cosinusDesireeEstimee;

          /* Affectation de la normale qui est retourner */
          for (unsigned int j = 0; j < 3; j++)
            n[j] = normaleEstimee[j];

          /* Construction du vecteur t'= +/- (d1-d3).[x1, 0, -x3] */
          aTbp[0] = (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[0][w];
          aTbp[1] = (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[1][w];
          aTbp[2] = -(2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[2][w];

          /* si c'est la deuxieme solution */
          if (w == 1)
            signeSinus = -1; /* car esp1*esp3 = -1 */
          else
            signeSinus = 1;
        } /* fin if (cosinusDesireeEstimee > cosinusAncien) */
      }   /* fin k */
    }     /* fin w */
  }       /* fin else */

  /* Calcul du vecteur de translation qui est retourner : t = (U * t') / d */
  for (unsigned int i = 0; i < 3; i++) {
    atb[i] = 0.0;
    for (unsigned int j = 0; j < 3; j++) {
      atb[i] += mU[i][j] * aTbp[j];
    }
    atb[i] /= distanceFictive;
  }

#ifdef DEBUG_Homographie
  printf("t' : ");
  std::cout << aTbp.t();
  printf("t/d : ");
  std::cout << atb.t();
  printf("n : ");
  std::cout << n.t();
#endif

  /* Calcul de la matrice de rotation R */

  /*****
        Calcul du sinus(theta) et du cosinus(theta)
        Remarque : sinus(theta) pourra changer de signe en fonction
        de la solution retenue (cf. ci-dessous)
  *****/
  sinusTheta =
      signeSinus * sqrt((sv[0] * sv[0] - sv[1] * sv[1]) * (sv[1] * sv[1] - sv[2] * sv[2])) / ((sv[0] + sv[2]) * sv[1]);

  cosinusTheta = (sv[1] * sv[1] + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

  /* construction de la matrice de rotation R' */
  aRbp[0][0] = cosinusTheta;
  aRbp[0][1] = 0;
  aRbp[0][2] = -sinusTheta;
  aRbp[1][0] = 0;
  aRbp[1][1] = 1;
  aRbp[1][2] = 0;
  aRbp[2][0] = sinusTheta;
  aRbp[2][1] = 0;
  aRbp[2][2] = cosinusTheta;

  /* multiplication Rint = U R' */
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      aRbint[i][j] = 0.0;
      for (unsigned int k = 0; k < 3; k++) {
        aRbint[i][j] += mU[i][k] * aRbp[k][j];
      }
    }
  }

  /* multiplication R = Rint . V^T */
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      aRb[i][j] = 0.0;
      for (unsigned int k = 0; k < 3; k++) {
        aRb[i][j] += aRbint[i][k] * mV[j][k];
      }
    }
  }
/*transpose_carre(aRb,3); */
#ifdef DEBUG_Homographie
  printf("R : %d\n", aRb.isARotationMatrix());
  std::cout << aRb << std::endl;
#endif
}

/*!

  Compute the camera displacement between two images from the homography \f$
  {^a}{\bf H}_b \f$.

  Camera displacement between \f$ {^a}{\bf p} \f$ and \f$ {^a}{\bf p} \f$ is
  represented as a rotation matrix \f$ {^a}{\bf R}_b \f$ and a translation
  vector \f$ ^a{\bf t}_b \f$ from which an homogeneous matrix can be build
  (vpHomogeneousMatrix).

  \param aHb : Input homography \f$ {^a}{\bf H}_b \f$.

  \param aRb : Rotation matrix as an output \f$ {^a}{\bf R}_b \f$.

  \param atb : Translation vector as an output \f$ ^a{\bf t}_b \f$.

  \param n : Normal vector to the plane as an output.
*/
void vpHomography::computeDisplacement(const vpHomography &aHb, vpRotationMatrix &aRb, vpTranslationVector &atb,
                                       vpColVector &n)
{
  /**** Declarations des variables ****/

  vpMatrix aRbint(3, 3);
  vpColVector svTemp(3), sv(3);
  vpMatrix mX(3, 2);
  vpColVector aTbp(3), normaleEstimee(3);
  double distanceFictive;
  double sinusTheta, cosinusTheta, signeSinus = 1;
  double s, determinantU, determinantV;
  unsigned int vOrdre[3];

  vpColVector normaleDesiree(3);
  normaleDesiree[0] = 0;
  normaleDesiree[1] = 0;
  normaleDesiree[2] = 1;

/**** Corps de la focntion ****/
#ifdef DEBUG_Homographie
  printf("debut : Homographie_EstimationDeplacementCamera\n");
#endif

  /* Allocation des matrices */
  vpMatrix mTempU(3, 3);
  vpMatrix mTempV(3, 3);
  vpMatrix mU(3, 3);
  vpMatrix mV(3, 3);
  vpMatrix aRbp(3, 3);

  vpMatrix mH(3, 3);
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      mH[i][j] = aHb[i][j];

  /* Preparation au calcul de la SVD */
  mTempU = mH;

  /*****
        Remarque : mTempU, svTemp et mTempV sont modifies par svd
        Il est necessaire apres de les trier dans l'ordre decroissant
        des valeurs singulieres
  *****/
  mTempU.svd(svTemp, mTempV);

  /* On va mettre les valeurs singulieres en ordre decroissant : */

  /* Determination de l'ordre des valeurs */
  if (svTemp[0] >= svTemp[1]) {
    if (svTemp[0] >= svTemp[2]) {
      if (svTemp[1] > svTemp[2]) {
        vOrdre[0] = 0;
        vOrdre[1] = 1;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 0;
        vOrdre[1] = 2;
        vOrdre[2] = 1;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 0;
      vOrdre[2] = 1;
    }
  } else {
    if (svTemp[1] >= svTemp[2]) {
      if (svTemp[0] > svTemp[2]) {
        vOrdre[0] = 1;
        vOrdre[1] = 0;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 1;
        vOrdre[1] = 2;
        vOrdre[2] = 0;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 1;
      vOrdre[2] = 0;
    }
  }
  /*****
        Tri decroissant des matrices U, V, sv
        en fonction des valeurs singulieres car
        hypothese : sv[0]>=sv[1]>=sv[2]>=0
  *****/

  for (unsigned int i = 0; i < 3; i++) {
    sv[i] = svTemp[vOrdre[i]];
    for (unsigned int j = 0; j < 3; j++) {
      mU[i][j] = mTempU[i][vOrdre[j]];
      mV[i][j] = mTempV[i][vOrdre[j]];
    }
  }

#ifdef DEBUG_Homographie
  printf("U : \n");
  std::cout << mU << std::endl;
  printf("V : \n");
  std::cout << mV << std::endl;
  printf("Valeurs singulieres : ");
  std::cout << sv.t();
#endif

  /* A verifier si necessaire!!! */
  determinantV = mV.det();
  determinantU = mU.det();

  s = determinantU * determinantV;

#ifdef DEBUG_Homographie
  printf("s = det(U) * det(V) = %f * %f = %f\n", determinantU, determinantV, s);
#endif
  if (s < 0)
    mV *= -1;

  /* d' = d2 */
  distanceFictive = sv[1];
#ifdef DEBUG_Homographie
  printf("d = %f\n", distanceFictive);
#endif
  n.resize(3);

  if (((sv[0] - sv[1]) < sing_threshold) && (sv[0] - sv[2]) < sing_threshold) {
    //#ifdef DEBUG_Homographie
    //   printf ("\nPure  rotation\n");
    //#endif
    /*****
          Cas ou le deplacement est une rotation pure
          et la normale reste indefini
          sv[0] = sv[1]= sv[2]
    *****/
    aTbp[0] = 0;
    aTbp[1] = 0;
    aTbp[2] = 0;

    n[0] = normaleDesiree[0];
    n[1] = normaleDesiree[1];
    n[2] = normaleDesiree[2];
  } else {
#ifdef DEBUG_Homographie
    printf("\nCas general\n");
#endif
    /* Cas general */

    /*****
          test pour determiner quelle est la bonne solution on teste
          d'abord quelle solution est plus proche de la perpendiculaire
          au plan de la cible constuction de la normale n
    *****/

    /*****
          Calcul de la normale au plan : n' = [ esp1*x1 , x2=0 , esp3*x3 ]
          dans l'ordre : cas (esp1=+1, esp3=+1) et (esp1=-1, esp3=+1)
    *****/
    mX[0][0] = sqrt((sv[0] * sv[0] - sv[1] * sv[1]) / (sv[0] * sv[0] - sv[2] * sv[2]));
    mX[1][0] = 0.0;
    mX[2][0] = sqrt((sv[1] * sv[1] - sv[2] * sv[2]) / (sv[0] * sv[0] - sv[2] * sv[2]));

    mX[0][1] = -mX[0][0];
    mX[1][1] = mX[1][0];
    mX[2][1] = mX[2][0];

    /* Il y a 4 solutions pour n : 2 par cas => n1, -n1, n2, -n2 */
    double cosinusAncien = 0.0;
    for (unsigned int w = 0; w < 2; w++) {   /* Pour les 2 cas */
      for (unsigned int k = 0; k < 2; k++) { /* Pour le signe */

        /* Calcul de la normale estimee : n = V.n' */
        for (unsigned int i = 0; i < 3; i++) {
          normaleEstimee[i] = 0.0;
          for (unsigned int j = 0; j < 3; j++) {
            normaleEstimee[i] += (2.0 * k - 1.0) * mV[i][j] * mX[j][w];
          }
        }

        /* Calcul du cosinus de l'angle entre la normale reelle et desire */
        double cosinusDesireeEstimee = 0.0;
        for (unsigned int i = 0; i < 3; i++)
          cosinusDesireeEstimee += normaleEstimee[i] * normaleDesiree[i];

        /*****
              Si la solution est meilleur
              Remarque : On ne teste pas le cas oppose (cos<0)
        *****/
        if (cosinusDesireeEstimee > cosinusAncien) {
          cosinusAncien = cosinusDesireeEstimee;

          /* Affectation de la normale qui est retourner */
          for (unsigned int j = 0; j < 3; j++)
            n[j] = normaleEstimee[j];

          /* Construction du vecteur t'= +/- (d1-d3).[x1, 0, -x3] */
          aTbp[0] = (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[0][w];
          aTbp[1] = (2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[1][w];
          aTbp[2] = -(2.0 * k - 1.0) * (sv[0] - sv[2]) * mX[2][w];

          /* si c'est la deuxieme solution */
          if (w == 1)
            signeSinus = -1; /* car esp1*esp3 = -1 */
          else
            signeSinus = 1;
        } /* fin if (cosinusDesireeEstimee > cosinusAncien) */
      }   /* fin k */
    }     /* fin w */
  }       /* fin else */

  /* Calcul du vecteur de translation qui est retourner : t = (U * t') / d */
  for (unsigned int i = 0; i < 3; i++) {
    atb[i] = 0.0;
    for (unsigned int j = 0; j < 3; j++) {
      atb[i] += mU[i][j] * aTbp[j];
    }
    atb[i] /= distanceFictive;
  }

#ifdef DEBUG_Homographie
  printf("t' : ");
  std::cout << aTbp.t();
  printf("t/d : ");
  std::cout << atb.t();
  printf("n : ");
  std::cout << n.t();
#endif

  /* Calcul de la matrice de rotation R */

  /*****
        Calcul du sinus(theta) et du cosinus(theta)
        Remarque : sinus(theta) pourra changer de signe en fonction
        de la solution retenue (cf. ci-dessous)
  *****/
  sinusTheta =
      signeSinus * sqrt((sv[0] * sv[0] - sv[1] * sv[1]) * (sv[1] * sv[1] - sv[2] * sv[2])) / ((sv[0] + sv[2]) * sv[1]);

  cosinusTheta = (sv[1] * sv[1] + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

  /* construction de la matrice de rotation R' */
  aRbp[0][0] = cosinusTheta;
  aRbp[0][1] = 0;
  aRbp[0][2] = -sinusTheta;
  aRbp[1][0] = 0;
  aRbp[1][1] = 1;
  aRbp[1][2] = 0;
  aRbp[2][0] = sinusTheta;
  aRbp[2][1] = 0;
  aRbp[2][2] = cosinusTheta;

  /* multiplication Rint = U R' */
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      aRbint[i][j] = 0.0;
      for (unsigned int k = 0; k < 3; k++) {
        aRbint[i][j] += mU[i][k] * aRbp[k][j];
      }
    }
  }

  /* multiplication R = Rint . V^T */
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      aRb[i][j] = 0.0;
      for (unsigned int k = 0; k < 3; k++) {
        aRb[i][j] += aRbint[i][k] * mV[j][k];
      }
    }
  }
/*transpose_carre(aRb,3); */
#ifdef DEBUG_Homographie
  printf("R : %d\n", aRb.isARotationMatrix());
  std::cout << aRb << std::endl;
#endif
}

void vpHomography::computeDisplacement(const vpHomography &H, const double x, const double y,
                                       std::list<vpRotationMatrix> &vR, std::list<vpTranslationVector> &vT,
                                       std::list<vpColVector> &vN)
{

#ifdef DEBUG_Homographie
  printf("debut : Homographie_EstimationDeplacementCamera\n");
#endif

  vR.clear();
  vT.clear();
  vN.clear();

  /**** Declarations des variables ****/
  int cas1 = 1, cas2 = 2, cas3 = 3, cas4 = 4;
  int cas = 0;
  bool norm1ok = false, norm2ok = false, norm3ok = false, norm4ok = false;

  double tmp, determinantU, determinantV, s, distanceFictive;
  vpMatrix mTempU(3, 3), mTempV(3, 3), U(3, 3), V(3, 3);

  vpRotationMatrix Rprim1, R1;
  vpRotationMatrix Rprim2, R2;
  vpRotationMatrix Rprim3, R3;
  vpRotationMatrix Rprim4, R4;
  vpTranslationVector Tprim1, T1;
  vpTranslationVector Tprim2, T2;
  vpTranslationVector Tprim3, T3;
  vpTranslationVector Tprim4, T4;
  vpColVector Nprim1(3), N1(3);
  vpColVector Nprim2(3), N2(3);
  vpColVector Nprim3(3), N3(3);
  vpColVector Nprim4(3), N4(3);

  vpColVector svTemp(3), sv(3);
  unsigned int vOrdre[3];
  vpColVector vTp(3);

  /* Preparation au calcul de la SVD */
  mTempU = H.convert();

  /*****
        Remarque : mTempU, svTemp et mTempV sont modifies par svd
        Il est necessaire apres de les trier dans l'ordre decroissant
        des valeurs singulieres
  *****/

  // cette fonction ne renvoit pas d'erreur
  mTempU.svd(svTemp, mTempV);

  /* On va mettre les valeurs singulieres en ordre decroissant : */
  /* Determination de l'ordre des valeurs */
  if (svTemp[0] >= svTemp[1]) {
    if (svTemp[0] >= svTemp[2]) {
      if (svTemp[1] > svTemp[2]) {
        vOrdre[0] = 0;
        vOrdre[1] = 1;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 0;
        vOrdre[1] = 2;
        vOrdre[2] = 1;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 0;
      vOrdre[2] = 1;
    }
  } else {
    if (svTemp[1] >= svTemp[2]) {
      if (svTemp[0] > svTemp[2]) {
        vOrdre[0] = 1;
        vOrdre[1] = 0;
        vOrdre[2] = 2;
      } else {
        vOrdre[0] = 1;
        vOrdre[1] = 2;
        vOrdre[2] = 0;
      }
    } else {
      vOrdre[0] = 2;
      vOrdre[1] = 1;
      vOrdre[2] = 0;
    }
  }
  /*****
        Tri decroissant des matrices U, V, sv
        en fonction des valeurs singulieres car
        hypothese : sv[0]>=sv[1]>=sv[2]>=0
  *****/
  for (unsigned int i = 0; i < 3; i++) {
    sv[i] = svTemp[vOrdre[i]];
    for (unsigned int j = 0; j < 3; j++) {
      U[i][j] = mTempU[i][vOrdre[j]];
      V[i][j] = mTempV[i][vOrdre[j]];
    }
  }

#ifdef DEBUG_Homographie
  printf("U : \n");
  Affiche(U);
  printf("V : \n");
  affiche(V);
  printf("Valeurs singulieres : ");
  affiche(sv);
#endif

  // calcul du determinant de U et V
  determinantU = U[0][0] * (U[1][1] * U[2][2] - U[1][2] * U[2][1]) + U[0][1] * (U[1][2] * U[2][0] - U[1][0] * U[2][2]) +
                 U[0][2] * (U[1][0] * U[2][1] - U[1][1] * U[2][0]);

  determinantV = V[0][0] * (V[1][1] * V[2][2] - V[1][2] * V[2][1]) + V[0][1] * (V[1][2] * V[2][0] - V[1][0] * V[2][2]) +
                 V[0][2] * (V[1][0] * V[2][1] - V[1][1] * V[2][0]);

  s = determinantU * determinantV;

#ifdef DEBUG_Homographie
  printf("s = det(U) * det(V) = %f * %f = %f\n", determinantU, determinantV, s);
#endif

  // deux cas sont a traiter :
  // distance Fictive = sv[1]
  // distance fictive = -sv[1]

  // pour savoir quelle est le bon signe,
  // on utilise le point qui appartient au plan
  // la contrainte est :
  // (h_31x_1 + h_32x_2 + h_33)/d > 0
  // et d = sd'

  tmp = H[2][0] * x + H[2][1] * y + H[2][2];

  if ((tmp / (sv[1] * s)) > 0)
    distanceFictive = sv[1];
  else
    distanceFictive = -sv[1];

  // il faut ensuite considerer l'ordre de multiplicite de chaque variable
  // 1er cas : d1<>d2<>d3
  // 2eme cas : d1=d2 <> d3
  // 3eme cas : d1 <>d2 =d3
  // 4eme cas : d1 =d2=d3

  if ((sv[0] - sv[1]) < sing_threshold) {
    if ((sv[1] - sv[2]) < sing_threshold)
      cas = cas4;
    else
      cas = cas2;
  } else {
    if ((sv[1] - sv[2]) < sing_threshold)
      cas = cas3;
    else
      cas = cas1;
  }

  Nprim1 = 0.0;
  Nprim2 = 0.0;
  Nprim3 = 0.0;
  Nprim4 = 0.0;

  // on filtre ensuite les diff'erentes normales possibles
  // condition : nm/d > 0
  // avec d = sd'
  // et n = Vn'

  // les quatres cas sont : ++, +-, -+ , --
  // dans tous les cas, Normale[1] = 0;
  Nprim1[1] = 0.0;
  Nprim2[1] = 0.0;
  Nprim3[1] = 0.0;
  Nprim4[1] = 0.0;

  // on calcule les quatres cas de normale

  if (cas == cas1) {
    // quatre normales sont possibles
    Nprim1[0] = sqrt((sv[0] * sv[0] - sv[1] * sv[1]) / (sv[0] * sv[0] - sv[2] * sv[2]));

    Nprim1[2] = sqrt((sv[1] * sv[1] - sv[2] * sv[2]) / (sv[0] * sv[0] - sv[2] * sv[2]));

    Nprim2[0] = Nprim1[0];
    Nprim2[2] = -Nprim1[2];
    Nprim3[0] = -Nprim1[0];
    Nprim3[2] = Nprim1[2];
    Nprim4[0] = -Nprim1[0];
    Nprim4[2] = -Nprim1[2];
  }
  if (cas == cas2) {
    // 2 normales sont possibles
    // x3 = +-1
    Nprim1[2] = 1;
    Nprim2[2] = -1;
  }

  if (cas == cas3) {
    // 2 normales sont possibles
    // x1 = +-1
    Nprim1[0] = 1;
    Nprim2[0] = -1;
  }
  // si on est dans le cas 4,
  // on considere que x reste indefini

  // on peut maintenant filtrer les solutions avec la contrainte
  // n^tm / d > 0
  // attention, il faut travailler avec la bonne normale,
  // soit Ni et non pas Nprimi
  if (cas == cas1) {
    N1 = V * Nprim1;

    tmp = N1[0] * x + N1[1] * y + N1[2];
    tmp /= (distanceFictive * s);
    norm1ok = (tmp > 0);

    N2 = V * Nprim2;
    tmp = N2[0] * x + N2[1] * y + N2[2];
    tmp /= (distanceFictive * s);
    norm2ok = (tmp > 0);

    N3 = V * Nprim3;
    tmp = N3[0] * x + N3[1] * y + N3[2];
    tmp /= (distanceFictive * s);
    norm3ok = (tmp > 0);

    N4 = V * Nprim4;
    tmp = N4[0] * x + N4[1] * y + N4[2];
    tmp /= (distanceFictive * s);
    norm4ok = (tmp > 0);
  }

  if (cas == cas2) {
    N1 = V * Nprim1;
    tmp = N1[0] * x + N1[1] * y + N1[2];
    tmp /= (distanceFictive * s);
    norm1ok = (tmp > 0);

    N2 = V * Nprim2;
    tmp = N2[0] * x + N2[1] * y + N2[2];
    tmp /= (distanceFictive * s);
    norm2ok = (tmp > 0);
  }

  if (cas == cas3) {
    N1 = V * Nprim1;
    tmp = N1[0] * x + N1[1] * y + N1[2];
    tmp /= (distanceFictive * s);
    norm1ok = (tmp > 0);

    N2 = V * Nprim2;
    tmp = N2[0] * x + N2[1] * y + N2[2];
    tmp /= (distanceFictive * s);
    norm2ok = (tmp > 0);
  }

  // on a donc maintenant les differents cas possibles
  // on peut ensuite remonter aux deplacements
  // on separe encore les cas 1,2,3
  // la, deux choix se posent suivant le signe de d
  if (distanceFictive > 0) {
    if (cas == cas1) {
      if (norm1ok) {

        // cos theta
        Rprim1[0][0] = (vpMath::sqr(sv[1]) + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

        Rprim1[2][2] = Rprim1[0][0];

        // sin theta
        Rprim1[2][0] = (sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] + sv[2]) * sv[1]);

        Rprim1[0][2] = -Rprim1[2][0];

        Rprim1[1][1] = 1.0;

        Tprim1[0] = Nprim1[0];
        Tprim1[1] = 0.0;
        Tprim1[2] = -Nprim1[2];

        Tprim1 *= (sv[0] - sv[2]);
      }

      if (norm2ok) {

        // cos theta
        Rprim2[0][0] = (vpMath::sqr(sv[1]) + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

        Rprim2[2][2] = Rprim2[0][0];

        // sin theta
        Rprim2[2][0] = -(sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] + sv[2]) * sv[1]);

        Rprim2[0][2] = -Rprim2[2][0];

        Rprim2[1][1] = 1.0;

        Tprim2[0] = Nprim2[0];
        Tprim2[1] = 0.0;
        Tprim2[2] = -Nprim2[2];

        Tprim2 *= (sv[0] - sv[2]);
      }

      if (norm3ok) {

        // cos theta
        Rprim3[0][0] = (vpMath::sqr(sv[1]) + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

        Rprim3[2][2] = Rprim3[0][0];

        // sin theta
        Rprim3[2][0] = -(sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] + sv[2]) * sv[1]);

        Rprim3[0][2] = -Rprim3[2][0];

        Rprim3[1][1] = 1.0;

        Tprim3[0] = Nprim3[0];
        Tprim3[1] = 0.0;
        Tprim3[2] = -Nprim3[2];

        Tprim3 *= (sv[0] - sv[2]);
      }

      if (norm4ok) {

        // cos theta
        Rprim4[0][0] = (vpMath::sqr(sv[1]) + sv[0] * sv[2]) / ((sv[0] + sv[2]) * sv[1]);

        Rprim4[2][2] = Rprim4[0][0];

        // sin theta
        Rprim4[2][0] = (sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] + sv[2]) * sv[1]);

        Rprim4[0][2] = -Rprim4[2][0];

        Rprim4[1][1] = 1.0;

        Tprim4[0] = Nprim4[0];
        Tprim4[1] = 0.0;
        Tprim4[2] = -Nprim4[2];

        Tprim4 *= (sv[0] - sv[2]);
      }
    }

    if (cas == cas2) {
      // 2 normales sont potentiellement candidates

      if (norm1ok) {
        Rprim1.eye();

        Tprim1 = Nprim1[0];
        Tprim1 *= (sv[2] - sv[0]);
      }

      if (norm2ok) {
        Rprim2.eye();

        Tprim2 = Nprim2[1];
        Tprim2 *= (sv[2] - sv[0]);
      }
    }
    if (cas == cas3) {
      if (norm1ok) {
        Rprim1.eye();

        Tprim1 = Nprim1[0];
        Tprim1 *= (sv[0] - sv[1]);
      }

      if (norm2ok) {
        Rprim2.eye();

        Tprim2 = Nprim2[1];
        Tprim2 *= (sv[0] - sv[1]);
      }
    }
    if (cas == cas4) {
      // on ne connait pas la normale dans ce cas la
      Rprim1.eye();
      Tprim1 = 0.0;
    }
  }

  if (distanceFictive < 0) {

    if (cas == cas1) {
      if (norm1ok) {

        // cos theta
        Rprim1[0][0] = (sv[0] * sv[2] - vpMath::sqr(sv[1])) / ((sv[0] - sv[2]) * sv[1]);

        Rprim1[2][2] = -Rprim1[0][0];

        // sin theta
        Rprim1[2][0] = (sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] - sv[2]) * sv[1]);

        Rprim1[0][2] = Rprim1[2][0];

        Rprim1[1][1] = -1.0;

        Tprim1[0] = Nprim1[0];
        Tprim1[1] = 0.0;
        Tprim1[2] = Nprim1[2];

        Tprim1 *= (sv[0] + sv[2]);
      }

      if (norm2ok) {

        // cos theta
        Rprim2[0][0] = (sv[0] * sv[2] - vpMath::sqr(sv[1])) / ((sv[0] - sv[2]) * sv[1]);

        Rprim2[2][2] = -Rprim2[0][0];

        // sin theta
        Rprim2[2][0] = -(sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] - sv[2]) * sv[1]);

        Rprim2[0][2] = Rprim2[2][0];

        Rprim2[1][1] = -1.0;

        Tprim2[0] = Nprim2[0];
        Tprim2[1] = 0.0;
        Tprim2[2] = Nprim2[2];

        Tprim2 *= (sv[0] + sv[2]);
      }

      if (norm3ok) {

        // cos theta
        Rprim3[0][0] = (sv[0] * sv[2] - vpMath::sqr(sv[1])) / ((sv[0] - sv[2]) * sv[1]);

        Rprim3[2][2] = -Rprim3[0][0];

        // sin theta
        Rprim3[2][0] = -(sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] - sv[2]) * sv[1]);

        Rprim3[0][2] = Rprim3[2][0];

        Rprim3[1][1] = -1.0;

        Tprim3[0] = Nprim3[0];
        Tprim3[1] = 0.0;
        Tprim3[2] = Nprim3[2];

        Tprim3 *= (sv[0] + sv[2]);
      }

      if (norm4ok) {
        // cos theta
        Rprim4[0][0] = (sv[0] * sv[2] - vpMath::sqr(sv[1])) / ((sv[0] - sv[2]) * sv[1]);

        Rprim4[2][2] = -Rprim4[0][0];

        // sin theta
        Rprim4[2][0] = (sqrt((vpMath::sqr(sv[0]) - vpMath::sqr(sv[1])) * (vpMath::sqr(sv[1]) - vpMath::sqr(sv[2])))) /
                       ((sv[0] - sv[2]) * sv[1]);

        Rprim4[0][2] = Rprim4[2][0];

        Rprim4[1][1] = -1.0;

        Tprim4[0] = Nprim4[0];
        Tprim4[1] = 0.0;
        Tprim4[2] = Nprim4[2];

        Tprim4 *= (sv[0] + sv[2]);
      }
    }
    if (cas == cas2) {
      // 2 normales sont potentiellement candidates

      if (norm1ok) {
        Rprim1.eye();
        Rprim1[0][0] = -1;
        Rprim1[1][1] = -1;

        Tprim1 = Nprim1[0];
        Tprim1 *= (sv[2] + sv[0]);
      }

      if (norm2ok) {
        Rprim2.eye();
        Rprim2[0][0] = -1;
        Rprim2[1][1] = -1;

        Tprim2 = Nprim2[1];
        Tprim2 *= (sv[2] + sv[0]);
      }
    }
    if (cas == cas3) {
      if (norm1ok) {
        Rprim1.eye();
        Rprim1[2][2] = -1;
        Rprim1[1][1] = -1;

        Tprim1 = Nprim1[0];
        Tprim1 *= (sv[2] + sv[0]);
      }

      if (norm2ok) {
        Rprim2.eye();
        Rprim2[2][2] = -1;
        Rprim2[1][1] = -1;

        Tprim2 = Nprim2[1];
        Tprim2 *= (sv[0] + sv[2]);
      }
    }

    // ON NE CONSIDERE PAS LE CAS NUMERO 4
  }
  // tous les Rprim et Tprim sont calcules
  // on peut maintenant recuperer la
  // rotation, et la translation
  // IL Y A JUSTE LE CAS D<0 ET CAS 4 QU'ON NE TRAITE PAS
  if ((distanceFictive > 0) || (cas != cas4)) {
    // on controle juste si les normales sont ok

    if (norm1ok) {
      R1 = s * U * Rprim1 * V.t();
      T1 = U * Tprim1;
      T1 /= (distanceFictive * s);
      N1 = V * Nprim1;

      // je rajoute le resultat
      vR.push_back(R1);
      vT.push_back(T1);
      vN.push_back(N1);
    }
    if (norm2ok) {
      R2 = s * U * Rprim2 * V.t();
      T2 = U * Tprim2;
      T2 /= (distanceFictive * s);
      N2 = V * Nprim2;

      // je rajoute le resultat
      vR.push_back(R2);
      vT.push_back(T2);
      vN.push_back(N2);
    }
    if (norm3ok) {
      R3 = s * U * Rprim3 * V.t();
      T3 = U * Tprim3;
      T3 /= (distanceFictive * s);
      N3 = V * Nprim3;
      // je rajoute le resultat
      vR.push_back(R3);
      vT.push_back(T3);
      vN.push_back(N3);
    }
    if (norm4ok) {
      R4 = s * U * Rprim4 * V.t();
      T4 = U * Tprim4;
      T4 /= (distanceFictive * s);
      N4 = V * Nprim4;

      // je rajoute le resultat
      vR.push_back(R4);
      vT.push_back(T4);
      vN.push_back(N4);
    }
  } else {
    std::cout << "On tombe dans le cas particulier ou le mouvement n'est pas "
                 "estimable!"
              << std::endl;
  }

// on peut ensuite afficher les resultats...
/* std::cout << "Analyse des resultats : "<< std::endl; */
/* if (cas==cas1) */
/* std::cout << "On est dans le cas 1" << std::endl; */
/* if (cas==cas2) */
/* std::cout << "On est dans le cas 2" << std::endl; */
/* if (cas==cas3) */
/* std::cout << "On est dans le cas 3" << std::endl; */
/* if (cas==cas4) */
/* std::cout << "On est dans le cas 4" << std::endl; */

/* if (distanceFictive < 0) */
/* std::cout << "d'<0" << std::endl; */
/* else */
/* std::cout << "d'>0" << std::endl; */

#ifdef DEBUG_Homographie
  printf("fin : Homographie_EstimationDeplacementCamera\n");
#endif
}
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS
