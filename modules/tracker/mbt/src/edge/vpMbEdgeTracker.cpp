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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpMbEdgeTracker.cpp
  \brief Make the complete tracking of an object by using its CAD model.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPolygon3D.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbtDistanceLine.h>
#include <visp3/mbt/vpMbtXmlParser.h>
#include <visp3/vision/vpPose.h>

#include <float.h>
#include <limits>
#include <map>
#include <sstream>
#include <string>

/*!
  Basic constructor
*/
vpMbEdgeTracker::vpMbEdgeTracker()
  : me(), lines(1), circles(1), cylinders(1), nline(0), ncircle(0), ncylinder(0), nbvisiblepolygone(0),
    percentageGdPt(0.4), scales(1), Ipyramid(0), scaleLevel(0), nbFeaturesForProjErrorComputation(0), m_factor(),
    m_robustLines(), m_robustCylinders(), m_robustCircles(), m_wLines(), m_wCylinders(), m_wCircles(), m_errorLines(),
    m_errorCylinders(), m_errorCircles(), m_L_edge(), m_error_edge(), m_w_edge(), m_weightedError_edge(),
    m_robust_edge(), m_featuresToBeDisplayedEdge()
{
  scales[0] = true;

#ifdef VISP_HAVE_OGRE
  faces.getOgreContext()->setWindowName("MBT Edge");
#endif
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbEdgeTracker::~vpMbEdgeTracker()
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL) {
          delete l;
        }
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        cy = *it;
        if (cy != NULL) {
          delete cy;
        }
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL) {
          delete ci;
        }
        ci = NULL;
      }

      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  cleanPyramid(Ipyramid);
}

/*!
  Set the moving edge parameters.

  \param p_me : an instance of vpMe containing all the desired parameters
*/
void vpMbEdgeTracker::setMovingEdge(const vpMe &p_me)
{
  this->me = p_me;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        vpMbtDistanceLine *l = *it;
        l->setMovingEdge(&(this->me));
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        vpMbtDistanceCylinder *cy = *it;
        cy->setMovingEdge(&(this->me));
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        vpMbtDistanceCircle *ci = *it;
        ci->setMovingEdge(&(this->me));
      }
    }
  }
}

/*!
  Compute the visual servoing loop to get the pose of the feature set.

  \exception vpTrackingException::notEnoughPointError if the number of
  detected feature is equal to zero.

  \param _I : The current image.
  \param lvl : The level in the pyramid scale.
 */
void vpMbEdgeTracker::computeVVS(const vpImage<unsigned char> &_I, const unsigned int lvl)
{
  double residu_1 = 1e3;
  double r = 1e3 - 1;

  unsigned int iter = 0;

  computeVVSInit();
  unsigned int nbrow = m_error_edge.getRows();

  bool reloop = true;

  bool isoJoIdentity_ = isoJoIdentity; // Backup since it can be modified if L is not full rank
  if (isoJoIdentity_)
    oJo.eye();

  /*** First phase ***/

  while (reloop == true && iter < 10) {
    double count = 0;

    computeVVSFirstPhase(_I, iter, count, lvl);

    count = count / (double)nbrow;
    if (count >= 0.85) {
      reloop = false;
    }

    computeVVSFirstPhasePoseEstimation(iter, isoJoIdentity_);

    iter++;
  }

  //   std::cout << "\t First minimization in " << iter << " iteration give as
  //   initial cMo: \n" << cMo << std::endl;

  /*** Second phase ***/
  vpHomogeneousMatrix cMoPrev;
  vpColVector W_true(nbrow);
  vpMatrix L_true;
  vpMatrix LVJ_true;

  double mu = m_initialMu;
  vpColVector m_error_prev;
  vpColVector m_w_prev;

  // To avoid to create these matrices each iteration
  vpMatrix LTL;
  vpColVector LTR;
  vpColVector v;

  iter = 0;
  m_w_edge = 1;

  // while ( ((int)((residu_1 - r)*1e8) !=0 )  && (iter<30))
  while (std::fabs((residu_1 - r) * 1e8) > std::numeric_limits<double>::epsilon() && (iter < m_maxIter)) {
    computeVVSInteractionMatrixAndResidu(_I);

    bool reStartFromLastIncrement = false;
    computeVVSCheckLevenbergMarquardt(iter, m_error_edge, m_error_prev, cMoPrev, mu, reStartFromLastIncrement,
                                      &m_w_edge, &m_w_prev);

    if (!reStartFromLastIncrement) {
      computeVVSWeights();

      L_true = m_L_edge;
      vpVelocityTwistMatrix cVo;

      if (computeCovariance) {
        L_true = m_L_edge;
        if (!isoJoIdentity_) {
          cVo.buildFrom(cMo);
          LVJ_true = (m_L_edge * cVo * oJo);
        }
      }

      double wi = 0.0, eri = 0.0;
      double num = 0.0, den = 0.0;
      if ((iter == 0) || m_computeInteraction) {
        for (unsigned int i = 0; i < nbrow; i++) {
          wi = m_w_edge[i] * m_factor[i];
          W_true[i] = wi;
          eri = m_error_edge[i];
          num += wi * vpMath::sqr(eri);
          den += wi;

          m_weightedError_edge[i] = wi * eri;

          for (unsigned int j = 0; j < 6; j++) {
            m_L_edge[i][j] = wi * m_L_edge[i][j];
          }
        }
      } else {
        for (unsigned int i = 0; i < nbrow; i++) {
          wi = m_w_edge[i] * m_factor[i];
          W_true[i] = wi;
          eri = m_error_edge[i];
          num += wi * vpMath::sqr(eri);
          den += wi;

          m_weightedError_edge[i] = wi * eri;
        }
      }

      residu_1 = r;
      r = sqrt(num / den); // Le critere d'arret prend en compte le poids

      computeVVSPoseEstimation(isoJoIdentity_, iter, m_L_edge, LTL, m_weightedError_edge, m_error_edge, m_error_prev,
                               LTR, mu, v, &m_w_edge, &m_w_prev);

      cMoPrev = cMo;
      cMo = vpExponentialMap::direct(v).inverse() * cMo;

    } // endif(!restartFromLast)

    iter++;
  }

  computeCovarianceMatrixVVS(isoJoIdentity_, W_true, cMoPrev, L_true, LVJ_true, m_error_edge);

  updateMovingEdgeWeights();
}

void vpMbEdgeTracker::computeVVSFirstPhase(const vpImage<unsigned char> &_I, const unsigned int iter, double &count,
                                           const unsigned int lvl)
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  double limite = 3;              // Une limite de 3 pixels
  limite = limite / cam.get_px(); // Transformation limite pixel en limite metre.

  unsigned int n = 0;

  // Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[lvl].begin(); it != lines[lvl].end(); ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      l->computeInteractionMatrixError(cMo);

      double fac = 1;
      if (iter == 0) {
        for (std::list<int>::const_iterator itindex = l->Lindex_polygon.begin(); itindex != l->Lindex_polygon.end();
             ++itindex) {
          int index = *itindex;
          if (l->hiddenface->isAppearing((unsigned int)index)) {
            fac = 0.2;
            break;
          }
          if (l->closeToImageBorder(_I, 10)) {
            fac = 0.1;
            break;
          }
        }
      }

      std::list<vpMeSite>::const_iterator itListLine;

      unsigned int indexFeature = 0;

      for (size_t a = 0; a < l->meline.size(); a++) {
        if (iter == 0 && l->meline[a] != NULL)
          itListLine = l->meline[a]->getMeList().begin();

        for (unsigned int i = 0; i < l->nbFeature[a]; i++) {
          for (unsigned int j = 0; j < 6; j++) {
            m_L_edge[n + i][j] = l->L[indexFeature][j]; // On remplit la matrice d'interaction globale
          }
          m_error_edge[n + i] = l->error[indexFeature]; // On remplit la matrice d'erreur

          if (m_error_edge[n + i] <= limite)
            count = count + 1.0; // Si erreur proche de 0 on incremente cur

          m_w_edge[n + i] = 0;

          if (iter == 0) {
            m_factor[n + i] = fac;
            vpMeSite site = *itListLine;
            if (site.getState() != vpMeSite::NO_SUPPRESSION)
              m_factor[n + i] = 0.2;
            ++itListLine;
          }

          // If pour la premiere extremite des moving edges
          if (indexFeature == 0) {
            e_cur = l->error[0];
            if (l->nbFeature[a] > 1) {
              e_next = l->error[1];
              if (fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next)) {
                m_w_edge[n + i] = 1 /*0.5*/;
              }
              e_prev = e_cur;
            } else
              m_w_edge[n + i] = 1;
          }

          // If pour la derniere extremite des moving edges
          else if (indexFeature == l->nbFeatureTotal - 1) {
            e_cur = l->error[indexFeature];
            if (fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev)) {
              m_w_edge[n + i] += 1 /*0.5*/;
            }
          }

          else {
            e_cur = l->error[indexFeature];
            e_next = l->error[indexFeature + 1];
            if (fabs(e_cur - e_prev) < limite) {
              m_w_edge[n + i] += 0.5;
            }
            if (fabs(e_cur - e_next) < limite) {
              m_w_edge[n + i] += 0.5;
            }
            e_prev = e_cur;
          }
          indexFeature++;
        }
        n += l->nbFeature[a];
      }
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[lvl].begin(); it != cylinders[lvl].end();
       ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      cy->computeInteractionMatrixError(cMo, _I);
      double fac = 1.0;

      std::list<vpMeSite>::const_iterator itCyl1;
      std::list<vpMeSite>::const_iterator itCyl2;
      if (iter == 0 && (cy->meline1 != NULL || cy->meline2 != NULL)) {
        itCyl1 = cy->meline1->getMeList().begin();
        itCyl2 = cy->meline2->getMeList().begin();
      }

      for (unsigned int i = 0; i < cy->nbFeature; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          m_L_edge[n + i][j] = cy->L[i][j]; // On remplit la matrice d'interaction globale
        }
        m_error_edge[n + i] = cy->error[i]; // On remplit la matrice d'erreur

        if (m_error_edge[n + i] <= limite)
          count = count + 1.0; // Si erreur proche de 0 on incremente cur

        m_w_edge[n + i] = 0;

        if (iter == 0) {
          m_factor[n + i] = fac;
          vpMeSite site;
          if (i < cy->nbFeaturel1) {
            site = *itCyl1;
            ++itCyl1;
          } else {
            site = *itCyl2;
            ++itCyl2;
          }
          if (site.getState() != vpMeSite::NO_SUPPRESSION)
            m_factor[n + i] = 0.2;
        }

        // If pour la premiere extremite des moving edges
        if (i == 0) {
          e_cur = cy->error[0];
          if (cy->nbFeature > 1) {
            e_next = cy->error[1];
            if (fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next)) {
              m_w_edge[n + i] = 1 /*0.5*/;
            }
            e_prev = e_cur;
          } else
            m_w_edge[n + i] = 1;
        }
        if (i == cy->nbFeaturel1) {
          e_cur = cy->error[i];
          if (cy->nbFeaturel2 > 1) {
            e_next = cy->error[i + 1];
            if (fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next)) {
              m_w_edge[n + i] = 1 /*0.5*/;
            }
            e_prev = e_cur;
          } else
            m_w_edge[n + i] = 1;
        }

        // If pour la derniere extremite des moving edges
        else if (i == cy->nbFeaturel1 - 1) {
          e_cur = cy->error[i];
          if (fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev)) {
            m_w_edge[n + i] += 1 /*0.5*/;
          }
        }
        // If pour la derniere extremite des moving edges
        else if (i == cy->nbFeature - 1) {
          e_cur = cy->error[i];
          if (fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev)) {
            m_w_edge[n + i] += 1 /*0.5*/;
          }
        }

        else {
          e_cur = cy->error[i];
          e_next = cy->error[i + 1];
          if (fabs(e_cur - e_prev) < limite) {
            m_w_edge[n + i] += 0.5;
          }
          if (fabs(e_cur - e_next) < limite) {
            m_w_edge[n + i] += 0.5;
          }
          e_prev = e_cur;
        }
      }

      n += cy->nbFeature;
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[lvl].begin(); it != circles[lvl].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      ci->computeInteractionMatrixError(cMo);
      double fac = 1.0;

      std::list<vpMeSite>::const_iterator itCir;
      if (iter == 0 && (ci->meEllipse != NULL)) {
        itCir = ci->meEllipse->getMeList().begin();
      }

      for (unsigned int i = 0; i < ci->nbFeature; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          m_L_edge[n + i][j] = ci->L[i][j]; // On remplit la matrice d'interaction globale
        }
        m_error_edge[n + i] = ci->error[i]; // On remplit la matrice d'erreur

        if (m_error_edge[n + i] <= limite)
          count = count + 1.0; // Si erreur proche de 0 on incremente cur

        m_w_edge[n + i] = 0;

        if (iter == 0) {
          m_factor[n + i] = fac;
          vpMeSite site = *itCir;
          if (site.getState() != vpMeSite::NO_SUPPRESSION)
            m_factor[n + i] = 0.2;
          ++itCir;
        }

        // If pour la premiere extremite des moving edges
        if (i == 0) {
          e_cur = ci->error[0];
          if (ci->nbFeature > 1) {
            e_next = ci->error[1];
            if (fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next)) {
              m_w_edge[n + i] = 1 /*0.5*/;
            }
            e_prev = e_cur;
          } else
            m_w_edge[n + i] = 1;
        }

        // If pour la derniere extremite des moving edges
        else if (i == ci->nbFeature - 1) {
          e_cur = ci->error[i];
          if (fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev)) {
            m_w_edge[n + i] += 1 /*0.5*/;
          }
        }

        else {
          e_cur = ci->error[i];
          e_next = ci->error[i + 1];
          if (fabs(e_cur - e_prev) < limite) {
            m_w_edge[n + i] += 0.5;
          }
          if (fabs(e_cur - e_next) < limite) {
            m_w_edge[n + i] += 0.5;
          }
          e_prev = e_cur;
        }
      }

      n += ci->nbFeature;
    }
  }
}

void vpMbEdgeTracker::computeVVSFirstPhaseFactor(const vpImage<unsigned char> &I, const unsigned int lvl)
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  unsigned int n = 0;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[lvl].begin(); it != lines[lvl].end(); ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      l->computeInteractionMatrixError(cMo);

      double fac = 1;
      for (std::list<int>::const_iterator itindex = l->Lindex_polygon.begin(); itindex != l->Lindex_polygon.end();
           ++itindex) {
        int index = *itindex;
        if (l->hiddenface->isAppearing((unsigned int)index)) {
          fac = 0.2;
          break;
        }
        if (l->closeToImageBorder(I, 10)) {
          fac = 0.1;
          break;
        }
      }

      unsigned int indexFeature = 0;
      for (size_t a = 0; a < l->meline.size(); a++) {
        std::list<vpMeSite>::const_iterator itListLine;
        if (l->meline[a] != NULL) {
          itListLine = l->meline[a]->getMeList().begin();

          for (unsigned int i = 0; i < l->nbFeature[a]; i++) {
            m_factor[n + i] = fac;
            vpMeSite site = *itListLine;
            if (site.getState() != vpMeSite::NO_SUPPRESSION)
              m_factor[n + i] = 0.2;
            ++itListLine;
            indexFeature++;
          }
          n += l->nbFeature[a];
        }
      }
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[lvl].begin(); it != cylinders[lvl].end();
       ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      cy->computeInteractionMatrixError(cMo, I);

      std::list<vpMeSite>::const_iterator itCyl1;
      std::list<vpMeSite>::const_iterator itCyl2;
      if ((cy->meline1 != NULL || cy->meline2 != NULL)) {
        itCyl1 = cy->meline1->getMeList().begin();
        itCyl2 = cy->meline2->getMeList().begin();

        double fac = 1.0;
        for (unsigned int i = 0; i < cy->nbFeature; i++) {
          m_factor[n + i] = fac;
          vpMeSite site;
          if (i < cy->nbFeaturel1) {
            site = *itCyl1;
            ++itCyl1;
          } else {
            site = *itCyl2;
            ++itCyl2;
          }
          if (site.getState() != vpMeSite::NO_SUPPRESSION)
            m_factor[n + i] = 0.2;
        }
        n += cy->nbFeature;
      }
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[lvl].begin(); it != circles[lvl].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      ci->computeInteractionMatrixError(cMo);

      std::list<vpMeSite>::const_iterator itCir;
      if (ci->meEllipse != NULL) {
        itCir = ci->meEllipse->getMeList().begin();
        double fac = 1.0;

        for (unsigned int i = 0; i < ci->nbFeature; i++) {
          m_factor[n + i] = fac;
          vpMeSite site = *itCir;
          if (site.getState() != vpMeSite::NO_SUPPRESSION)
            m_factor[n + i] = 0.2;
          ++itCir;
        }
        n += ci->nbFeature;
      }
    }
  }
}

void vpMbEdgeTracker::computeVVSFirstPhasePoseEstimation(const unsigned int iter, bool &isoJoIdentity_)
{
  unsigned int nerror = m_weightedError_edge.getRows();

  double wi, eri;
  if ((iter == 0) || m_computeInteraction) {
    for (unsigned int i = 0; i < nerror; i++) {
      wi = m_w_edge[i] * m_factor[i];
      eri = m_error_edge[i];

      m_weightedError_edge[i] = wi * eri;

      for (unsigned int j = 0; j < 6; j++) {
        m_L_edge[i][j] = wi * m_L_edge[i][j];
      }
    }
  } else {
    for (unsigned int i = 0; i < nerror; i++) {
      wi = m_w_edge[i] * m_factor[i];
      eri = m_error_edge[i];

      m_weightedError_edge[i] = wi * eri;
    }
  }

  vpVelocityTwistMatrix cVo;

  // If all the 6 dof should be estimated, we check if the interaction matrix
  // is full rank. If not we remove automatically the dof that cannot be
  // estimated This is particularly useful when consering circles (rank 5) and
  // cylinders (rank 4)
  if (isoJoIdentity_) {
    cVo.buildFrom(cMo);

    vpMatrix K; // kernel
    unsigned int rank = (m_L_edge * cVo).kernel(K);
    if (rank == 0) {
      throw vpException(vpException::fatalError, "Rank=0, cannot estimate the pose !");
    }
    if (rank != 6) {
      vpMatrix I; // Identity
      I.eye(6);
      oJo = I - K.AtA();

      isoJoIdentity_ = false;
    }
  }

  vpColVector v;
  vpMatrix LTL;
  vpColVector LTR;

  if (isoJoIdentity_) {
    LTL = m_L_edge.AtA();
    computeJTR(m_L_edge, m_weightedError_edge, LTR);
    v = -0.7 * LTL.pseudoInverse(LTL.getRows() * std::numeric_limits<double>::epsilon()) * LTR;
  } else {
    cVo.buildFrom(cMo);
    vpMatrix LVJ = (m_L_edge * cVo * oJo);
    vpMatrix LVJTLVJ = (LVJ).AtA();
    vpColVector LVJTR;
    computeJTR(LVJ, m_weightedError_edge, LVJTR);
    v = -0.7 * LVJTLVJ.pseudoInverse(LVJTLVJ.getRows() * std::numeric_limits<double>::epsilon()) * LVJTR;
    v = cVo * v;
  }

  cMo = vpExponentialMap::direct(v).inverse() * cMo;
}

void vpMbEdgeTracker::computeVVSInit()
{
  // Nombre de moving edges
  unsigned int nbrow = 0;
  unsigned int nberrors_lines = 0;
  unsigned int nberrors_cylinders = 0;
  unsigned int nberrors_circles = 0;

  nbrow = initMbtTracking(nberrors_lines, nberrors_cylinders, nberrors_circles);

  if (nbrow == 0) {
    throw vpTrackingException(vpTrackingException::notEnoughPointError,
                              "No data found to compute the interaction matrix...");
  }

  m_L_edge.resize(nbrow, 6, false, false);
  m_error_edge.resize(nbrow, false);

  m_weightedError_edge.resize(nbrow, false);
  m_w_edge.resize(nbrow, false);
  m_w_edge = 1;
  m_factor.resize(nbrow, false);
  m_factor = 1;

  m_robustLines.resize(nberrors_lines);
  m_robustCylinders.resize(nberrors_cylinders);
  m_robustCircles.resize(nberrors_circles);
  m_robustLines.setThreshold(2.0 / cam.get_px());
  m_robustCylinders.setThreshold(2.0 / cam.get_px());
  m_robustCircles.setThreshold(vpMath::sqr(2.0 / cam.get_px()));

  m_wLines.resize(nberrors_lines, false);
  m_wLines = 1;
  m_wCylinders.resize(nberrors_cylinders, false);
  m_wCylinders = 1;
  m_wCircles.resize(nberrors_circles, false);
  m_wCircles = 1;

  m_errorLines.resize(nberrors_lines, false);
  m_errorCylinders.resize(nberrors_cylinders, false);
  m_errorCircles.resize(nberrors_circles, false);
}

void vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu()
{
  throw vpException(vpException::fatalError, "vpMbEdgeTracker::"
                                             "computeVVSInteractionMatrixAndR"
                                             "esidu() should not be called!");
}

void vpMbEdgeTracker::computeVVSInteractionMatrixAndResidu(const vpImage<unsigned char> &_I)
{
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  unsigned int n = 0;
  unsigned int nlines = 0;
  unsigned int ncylinders = 0;
  unsigned int ncircles = 0;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      l->computeInteractionMatrixError(cMo);
      for (unsigned int i = 0; i < l->nbFeatureTotal; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          m_L_edge[n + i][j] = l->L[i][j];
          m_error_edge[n + i] = l->error[i];
          m_errorLines[nlines + i] = m_error_edge[n + i];
        }
      }
      n += l->nbFeatureTotal;
      nlines += l->nbFeatureTotal;
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      cy->computeInteractionMatrixError(cMo, _I);
      for (unsigned int i = 0; i < cy->nbFeature; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          m_L_edge[n + i][j] = cy->L[i][j];
          m_error_edge[n + i] = cy->error[i];
          m_errorCylinders[ncylinders + i] = m_error_edge[n + i];
        }
      }

      n += cy->nbFeature;
      ncylinders += cy->nbFeature;
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      ci->computeInteractionMatrixError(cMo);
      for (unsigned int i = 0; i < ci->nbFeature; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          m_L_edge[n + i][j] = ci->L[i][j];
          m_error_edge[n + i] = ci->error[i];
          m_errorCircles[ncircles + i] = m_error_edge[n + i];
        }
      }

      n += ci->nbFeature;
      ncircles += ci->nbFeature;
    }
  }
}

void vpMbEdgeTracker::computeVVSWeights()
{
  unsigned int nberrors_lines = m_errorLines.getRows(), nberrors_cylinders = m_errorCylinders.getRows(),
               nberrors_circles = m_errorCircles.getRows();

  if (nberrors_lines > 0)
    m_robustLines.MEstimator(vpRobust::TUKEY, m_errorLines, m_wLines);
  if (nberrors_cylinders > 0)
    m_robustCylinders.MEstimator(vpRobust::TUKEY, m_errorCylinders, m_wCylinders);
  if (nberrors_circles > 0)
    m_robustCircles.MEstimator(vpRobust::TUKEY, m_errorCircles, m_wCircles);

  m_w_edge.insert(0, m_wLines);
  m_w_edge.insert(m_wLines.getRows(), m_wCylinders);
  m_w_edge.insert(m_wLines.getRows() + m_wCylinders.getRows(), m_wCircles);
}

/*!
  Compute the projection error of the model.
  This approach compares the gradient direction around samples of each lines
  of the model with their direction. Error is expressed in degrees between 0
  and 90.

  \param _I : Image in which the model appears.
*/
void vpMbEdgeTracker::computeProjectionError(const vpImage<unsigned char> &_I)
{
  projectionError = 0.0;
  unsigned int nbFeatures = 0;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    vpMbtDistanceLine *l = *it;
    if (l->isVisible() && l->isTracked()) {
      for (size_t a = 0; a < l->meline.size(); a++) {
        if (l->meline[a] != NULL) {
          double lineNormGradient;
          unsigned int lineNbFeatures;
          l->meline[a]->computeProjectionError(_I, lineNormGradient, lineNbFeatures, m_SobelX, m_SobelY,
                                               m_projectionErrorDisplay, m_projectionErrorDisplayLength,
                                               m_projectionErrorDisplayThickness);
          projectionError += lineNormGradient;
          nbFeatures += lineNbFeatures;
        }
      }
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    vpMbtDistanceCylinder *cy = *it;
    if (cy->isVisible() && cy->isTracked()) {
      if (cy->meline1 != NULL) {
        double cylinderNormGradient = 0;
        unsigned int cylinderNbFeatures = 0;
        cy->meline1->computeProjectionError(_I, cylinderNormGradient, cylinderNbFeatures, m_SobelX, m_SobelY,
                                            m_projectionErrorDisplay, m_projectionErrorDisplayLength,
                                            m_projectionErrorDisplayThickness);
        projectionError += cylinderNormGradient;
        nbFeatures += cylinderNbFeatures;
      }

      if (cy->meline2 != NULL) {
        double cylinderNormGradient = 0;
        unsigned int cylinderNbFeatures = 0;
        cy->meline2->computeProjectionError(_I, cylinderNormGradient, cylinderNbFeatures, m_SobelX, m_SobelY,
                                            m_projectionErrorDisplay, m_projectionErrorDisplayLength,
                                            m_projectionErrorDisplayThickness);
        projectionError += cylinderNormGradient;
        nbFeatures += cylinderNbFeatures;
      }
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    vpMbtDistanceCircle *c = *it;
    if (c->isVisible() && c->isTracked() && c->meEllipse != NULL) {
      double circleNormGradient = 0;
      unsigned int circleNbFeatures = 0;
      c->meEllipse->computeProjectionError(_I, circleNormGradient, circleNbFeatures, m_SobelX, m_SobelY,
                                           m_projectionErrorDisplay, m_projectionErrorDisplayLength,
                                           m_projectionErrorDisplayThickness);
      projectionError += circleNormGradient;
      nbFeatures += circleNbFeatures;
    }
  }

  if (nbFeatures > 0) {
    projectionError = vpMath::deg(projectionError / (double)nbFeatures);
  } else {
    projectionError = 90.0;
  }

  nbFeaturesForProjErrorComputation = nbFeatures;
  //  std::cout << "Norm Gradient = " << errorGradient << std::endl;
}

/*!
  Check if the tracking failed.

  \throw vpTrackingException::fatalError if the test fails.
*/
void vpMbEdgeTracker::testTracking()
{
  int nbExpectedPoint = 0;
  int nbGoodPoint = 0;
  int nbBadPoint = 0;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    vpMbtDistanceLine *l = *it;
    if (l->isVisible() && l->isTracked()) {
      for (size_t a = 0; a < l->meline.size(); a++) {
        if (l->meline[a] != NULL) {
          nbExpectedPoint += (int)l->meline[a]->expecteddensity;
          for (std::list<vpMeSite>::const_iterator itme = l->meline[a]->getMeList().begin();
               itme != l->meline[a]->getMeList().end(); ++itme) {
            vpMeSite pix = *itme;
            if (pix.getState() == vpMeSite::NO_SUPPRESSION)
              nbGoodPoint++;
            else
              nbBadPoint++;
          }
        }
      }
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    vpMbtDistanceCylinder *cy = *it;
    if ((cy->meline1 != NULL && cy->meline2 != NULL) && cy->isVisible() && cy->isTracked()) {
      nbExpectedPoint += (int)cy->meline1->expecteddensity;
      for (std::list<vpMeSite>::const_iterator itme1 = cy->meline1->getMeList().begin();
           itme1 != cy->meline1->getMeList().end(); ++itme1) {
        vpMeSite pix = *itme1;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoint++;
        else
          nbBadPoint++;
      }
      nbExpectedPoint += (int)cy->meline2->expecteddensity;
      for (std::list<vpMeSite>::const_iterator itme2 = cy->meline2->getMeList().begin();
           itme2 != cy->meline2->getMeList().end(); ++itme2) {
        vpMeSite pix = *itme2;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoint++;
        else
          nbBadPoint++;
      }
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci->isVisible() && ci->isTracked() && ci->meEllipse != NULL) {
      nbExpectedPoint += ci->meEllipse->getExpectedDensity();
      for (std::list<vpMeSite>::const_iterator itme = ci->meEllipse->getMeList().begin();
           itme != ci->meEllipse->getMeList().end(); ++itme) {
        vpMeSite pix = *itme;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoint++;
        else
          nbBadPoint++;
      }
    }
  }

  // Compare the number of good points with the min between the number of
  // expected points and number of points that are tracked
  int nb_min = (int)vpMath::minimum(percentageGdPt * nbExpectedPoint, percentageGdPt * (nbGoodPoint + nbBadPoint));
  // int nb_min = (std::min)(val1, val2);
  if (nbGoodPoint < nb_min || nbExpectedPoint < 2) {
    std::ostringstream oss;
    oss << "Not enough moving edges (" << nbGoodPoint << ") to track the object: expected " << nb_min
        << ". Try to reduce the threshold=" << percentageGdPt
        << " using vpMbTracker::setGoodMovingEdgesRatioThreshold()";
    throw vpTrackingException(vpTrackingException::fatalError, oss.str());
  }
}

/*!
  Compute each state of the tracking procedure for all the feature sets.

  If the tracking is considered as failed an exception is thrown.

  \param I : The image.
 */
void vpMbEdgeTracker::track(const vpImage<unsigned char> &I)
{
  track(&I, NULL);
}

void vpMbEdgeTracker::track(const vpImage<vpRGBa> &I)
{
  vpImageConvert::convert(I, m_I);
  track(&m_I, &I);
}

void vpMbEdgeTracker::track(const vpImage<unsigned char> * const I, const vpImage<vpRGBa> * const)
{
  initPyramid(*I, Ipyramid);

  unsigned int lvl = (unsigned int)scales.size();
  do {
    lvl--;

    projectionError = 90.0;

    if (scales[lvl]) {
      vpHomogeneousMatrix cMo_1 = cMo;
      try {
        downScale(lvl);

        try {
          trackMovingEdge(*Ipyramid[lvl]);
        } catch (...) {
          vpTRACE("Error in moving edge tracking");
          throw;
        }

        // initialize the vector that contains the error and the matrix that
        // contains the interaction matrix AY: Useless as it is done in
        // coputeVVS()
        /*
        for(std::list<vpMbtDistanceLine*>::const_iterator
        it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){ l = *it; if
        (l->isVisible()){ l->initInteractionMatrixError();
          }
        }

        for(std::list<vpMbtDistanceCylinder*>::const_iterator
        it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){ cy = *it;
          if(cy->isVisible()) {
            cy->initInteractionMatrixError();
          }
        }

        for(std::list<vpMbtDistanceCircle*>::const_iterator
        it=circles[lvl].begin(); it!=circles[lvl].end(); ++it){ ci = *it; if
        (ci->isVisible()){ ci->initInteractionMatrixError();
          }
        }
        */

        try {
          computeVVS(*Ipyramid[lvl], lvl);
        } catch (...) {
          covarianceMatrix = -1;
          throw; // throw the original exception
        }

        testTracking();

        if (displayFeatures) {
          m_featuresToBeDisplayedEdge = getFeaturesForDisplayEdge();
        }

        // Looking for new visible face
        bool newvisibleface = false;
        visibleFace(*I, cMo, newvisibleface);

        // cam.computeFov(I.getWidth(), I.getHeight());
        if (useScanLine) {
          faces.computeClippedPolygons(cMo, cam);
          faces.computeScanLineRender(cam, I->getWidth(), I->getHeight());
        }

        updateMovingEdge(*I);

        initMovingEdge(*I, cMo);
        // Reinit the moving edge for the lines which need it.
        reinitMovingEdge(*I, cMo);

        if (computeProjError)
          computeProjectionError(*I);

        upScale(lvl);
      } catch (const vpException &e) {
        if (lvl != 0) {
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        } else {
          upScale(lvl);
          throw(e);
        }
      }
    }
  } while (lvl != 0);

  cleanPyramid(Ipyramid);
}

/*!
 Initialize the tracking.

 \param I : The image.
*/
void vpMbEdgeTracker::init(const vpImage<unsigned char> &I)
{
  if (!modelInitialised) {
    throw vpException(vpException::fatalError, "model not initialized");
  }

  bool a = false;

#ifdef VISP_HAVE_OGRE
  if (useOgre) {
    if (!faces.isOgreInitialised()) {
      faces.setBackgroundSizeOgre(I.getHeight(), I.getWidth());
      faces.setOgreShowConfigDialog(ogreShowConfigDialog);
      faces.initOgre(cam);
      // Turn off Ogre config dialog display for the next call to this
      // function since settings are saved in the ogre.cfg file and used
      // during the next call
      ogreShowConfigDialog = false;
    }
  }
#endif

  if (clippingFlag > 2)
    cam.computeFov(I.getWidth(), I.getHeight());

  visibleFace(I, cMo, a);
  resetMovingEdge();

  if (useScanLine) {
    if (clippingFlag <= 2)
      cam.computeFov(I.getWidth(), I.getHeight());

    faces.computeClippedPolygons(cMo, cam);
    faces.computeScanLineRender(cam, I.getWidth(), I.getHeight());
  }

  initPyramid(I, Ipyramid);
  unsigned int i = (unsigned int)scales.size();
  do {
    i--;
    if (scales[i]) {
      downScale(i);
      initMovingEdge(*Ipyramid[i], cMo);
      upScale(i);
    }
  } while (i != 0);

  cleanPyramid(Ipyramid);
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I : grayscale image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void vpMbEdgeTracker::setPose(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;

  init(I);
}

/*!
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.

  \param I_color : color image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void vpMbEdgeTracker::setPose(const vpImage<vpRGBa> &I_color, const vpHomogeneousMatrix &cdMo)
{
  cMo = cdMo;

  vpImageConvert::convert(I_color, m_I);
  init(m_I);
}

/*!
  Load the xml configuration file.
  From the configuration file
  initialize the parameters corresponding to the objects: moving-edges, camera
  and visibility angles.

  \warning To clean up memory allocated by the xml library, the user has to
  call vpXmlParser::cleanup() before the exit().

  \param configFile : full name of the xml file.

  \sa loadConfigFile(const char*), vpXmlParser::cleanup()
*/
void vpMbEdgeTracker::loadConfigFile(const std::string &configFile)
{
  // Load projection error config
  vpMbTracker::loadConfigFile(configFile);

#ifdef VISP_HAVE_XML2
  vpMbtXmlParser xmlp;

  xmlp.setCameraParameters(cam);
  xmlp.setAngleAppear(vpMath::deg(angleAppears));
  xmlp.setAngleDisappear(vpMath::deg(angleDisappears));
  xmlp.setMovingEdge(me);

  try {
    std::cout << " *********** Parsing XML for Mb Edge Tracker ************ " << std::endl;
    xmlp.parse(configFile);
  } catch (...) {
    throw vpException(vpException::ioError, "Cannot open XML file \"%s\"", configFile.c_str());
  }

  vpCameraParameters camera;
  vpMe meParser;
  xmlp.getCameraParameters(camera);
  xmlp.getMe(meParser);

  setCameraParameters(camera);
  setMovingEdge(meParser);
  angleAppears = vpMath::rad(xmlp.getAngleAppear());
  angleDisappears = vpMath::rad(xmlp.getAngleDisappear());

  if (xmlp.hasNearClippingDistance())
    setNearClippingDistance(xmlp.getNearClippingDistance());

  if (xmlp.hasFarClippingDistance())
    setFarClippingDistance(xmlp.getFarClippingDistance());

  if (xmlp.getFovClipping())
    setClipping(clippingFlag | vpPolygon3D::FOV_CLIPPING);

  useLodGeneral = xmlp.getLodState();
  minLineLengthThresholdGeneral = xmlp.getMinLineLengthThreshold();
  minPolygonAreaThresholdGeneral = xmlp.getMinPolygonAreaThreshold();

  applyLodSettingInConfig = false;
  if (this->getNbPolygon() > 0) {
    applyLodSettingInConfig = true;
    setLod(useLodGeneral);
    setMinLineLengthThresh(minLineLengthThresholdGeneral);
    setMinPolygonAreaThresh(minPolygonAreaThresholdGeneral);
  }

#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile.c_str());
#endif
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible faces).
*/
void vpMbEdgeTracker::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo_,
                              const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                              const bool displayFullModel)
{
  //Display first the Moving-Edges
  if (displayFeatures) {
    displayFeaturesOnImage(I);
  }

  std::vector<std::vector<double> > models = vpMbEdgeTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    } else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double mu20 = models[i][3];
      double mu11 = models[i][4];
      double mu02 = models[i][5];
      vpDisplay::displayEllipse(I, center, mu20, mu11, mu02, true, col, thickness);
    }
  }

#ifdef VISP_HAVE_OGRE
  if (useOgre)
    faces.displayOgre(cMo_);
#endif
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non
  visible surfaces).
*/
void vpMbEdgeTracker::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo_,
                              const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                              const bool displayFullModel)
{
  //Display first the Moving-Edges
  if (displayFeatures) {
    displayFeaturesOnImage(I);
  }

  std::vector<std::vector<double> > models = vpMbEdgeTracker::getModelForDisplay(I.getWidth(), I.getHeight(), cMo_, camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    if (vpMath::equal(models[i][0], 0)) {
      vpImagePoint ip1(models[i][1], models[i][2]);
      vpImagePoint ip2(models[i][3], models[i][4]);
      vpDisplay::displayLine(I, ip1, ip2, col, thickness);
    } else if (vpMath::equal(models[i][0], 1)) {
      vpImagePoint center(models[i][1], models[i][2]);
      double mu20 = models[i][3];
      double mu11 = models[i][4];
      double mu02 = models[i][5];
      vpDisplay::displayEllipse(I, center, mu20, mu11, mu02, true, col, thickness);
    }
  }

#ifdef VISP_HAVE_OGRE
  if (useOgre)
    faces.displayOgre(cMo_);
#endif
}

std::vector<std::vector<double> > vpMbEdgeTracker::getFeaturesForDisplayEdge()
{
  std::vector<std::vector<double> > features;

  const unsigned int lvl = 0;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[lvl].begin(); it != lines[lvl].end(); ++it) {
    vpMbtDistanceLine *l = *it;
    if (l->isVisible() && l->isTracked()) {
      std::vector<std::vector<double> > currentFeatures = l->getFeaturesForDisplay();
      features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[lvl].begin(); it != cylinders[lvl].end();
       ++it) {
    vpMbtDistanceCylinder *cy = *it;
    if (cy->isVisible() && cy->isTracked()) {
      std::vector<std::vector<double> > currentFeatures = cy->getFeaturesForDisplay();
      features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[lvl].begin(); it != circles[lvl].end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci->isVisible() && ci->isTracked()) {
      std::vector<std::vector<double> > currentFeatures = ci->getFeaturesForDisplay();
      features.insert(features.end(), currentFeatures.begin(), currentFeatures.end());
    }
  }

  return features;
}

/*!
  Return a list of primitives parameters to display the model at a given pose and camera parameters.
  - Line parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`
  - Ellipse parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<mu20>`, `<mu11>`, `<mu02>`

  \param width : Image width.
  \param height : Image height.
  \param cMo_ : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbEdgeTracker::getModelForDisplay(unsigned int width, unsigned int height,
                                                                      const vpHomogeneousMatrix &cMo_,
                                                                      const vpCameraParameters &camera,
                                                                      const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
           ++it) {
        std::vector<std::vector<double> > currentModel =
          (*it)->getModelForDisplay(width, height, cMo_, camera, displayFullModel);
        models.insert(models.end(), currentModel.begin(), currentModel.end());
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
           it != cylinders[scaleLevel].end(); ++it) {
        std::vector<std::vector<double> > currentModel =
          (*it)->getModelForDisplay(width, height, cMo_, camera, displayFullModel);
        models.insert(models.end(), currentModel.begin(), currentModel.end());
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
           it != circles[scaleLevel].end(); ++it) {
        std::vector<double> paramsCircle = (*it)->getModelForDisplay(cMo_, camera, displayFullModel);
        models.push_back(paramsCircle);
      }
      break; // displaying model on one scale only
    }
  }

  return models;
}

void vpMbEdgeTracker::displayFeaturesOnImage(const vpImage<unsigned char> &I)
{
  for (size_t i = 0; i < m_featuresToBeDisplayedEdge.size(); i++) {
    if (vpMath::equal(m_featuresToBeDisplayedEdge[i][0], 0)) {
      vpImagePoint ip(m_featuresToBeDisplayedEdge[i][1], m_featuresToBeDisplayedEdge[i][2]);
      int state = static_cast<int>(m_featuresToBeDisplayedEdge[i][3]);

      switch (state) {
      case vpMeSite::NO_SUPPRESSION:
        vpDisplay::displayCross(I, ip, 3, vpColor::green, 1);
        break;

      case vpMeSite::CONSTRAST:
        vpDisplay::displayCross(I, ip, 3, vpColor::blue, 1);
        break;

      case vpMeSite::THRESHOLD:
        vpDisplay::displayCross(I, ip, 3, vpColor::purple, 1);
        break;

      case vpMeSite::M_ESTIMATOR:
        vpDisplay::displayCross(I, ip, 3, vpColor::red, 1);
        break;

      case vpMeSite::TOO_NEAR:
        vpDisplay::displayCross(I, ip, 3, vpColor::cyan, 1);
        break;

      default:
        vpDisplay::displayCross(I, ip, 3, vpColor::yellow, 1);
      }
    }
  }
}

void vpMbEdgeTracker::displayFeaturesOnImage(const vpImage<vpRGBa> &I)
{
  for (size_t i = 0; i < m_featuresToBeDisplayedEdge.size(); i++) {
    if (vpMath::equal(m_featuresToBeDisplayedEdge[i][0], 0)) {
      vpImagePoint ip(m_featuresToBeDisplayedEdge[i][1], m_featuresToBeDisplayedEdge[i][2]);
      int state = static_cast<int>(m_featuresToBeDisplayedEdge[i][3]);

      switch (state) {
      case vpMeSite::NO_SUPPRESSION:
        vpDisplay::displayCross(I, ip, 3, vpColor::green, 1);
        break;

      case vpMeSite::CONSTRAST:
        vpDisplay::displayCross(I, ip, 3, vpColor::blue, 1);
        break;

      case vpMeSite::THRESHOLD:
        vpDisplay::displayCross(I, ip, 3, vpColor::purple, 1);
        break;

      case vpMeSite::M_ESTIMATOR:
        vpDisplay::displayCross(I, ip, 3, vpColor::red, 1);
        break;

      case vpMeSite::TOO_NEAR:
        vpDisplay::displayCross(I, ip, 3, vpColor::cyan, 1);
        break;

      default:
        vpDisplay::displayCross(I, ip, 3, vpColor::yellow, 1);
      }
    }
  }
}

/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the
  lines.

  \param I : The image.
  \param _cMo : The pose of the camera used to initialize the moving edges.
*/
void vpMbEdgeTracker::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  const bool doNotTrack = false;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    vpMbtDistanceLine *l = *it;
    bool isvisible = false;

    for (std::list<int>::const_iterator itindex = l->Lindex_polygon.begin(); itindex != l->Lindex_polygon.end();
         ++itindex) {
      int index = *itindex;
      if (index == -1)
        isvisible = true;
      else {
        if (l->hiddenface->isVisible((unsigned int)index))
          isvisible = true;
      }
    }

    // Si la ligne n'appartient a aucune face elle est tout le temps visible
    if (l->Lindex_polygon.empty())
      isvisible = true; // Not sure that this can occur

    if (isvisible) {
      l->setVisible(true);
      l->updateTracked();
      if (l->meline.empty() && l->isTracked())
        l->initMovingEdge(I, _cMo, doNotTrack, m_mask);
    } else {
      l->setVisible(false);
      for (size_t a = 0; a < l->meline.size(); a++) {
        if (l->meline[a] != NULL)
          delete l->meline[a];
        if (a < l->nbFeature.size())
          l->nbFeature[a] = 0;
      }
      l->nbFeatureTotal = 0;
      l->meline.clear();
      l->nbFeature.clear();
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    vpMbtDistanceCylinder *cy = *it;

    bool isvisible = false;

    int index = cy->index_polygon;
    if (index == -1)
      isvisible = true;
    else {
      if (cy->hiddenface->isVisible((unsigned int)index + 1) || cy->hiddenface->isVisible((unsigned int)index + 2) ||
          cy->hiddenface->isVisible((unsigned int)index + 3) || cy->hiddenface->isVisible((unsigned int)index + 4))
        isvisible = true;
    }
    //    vpTRACE("cyl with index %d is visible: %d", index, isvisible);

    if (isvisible) {
      cy->setVisible(true);
      if (cy->meline1 == NULL || cy->meline2 == NULL) {
        if (cy->isTracked())
          cy->initMovingEdge(I, _cMo, doNotTrack, m_mask);
      }
    } else {
      cy->setVisible(false);
      if (cy->meline1 != NULL)
        delete cy->meline1;
      if (cy->meline2 != NULL)
        delete cy->meline2;
      cy->meline1 = NULL;
      cy->meline2 = NULL;
      cy->nbFeature = 0;
      cy->nbFeaturel1 = 0;
      cy->nbFeaturel2 = 0;
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    bool isvisible = false;

    int index = ci->index_polygon;
    if (index == -1)
      isvisible = true;
    else {
      if (ci->hiddenface->isVisible((unsigned int)index))
        isvisible = true;
    }

    if (isvisible) {
      ci->setVisible(true);
      if (ci->meEllipse == NULL) {
        if (ci->isTracked())
          ci->initMovingEdge(I, _cMo, doNotTrack, m_mask);
      }
    } else {
      ci->setVisible(false);
      if (ci->meEllipse != NULL)
        delete ci->meEllipse;
      ci->meEllipse = NULL;
      ci->nbFeature = 0;
    }
  }
}

/*!
  Track the moving edges in the image.

  \param I : the image.
*/
void vpMbEdgeTracker::trackMovingEdge(const vpImage<unsigned char> &I)
{
  const bool doNotTrack = false;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    vpMbtDistanceLine *l = *it;
    if (l->isVisible() && l->isTracked()) {
      if (l->meline.empty()) {
        l->initMovingEdge(I, cMo, doNotTrack, m_mask);
      }
      l->trackMovingEdge(I);
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    vpMbtDistanceCylinder *cy = *it;
    if (cy->isVisible() && cy->isTracked()) {
      if (cy->meline1 == NULL || cy->meline2 == NULL) {
        cy->initMovingEdge(I, cMo, doNotTrack, m_mask);
      }
      cy->trackMovingEdge(I, cMo);
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    vpMbtDistanceCircle *ci = *it;
    if (ci->isVisible() && ci->isTracked()) {
      if (ci->meEllipse == NULL) {
        ci->initMovingEdge(I, cMo, doNotTrack, m_mask);
      }
      ci->trackMovingEdge(I, cMo);
    }
  }
}

/*!
  Update the moving edges at the end of the virtual visual servoing.

  \param I : the image.
*/
void vpMbEdgeTracker::updateMovingEdge(const vpImage<unsigned char> &I)
{
  vpMbtDistanceLine *l;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      l->updateMovingEdge(I, cMo);
      if (l->nbFeatureTotal == 0 && l->isVisible()) {
        l->Reinit = true;
      }
    }
  }

  vpMbtDistanceCylinder *cy;
  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      cy->updateMovingEdge(I, cMo);
      if ((cy->nbFeaturel1 == 0 || cy->nbFeaturel2 == 0) && cy->isVisible()) {
        cy->Reinit = true;
      }
    }
  }

  vpMbtDistanceCircle *ci;
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      ci->updateMovingEdge(I, cMo);
      if (ci->nbFeature == 0 && ci->isVisible()) {
        ci->Reinit = true;
      }
    }
  }
}

void vpMbEdgeTracker::updateMovingEdgeWeights()
{
  unsigned int n = 0;

  vpMbtDistanceLine *l;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      unsigned int indexLine = 0;
      double wmean = 0;
      for (size_t a = 0; a < l->meline.size(); a++) {
        if (l->nbFeature[a] > 0) {
          std::list<vpMeSite>::iterator itListLine;
          itListLine = l->meline[a]->getMeList().begin();

          for (unsigned int i = 0; i < l->nbFeature[a]; i++) {
            wmean += m_w_edge[n + indexLine];
            vpMeSite p = *itListLine;
            if (m_w_edge[n + indexLine] < 0.5) {
              p.setState(vpMeSite::M_ESTIMATOR);

              *itListLine = p;
            }

            ++itListLine;
            indexLine++;
          }
        }
      }
      n += l->nbFeatureTotal;

      if (l->nbFeatureTotal != 0)
        wmean /= l->nbFeatureTotal;
      else
        wmean = 1;

      l->setMeanWeight(wmean);

      if (wmean < 0.8)
        l->Reinit = true;
    }
  }

  // Same thing with cylinders as with lines
  vpMbtDistanceCylinder *cy;
  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      double wmean = 0;
      std::list<vpMeSite>::iterator itListCyl1;
      std::list<vpMeSite>::iterator itListCyl2;

      if (cy->nbFeature > 0) {
        itListCyl1 = cy->meline1->getMeList().begin();
        itListCyl2 = cy->meline2->getMeList().begin();

        for (unsigned int i = 0; i < cy->nbFeaturel1; i++) {
          wmean += m_w_edge[n + i];
          vpMeSite p = *itListCyl1;
          if (m_w_edge[n + i] < 0.5) {
            p.setState(vpMeSite::M_ESTIMATOR);

            *itListCyl1 = p;
          }

          ++itListCyl1;
        }
      }

      if (cy->nbFeaturel1 != 0)
        wmean /= cy->nbFeaturel1;
      else
        wmean = 1;

      cy->setMeanWeight1(wmean);

      if (wmean < 0.8) {
        cy->Reinit = true;
      }

      wmean = 0;
      for (unsigned int i = cy->nbFeaturel1; i < cy->nbFeature; i++) {
        wmean += m_w_edge[n + i];
        vpMeSite p = *itListCyl2;
        if (m_w_edge[n + i] < 0.5) {
          p.setState(vpMeSite::M_ESTIMATOR);

          *itListCyl2 = p;
        }

        ++itListCyl2;
      }

      if (cy->nbFeaturel2 != 0)
        wmean /= cy->nbFeaturel2;
      else
        wmean = 1;

      cy->setMeanWeight2(wmean);

      if (wmean < 0.8) {
        cy->Reinit = true;
      }

      n += cy->nbFeature;
    }
  }

  // Same thing with circles as with lines
  vpMbtDistanceCircle *ci;
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      double wmean = 0;
      std::list<vpMeSite>::iterator itListCir;

      if (ci->nbFeature > 0) {
        itListCir = ci->meEllipse->getMeList().begin();
      }

      wmean = 0;
      for (unsigned int i = 0; i < ci->nbFeature; i++) {
        wmean += m_w_edge[n + i];
        vpMeSite p = *itListCir;
        if (m_w_edge[n + i] < 0.5) {
          p.setState(vpMeSite::M_ESTIMATOR);

          *itListCir = p;
        }

        ++itListCir;
      }

      if (ci->nbFeature != 0)
        wmean /= ci->nbFeature;
      else
        wmean = 1;

      ci->setMeanWeight(wmean);

      if (wmean < 0.8) {
        ci->Reinit = true;
      }

      n += ci->nbFeature;
    }
  }
}

/*!
  Reinitialize the lines if it is required.

  A line is reinitialized if the 2D line do not match enough with the
  projected 3D line.

  \param I : the image.
  \param _cMo : the pose of the used to re-initialize the moving edges
*/
void vpMbEdgeTracker::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtDistanceLine *l;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      if (l->Reinit && l->isVisible())
        l->reinitMovingEdge(I, _cMo, m_mask);
    }
  }

  vpMbtDistanceCylinder *cy;
  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      if (cy->Reinit && cy->isVisible())
        cy->reinitMovingEdge(I, _cMo, m_mask);
    }
  }

  vpMbtDistanceCircle *ci;
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      if (ci->Reinit && ci->isVisible())
        ci->reinitMovingEdge(I, _cMo, m_mask);
    }
  }
}

void vpMbEdgeTracker::resetMovingEdge()
{
  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        for (size_t a = 0; a < (*it)->meline.size(); a++) {
          if ((*it)->meline[a] != NULL) {
            delete (*it)->meline[a];
            (*it)->meline[a] = NULL;
          }
        }

        (*it)->meline.clear();
        (*it)->nbFeature.clear();
        (*it)->nbFeatureTotal = 0;
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        if ((*it)->meline1 != NULL) {
          delete (*it)->meline1;
          (*it)->meline1 = NULL;
        }
        if ((*it)->meline2 != NULL) {
          delete (*it)->meline2;
          (*it)->meline2 = NULL;
        }

        (*it)->nbFeature = 0;
        (*it)->nbFeaturel1 = 0;
        (*it)->nbFeaturel2 = 0;
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        if ((*it)->meEllipse != NULL) {
          delete (*it)->meEllipse;
          (*it)->meEllipse = NULL;
        }
        (*it)->nbFeature = 0;
      }
    }
  }
}

/*!
  Add a line belonging to the \f$ index \f$ the polygon to the list of lines.
  It is defined by its two extremities.

  If the line already exists, the ploygone's index is added to the list of
  polygon to which it belongs.

  \param P1 : The first extremity of the line.
  \param P2 : The second extremity of the line.
  \param polygon : The index of the polygon to which the line belongs.
  \param name : the optional name of the line
*/
void vpMbEdgeTracker::addLine(vpPoint &P1, vpPoint &P2, int polygon, std::string name)
{
  {
    // suppress line already in the model
    bool already_here = false;
    vpMbtDistanceLine *l;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        downScale(i);
        for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
          l = *it;
          if ((samePoint(*(l->p1), P1) && samePoint(*(l->p2), P2)) ||
              (samePoint(*(l->p1), P2) && samePoint(*(l->p2), P1))) {
            already_here = true;
            l->addPolygon(polygon);
            l->hiddenface = &faces;
          }
        }

        if (!already_here) {
          l = new vpMbtDistanceLine;

          l->setCameraParameters(cam);
          l->buildFrom(P1, P2);
          l->addPolygon(polygon);
          l->setMovingEdge(&me);
          l->hiddenface = &faces;
          l->useScanLine = useScanLine;

          l->setIndex(nline);
          l->setName(name);

          if (clippingFlag != vpPolygon3D::NO_CLIPPING)
            l->getPolygon().setClipping(clippingFlag);

          if ((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
            l->getPolygon().setNearClippingDistance(distNearClip);

          if ((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
            l->getPolygon().setFarClippingDistance(distFarClip);

          nline += 1;
          lines[i].push_back(l);
        }
        upScale(i);
      }
    }
  }
}

/*!
  Remove a line using its name.

  \param name : The name of the line to remove.
*/
void vpMbEdgeTracker::removeLine(const std::string &name)
{
  vpMbtDistanceLine *l;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (name.compare(l->getName()) == 0) {
          lines[i].erase(it);
          break;
        }
      }
    }
  }
}

/*!
  Add a circle to the list of circles.

  \param P1 : Center of the circle.
  \param P2,P3 : Two points on the plane containing the circle. With the
  center of the circle we have 3 points defining the plane that contains the
  circle. \param r : Radius of the circle. \param idFace : Id of the face that
  is associated to the circle to handle visibility test. \param name : the
  optional name of the circle.
*/
void vpMbEdgeTracker::addCircle(const vpPoint &P1, const vpPoint &P2, const vpPoint &P3, const double r, int idFace,
                                const std::string &name)
{
  {
    bool already_here = false;
    vpMbtDistanceCircle *ci;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        downScale(i);
        for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
          ci = *it;
          if ((samePoint(*(ci->p1), P1) && samePoint(*(ci->p2), P2) && samePoint(*(ci->p3), P3)) ||
              (samePoint(*(ci->p1), P1) && samePoint(*(ci->p2), P3) && samePoint(*(ci->p3), P2))) {
            already_here =
                (std::fabs(ci->radius - r) < std::numeric_limits<double>::epsilon() * vpMath::maximum(ci->radius, r));
          }
        }

        if (!already_here) {
          ci = new vpMbtDistanceCircle;

          ci->setCameraParameters(cam);
          ci->buildFrom(P1, P2, P3, r);
          ci->setMovingEdge(&me);
          ci->setIndex(ncircle);
          ci->setName(name);
          ci->index_polygon = idFace;
          ci->hiddenface = &faces;

          //        if(clippingFlag != vpPolygon3D::NO_CLIPPING)
          //          ci->getPolygon().setClipping(clippingFlag);

          //        if((clippingFlag & vpPolygon3D::NEAR_CLIPPING) ==
          //        vpPolygon3D::NEAR_CLIPPING)
          //          ci->getPolygon().setNearClippingDistance(distNearClip);

          //        if((clippingFlag & vpPolygon3D::FAR_CLIPPING) ==
          //        vpPolygon3D::FAR_CLIPPING)
          //          ci->getPolygon().setFarClippingDistance(distFarClip);

          ncircle += 1;
          circles[i].push_back(ci);
        }
        upScale(i);
      }
    }
  }
}

/*!
  Add a cylinder to the list of cylinders.

  \param P1 : The first extremity of the axis.
  \param P2 : The second extremity of the axis.
  \param r : The radius of the cylinder.
  \param idFace : The index of the face.
  \param name : the optional name of the cylinder
*/
void vpMbEdgeTracker::addCylinder(const vpPoint &P1, const vpPoint &P2, const double r, int idFace,
                                  const std::string &name)
{
  {
    bool already_here = false;
    vpMbtDistanceCylinder *cy;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        downScale(i);
        for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
             ++it) {
          cy = *it;
          if ((samePoint(*(cy->p1), P1) && samePoint(*(cy->p2), P2)) ||
              (samePoint(*(cy->p1), P2) && samePoint(*(cy->p2), P1))) {
            already_here =
                (std::fabs(cy->radius - r) < std::numeric_limits<double>::epsilon() * vpMath::maximum(cy->radius, r));
          }
        }

        if (!already_here) {
          cy = new vpMbtDistanceCylinder;

          cy->setCameraParameters(cam);
          cy->buildFrom(P1, P2, r);
          cy->setMovingEdge(&me);
          cy->setIndex(ncylinder);
          cy->setName(name);
          cy->index_polygon = idFace;
          cy->hiddenface = &faces;
          ncylinder += 1;
          cylinders[i].push_back(cy);
        }
        upScale(i);
      }
    }
  }
}

/*!
  Remove a cylinder by its name.

  \param name : The name of the cylinder to remove.
*/
void vpMbEdgeTracker::removeCylinder(const std::string &name)
{
  vpMbtDistanceCylinder *cy;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceCylinder *>::iterator it = cylinders[i].begin(); it != cylinders[i].end(); ++it) {
        cy = *it;
        if (name.compare(cy->getName()) == 0) {
          cylinders[i].erase(it);
          break;
        }
      }
    }
  }
}

/*!
  Remove a circle by its name.

  \param name : The name of the circle to remove.
*/
void vpMbEdgeTracker::removeCircle(const std::string &name)
{
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i++) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceCircle *>::iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (name.compare(ci->getName()) == 0) {
          circles[i].erase(it);
          break;
        }
      }
    }
  }
}

/*!
  Add a polygon to the list of polygons.

  \param p : The polygon to add.
*/
void vpMbEdgeTracker::addPolygon(vpMbtPolygon &p)
{
  unsigned int nbpt = p.getNbPoint();
  if (nbpt > 0) {
    for (unsigned int i = 0; i < nbpt - 1; i++)
      addLine(p.p[i], p.p[i + 1], p.getIndex());
    addLine(p.p[nbpt - 1], p.p[0], p.getIndex());
  }
}

/*!
  Detect the visible faces in the image and says if a new one appeared.

  \warning If in one iteration one face appears and one disappears, then the
  function will not detect the new face.

  \param _I : Image to test if a face is entirely in the image.
  \param _cMo : The pose of the camera used to project the 3D model into the
  image. \param newvisibleline : This parameter is set to true if a new face
  appeared.
*/
void vpMbEdgeTracker::visibleFace(const vpImage<unsigned char> &_I, const vpHomogeneousMatrix &_cMo,
                                  bool &newvisibleline)
{
  unsigned int n;
  bool changed = false;

  if (!useOgre) {
    // n = faces.setVisible(_I.getWidth(), _I.getHeight(), cam, _cMo, vpMath::rad(89), vpMath::rad(89),
    // changed);
    n = faces.setVisible(_I.getWidth(), _I.getHeight(), cam, _cMo, angleAppears, angleDisappears, changed);
  } else {
#ifdef VISP_HAVE_OGRE
    n = faces.setVisibleOgre(_I.getWidth(), _I.getHeight(), cam, _cMo, angleAppears, angleDisappears, changed);
#else
    n = faces.setVisible(_I.getWidth(), _I.getHeight(), cam, _cMo, angleAppears, angleDisappears, changed);
#endif
  }

  if (n > nbvisiblepolygone) {
    // cout << "une nouvelle face est visible " << endl;
    newvisibleline = true;
  } else
    newvisibleline = false;

  nbvisiblepolygone = n;
}

/*!
  Add the lines to track from the polygon description. If the polygon has only
  two points, it defines a single line that is always visible. If it has three
  or more corners, it defines a face. In that case the visibility of the face
  is computed in order to track the corresponding lines only if the face is
  visible.

  The id of the polygon is supposed to be set prior calling this function.

  This method is called from the loadModel() one to add a face of the object
  to track.

  \param polygon : The polygon describing the set of lines that has to be
  tracked.
*/
void vpMbEdgeTracker::initFaceFromCorners(vpMbtPolygon &polygon)
{
  unsigned int nbpt = polygon.getNbPoint();
  if (nbpt > 0) {
    for (unsigned int i = 0; i < nbpt - 1; i++)
      vpMbEdgeTracker::addLine(polygon.p[i], polygon.p[i + 1], polygon.getIndex(), polygon.getName());
    vpMbEdgeTracker::addLine(polygon.p[nbpt - 1], polygon.p[0], polygon.getIndex(), polygon.getName());
  }
}
/*!
  Add the lines to track from the polygon description. If the polygon has only
  two points, it defines a single line that is always visible. If it has three
  or more corners, it defines a face. In that case the visibility of the face
  is computed in order to track the corresponding lines only if the face is
  visible.

  The id of the polygon is supposed to be set prior calling this function.

  This method is called from the loadModel() one to add a face of the object
  to track.

  \param polygon : The polygon describing the set of lines that has to be
  tracked.
*/
void vpMbEdgeTracker::initFaceFromLines(vpMbtPolygon &polygon)
{
  unsigned int nbpt = polygon.getNbPoint();
  if (nbpt > 0) {
    for (unsigned int i = 0; i < nbpt - 1; i++)
      vpMbEdgeTracker::addLine(polygon.p[i], polygon.p[i + 1], polygon.getIndex(), polygon.getName());
  }
}

unsigned int vpMbEdgeTracker::initMbtTracking(unsigned int &nberrors_lines, unsigned int &nberrors_cylinders,
                                              unsigned int &nberrors_circles)
{
  unsigned int nbrow = 0;
  nberrors_lines = 0;
  nberrors_cylinders = 0;
  nberrors_circles = 0;

  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {

    vpMbtDistanceLine *l = *it;

    if (l->isTracked()) {
      l->initInteractionMatrixError();
      nbrow += l->nbFeatureTotal;
      nberrors_lines += l->nbFeatureTotal;
    }
  }

  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    vpMbtDistanceCylinder *cy = *it;

    if (cy->isTracked()) {
      cy->initInteractionMatrixError();
      nbrow += cy->nbFeature;
      nberrors_cylinders += cy->nbFeature;
    }
  }

  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    vpMbtDistanceCircle *ci = *it;

    if (ci->isTracked()) {
      ci->initInteractionMatrixError();
      nbrow += ci->nbFeature;
      nberrors_circles += ci->nbFeature;
    }
  }

  return nbrow;
}

/*!
  Add a circle to track from its center, 3 points (including the center)
  defining the plane that contain the circle and its radius.

  \param p1 : Center of the circle.
  \param p2,p3 : Two points on the plane containing the circle. With the
  center of the circle we have 3 points defining the plane that contains the
  circle. \param radius : Radius of the circle. \param idFace : Index of the
  face associated to the circle to handle visibility test. \param name : The
  optional name of the circle.
*/
void vpMbEdgeTracker::initCircle(const vpPoint &p1, const vpPoint &p2, const vpPoint &p3, const double radius,
                                 const int idFace, const std::string &name)
{
  addCircle(p1, p2, p3, radius, (int)idFace, name);
}

/*!
  Add a cylinder to track from two points on the axis (defining the length of
  the cylinder) and its radius.

  \param p1 : First point on the axis.
  \param p2 : Second point on the axis.
  \param radius : Radius of the cylinder.
  \param idFace : Id of the face that is associated to the cylinder to handle
  visibility test. \param name : The optional name of the cylinder.
*/
void vpMbEdgeTracker::initCylinder(const vpPoint &p1, const vpPoint &p2, const double radius, const int idFace,
                                   const std::string &name)
{
  addCylinder(p1, p2, radius, (int)idFace, name);
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose.

*/
void vpMbEdgeTracker::resetTracker()
{
  this->cMo.eye();
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL)
          delete l;
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        cy = *it;
        if (cy != NULL)
          delete cy;
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL)
          delete ci;
        ci = NULL;
      }
      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  faces.reset();

  useScanLine = false;

#ifdef VISP_HAVE_OGRE
  useOgre = false;
#endif

  m_computeInteraction = true;
  nline = 0;
  ncylinder = 0;
  m_lambda = 1.0;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;

  angleAppears = vpMath::rad(89);
  angleDisappears = vpMath::rad(89);
  clippingFlag = vpPolygon3D::NO_CLIPPING;

  m_optimizationMethod = vpMbTracker::GAUSS_NEWTON_OPT;

  // reinitialization of the scales.
  this->setScales(scales);
}

/*!
  Re-initialize the model used by the tracker.

  \param I : The image containing the object to initialize.
  \param cad_name : Path to the file containing the 3D model description.
  \param cMo_ : The new vpHomogeneousMatrix between the camera and the new
  model
  \param verbose : verbose option to print additional information when
  loading CAO model files which include other CAO model files.
  \param T : optional transformation matrix (currently only for .cao) to transform
  3D points expressed in the original object frame to the desired object frame.
*/
void vpMbEdgeTracker::reInitModel(const vpImage<unsigned char> &I, const std::string &cad_name,
                                  const vpHomogeneousMatrix &cMo_, const bool verbose,
                                  const vpHomogeneousMatrix &T)
{
  this->cMo.eye();
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;
  vpMbtDistanceCircle *ci;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        if (l != NULL)
          delete l;
        l = NULL;
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        cy = *it;
        if (cy != NULL)
          delete cy;
        cy = NULL;
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        ci = *it;
        if (ci != NULL)
          delete ci;
        ci = NULL;
      }

      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }

  faces.reset();

  // compute_interaction=1;
  nline = 0;
  ncylinder = 0;
  ncircle = 0;
  // lambda = 1;
  nbvisiblepolygone = 0;

  loadModel(cad_name, verbose, T);
  initFromPose(I, cMo_);
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a
  vpMeSite with its flag "state" equal to 0. Only these points are used
  during the virtual visual servoing stage.

  \exception vpException::dimensionError if level does not represent a used
  level.

  \return the number of good points.
*/
unsigned int vpMbEdgeTracker::getNbPoints(const unsigned int level) const
{
  if ((level > scales.size()) || !scales[level]) {
    throw vpException(vpException::dimensionError, "Cannot get the number of points for level %d: level is not used",
                      level);
  }

  unsigned int nbGoodPoints = 0;
  vpMbtDistanceLine *l;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[level].begin(); it != lines[level].end(); ++it) {
    l = *it;
    if (l->isVisible() && l->isTracked()) {
      for (size_t a = 0; a < l->meline.size(); a++) {
        if (l->nbFeature[a] != 0)
          for (std::list<vpMeSite>::const_iterator itme = l->meline[a]->getMeList().begin();
               itme != l->meline[a]->getMeList().end(); ++itme) {
            if (itme->getState() == vpMeSite::NO_SUPPRESSION)
              nbGoodPoints++;
          }
      }
    }
  }

  vpMbtDistanceCylinder *cy;
  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[level].begin(); it != cylinders[level].end();
       ++it) {
    cy = *it;
    if (cy->isVisible() && cy->isTracked() && (cy->meline1 != NULL || cy->meline2 != NULL)) {
      for (std::list<vpMeSite>::const_iterator itme1 = cy->meline1->getMeList().begin();
           itme1 != cy->meline1->getMeList().end(); ++itme1) {
        if (itme1->getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoints++;
      }
      for (std::list<vpMeSite>::const_iterator itme2 = cy->meline2->getMeList().begin();
           itme2 != cy->meline2->getMeList().end(); ++itme2) {
        if (itme2->getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoints++;
      }
    }
  }

  vpMbtDistanceCircle *ci;
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[level].begin(); it != circles[level].end(); ++it) {
    ci = *it;
    if (ci->isVisible() && ci->isTracked() && ci->meEllipse != NULL) {
      for (std::list<vpMeSite>::const_iterator itme = ci->meEllipse->getMeList().begin();
           itme != ci->meEllipse->getMeList().end(); ++itme) {
        if (itme->getState() == vpMeSite::NO_SUPPRESSION)
          nbGoodPoints++;
      }
    }
  }

  return nbGoodPoints;
}

/*!
  Set the scales to use to realize the tracking. The vector of boolean
  activates or not the scales to set for the object tracking. The first
  element of the list correspond to the tracking on the full image, the second
  element corresponds to the tracking on an image subsampled by two.

  Using multi scale tracking allows to track the object with greater moves. It
  requires the computation of a pyramid of images, but the total tracking can
  be faster than a tracking based only on the full scale. The pose is computed
  from the smallest image to the biggest. This may be dangerous if the object
  to track is small in the image, because the subsampled scale(s) will have
  only few points to compute the pose (it could result in a loss of
  precision).

  \warning This method must be used before the tracker has been initialized (
  before the call of the loadConfigFile() or loadModel() methods).

  \warning At least one level must be activated.

  \param scale : The vector describing the levels to use.
*/
void vpMbEdgeTracker::setScales(const std::vector<bool> &scale)
{
  unsigned int nbActivatedLevels = 0;
  for (unsigned int i = 0; i < scale.size(); i++) {
    if (scale[i]) {
      nbActivatedLevels++;
    }
  }

  if (scale.empty() || (nbActivatedLevels == 0)) {
    vpERROR_TRACE(" !! WARNING : must use at least one level for the "
                  "tracking. Use the global one");
    this->scales.resize(0);
    this->scales.push_back(true);

    lines.resize(1);
    lines[0].clear();

    cylinders.resize(1);
    cylinders[0].clear();

    circles.resize(1);
    circles[0].clear();
  } else {
    this->scales = scale;

    lines.resize(scale.size());
    cylinders.resize(scale.size());
    circles.resize(scale.size());

    for (unsigned int i = 0; i < lines.size(); i++) {
      lines[i].clear();
      cylinders[i].clear();
      circles[i].clear();
    }
  }
}

/*!
  Set the far distance for clipping.

  \param dist : Far clipping value.
*/
void vpMbEdgeTracker::setFarClippingDistance(const double &dist)
{
  if ((clippingFlag & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING && dist <= distNearClip)
    std::cerr << "Far clipping value cannot be inferior than near clipping "
                 "value. Far clipping won't be considered."
              << std::endl;
  else if (dist < 0)
    std::cerr << "Far clipping value cannot be inferior than 0. Far clipping "
                 "won't be considered."
              << std::endl;
  else {
    vpMbTracker::setFarClippingDistance(dist);
    vpMbtDistanceLine *l;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
          l = *it;
          l->getPolygon().setFarClippingDistance(distFarClip);
        }
      }
    }
  }
}

/*!
  Set the near distance for clipping.

  \param dist : Near clipping value.
*/
void vpMbEdgeTracker::setNearClippingDistance(const double &dist)
{
  if ((clippingFlag & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING && dist >= distFarClip)
    std::cerr << "Near clipping value cannot be superior than far clipping "
                 "value. Near clipping won't be considered."
              << std::endl;
  else if (dist < 0)
    std::cerr << "Near clipping value cannot be inferior than 0. Near "
                 "clipping won't be considered."
              << std::endl;
  else {
    vpMbTracker::setNearClippingDistance(dist);
    vpMbtDistanceLine *l;

    for (unsigned int i = 0; i < scales.size(); i += 1) {
      if (scales[i]) {
        for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
          l = *it;
          l->getPolygon().setNearClippingDistance(distNearClip);
        }
      }
    }
  }
}

/*!
  Specify which clipping to use.

  \sa vpMbtPolygonClipping

  \param flags : New clipping flags.
*/
void vpMbEdgeTracker::setClipping(const unsigned int &flags)
{
  vpMbTracker::setClipping(flags);

  vpMbtDistanceLine *l;

  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        l = *it;
        l->getPolygon().setClipping(clippingFlag);
      }
    }
  }
}

/*!
  Compute the pyramid of image associated to the image in parameter. The
  scales computed are the ones corresponding to the scales  attribute of the
  class. If OpenCV is detected, the functions used to computed a smoothed
  pyramid come from OpenCV, otherwise a simple subsampling (no smoothing, no
  interpolation) is realized.

  \warning The pyramid contains pointers to vpImage. To properly deallocate
  the pyramid. All the element but the first (which is a pointer to the input
  image) must be freed. A proper cleaning is implemented in the cleanPyramid()
  method.

  \param _I : The input image.
  \param _pyramid : The pyramid of image to build from the input image.
*/
void vpMbEdgeTracker::initPyramid(const vpImage<unsigned char> &_I,
                                  std::vector<const vpImage<unsigned char> *> &_pyramid)
{
  _pyramid.resize(scales.size());

  if (scales[0]) {
    _pyramid[0] = &_I;
  } else {
    _pyramid[0] = NULL;
  }

  for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
    if (scales[i]) {
      unsigned int cScale = static_cast<unsigned int>(pow(2., (int)i));
      vpImage<unsigned char> *I = new vpImage<unsigned char>(_I.getHeight() / cScale, _I.getWidth() / cScale);
#if (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x020408))
      IplImage *vpI0 = cvCreateImageHeader(cvSize((int)_I.getWidth(), (int)_I.getHeight()), IPL_DEPTH_8U, 1);
      vpI0->imageData = (char *)(_I.bitmap);
      IplImage *vpI =
          cvCreateImage(cvSize((int)(_I.getWidth() / cScale), (int)(_I.getHeight() / cScale)), IPL_DEPTH_8U, 1);
      cvResize(vpI0, vpI, CV_INTER_NN);
      vpImageConvert::convert(vpI, *I);
      cvReleaseImage(&vpI);
      vpI0->imageData = NULL;
      cvReleaseImageHeader(&vpI0);
#else
      for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale) {
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale) {
          (*I)[k][l] = _I[ii][jj];
        }
      }
#endif
      _pyramid[i] = I;
    } else {
      _pyramid[i] = NULL;
    }
  }
}

/*!
  Clean the pyramid of image allocated with the initPyramid() method. The
  vector has a size equal to zero at the end of the method.

  \param _pyramid : The pyramid of image to clean.
*/
void vpMbEdgeTracker::cleanPyramid(std::vector<const vpImage<unsigned char> *> &_pyramid)
{
  if (_pyramid.size() > 0) {
    _pyramid[0] = NULL;
    for (unsigned int i = 1; i < _pyramid.size(); i += 1) {
      if (_pyramid[i] != NULL) {
        delete _pyramid[i];
        _pyramid[i] = NULL;
      }
    }
    _pyramid.resize(0);
  }
}

/*!
  Get the list of the lines tracked for the specified level. Each line
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param level : Level corresponding to the list to return.
  \param linesList : The list of the lines of the model.
*/
void vpMbEdgeTracker::getLline(std::list<vpMbtDistanceLine *> &linesList, const unsigned int level) const
{
  if (level > scales.size() || !scales[level]) {
    std::ostringstream oss;
    oss << level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  linesList = lines[level];
}

/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param level : Level corresponding to the list to return.
  \param cylindersList : The list of the cylinders of the model.
*/
void vpMbEdgeTracker::getLcylinder(std::list<vpMbtDistanceCylinder *> &cylindersList, const unsigned int level) const
{
  if (level > scales.size() || !scales[level]) {
    std::ostringstream oss;
    oss << level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  cylindersList = cylinders[level];
}

/*!
  Get the list of the circles tracked for the specified level. Each circle
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not
  correspond to an used level.

  \param level : Level corresponding to the list to return.
  \param circlesList : The list of the circles of the model.
*/
void vpMbEdgeTracker::getLcircle(std::list<vpMbtDistanceCircle *> &circlesList, const unsigned int level) const
{
  if (level > scales.size() || !scales[level]) {
    std::ostringstream oss;
    oss << level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  circlesList = circles[level];
}

/*!
  Modify the camera parameters to have them corresponding to the current
  scale. The new parameters are divided by \f$ 2^{\_scale} \f$.

  \param _scale : Scale to use.
*/
void vpMbEdgeTracker::downScale(const unsigned int _scale)
{
  const double ratio = pow(2., (int)_scale);
  scaleLevel = _scale;

  vpMatrix K = cam.get_K();

  K[0][0] /= ratio;
  K[1][1] /= ratio;
  K[0][2] /= ratio;
  K[1][2] /= ratio;

  cam.initFromCalibrationMatrix(K);
}

/*!
  Modify the camera parameters to have them corresponding to the current
  scale. The new parameters are multiplied by \f$ 2^{\_scale} \f$.

  \param _scale : Scale to use.
*/
void vpMbEdgeTracker::upScale(const unsigned int _scale)
{
  const double ratio = pow(2., (int)_scale);
  scaleLevel = 0;

  vpMatrix K = cam.get_K();

  K[0][0] *= ratio;
  K[1][1] *= ratio;
  K[0][2] *= ratio;
  K[1][2] *= ratio;

  cam.initFromCalibrationMatrix(K);
}

/*!
  Re initialize the moving edges associated to a given level. This method is
  used to re-initialize the level if the tracking failed on this level but
  succeeded on the other one.

  \param _lvl : The level to re-initialize.
*/
void vpMbEdgeTracker::reInitLevel(const unsigned int _lvl)
{
  unsigned int scaleLevel_1 = scaleLevel;
  scaleLevel = _lvl;

  vpMbtDistanceLine *l;
  for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[scaleLevel].begin(); it != lines[scaleLevel].end();
       ++it) {
    if ((*it)->isTracked()) {
      l = *it;
      l->reinitMovingEdge(*Ipyramid[_lvl], cMo, m_mask);
    }
  }

  vpMbtDistanceCylinder *cy;
  for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[scaleLevel].begin();
       it != cylinders[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      cy = *it;
      cy->reinitMovingEdge(*Ipyramid[_lvl], cMo, m_mask);
    }
  }

  vpMbtDistanceCircle *ci;
  for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[scaleLevel].begin();
       it != circles[scaleLevel].end(); ++it) {
    if ((*it)->isTracked()) {
      ci = *it;
      ci->reinitMovingEdge(*Ipyramid[_lvl], cMo, m_mask);
    }
  }

  trackMovingEdge(*Ipyramid[_lvl]);
  updateMovingEdge(*Ipyramid[_lvl]);
  scaleLevel = scaleLevel_1;
}

/*!
  Set if the polygons that have the given name have to be considered during
  the tracking phase.

  \param name : name of the polygon(s).
  \param useEdgeTracking : True if it has to be considered, False otherwise.
*/
void vpMbEdgeTracker::setUseEdgeTracking(const std::string &name, const bool &useEdgeTracking)
{
  for (unsigned int i = 0; i < scales.size(); i += 1) {
    if (scales[i]) {
      for (std::list<vpMbtDistanceLine *>::const_iterator it = lines[i].begin(); it != lines[i].end(); ++it) {
        /*(*it)->setTracked(useEdgeTracking);
        for(std::list<int>::const_iterator
        itpoly=(*it)->Lindex_polygon.begin();
        itpoly!=(*it)->Lindex_polygon.end(); ++itpoly){
          if(faces[(*itpoly)]->getName() != name){
            (*it)->setTracked(true);
            break;
          }
        }*/

        (*it)->setTracked(name, useEdgeTracking);
      }

      for (std::list<vpMbtDistanceCylinder *>::const_iterator it = cylinders[i].begin(); it != cylinders[i].end();
           ++it) {
        if (faces[(unsigned)(*it)->index_polygon]->getName() == name) {
          (*it)->setTracked(useEdgeTracking);
        }
      }

      for (std::list<vpMbtDistanceCircle *>::const_iterator it = circles[i].begin(); it != circles[i].end(); ++it) {
        if (faces[(unsigned)(*it)->index_polygon]->getName() == name) {
          (*it)->setTracked(useEdgeTracking);
        }
      }
    }
  }
}
