/****************************************************************************
 *
 * $Id: vpMbEdgeTracker.cpp 2807 2010-09-14 10:14:54Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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



#include <visp/vpDebug.h>
#include <visp/vpPose.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpImageIo.h>
#include <visp/vpRobust.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpMatrixException.h>
#include <visp/vpMath.h>
#include <visp/vpException.h>
#include <visp/vpTrackingException.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpMbtDistanceLine.h>
#include <visp/vpMbtXmlParser.h>

#include <limits>
#include <string>
#include <sstream>
#include <float.h>

/*!
  Basic constructor
*/
vpMbEdgeTracker::vpMbEdgeTracker()
{
  index_polygon =0;
  compute_interaction=1;
  nline = 0;
  ncylinder = 0;
  lambda = 1;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
  displayMe = false;
  computeCovariance = false;

  lines.resize(1);
  cylinders.resize(1);
  scales.resize(1);
  scales[0] = true;
  lines[0].clear();
  cylinders[0].clear();
  Ipyramid.resize(0);
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbEdgeTracker::~vpMbEdgeTracker()
{
  vpMbtDistanceLine *l ;
  vpMbtDistanceCylinder *c;
  
  for (unsigned int i = 0; i < lines.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        l = *it;
        if (l!=NULL){
          delete l ;
        }
        l = NULL ;
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        c = *it;
        if (c!=NULL){
          delete c ;
        }
        c = NULL ;
      }

      lines[i].clear();
      cylinders[i].clear();
    }
  }
  cleanPyramid(Ipyramid);
}

/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbEdgeTracker::setMovingEdge(const vpMe &_me)
{
  this->me = _me;

  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        (*it)->setMovingEdge(&me) ;
      }

      vpMbtDistanceCylinder *cy;
      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        cy = *it;
        cy->setMovingEdge(&me) ;
      }
    }
  }
}


/*!
  Compute the visual servoing loop to get the pose of the feature set.
  
  \exception vpTrackingException::notEnoughPointError if the number of detected 
  feature is equal to zero. 
  
  \param _I : The current image. 
 */
void
vpMbEdgeTracker::computeVVS(const vpImage<unsigned char>& _I)
{
  double residu_1 =1e3;
  double r =1e3-1;
  vpMatrix LTL;
  vpColVector LTR;

  // compute the interaction matrix and its pseudo inverse
  vpMbtDistanceLine *l ;
  vpMbtDistanceCylinder *cy ;

  vpColVector w;
  vpColVector weighted_error;
  vpColVector factor;

  unsigned int iter = 0;

  //Nombre de moving edges
  unsigned int nbrow  = 0;
  unsigned int nberrors_lines = 0;
  unsigned int nberrors_cylinders = 0;

  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    nbrow += l->nbFeature ;
    nberrors_lines+=l->nbFeature;
    l->initInteractionMatrixError() ;
  }

  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    nbrow += cy->nbFeature ;
    nberrors_cylinders += cy->nbFeature ;
    cy->initInteractionMatrixError() ;
  }
  
  if (nbrow==0){
    vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
  }
  
  vpMatrix L(nbrow,6), Lp;

  // compute the error vector
  vpColVector error(nbrow);
  unsigned int nerror = error.getRows();
  vpColVector v ;

  double limite = 3; //Une limite de 3 pixels
  limite = limite / cam.get_px(); //Transformation limite pixel en limite metre.
  
  //Parametre pour la premiere phase d'asservissement
  double e_prev = 0, e_cur, e_next;
  bool reloop = true;
  double count = 0;
  
  /*** First phase ***/

  while ( reloop == true && iter<10)
  {
    if(iter==0)
    {
      weighted_error.resize(nerror) ;
      w.resize(nerror);
      w = 0;
      factor.resize(nerror);
      factor = 1;
    }
    
    count = 0;

    unsigned int n = 0;
    reloop = false;
    for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
      l = *it;
      l->computeInteractionMatrixError(cMo);
      
      double fac = 1;
      if (iter == 0)
      {
        for(std::list<int>::const_iterator it = l->Lindex_polygon.begin(); it!=l->Lindex_polygon.end(); ++it){
          int index = *it;
          if (l->hiddenface->isAppearing(index))
          {
            fac = 0.2;
            break;
          }
          if(l->closeToImageBorder(_I, 10))
          {
            fac = 0.1;
            break;
          }
        }
      }
      
      std::list<vpMeSite>::const_iterator itListLine;
      if (iter == 0 && l->meline != NULL)
        itListLine = l->meline->getMeList().begin();
      
      for (unsigned int i=0 ; i < l->nbFeature ; i++)
      {
        for (unsigned int j=0; j < 6 ; j++)
        {
          L[n+i][j] = l->L[i][j]; //On remplit la matrice d'interaction globale
        }
        error[n+i] = l->error[i]; //On remplit la matrice d'erreur

        if (error[n+i] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n+i] = 0;

        if (iter == 0)
        {
          factor[n+i] = fac;
          vpMeSite site = *itListLine;
          if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
          ++itListLine;
        }

        //If pour la premiere extremite des moving edges
        if (i == 0)
        {
          e_cur = l->error[0];
          if (l->nbFeature > 1)
          {
            e_next = l->error[1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
            }
            e_prev = e_cur;
          }
          else w[n+i] = 1;
        }

        //If pour la derniere extremite des moving edges
        else if(i == l->nbFeature-1)
        {
          e_cur = l->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          }
        }

        else
        {
          e_cur = l->error[i];
          e_next = l->error[i+1];
          if ( fabs(e_cur - e_prev) < limite )
          {
            w[n+i] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite )
          {
            w[n+i] += 0.5;
          }
          e_prev = e_cur;
        }
      }
      
      n+= l->nbFeature ;
    }


    for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
      cy = *it;
      cy->computeInteractionMatrixError(cMo, _I);
      double fac = 0.2;

      std::list<vpMeSite>::const_iterator itCyl1;
      std::list<vpMeSite>::const_iterator itCyl2;
      if (iter == 0 && (cy->meline1 != NULL || cy->meline2 != NULL)){
        itCyl1 = cy->meline1->getMeList().begin();
        itCyl2 = cy->meline2->getMeList().begin();
      }

      for(unsigned int i=0 ; i < cy->nbFeature ; i++){
        for(unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = cy->L[i][j]; //On remplit la matrice d'interaction globale
        }
        error[n+i] = cy->error[i]; //On remplit la matrice d'erreur

        if (error[n+i] <= limite) count = count+1.0; //Si erreur proche de 0 on incremente cur

        w[n+i] = 0;

        if (iter == 0)
        {
          factor[n+i] = fac;
          vpMeSite site;
          if(i<cy->nbFeaturel1) {
            site= *itCyl1;
            ++itCyl1;
          }
          else{
            site= *itCyl2;
            ++itCyl2;
          }
          if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
        }

        //If pour la premiere extremite des moving edges
        if (i == 0)
        {
          e_cur = cy->error[0];
          if (cy->nbFeature > 1)
          {
            e_next = cy->error[1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
            }
            e_prev = e_cur;
          }
          else w[n+i] = 1;
        }
        if (i == cy->nbFeaturel1)
        {
          e_cur = cy->error[i];
          if (cy->nbFeaturel2 > 1)
          {
            e_next = cy->error[i+1];
            if ( fabs(e_cur - e_next) < limite && vpMath::sign(e_cur) == vpMath::sign(e_next) )
            {
              w[n+i] = 1/*0.5*/;
            }
            e_prev = e_cur;
          }
          else w[n+i] = 1;
        }

        //If pour la derniere extremite des moving edges
        else if(i == cy->nbFeaturel1-1)
        {
          e_cur = cy->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          }
        }
        //If pour la derniere extremite des moving edges
        else if(i == cy->nbFeature-1)
        {
          e_cur = cy->error[i];
          if ( fabs(e_cur - e_prev) < limite && vpMath::sign(e_cur) == vpMath::sign(e_prev) )
          {
            w[n+i] += 1/*0.5*/;
          }
        }

        else
        {
          e_cur = cy->error[i];
          e_next = cy->error[i+1];
          if ( fabs(e_cur - e_prev) < limite ){
            w[n+i] += 0.5;
          }
          if ( fabs(e_cur - e_next) < limite ){
            w[n+i] += 0.5;
          }
          e_prev = e_cur;
        }
      }

      n+= cy->nbFeature ;
    }
    
    count = count / (double)nbrow;
    if (count < 0.85){
      reloop = true;
    }

    double num=0;
    double den=0;

    double wi ; double eri ;
    for(unsigned int i = 0; i < nerror; i++){
      wi = w[i]*factor[i];
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi ;

      weighted_error[i] =  wi*eri ;
    }

    if((iter==0) || compute_interaction){
      for (unsigned int i=0 ; i < nerror ; i++){
        for (unsigned int j=0 ; j < 6 ; j++){
          L[i][j] = w[i]*factor[i]*L[i][j] ;
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -0.7*LTL.pseudoInverse(LTL.getRows()*DBL_EPSILON)*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
//   cout << "\t First minimization in " << iter << " iteration " << endl ;
  
/*** Second phase ***/

  vpRobust robust_lines(nberrors_lines);
  vpRobust robust_cylinders(nberrors_cylinders);
  robust_lines.setIteration(0) ;
  robust_cylinders.setIteration(0) ;
  iter = 0;
  vpColVector w_lines(nberrors_lines);
  vpColVector w_cylinders(nberrors_cylinders);
  vpColVector error_lines(nberrors_lines);
  vpColVector error_cylinders(nberrors_cylinders);

  vpColVector error_vec;
  vpColVector W_true;
  vpMatrix L_true;
  
  while ( ((int)((residu_1 - r)*1e8) !=0 )  && (iter<30))
  {
    unsigned int n = 0 ;
    unsigned int nlines = 0;
    unsigned int ncylinders = 0;
    for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
      l = *it;
      l->computeInteractionMatrixError(cMo) ;
      for (unsigned int i=0 ; i < l->nbFeature ; i++){
        for (unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = l->L[i][j];
          error[n+i] = l->error[i];
          error_lines[nlines+i] = error[n+i];
        }
      }
      n+= l->nbFeature;
      nlines+= l->nbFeature;
    }

    for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
      cy = *it;
      cy->computeInteractionMatrixError(cMo, _I) ;
      for(unsigned int i=0 ; i < cy->nbFeature ; i++){
        for(unsigned int j=0; j < 6 ; j++){
          L[n+i][j] = cy->L[i][j];
          error[n+i] = cy->error[i];
          error_cylinders[ncylinders+i] = error[n+i];
        }
      }

      n+= cy->nbFeature ;
      ncylinders+= cy->nbFeature ;
    }
    
    if(iter==0)
    {
      weighted_error.resize(nerror);
      w.resize(nerror);
      w = 1;
      w_lines.resize(nberrors_lines);
      w_lines = 1;
      w_cylinders.resize(nberrors_cylinders);
      w_cylinders = 1;

      robust_lines.setThreshold(2/cam.get_px());
      robust_cylinders.setThreshold(2/cam.get_px());
			if(nberrors_lines > 0)
				robust_lines.MEstimator(vpRobust::TUKEY, error_lines,w_lines);
      if(nberrors_cylinders > 0){
       robust_cylinders.MEstimator(vpRobust::TUKEY, error_cylinders,w_cylinders);
      }
    }
    else
    {
      robust_lines.setIteration(iter);
      robust_cylinders.setIteration(iter);
			if(nberrors_lines > 0)
				robust_lines.MEstimator(vpRobust::TUKEY, error_lines, w_lines);
      if(nberrors_cylinders > 0){
        robust_cylinders.MEstimator(vpRobust::TUKEY, error_cylinders,w_cylinders);
      }
    }

    unsigned int cpt = 0;
    while(cpt<nbrow){
      if(cpt<nberrors_lines){
        w[cpt] = w_lines[cpt];
      }
      else{
        w[cpt] = w_cylinders[cpt-nberrors_lines];
      }
      cpt++;
    }

    residu_1 = r;

    double num=0;
    double den=0;
    double wi;
    double eri;
    
    L_true = L;
    W_true = vpColVector(nerror);
    
    for(unsigned int i=0; i<nerror; i++){
      wi = w[i]*factor[i];
      W_true[i] = wi*wi;
      eri = error[i];
      num += wi*vpMath::sqr(eri);
      den += wi;

      weighted_error[i] =  wi*eri ;
    }
    
    r = sqrt(num/den); //Le critere d'arret prend en compte le poids

    if((iter==0)|| compute_interaction){
      for (unsigned int i=0 ; i < nerror ; i++){
        for (unsigned int j=0 ; j < 6 ; j++){
          L[i][j] = w[i]*factor[i]*L[i][j];
        }
      }
    }

    LTL = L.AtA();
    computeJTR(L, weighted_error, LTR);
    v = -lambda*LTL.pseudoInverse(LTL.getRows()*DBL_EPSILON)*LTR;
    cMo =  vpExponentialMap::direct(v).inverse() * cMo;

    iter++;
  }
  
  vpMatrix D; //Should be the M.diag(wi) * M.diag(wi).transpose() =  (M.diag(wi^2))  which is more efficient
  D.diag(W_true);
  
  if(computeCovariance){
    covarianceMatrix = vpMatrix::computeCovarianceMatrix(L_true,v,-lambda*error,D);
  }
  
  unsigned int n =0 ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    {
      double wmean = 0 ;
      std::list<vpMeSite>::iterator itListLine;
      if (l->nbFeature > 0) itListLine = l->meline->getMeList().begin();
      
      for (unsigned int i=0 ; i < l->nbFeature ; i++){
        wmean += w[n+i] ;
        vpMeSite p = *itListLine;
        if (w[n+i] < 0.5){
          p.setState(vpMeSite::M_ESTIMATOR);
          
          *itListLine = p;
        }

        ++itListLine;
      }
      n+= l->nbFeature ;
      
      if (l->nbFeature!=0)
        wmean /= l->nbFeature ;
      else
        wmean = 1;
            
      l->setMeanWeight(wmean);

      if (wmean < 0.8)
        l->Reinit = true;
    }
  }


  // Same thing with cylinders as with lines
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    double wmean = 0 ;
    std::list<vpMeSite>::iterator itListCyl1;
    std::list<vpMeSite>::iterator itListCyl2;
    if (cy->nbFeature > 0){
      itListCyl1 = cy->meline1->getMeList().begin();
      itListCyl2 = cy->meline2->getMeList().begin();
    }

    wmean = 0;
    for(unsigned int i=0 ; i < cy->nbFeaturel1 ; i++){
      wmean += w[n+i] ;
      vpMeSite p = *itListCyl1;
      if (w[n+i] < 0.5){
        p.setState(vpMeSite::M_ESTIMATOR);
          
        *itListCyl1 = p;
      }

      ++itListCyl1;
    }

    if (cy->nbFeaturel1!=0)
      wmean /= cy->nbFeaturel1 ;
    else
      wmean = 1;

    cy->setMeanWeight1(wmean);

    if (wmean < 0.8){
      cy->Reinit = true;
    }

    wmean = 0;
    for(unsigned int i=cy->nbFeaturel1 ; i < cy->nbFeature ; i++){
      wmean += w[n+i] ;
      vpMeSite p = *itListCyl2;
      if (w[n+i] < 0.5){
        p.setState(vpMeSite::M_ESTIMATOR);
        
        *itListCyl2 = p;
      }

      ++itListCyl2;
    }

    if (cy->nbFeaturel2!=0)
      wmean /= cy->nbFeaturel2 ;
    else
      wmean = 1;

    cy->setMeanWeight2(wmean);

    if (wmean < 0.8){
      cy->Reinit = true;
    }

    n+= cy->nbFeature ;
  }
}

/*!
  Check if the tracking failed.
  
  \throw vpTrackingException::fatalError if the test fails. 
*/
void
vpMbEdgeTracker::testTracking()
{
  int nbExpectedPoint = 0;
  int nbGoodPoint = 0;
  int nbBadPoint = 0;
  
  vpMbtDistanceLine *l ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    if (l->isVisible() && l->meline != NULL)
    {
      nbExpectedPoint += (int)l->meline->expecteddensity;
      for(std::list<vpMeSite>::const_iterator it=l->meline->getMeList().begin(); it!=l->meline->getMeList().end(); ++it){
        vpMeSite pix = *it;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoint++;
        else nbBadPoint++;
      }
    }
  }    


  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    if (cy->meline1 !=NULL && cy->meline2 != NULL)
    {
      nbExpectedPoint += (int)cy->meline1->expecteddensity;
      for(std::list<vpMeSite>::const_iterator it=cy->meline1->getMeList().begin(); it!=cy->meline1->getMeList().end(); ++it){
        vpMeSite pix = *it;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoint++;
        else nbBadPoint++;
      }
      nbExpectedPoint += (int)cy->meline2->expecteddensity;
      for(std::list<vpMeSite>::const_iterator it=cy->meline2->getMeList().begin(); it!=cy->meline2->getMeList().end(); ++it){
        vpMeSite pix = *it;
        if (pix.getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoint++;
        else nbBadPoint++;
      }
    }
  }

  if (nbGoodPoint < percentageGdPt *(nbGoodPoint+nbBadPoint) || nbExpectedPoint < 2)
  {
    std::cout << "nbGoodPoint :" << nbGoodPoint << std::endl;
    std::cout << "nbBadPoint :" << nbBadPoint << std::endl;
    std::cout << "nbExpectedPoint :" << nbExpectedPoint << std::endl;
    std::cout << "percentageGdPt: " << percentageGdPt << std::endl;
    throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
  }      
}


/*!
  Compute each state of the tracking procedure for all the feature sets.
  
  If the tracking is considered as failed an exception is thrown.
  
  \param I : The image.
 */
void
vpMbEdgeTracker::track(const vpImage<unsigned char> &I)
{
  initPyramid(I, Ipyramid);
  
//  for (int lvl = ((int)scales.size()-1); lvl >= 0; lvl -= 1)
  unsigned int lvl = scales.size();
  do{
    lvl--;
    if(scales[lvl]){
      vpHomogeneousMatrix cMo_1 = cMo;
      try
      {
        downScale(lvl);

        try
        {  
          trackMovingEdge(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge tracking") ;
          throw ;
        }

        // initialize the vector that contains the error and the matrix that contains
        // the interaction matrix
        vpMbtDistanceLine *l ;
        for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
          l = *it;
          if (l->isVisible()){
            l->initInteractionMatrixError() ;
          }
        }  

        vpMbtDistanceCylinder *cy ;
        for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
          cy = *it;
          cy->initInteractionMatrixError();
        }

        try
        {
          computeVVS(*Ipyramid[lvl]);
        }
        catch(...)
        {
          vpTRACE("Error in computeVVS") ;
          throw vpException(vpException::fatalError, "Error in computeVVS");
        }
        
        try
        {
          testTracking();
        }
        catch(...)
        {
          throw vpTrackingException(vpTrackingException::fatalError, "test Tracking fail");
        }
        
        if (displayMe)
        {
          if(lvl == 0){
            for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
              l = *it;
              if (l->isVisible()){
                l->displayMovingEdges(I);
              }
            }

            for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
              cy = *it;
              cy->displayMovingEdges(I);
            }
          }
        }
        
        try
        {
          updateMovingEdge(I);
        }
        catch(...)
        {
          vpTRACE("Error in moving edge updating") ;
          throw ;
        }
        
        // Looking for new visible face
        bool newvisibleface = false ;
        visibleFace(cMo, newvisibleface) ;
        initMovingEdge(I,cMo) ;

        // Reinit the moving edge for the lines which need it.
        reinitMovingEdge(I,cMo);
        upScale(lvl);
      }
      catch(...)
      {
        if(lvl != 0){
          cMo = cMo_1;
          reInitLevel(lvl);
          upScale(lvl);
        }
        else{
          upScale(lvl);
          throw ;
        }
      }
    }
  } while(lvl != 0);

  cleanPyramid(Ipyramid);
}

/*!
 Initialize the tracking.
 
 \param I : The image.
*/
void vpMbEdgeTracker::init(const vpImage<unsigned char>& I)
{
	bool a = false;
  
  initPyramid(I, Ipyramid);
  visibleFace(cMo, a);
  unsigned int i=scales.size();
  do {
    i--;
    if(scales[i]){
      downScale(i);
      initMovingEdge(*Ipyramid[i], cMo);
      upScale(i);
    }
  } while(i != 0);
  
  cleanPyramid(Ipyramid);
}

/*!
  Load the xml configuration file.
  From the configuration file parameters write initialize the corresponding objects (Ecm, camera).

  \param _filename : full name of the xml file.
*/
void 
vpMbEdgeTracker::loadConfigFile(const std::string& _filename)
{
  loadConfigFile(_filename.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file parameters write initialize the corresponding objects (Ecm, camera).

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param filename : full name of the xml file.
*/
void
vpMbEdgeTracker::loadConfigFile(const char* filename)
{
#ifdef VISP_HAVE_XML2
  vpMbtXmlParser xmlp;
  
  try{
    xmlp.parse(filename);
  }
  catch(...){
    vpERROR_TRACE("Can't open XML file \"%s\"\n ",filename);
    throw vpException(vpException::ioError, "problem to parse configuration file.");
  }

  vpCameraParameters camera;
  vpMe meParser;
  xmlp.getCameraParameters(camera);
  xmlp.getMe(meParser);
  setCameraParameters(camera);
  setMovingEdge(meParser);
#else
	vpTRACE("You need the libXML2 to read the config file %s", filename);
#endif
}


/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible surfaces).
*/
void
vpMbEdgeTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness, const bool displayFullModel)
{
  vpMbtDistanceLine *l ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        l = *it;
        l->display(I,_cMo, cam, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, _cMo, cam, col, thickness);
      }

      break ; //displaying model on one scale only
    }
  }
}

/*!
  Display the 3D model from a given position of the camera.

  \param I : The image.
  \param _cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : If true, the full model is displayed (even the non visible surfaces).
*/
void
vpMbEdgeTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters &cam,
											const vpColor& col,
											const unsigned int thickness, const bool displayFullModel)
{
  vpMbtDistanceLine *l ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        l = *it;
        l->display(I,_cMo, cam, col, thickness, displayFullModel) ;
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, _cMo, cam, col, thickness) ;
      }

      break ; //displaying model on one scale only
    }
  }
}


/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the lines.
  
  \param I : The image.
  \param _cMo : The pose of the camera used to initialize the moving edges.
*/
void
vpMbEdgeTracker::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtDistanceLine *l ;
  
  lines[scaleLevel].front() ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    bool isvisible = false ;

    for(std::list<int>::const_iterator it=l->Lindex_polygon.begin(); it!=l->Lindex_polygon.end(); ++it){
      int index = *it;
      if (index ==-1) isvisible =true ;
      else
      {
        if (l->hiddenface->isVisible(index)) isvisible = true ;
      }
    }

    //Si la ligne n'appartient a aucune face elle est tout le temps visible
    if (l->Lindex_polygon.empty()) isvisible = true;

    if (isvisible)
    {
      l->setVisible(true) ;
      if (l->meline==NULL)
      {
        //cout << "init me line "<< l->getIndex() <<endl ;
        l->initMovingEdge(I, _cMo) ;
      }
    }
    else
    {
      l->setVisible(false) ;
      if (l->meline!=NULL) delete l->meline;
      l->meline=NULL;
    }
  }   


  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    if (cy->meline1==NULL || cy->meline2==NULL){
      cy->initMovingEdge(I, _cMo) ;
    }
  }
}


/*!
  Track the moving edges in the image.
  
  \param I : the image.
*/
void
vpMbEdgeTracker::trackMovingEdge(const vpImage<unsigned char> &I)
{
  vpMbtDistanceLine *l ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    if(l->isVisible() == true){
      if(l->meline == NULL){
        l->initMovingEdge(I, cMo);
      }
      l->trackMovingEdge(I, cMo) ;
    }
  }

  vpMbtDistanceCylinder *cy;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    if(cy->meline1 == NULL || cy->meline2 == NULL){
      cy->initMovingEdge(I, cMo);
    }
    cy->trackMovingEdge(I, cMo) ;
  }
}


/*!
  Update the moving edges at the end of the virtual visual servoing.
  
  \param I : the image.
*/
void
vpMbEdgeTracker::updateMovingEdge(const vpImage<unsigned char> &I)
{
  vpMbtDistanceLine *l ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    l->updateMovingEdge(I, cMo) ;
    if (l->nbFeature == 0 && l->isVisible()){
      l->Reinit = true;
    }
  }


  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    cy->updateMovingEdge(I, cMo) ;
    if(cy->nbFeaturel1 == 0 || cy->nbFeaturel2 == 0){
      cy->Reinit = true;
    }
  }
}


/*!
  Reinitialize the lines if it is required.
  
  A line is reinitialized if the 2D line do not match enough with the projected 3D line.
  
  \param I : the image.
  \param _cMo : the pose of the used to re-initialise the moving edges
*/
void
vpMbEdgeTracker::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &_cMo)
{
  vpMbtDistanceLine *l ;  
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    if (l->Reinit && l->isVisible())
      l->reinitMovingEdge(I, _cMo);
  }

  vpMbtDistanceCylinder*cy;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    if (cy->Reinit)
      cy->reinitMovingEdge(I, _cMo);
  }
}


/*!
  Check if two vpPoints are similar.
  
  To be similar : \f$ (X_1 - X_2)^2 + (Y_1 - Y_2)^2 + (Z_1 - Z_2)^2 < threshold \f$.
  
  \param P1 : The first point to compare
  \param P2 : The second point to compare
  \param threshold : The threshold  used to decide if the points are similar or not.
*/
bool samePoint(const vpPoint &P1, const vpPoint &P2, double threshold=1e-5)
{
  double d = vpMath::sqr(P1.get_oX() - P2.get_oX())+
  vpMath::sqr(P1.get_oY() - P2.get_oY())+
  vpMath::sqr(P1.get_oZ() - P2.get_oZ()) ;
  if (d  < threshold)
    return true ;
  else
    return false ;
}


/*!
  Add a line belonging to the \f$ index \f$ th polygone to the list of lines. It is defined by its two extremities.
  
  If the line already exists, the ploygone's index is added to the list of polygon to which it belongs.
  
  \param P1 : The first extremity of the line.
  \param P2 : The second extremity of the line.
  \param polygone : The index of the polygon to which the line belongs.
  \param name : the optionnal name of the line 
*/
void
vpMbEdgeTracker::addLine(vpPoint &P1, vpPoint &P2, int polygone, std::string name)
{
  //suppress line already in the model
  
  bool already_here = false ;
  vpMbtDistanceLine *l ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      downScale(i);
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        l = *it;
        if((samePoint(*(l->p1),P1) && samePoint(*(l->p2),P2)) ||
           (samePoint(*(l->p1),P2) && samePoint(*(l->p2),P1)) ){
          already_here = true ;
          l->Lindex_polygon.push_back(polygone);
          l->hiddenface = &faces ;
        }
      }

      if (!already_here){
        l = new vpMbtDistanceLine ;

        l->setCameraParameters(cam) ;
        l->buildFrom(P1,P2) ;
        l->Lindex_polygon.push_back(polygone);
        l->setMovingEdge(&me) ;
        l->hiddenface = &faces ;
        l->setIndex(nline) ;
        l->setName(name);
        nline +=1 ;
        lines[i].push_back(l);
      }
      upScale(i);
    }
  }
}

/*!
  Remove a line using its name. 
  
  \param name : The name of the line to remove. 
*/
void
vpMbEdgeTracker::removeLine(const std::string& name)
{
  vpMbtDistanceLine *l;
  
  for(unsigned int i=0; i<scales.size(); i++){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        l = *it;
        if (name.compare(l->getName()) == 0){
          lines[i].erase(it);
          break;
        }
      }
    }
  }
}




/*!
  Add a cylinder to the list of cylinders.

  \param P1 : The first extremity of the axis.
  \param P2 : The second extremity of the axis.
  \param r : The radius of the cylinder.
  \param name : the optionnal name of the cylinder
*/
void
vpMbEdgeTracker::addCylinder(const vpPoint &P1, const vpPoint &P2, const double r, const std::string& name)
{
  bool already_here = false ;
  vpMbtDistanceCylinder *cy ;

  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      downScale(i);
      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        cy = *it;
        if((samePoint(*(cy->p1),P1) && samePoint(*(cy->p2),P2)) ||
           (samePoint(*(cy->p1),P2) && samePoint(*(cy->p2),P1)) ){
          already_here = (std::fabs(cy->radius - r) < std::numeric_limits<double>::epsilon() * vpMath::maximum(cy->radius, r));
        }
      }

      if (!already_here){
        cy = new vpMbtDistanceCylinder ;

        cy->setCameraParameters(cam);
        cy->buildFrom(P1, P2, r);
        cy->setMovingEdge(&me);
        cy->setIndex(ncylinder);
        cy->setName(name);
        ncylinder +=1;
        cylinders[i].push_back(cy);
      }
      upScale(i);
    }
  }
}


/*!
  Remove a cylinder by its name.

  \param name : The name of the cylinder to remove.
*/
void
vpMbEdgeTracker::removeCylinder(const std::string& name)
{
  vpMbtDistanceCylinder *cy;

  for(unsigned int i=0; i<scales.size(); i++){
    if(scales[i]){
      for(std::list<vpMbtDistanceCylinder*>::iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        cy = *it;
        if (name.compare(cy->getName()) == 0){
          cylinders[i].erase(it);
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
void
vpMbEdgeTracker::addPolygon(vpMbtPolygon &p)
{
  p.setIndex(index_polygon) ;
  faces.addPolygon(&p) ;

  unsigned int nbpt = p.getNbPoint() ;
  if(nbpt > 0){
    for (unsigned int i=0 ; i < nbpt-1 ; i++)
      addLine(p.p[i], p.p[i+1], index_polygon) ;
    addLine(p.p[nbpt-1], p.p[0], index_polygon) ;
  }
  
  index_polygon++ ;
}


/*!
  Detect the visible faces in the image and says if a new one appeared.
  
  \warning If in one iteration one face appears and one disappears, then the 
  function will not detect the new face. 
  
  \param cMo : The pose of the camera used to project the 3D model into the image.
  \param newvisibleline : This parameter is set to true if a new face appeared.
*/
void
vpMbEdgeTracker::visibleFace(const vpHomogeneousMatrix &cMo, bool &newvisibleline)
{
  unsigned int n ;

  n = faces.setVisible(cMo) ;
//  cout << "visible face " << n << endl ;
  if (n > nbvisiblepolygone)
  {
    //cout << "une nouvelle face est visible " << endl ;
    newvisibleline = true ;
  }
  else
    newvisibleline = false ;

  nbvisiblepolygone= n ;
}


/*!
  Load a 3D model contained in a file. This file is either a vrml or a cao file.
  
  \param file : Full name the file containing the 3D model description.
  The extension of this file is either .wrl or .cao.

  \sa vpMbTracker::loadModel()
*/
void
vpMbEdgeTracker::loadModel(const char* file)
{
  std::string model(file);
  vpMbTracker::loadModel(model);
}

/*!
  Load a 3D model contained in a file.
  
  \param file : Full name the file containing the 3D model description.
  The extension of this file is either .wrl or .cao.

  \sa vpMbTracker::loadModel()
*/
void
vpMbEdgeTracker::loadModel(const std::string &file)
{
  vpMbTracker::loadModel(file);
}

/*!
  Add a face to track from its corners (in the object frame). This method is
  called from the loadModel() one to add a face of the object to track. 
  The initialisation of the face depends on the primitive to track.
  
  \param _corners : The vector of corners representing the face.
  \param _indexFace : The index of the face.
*/
void 
vpMbEdgeTracker::initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace)
{
  vpMbtPolygon *polygon = NULL;
  polygon = new vpMbtPolygon;
  polygon->setNbPoint(_corners.size());
  polygon->setIndex((int)_indexFace);
  for(unsigned int j = 0; j < _corners.size(); j++) {
    polygon->addPoint(j, _corners[j]);
  }
  addPolygon(*polygon);

  delete polygon;
  polygon = NULL;
}

/*!
  Add a cylinder to track from tow points on the axis (defining the length of
  the cylinder) and its radius.

  \param _p1 : First point on the axis.
  \param _p2 : Second point on the axis.
  \param _radius : Radius of the cylinder.
  \param _indexCylinder : Index of the cylinder.
*/
void
vpMbEdgeTracker::initCylinder(const vpPoint& _p1, const vpPoint _p2, const double _radius, const unsigned int _indexCylinder)
{
  if(_indexCylinder != 0){
    ncylinder = _indexCylinder;
  }
  addCylinder(_p1, _p2, _radius);
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose. 
  
*/
void
vpMbEdgeTracker::resetTracker()
{
  this->cMo.setIdentity();
  vpMbtDistanceLine *l;
  vpMbtDistanceCylinder *cy;

  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[i].begin(); it!=lines[i].end(); ++it){
        l = *it;
        if (l!=NULL) delete l ;
        l = NULL ;
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[i].begin(); it!=cylinders[i].end(); ++it){
        cy = *it;
        if (cy!=NULL) delete cy;
        cy = NULL;
      }
      lines[i].clear();
      cylinders[i].clear();
    }
  }
  
  faces.reset();
  
  index_polygon =0;
  compute_interaction=1;
  nline = 0;
  ncylinder = 0;
  lambda = 1;
  nbvisiblepolygone = 0;
  percentageGdPt = 0.4;
  
  // reinitialisation of the scales.
  this->setScales(scales);
}



/*!
  Re-initialise the model used by the tracker.  
  
  \param _I : The image containing the object to initialize.
  \param _cad_name : Path to the file containing the 3D model description.
  \param _cMo : The new vpHomogeneousMatrix between the camera and the new model
*/
void
vpMbEdgeTracker::reInitModel(const vpImage<unsigned char>& _I, const char* _cad_name, const vpHomogeneousMatrix& _cMo)
{
  resetTracker();
  loadModel(_cad_name);
  initFromPose(_I, _cMo);
}

/*!
  Return the number of good points (vpMeSite) tracked. A good point is a 
  vpMeSite with its flag "suppress" equal to 0. Only these points are used 
  during the virtual visual servoing stage. 
  
  \exception vpException::dimensionError if _level does not represent a used 
  level.
  
  \return the number of good points. 
*/
unsigned int 
vpMbEdgeTracker::getNbPoints(const unsigned int _level)
{
  if((_level > scales.size()) || !scales[_level]){
    throw vpException(vpException::dimensionError, "Level is not used");
  }
  
  unsigned int nbGoodPoints = 0;
  vpMbtDistanceLine *l ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[_level].begin(); it!=lines[_level].end(); ++it){
    l = *it;
    if (l->isVisible() && l->meline != NULL)
    {
      for(std::list<vpMeSite>::const_iterator it=l->meline->getMeList().begin(); it!=l->meline->getMeList().end(); ++it){
        if (it->getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoints++;
      }
    }
  }

  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[_level].begin(); it!=cylinders[_level].end(); ++it){
    cy = *it;
    if (cy->meline1 != NULL || cy->meline2 != NULL)
    {
      for(std::list<vpMeSite>::const_iterator it=cy->meline1->getMeList().begin(); it!=cy->meline1->getMeList().end(); ++it){
        if (it->getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoints++;
      }
      for(std::list<vpMeSite>::const_iterator it=cy->meline2->getMeList().begin(); it!=cy->meline2->getMeList().end(); ++it){
        if (it->getState() == vpMeSite::NO_SUPPRESSION) nbGoodPoints++;
      }
    }
  }

  return nbGoodPoints;
}


/*!
  Return the polygon (face) "index".
  
  \exception vpException::dimensionError if index does not represent a good 
  polygon.
  
  \param _index : Index of the polygon to return.
  \return Pointer to the polygon index.
*/
vpMbtPolygon* 
vpMbEdgeTracker::getPolygon(const unsigned int _index)
{
  if(_index >= static_cast<unsigned int>(faces.getPolygon().size()) ){
    throw vpException(vpException::dimensionError, "index out of range");
  }
  std::list<vpMbtPolygon*>::const_iterator it = faces.getPolygon().begin();
  std::advance(it, _index);
  return *it;
}

/*!
  Get the number of polygon (face) representing the object to track.
  
  \return Number of polygon.
*/
unsigned int 
vpMbEdgeTracker::getNbPolygon() 
{
  return static_cast<unsigned int>(faces.getPolygon().size());
}

/*!
  Set the scales to use to realise the tracking. The vector of boolean activates
  or not the scales to se for the object tracking. The first element of the list
  correspond to the tracking on the full image, the second element corresponds 
  to the tracking on an image sbsampled by two. 
  
  Using multi scale tracking allows to track the object with greater moves. It 
  requires the computation of a pyramid of images, but the total tracking can be
  faster than a tracking based only on the full scale. The pose is computed from
  the smallest image to the biggest. This may be dangerous if the object to 
  track is small in the image, because the subsampled scale(s) will have only 
  few points to compute the pose (it could result in a loss of precision). 
  
  \warning This method must be used before the tracker has been initialised (
  before the call of the loadConfigFile() or loadModel() methods). 
  
  \warning At least one level must be activated. 
  
  \param _scales : The vector describing the levels to use. 
*/
void 
vpMbEdgeTracker::setScales(const std::vector<bool>& _scales)
{
  unsigned int nbActivatedLevels = 0;
  for (unsigned int i = 0; i < _scales.size(); i += 1){
    if(_scales[i]){
      nbActivatedLevels++;
    }
  }
  if((_scales.size() < 1) || (nbActivatedLevels == 0)){
    vpERROR_TRACE(" !! WARNING : must use at least one level for the tracking. Use the global one");
    scales.resize(0);
    scales.push_back(true);
    lines.resize(1);
    lines[0].clear();
    cylinders.resize(1);
    cylinders[0].clear();
  }
  else{
    scales = _scales;
    lines.resize(_scales.size());
    cylinders.resize(_scales.size());
    for (unsigned int i = 0; i < lines.size(); i += 1){
      lines[i].clear();
      cylinders[i].clear();
    }
  }
}

/*!
  Compute the pyramid of image associated to the image in parameter. The scales 
  computed are the ones corresponding to the scales  attribte of the class. If 
  OpenCV is detected, the functions used to computed a smoothed pyramid come 
  from OpenCV, otherwise a simple subsampling (no smoothing, no interpolation) 
  is realised. 
  
  \warning The pyramid contains pointers to vpImage. To properly deallocate the
  pyramid. All the element but the first (which is a pointer to the input image)
  must be freed. A proper cleaning is implemented in the cleanPyramid() method. 
  
  \param _I : The input image.
  \param _pyramid : The pyramid of image to build from the input image.
*/
void 
vpMbEdgeTracker::initPyramid(const vpImage<unsigned char>& _I, std::vector< const vpImage<unsigned char>* >& _pyramid)
{
  _pyramid.resize(scales.size());
  
  if(scales[0]){
    _pyramid[0] = &_I;
  }
  else{
    _pyramid[0] = NULL;
  }
  
  for(unsigned int i=1; i<_pyramid.size(); i += 1){
    if(scales[i]){
      unsigned int cScale = static_cast<unsigned int>(pow(2., (int)i));
      vpImage<unsigned char>* I = new vpImage<unsigned char>(_I.getHeight() / cScale, _I.getWidth() / cScale);
#ifdef VISP_HAVE_OPENCV
      IplImage* vpI0 = cvCreateImageHeader(cvSize((int)_I.getWidth(), (int)_I.getHeight()), IPL_DEPTH_8U, 1);
      vpI0->imageData = (char*)(_I.bitmap);
      IplImage* vpI = cvCreateImage(cvSize((int)(_I.getWidth() / cScale), (int)(_I.getHeight() / cScale)), IPL_DEPTH_8U, 1);
      cvResize(vpI0, vpI, CV_INTER_NN);
      vpImageConvert::convert(vpI, *I);
      cvReleaseImage(&vpI);  
      vpI0->imageData = NULL;
      cvReleaseImageHeader(&vpI0);    
#else
      for (unsigned int k = 0, ii = 0; k < I->getHeight(); k += 1, ii += cScale){
        for (unsigned int l = 0, jj = 0; l < I->getWidth(); l += 1, jj += cScale){
          (*I)[k][l] = _I[ii][jj];
        }
      }
#endif   
      _pyramid[i] = I;
    }
    else{
      _pyramid[i] = NULL;
    }
  }
}

/*!
  Clean the pyramid of image allocated with the initPyramid() method. The vector
  has a size equal to zero at the end of the method. 
  
  \param _pyramid : The pyramid of image to clean.
*/
void 
vpMbEdgeTracker::cleanPyramid(std::vector< const vpImage<unsigned char>* >& _pyramid)
{
  if(_pyramid.size() > 0){
    _pyramid[0] = NULL;
    for (unsigned int i = 1; i < _pyramid.size(); i += 1){
      if(_pyramid[i] != NULL){
        delete _pyramid[i];
        _pyramid[i] = NULL;
      }
    }
    _pyramid.resize(0);
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  Get the list of the lines tracked for the specified level. Each line contains 
  the list of the vpMeSite. 
  
  \throw vpException::dimensionError if the parameter does not correspond to an 
  used level. 

  \warning The returned list is dynamically allocated and must be freed.
  
  \param _level : Level corresponding to the list to return. 
  \return Pointer to the list of the lines tracked. 
*/
vp_deprecated vpList<vpMbtDistanceLine *>*
vpMbEdgeTracker::getLline(const unsigned int _level)
{
  if(_level > scales.size() || !scales[_level]){
    std::ostringstream oss;
    oss << _level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";    
    throw vpException(vpException::dimensionError, errorMsg);
  }
  
  vpList<vpMbtDistanceLine *>* l = new vpList<vpMbtDistanceLine*>;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[_level].begin(); it!=lines[_level].end(); ++it){
    l->addRight(*it);
  }

  return l;
}


/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the parameter does not correspond to an
  used level.

  \warning The returned list is dynamically allocated and must be freed.

  \param _level : Level corresponding to the list to return.
  \return Pointer to the list of the cylinders tracked.
*/
vp_deprecated vpList<vpMbtDistanceCylinder *>*
vpMbEdgeTracker::getLcylinder(const unsigned int _level)
{
  if(_level > scales.size() || !scales[_level]){
    std::ostringstream oss;
    oss << _level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  vpList<vpMbtDistanceCylinder*>* c = new vpList<vpMbtDistanceCylinder*>;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[_level].begin(); it!=cylinders[_level].end(); ++it){
    c->addRight(*it);
  }

  return c;
}

/*!
 Initialize the tracking thanks to the initial pose of the camera.
 
 \param I : The image.
 \param _cMo : The initial pose used to initialize the tracking.
*/
vp_deprecated void
vpMbEdgeTracker::init(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &_cMo)
{
  this->cMo = _cMo;
  init(I);
}
#endif

/*!
  Get the list of the lines tracked for the specified level. Each line contains
  the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param _level : Level corresponding to the list to return.
  \param linesList : The list of the lines of the model.
*/
void
vpMbEdgeTracker::getLline(std::list<vpMbtDistanceLine *>& linesList, const unsigned int _level)
{
  if(_level > scales.size() || !scales[_level]){
    std::ostringstream oss;
    oss << _level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  linesList = lines[_level];
}


/*!
  Get the list of the cylinders tracked for the specified level. Each cylinder
  contains the list of the vpMeSite.

  \throw vpException::dimensionError if the second parameter does not correspond
  to an used level.

  \param _level : Level corresponding to the list to return.
  \param cylindersList : The list of the cylinders of the model.
*/
void
vpMbEdgeTracker::getLcylinder(std::list<vpMbtDistanceCylinder *>& cylindersList, const unsigned int _level)
{
  if(_level > scales.size() || !scales[_level]){
    std::ostringstream oss;
    oss << _level;
    std::string errorMsg = "level " + oss.str() + " is not used, cannot get its distance lines.";
    throw vpException(vpException::dimensionError, errorMsg);
  }

  cylindersList = cylinders[_level];
}


/*!
  Modify the camera parameters to have them corresponding to the current scale.
  The new parameters are divided by \f$ 2^{\_scale} \f$. 
  
  \param _scale : Scale to use. 
*/
void 
vpMbEdgeTracker::downScale(const unsigned int _scale)
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
  Modify the camera parameters to have them corresponding to the current scale.
  The new parameters are multiplied by \f$ 2^{\_scale} \f$. 
  
  \param _scale : Scale to use. 
*/
void 
vpMbEdgeTracker::upScale(const unsigned int _scale)
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
  Re initialise the moving edges associated to a given level. This method is 
  used to re-initialise the level if the tracking failed on this level but 
  succedded on the other one. 
  
  \param _lvl : The level to re-initialise.
*/
void 
vpMbEdgeTracker::reInitLevel(const unsigned int _lvl)
{
  unsigned int scaleLevel_1 = scaleLevel;
  scaleLevel = _lvl;

  vpMbtDistanceLine *l;  
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
    l = *it;
    l->reinitMovingEdge(*Ipyramid[_lvl], cMo);
  }


  vpMbtDistanceCylinder *cy;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
    cy = *it;
    cy->reinitMovingEdge(*Ipyramid[_lvl], cMo);
  }
  
  trackMovingEdge(*Ipyramid[_lvl]);
  updateMovingEdge(*Ipyramid[_lvl]);
  scaleLevel = scaleLevel_1;
}

