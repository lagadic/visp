/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Description:
 * Hybrid tracker based on edges (vpMbt) and points of interests (KLT)
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp/vpMbEdgeKltTracker.h>

#ifdef VISP_HAVE_OPENCV

vpMbEdgeKltTracker::vpMbEdgeKltTracker()
{
  compute_interaction = true;
  computeCovariance = false;
  
  lambda = 0.8;
  thresholdKLT = 2.0;
  thresholdMBT = 2.0;
  maxIter = 200;
  vpMbKltTracker::setMaxIter(30);
  
#ifdef VISP_HAVE_OGRE
  vpMbKltTracker::faces.getOgreContext()->setWindowName("MBT Hybrid");
#endif
}

/*!
  Basic constructor

*/
vpMbEdgeKltTracker::~vpMbEdgeKltTracker()
{
}

/*!
  Initialization of the tracker using a known initial pose.
  The 3D model must first have been loaded.

  \param I : Input image.
*/
void 
vpMbEdgeKltTracker::init(const vpImage<unsigned char>& I)
{
  vpMbKltTracker::init(I);
  
  initPyramid(I, Ipyramid);
  
  unsigned int n = 0;
  for(unsigned int i = 0; i < vpMbKltTracker::faces.size() ; i++){
      if(vpMbKltTracker::faces[i]->isVisible()){
        vpMbEdgeTracker::faces[i]->isvisible = true;
        n++;
      }
      else
        vpMbEdgeTracker::faces[i]->isvisible = false;
  }
  vpMbEdgeTracker::nbvisiblepolygone = n;
  
  unsigned int i = (unsigned int)scales.size();
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
  Set the pose to be used in entry of the next call to the track() function.
  This pose will be just used once.
  
  \warning This function has to be called after the initialization of the tracker.
  
  \param I : image corresponding to the desired pose.
  \param cdMo : Pose to affect.
*/
void           
vpMbEdgeKltTracker::setPose( const vpImage<unsigned char> &I, const vpHomogeneousMatrix& cdMo)
{
  if(firstTrack){
    vpMbKltTracker::setPose(I, cdMo);
    
    vpMbtDistanceLine *l;
    lines[scale Level].front() ;
    for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
      l = *it;
      if(l->meline != NULL){
        delete l->meline;
        l->meline = NULL;
      }
    }
    
    initPyramid(I, Ipyramid);
    
    unsigned int n = 0;
    for(unsigned int i = 0; i < vpMbKltTracker::faces.size() ; i++){
        if(vpMbKltTracker::faces[i]->isVisible()){
          vpMbEdgeTracker::faces[i]->isvisible = true;
          n++;
        }
        else
          vpMbEdgeTracker::faces[i]->isvisible = false;
    }
    vpMbEdgeTracker::nbvisiblepolygone = n;
    
    unsigned int i = (unsigned int)scales.size();
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
}

/*!
  Reset the tracker. The model is removed and the pose is set to identity.
  The tracker needs to be initialized with a new model and a new pose. 
*/
void    
vpMbEdgeKltTracker::resetTracker()
{
  vpMbEdgeTracker::resetTracker();
  vpMbKltTracker::resetTracker();
}

unsigned int
vpMbEdgeKltTracker::initMbtTracking(const unsigned int lvl)
{
  vpMbtDistanceLine *l ;
  vpMbtDistanceCylinder *cy ;

  if(lvl  >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "lvl not used.");
  }

  unsigned int nbrow  = 0;
  for(std::list<vpMbtDistanceLine*>::iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    l = *it;
    nbrow += l->nbFeature ;
    l->initInteractionMatrixError() ;
  }
  
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    cy = *it;
    nbrow += cy->nbFeature ;
    cy->initInteractionMatrixError() ;
  }
  
  return nbrow;  
}

/*!
  Load the xml configuration file. An example of such a file is provided in loadConfigFile(const char*) documentation.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges, KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param configFile : full name of the xml file.

  \sa loadConfigFile(const char*), vpXmlParser::cleanup()
*/
void 
vpMbEdgeKltTracker::loadConfigFile(const std::string& configFile)
{
  vpMbEdgeKltTracker::loadConfigFile(configFile.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file initialize the parameters corresponding to the objects: moving-edges, KLT, camera.

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param configFile : full name of the xml file.

  The XML configuration file has the following form:
  \code
<?xml version="1.0"?>
<conf>
  <ecm>
    <mask>
      <size>5</size>
      <nb_mask>180</nb_mask>
    </mask>
    <range>
      <tracking>10</tracking>
    </range>
    <contrast>
      <edge_threshold>7000</edge_threshold>
      <mu1>0.5</mu1>
      <mu2>0.5</mu2>
    </contrast>
  </ecm>
  <sample>
    <step>4</step>
    <nb_sample>250</nb_sample>
  </sample>
  <camera>
    <width>640</width>
    <height>480</height>
    <u0>320</u0>
    <v0>240</v0>
    <px>686.24</px>
    <py>686.24</py>
  </camera>
  <face>
    <angle_appear>65</angle_appear>
    <angle_disappear>85</angle_disappear>
  </face>
  <klt>
    <mask_border>10</mask_border>
    <max_features>10000</max_features>
    <window_size>5</window_size>
    <quality>0.02</quality>
    <min_distance>10</min_distance>
    <harris>0.02</harris>
    <size_block>3</size_block>
    <pyramid_lvl>3</pyramid_lvl>
  </klt>
</conf>
  \endcode

  \sa loadConfigFile(const std::string&), vpXmlParser::cleanup()
*/
void
vpMbEdgeKltTracker::loadConfigFile(const char* configFile)
{
#ifdef VISP_HAVE_XML2
  vpMbEdgeTracker::loadConfigFile(configFile);
  vpMbKltTracker::loadConfigFile(configFile);
#else
  vpTRACE("You need the libXML2 to read the config file %s", configFile);
#endif
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the 
  loadCAOModel() method. 

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao. 

  \param modelFile : the file containing the model.
*/
void
vpMbEdgeKltTracker::loadModel(const std::string& modelFile)
{
  vpMbTracker::loadModel(modelFile);
}

/*!
  Realize the post tracking operations. Mostly visibility tests
*/
bool
vpMbEdgeKltTracker::postTracking(const vpImage<unsigned char>& I, vpColVector &w_mbt, vpColVector &w_klt,
                                 const unsigned int lvl)
{
  bool reInit = vpMbKltTracker::postTracking(I, w_klt);
  
  postTrackingMbt(w_mbt,lvl);
  
  if (displayFeatures)
  {
    if(lvl == 0){
      vpMbtDistanceLine* l;
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
        l = *it;
        if (l->isVisible()){
          l->displayMovingEdges(I);
        }
      }
      
      vpMbtDistanceCylinder *cy ;
      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
        cy = *it;
        cy->displayMovingEdges(I);
      }
    }
  }
  
  if(reInit)
    return true;
  
  vpMbEdgeTracker::updateMovingEdge(I);
  unsigned int n = 0;
  for(unsigned int i = 0; i < vpMbKltTracker::faces.size() ; i++){
      if(vpMbKltTracker::faces[i]->isVisible()){
        vpMbEdgeTracker::faces[i]->isvisible = true;
        n++;
      }
      else
        vpMbEdgeTracker::faces[i]->isvisible = false;
  }
  vpMbEdgeTracker::nbvisiblepolygone = n;
  
  vpMbEdgeTracker::initMovingEdge(I, cMo) ;
  vpMbEdgeTracker::reinitMovingEdge(I, cMo);
  
  return false;
}

/*!
  post tracking computation. Compute the mean weight of a line and, check the
  weight associated to a site (to eventually remove an outlier) and eventually
  set a flag to re-initialize the line.

  \warning level parameter not yet implemented.

  \param w : Vector of weight associated to the residual.
  \param lvl : Optional parameter to specify the level to track.
*/
void
vpMbEdgeKltTracker::postTrackingMbt(vpColVector &w, const unsigned int lvl)
{

  if(lvl  >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }
  unsigned int n =0 ;
  vpMbtDistanceLine* l;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
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
  vpMbtDistanceCylinder *cy ;
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
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
  Realize the VVS loop for the tracking

  \param I : current image.
  \param nbInfos : Size of the features.
  \param w_mbt : weight vector for MBT.
  \param w_klt : weight vector for KLT.
  \param lvl : level of the pyramid.
*/
void
vpMbEdgeKltTracker::computeVVS(const vpImage<unsigned char>& I, const unsigned int &nbInfos, vpColVector &w_mbt, vpColVector &w_klt, const unsigned int lvl)
{
  vpColVector factor;
  unsigned int nbrow = trackFirstLoop(I, factor, lvl);
  
  if(nbrow < 4 && nbInfos < 4){
    vpERROR_TRACE("\n\t\t Error-> not enough data") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }
  else if(nbrow < 4)
    nbrow = 0;
  
  double residu = 0;
  double residu_1 = -1;
  unsigned int iter = 0;

  vpMatrix *J;
  vpMatrix J_mbt, J_klt;     // interaction matrix
  vpColVector *R;
  vpColVector R_mbt, R_klt;  // residu
  vpMatrix J_true;
  vpColVector R_true;
  vpColVector w_true;
  
  if(nbrow != 0){
    J_mbt.resize(nbrow,6);
    R_mbt.resize(nbrow);
  }
  
  if(nbInfos != 0){
    J_klt.resize(2*nbInfos,6);
    R_klt.resize(2*nbInfos);
  }
  
  vpColVector w;  // weight from MEstimator
  vpColVector v;  // "speed" for VVS
  vpRobust robust_mbt(0), robust_klt(0);
  vpHomography H;

  vpMatrix JTJ, JTR;
  
  double factorMBT = 1.0;
  double factorKLT = 1.0;
  
  //More efficient weight repartition for hybrid tracker should come soon...
//   factorMBT = 1.0 - (double)nbrow / (double)(nbrow + nbInfos);
//   factorKLT = 1.0 - factorMBT;
  factorMBT = 0.35;
  factorKLT = 0.65;
  
  double residuMBT = 0;
  double residuKLT = 0;
  
  while( ((int)((residu - residu_1)*1e8) !=0 )  && (iter<maxIter) ){   
    J = new vpMatrix(); 
    R = new vpColVector();
    
    if(nbrow >= 4)
      trackSecondLoop(I,J_mbt,R_mbt,cMo,lvl);
      
    if(nbInfos >= 4){
      unsigned int shift = 0;
      for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
        if(vpMbKltTracker::faces[i]->isVisible() && vpMbKltTracker::faces[i]->hasEnoughPoints()){
          vpSubColVector subR(R_klt, shift, 2*vpMbKltTracker::faces[i]->getNbPointsCur());
          vpSubMatrix subJ(J_klt, shift, 0, 2*vpMbKltTracker::faces[i]->getNbPointsCur(), 6);
          vpMbKltTracker::faces[i]->computeHomography(ctTc0, H);
          vpMbKltTracker::faces[i]->computeInteractionMatrixAndResidu(subR, subJ);
          shift += 2*vpMbKltTracker::faces[i]->getNbPointsCur();
        }
      }
    }
    
    if(iter == 0){
      w.resize(nbrow + 2*nbInfos);
      w=1;
      
      w_mbt.resize(nbrow);
      w_mbt = 1;
      robust_mbt.resize(nbrow);
      
      w_klt.resize(2*nbInfos);
      w_klt = 1;
      robust_klt.resize(2*nbInfos);
      
      w_true.resize(nbrow + 2*nbInfos);
    }

      /* robust */   
    if(nbrow > 3){
      residuMBT = 0;
      for(unsigned int i = 0; i < R_mbt.getRows(); i++)
        residuMBT += fabs(R_mbt[i]);
      residuMBT /= R_mbt.getRows();
      
      robust_mbt.setIteration(iter);
      robust_mbt.setThreshold(thresholdMBT/cam.get_px());
      robust_mbt.MEstimator( vpRobust::TUKEY, R_mbt, w_mbt);
      J->stackMatrices(J_mbt);
      R->stackMatrices(R_mbt);
    }
    
    if(nbInfos > 3){
      residuKLT = 0;
      for(unsigned int i = 0; i < R_klt.getRows(); i++)
        residuKLT += fabs(R_klt[i]);
      residuKLT /= R_klt.getRows();
      
      robust_klt.setIteration(iter);
      robust_klt.setThreshold(thresholdKLT/cam.get_px());
      robust_klt.MEstimator( vpRobust::TUKEY, R_klt, w_klt);
      
      J->stackMatrices(J_klt);
      R->stackMatrices(R_klt);
    }

    unsigned int cpt = 0;
    while(cpt< (nbrow+2*nbInfos)){
      if(cpt<(unsigned)nbrow){
        w[cpt] = ((w_mbt[cpt] * factor[cpt]) * factorMBT) ;
      }
      else
        w[cpt] = (w_klt[cpt-nbrow] * factorKLT);
      cpt++;
    }
    
    if(computeCovariance){
      R_true = (*R);
      J_true = (*J);
    }

    residu_1 = residu;
    residu = 0;    
    double num = 0;
    double den = 0;
    for (unsigned int i = 0; i < static_cast<unsigned int>(R->getRows()); i++){
      num += w[i]*vpMath::sqr((*R)[i]);
      den += w[i];
      
      w_true[i] = w[i]*w[i];
      (*R)[i] *= w[i];
      if(compute_interaction){
        for (unsigned int j = 0; j < 6; j += 1){
          (*J)[i][j] *= w[i];
        }
      }
    }

    residu = sqrt(num/den);

    JTJ = J->AtA();
    computeJTR(*J, *R, JTR);
    v = -lambda * JTJ.pseudoInverse() * JTR;
    cMo = vpExponentialMap::direct(v).inverse() * cMo;
    ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
    
    iter++;
    
    delete J;
    delete R;
  }
  
  if(computeCovariance){
    vpMatrix D;
    D.diag(w_true);
    covarianceMatrix = vpMatrix::computeCovarianceMatrix(J_true,v,-lambda*R_true,D);
  }
}

/*!
  Realize the tracking of the object in the image.

  \throw vpException : if the tracking is supposed to have failed.

  \param I : the input image.
*/
void
vpMbEdgeKltTracker::track(const vpImage<unsigned char>& I)
{ 
  unsigned int nbInfos;
  unsigned int nbFaceUsed;
  vpColVector w_klt;
  
  vpMbKltTracker::preTracking(I, nbInfos, nbFaceUsed);
  
  if(nbInfos >= 4)
    vpMbKltTracker::computeVVS(nbInfos, w_klt);
  else{
    nbInfos = 0;
    std::cout << "[ERROR] Unable to init with KLT" << std::endl;
  }
  
  vpMbEdgeTracker::trackMovingEdge(I);
 
  vpColVector w_mbt;
  computeVVS(I, nbInfos, w_mbt, w_klt);
  
  if(postTracking(I, w_mbt, w_klt)){
    vpMbKltTracker::reinit(I);
    
    initPyramid(I, Ipyramid);
  
    unsigned int n = 0;
    for(unsigned int i = 0; i < vpMbKltTracker::faces.size() ; i++){
        if(vpMbKltTracker::faces[i]->isVisible()){
          vpMbEdgeTracker::faces[i]->isvisible = true;
          n++;
        }
        else
          vpMbEdgeTracker::faces[i]->isvisible = false;
    }
    vpMbEdgeTracker::nbvisiblepolygone = n;
    
    unsigned int i = (unsigned int)scales.size();
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
}

unsigned int
vpMbEdgeKltTracker::trackFirstLoop(const vpImage<unsigned char>& I, vpColVector &factor, const unsigned int lvl)
{
  vpMbtDistanceLine *l ;
  vpMbtDistanceCylinder *cy ;

  if(lvl  >= scales.size() || !scales[lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }

  unsigned int nbrow  = initMbtTracking(lvl);
  
  if (nbrow==0){
//     vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
//     throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
      return nbrow;
  }
  
  factor.resize(nbrow);
  factor = 1;
    
  unsigned int n = 0;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    l = *it;
    l->computeInteractionMatrixError(cMo);
    
    double fac = 1;
    for(std::list<int>::const_iterator it = l->Lindex_polygon.begin(); it!=l->Lindex_polygon.end(); ++it){
      int index = *it;
      if (l->hiddenface->isAppearing((unsigned int)index)) {
        fac = 0.2;
        break;
      }
      if(l->closeToImageBorder(I, 10)){
        fac = 0.1;
        break;
      }
    }
    
    std::list<vpMeSite>::const_iterator itListLine;
    if (l->meline != NULL)
      itListLine = l->meline->getMeList().begin();
    
    for (unsigned int i=0 ; i < l->nbFeature ; i++){
        factor[n+i] = fac;
        vpMeSite site = *itListLine;
        if (site.getState() != vpMeSite::NO_SUPPRESSION) factor[n+i] = 0.2;
        ++itListLine;
    }   
    n+= l->nbFeature ;
  }
  
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    cy = *it;
    cy->computeInteractionMatrixError(cMo, I);
    double fac = 1.0;

    std::list<vpMeSite>::const_iterator itCyl1;
    std::list<vpMeSite>::const_iterator itCyl2;
    if ((cy->meline1 != NULL || cy->meline2 != NULL)){
      itCyl1 = cy->meline1->getMeList().begin();
      itCyl2 = cy->meline2->getMeList().begin();
    }

    for(unsigned int i=0 ; i < cy->nbFeature ; i++){
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

    n+= cy->nbFeature ;
  }
  
  return nbrow;
}

void 
vpMbEdgeKltTracker::trackSecondLoop(const vpImage<unsigned char>& I,  vpMatrix &L, vpColVector &error,
                                    vpHomogeneousMatrix& cMo, const unsigned int lvl)
{
  vpMbtDistanceLine* l;
  vpMbtDistanceCylinder *cy ;
  
  unsigned int n = 0 ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
    l = *it;
    l->computeInteractionMatrixError(cMo) ;
    for (unsigned int i=0 ; i < l->nbFeature ; i++){
      for (unsigned int j=0; j < 6 ; j++){
        L[n+i][j] = l->L[i][j];
        error[n+i] = l->error[i];
      }
    }
    n+= l->nbFeature;
  }
  
  for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[lvl].begin(); it!=cylinders[lvl].end(); ++it){
    cy = *it;
    cy->computeInteractionMatrixError(cMo, I) ;
    for(unsigned int i=0 ; i < cy->nbFeature ; i++){
      for(unsigned int j=0; j < 6 ; j++){
        L[n+i][j] = cy->L[i][j];
        error[n+i] = cy->error[i];
      }
    }

    n+= cy->nbFeature ;
  }
}

/*!
  Set the camera parameters

  \param cam : the new camera parameters
*/
void
vpMbEdgeKltTracker::setCameraParameters(const vpCameraParameters& cam)
{
  this->cam = cam;
  
  vpMbEdgeTracker::setCameraParameters(cam);
  vpMbKltTracker::setCameraParameters(cam);
}

/*!
  Initialise a new face from the coordinates given in parameter.

  \param corners : Coordinates of the corners of the face in the object frame.
  \param indexFace : index of the face (depends on the vrml file organization).
*/
void
vpMbEdgeKltTracker::initFaceFromCorners(const std::vector<vpPoint>& corners, const unsigned int indexFace)
{
  vpMbEdgeTracker::initFaceFromCorners(corners, indexFace);
  vpMbKltTracker::initFaceFromCorners(corners, indexFace);
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
vpMbEdgeKltTracker::initCylinder(const vpPoint& _p1, const vpPoint _p2, const double _radius, const unsigned int _indexCylinder)
{
  vpMbEdgeTracker::initCylinder(_p1, _p2, _radius, _indexCylinder);
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : boolean to say if all the model has to be displayed.
*/
void
vpMbEdgeKltTracker::display(const vpImage<unsigned char>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters & cam,
                            const vpColor& col , const unsigned int thickness, const bool displayFullModel)
{  
  vpMbtDistanceLine *l ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        l = *it;
        l->display(I,cMo, cam, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, cMo, cam, col, thickness);
      }

      break ; //displaying model on one scale only
    }
  }
  
  for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
    if(displayFeatures && vpMbKltTracker::faces[i]->hasEnoughPoints() && vpMbKltTracker::faces[i]->isVisible()) {
        vpMbKltTracker::faces[i]->displayPrimitive(I);
    }
  }
  
#ifdef VISP_HAVE_OGRE
  if(vpMbKltTracker::useOgre)
    vpMbKltTracker::faces.displayOgre(cMo);
#endif
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param I : The color image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : boolean to say if all the model has to be displayed.
*/
void
vpMbEdgeKltTracker::display(const vpImage<vpRGBa>& I, const vpHomogeneousMatrix &cMo, const vpCameraParameters & cam,
                            const vpColor& col , const unsigned int thickness, const bool displayFullModel)
{ 
  vpMbtDistanceLine *l ;
  
  for (unsigned int i = 0; i < scales.size(); i += 1){
    if(scales[i]){
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[scaleLevel].begin(); it!=lines[scaleLevel].end(); ++it){
        l = *it;
        l->display(I,cMo, cam, col, thickness, displayFullModel);
      }

      for(std::list<vpMbtDistanceCylinder*>::const_iterator it=cylinders[scaleLevel].begin(); it!=cylinders[scaleLevel].end(); ++it){
        (*it)->display(I, cMo, cam, col, thickness);
      }

      break ; //displaying model on one scale only
    }
  }
  
  for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
    if(displayFeatures && vpMbKltTracker::faces[i]->hasEnoughPoints() && vpMbKltTracker::faces[i]->isVisible()) {
        vpMbKltTracker::faces[i]->displayPrimitive(I);
    }
  }
  
#ifdef VISP_HAVE_OGRE
  if(vpMbKltTracker::useOgre)
    vpMbKltTracker::faces.displayOgre(cMo);
#endif
}

#endif //VISP_HAVE_OPENCV
