/****************************************************************************
 *
 * $Id:$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
  
  lambda = 0.8;
  thresholdKLT = 5.0;
  thresholdMBT = 2.0;
  maxIter = 200;
  vpMbKltTracker::setMaxIter(30);
}

/*!
  Basic constructor

*/
vpMbEdgeKltTracker::~vpMbEdgeKltTracker()
{
}

/*!
  Initialisation of the tracker using a known initial pose.
  The 3D model must first have been loaded.

  \param _I : Input image.
*/
void 
vpMbEdgeKltTracker::init(const vpImage<unsigned char>& _I)
{
  vpMbEdgeTracker::init(_I);
  vpMbKltTracker::init(_I);
}

int 
vpMbEdgeKltTracker::initMbtTracking(const unsigned int _lvl)
{
  vpMbtDistanceLine *l ;

  if(_lvl  >= scales.size() || !scales[_lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }

  int nbrow  = 0;
  for(std::list<vpMbtDistanceLine*>::iterator it=lines[_lvl].begin(); it!=lines[_lvl].end(); ++it){
    l = *it;
    nbrow += l->nbFeature ;
    l->initInteractionMatrixError() ;
  }
  
  return nbrow;  
}

/*!
  Load the xml configuration file.
  From the configuration file parameters write initialize the corresponding objects (Ecm, camera).

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \param _filename : full name of the xml file.
*/
void 
vpMbEdgeKltTracker::loadConfigFile(const std::string& _filename)
{
  vpMbEdgeKltTracker::loadConfigFile(_filename.c_str());
}

/*!
  Load the xml configuration file.
  From the configuration file parameters initialize the corresponding objects (Ecm, camera).

  \warning To clean up memory allocated by the xml library, the user has to call
  vpXmlParser::cleanup() before the exit().

  \throw vpException::ioError if the file has not been properly parsed (file not
  found or wrong format for the data). 

  \param filename : full name of the xml file.

  \sa vpXmlParser::cleanup()
*/
void
vpMbEdgeKltTracker::loadConfigFile(const char* filename)
{
#ifdef VISP_HAVE_XML2
  vpMbEdgeTracker::loadConfigFile(filename);
  vpMbKltTracker::loadConfigFile(filename);
#else
  vpTRACE("You need the libXML2 to read the config file %s", filename);
#endif
}

/*!
  Load a 3D model from the file in parameter. This file must either be a vrml
  file (.wrl) or a CAO file (.cao). CAO format is described in the 
  loadCAOModel() method. 

  \throw vpException::ioError if the file cannot be open, or if its extension is
  not wrl or cao. 

  \param _modelFile : the file containing the model.
*/
void
vpMbEdgeKltTracker::loadModel(const std::string& _modelFile)
{
  vpMbTracker::loadModel(_modelFile);
}

/*!
  Realise the post tracking operations. Mostly visibility tests
*/
bool
vpMbEdgeKltTracker::postTracking(const vpImage<unsigned char>& _I, vpColVector &w_mbt, vpColVector &w_klt, const unsigned int lvl)
{
  post_tracking_mbt(w_mbt,lvl);
  
//   if (displayMe)
  {
    if(lvl == 0){
      vpMbtDistanceLine* l;
      for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[lvl].begin(); it!=lines[lvl].end(); ++it){
        l = *it;
        if (l->isVisible()){
          l->displayMovingEdges(_I);
        }
      }
    }
  }
  
  vpMbEdgeTracker::updateMovingEdge(_I);
  bool useless = false ;
  vpMbEdgeTracker::visibleFace(cMo, useless) ;
  vpMbEdgeTracker::initMovingEdge(_I, cMo) ;
  vpMbEdgeTracker::reinitMovingEdge(_I, cMo);
  
  return vpMbKltTracker::postTracking(_I, w_klt);
}

/*!
  post tracking computation. Compute the mean weight of a line and, check the
  weight associated to a site (to eventually remove an outlier) and eventually
  set a flag to re-initialise the line.

  \warning _level parameter not yet implemented.

  \param w : Vector of weight associated to the residu.
  \param _lvl : Optional parameter to specify the level to track.
*/
void
vpMbEdgeKltTracker::post_tracking_mbt(vpColVector &w, const unsigned int _lvl)
{

  if(_lvl  >= scales.size() || !scales[_lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }
  unsigned int n =0 ;
  vpMbtDistanceLine* l;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[_lvl].begin(); it!=lines[_lvl].end(); ++it){
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
}

/*!
  Realise the VVS loop for the tracking

  \param _I : current image.
  \param nbInfos : Size of the features.
  \param w_mbt : weight vector for MBT.
  \param w_klt : weight vector for KLT.
  \param lvl : level of the pyramid.
*/
void
vpMbEdgeKltTracker::computeVVS(const vpImage<unsigned char>& _I, const unsigned int &nbInfos, vpColVector &w_mbt, vpColVector &w_klt, const unsigned int lvl)
{
  vpColVector factor;
  int nbrow = trackFirstLoop(_I, factor, lvl);
  
  if(nbrow < 4 && nbInfos < 4){
    vpERROR_TRACE("\n\t\t Error-> not enough data") ;
    throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data");
  }
  else if(nbrow < 4)
    nbrow = 0;
  
  double residu = 0;
  double residu_1 = -1;
  unsigned int iter = 0;

  vpMatrix J(nbrow,6);     // interaction matrix
  vpColVector R;  // residu
  vpMatrix J_mbt(nbrow,6), J_klt(2*nbInfos,6);
  vpColVector R_mbt(nbrow), R_klt(2*nbInfos);
  
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
    J.init(); 
    R.init();
    
    if(nbrow >= 4)
      trackSecondLoop(J_mbt,R_mbt,cMo,lvl);
      
    if(nbInfos >= 4){
      unsigned int shift = 0;
      for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
        if(vpMbKltTracker::faces[i]->getIsTracked() && vpMbKltTracker::faces[i]->hasEnoughPoints()){
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
      J.stackMatrices(J_mbt);
      R.stackMatrices(R_mbt);
    }

    if(nbInfos > 3){
      residuKLT = 0;
      for(unsigned int i = 0; i < R_klt.getRows(); i++)
        residuKLT += fabs(R_klt[i]);
      residuKLT /= R_klt.getRows();
      
      robust_klt.setIteration(iter);
      robust_klt.setThreshold(thresholdKLT/cam.get_px());
      robust_klt.MEstimator( vpRobust::TUKEY, R_klt, w_klt);
      J.stackMatrices(J_klt);
      R.stackMatrices(R_klt);
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

    residu_1 = residu;
    residu = 0;    
    double num = 0;
    double den = 0;
    for (unsigned int i = 0; i < static_cast<unsigned int>(R.getRows()); i++){
      num += w[i]*vpMath::sqr(R[i]);
      den += w[i];
      R[i] *= w[i];
      if(compute_interaction){
        for (unsigned int j = 0; j < 6; j += 1){
          J[i][j] *= w[i];
        }
      }
    }

    residu = sqrt(num/den);

    JTJ = J.AtA();
    computeJTR(J, R, JTR);
    v = -lambda * JTJ.pseudoInverse() * JTR;
    cMo = vpExponentialMap::direct(v).inverse() * cMo;
    ctTc0 = vpExponentialMap::direct(v).inverse() * ctTc0;
    
    iter++;
  }
}

/*!
  Realise the tracking of the object in the image

  \throw vpException : if the tracking is supposed to have failed

  \param _I : the input image
*/
void
vpMbEdgeKltTracker::track(const vpImage<unsigned char>& _I)
{ 
  unsigned int nbInfos;
  unsigned int nbFaceUsed;
  vpColVector w_klt;
  
  vpMbKltTracker::preTracking(_I,nbInfos,nbFaceUsed);
  
  if(nbInfos >= 4){
    vpMbKltTracker::computeVVS(nbInfos, w_klt);
//     setPose(vpMbKltTracker::getPose());
  }
  else{
    nbInfos = 0;
    std::cout << "[ERROR] Unable to init with KLT" << std::endl;
  }
  
  vpMbEdgeTracker::trackMovingEdge(_I);
 
  vpColVector w_mbt;
  computeVVS(_I, nbInfos, w_mbt, w_klt);
  
  if(postTracking(_I, w_mbt, w_klt))
    init(_I);
}

int 
vpMbEdgeKltTracker::trackFirstLoop(const vpImage<unsigned char>& _I, vpColVector &factor, const unsigned int _lvl)
{
  vpMbtDistanceLine *l ;

  if(_lvl  >= scales.size() || !scales[_lvl]){
    throw vpException(vpException::dimensionError, "_lvl not used.");
  }

  int nbrow  = initMbtTracking(_lvl);
  
  if (nbrow==0){
//     vpERROR_TRACE("\n\t\t Error-> not enough data in the interaction matrix...") ;
//     throw vpTrackingException(vpTrackingException::notEnoughPointError, "\n\t\t Error-> not enough data in the interaction matrix...");
      return nbrow;
  }
  
  factor.resize(nbrow);
  factor = 1;
    
  unsigned int n = 0;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[_lvl].begin(); it!=lines[_lvl].end(); ++it){
    l = *it;
    l->computeInteractionMatrixError(cMo);
    
    double fac = 1;
    for(std::list<int>::const_iterator it = l->Lindex_polygon.begin(); it!=l->Lindex_polygon.end(); ++it){
      int index = *it;
      if (l->hiddenface->isAppearing(index)) {
        fac = 0.2;
        break;
      }
      if(l->closeToImageBorder(_I, 10)){
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
  
  return nbrow;
}

void 
vpMbEdgeKltTracker::trackSecondLoop(vpMatrix &_L, vpColVector &_error, vpHomogeneousMatrix& _cMo, const unsigned int _lvl)
{
  vpMbtDistanceLine* l;
  
  unsigned int n = 0 ;
  for(std::list<vpMbtDistanceLine*>::const_iterator it=lines[_lvl].begin(); it!=lines[_lvl].end(); ++it){
    l = *it;
    l->computeInteractionMatrixError(_cMo) ;
    for (unsigned int i=0 ; i < l->nbFeature ; i++){
      for (unsigned int j=0; j < 6 ; j++){
        _L[n+i][j] = l->L[i][j];
        _error[n+i] = l->error[i];
//         error_lines[nlines+i] = error[n+i];
      }
    }
    n+= l->nbFeature;
  }
}

/*!
  Set the camera parameters

  \param _cam : the new camera parameters
*/
void
vpMbEdgeKltTracker::setCameraParameters(const vpCameraParameters& _cam)
{
  cam = _cam;
  cameraInitialised = true;
  
  vpMbEdgeTracker::setCameraParameters(_cam);
  vpMbKltTracker::setCameraParameters(_cam);
}

/*!
  set the current pose.

  \param _cMo : the current pose.
*/
void   
vpMbEdgeKltTracker::setPose(const vpHomogeneousMatrix &_cMo) 
{ 
  cMo = _cMo ;
  vpMbKltTracker::setPose(cMo);
//   vpMbEdgeTracker::setPose(cMo);
}

/*!
  Initialise a new face from the coordinates given in parameter.

  \param _corners : Coordinates of the corners of the face in the object frame.
  \param _indexFace : index of the face (depends on the vrml file organisation).
*/
void
vpMbEdgeKltTracker::initFaceFromCorners(const std::vector<vpPoint>& _corners, const unsigned int _indexFace)
{
  vpMbEdgeTracker::initFaceFromCorners(_corners, _indexFace);
  vpMbKltTracker::initFaceFromCorners(_corners, _indexFace);
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param _I : The image .
  \param _cMo : Pose used to project the 3D model into the image.
  \param _cam : The camera parameters.
  \param _col : The desired color.
  \param _l : The thickness of the lines.
  \param displayFullModel : boolean to say if all the model has to be displayed.
*/
void
vpMbEdgeKltTracker::display(const vpImage<unsigned char>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters & _cam, const vpColor& _col , const unsigned int _l, const bool displayFullModel)
{
  vpMbEdgeTracker::display(_I, _cMo, _cam, _col, _l, displayFullModel);
//   vpMbKltTracker::display(_I, _cMo, _cam, _col, _l, displayFullModel); // Not used because, it would display twice the edges
  
  for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
    if(displayFullModel || vpMbKltTracker::faces[i]->getIsTracked())
    {
      vpMbKltTracker::faces[i]->changeFrame(_cMo);      
      if(vpMbKltTracker::faces[i]->hasEnoughPoints())
        vpMbKltTracker::faces[i]->displayPrimitive(_I);
//       if(facesTracker[i].hasEnoughPoints())
//         faces[i]->displayNormal(_I);
    }
  }
}

/*!
  Display the 3D model at a given position using the given camera parameters

  \param _I : The color image .
  \param _cMo : Pose used to project the 3D model into the image.
  \param _cam : The camera parameters.
  \param _col : The desired color.
  \param _l : The thickness of the lines.
  \param displayFullModel : boolean to say if all the model has to be displayed.
*/
void
vpMbEdgeKltTracker::display(const vpImage<vpRGBa>& _I, const vpHomogeneousMatrix &_cMo, const vpCameraParameters & _cam, const vpColor& _col , const unsigned int _l, const bool displayFullModel)
{
  vpMbEdgeTracker::display(_I, _cMo, _cam, _col, _l, displayFullModel);
//   vpMbKltTracker::display(_I, _cMo, _cam, _col, _l, displayFullModel);// Not used because, it would display twice the edges
  
  for (unsigned int i = 0; i < vpMbKltTracker::faces.size(); i += 1){
    if(displayFullModel || vpMbKltTracker::faces[i]->getIsTracked())
    {
      vpMbKltTracker::faces[i]->changeFrame(_cMo);      
      if(vpMbKltTracker::faces[i]->hasEnoughPoints())
        vpMbKltTracker::faces[i]->displayPrimitive(_I);
//       if(facesTracker[i].hasEnoughPoints())
//         faces[i]->displayNormal(_I);
    }
  }
}

#endif //VISP_HAVE_OPENCV