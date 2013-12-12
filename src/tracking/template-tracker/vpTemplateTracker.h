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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
 \file vpTemplateTracker.h
 \brief
*/

#ifndef vpTemplateTracker_hh
#define vpTemplateTracker_hh

#include <math.h>

#include <visp/vpTemplateTrackerHeader.h>
#include <visp/vpTemplateTrackerZone.h>
#include <visp/vpTemplateTrackerWarp.h>
#include <visp/vpImageFilter.h>

class VISP_EXPORT vpTemplateTracker
{
  protected:
    //traitement pyramidal
    unsigned int                nbLvlPyr;
    unsigned int                l0Pyr;
    bool                        pyrInitialised;
    
    vpTemplateTrackerPoint     *ptTemplate;
    vpTemplateTrackerPoint    **ptTemplatePyr;
    bool                        ptTemplateInit;
    unsigned int                templateSize;
    unsigned int               *templateSizePyr;
    bool                       *ptTemplateSelect;
    bool                      **ptTemplateSelectPyr;
    bool                        ptTemplateSelectInit;
    unsigned int                templateSelectSize;

    #ifndef DOXYGEN_SHOULD_SKIP_THIS
    vpTemplateTrackerPointSuppMIInv *ptTemplateSupp; //pour inverse et compo
    vpTemplateTrackerPointSuppMIInv **ptTemplateSuppPyr;  //pour inverse et compo
    #endif

    bool                        ptTemplateSuppInit;
    vpTemplateTrackerPointCompo *ptTemplateCompo;    //pour ESM
    vpTemplateTrackerPointCompo **ptTemplateCompoPyr;   //pour ESM
    vpTemplateTrackerZone               *zoneTracked;
    vpTemplateTrackerZone               *zoneTrackedPyr;
    
    vpImage<unsigned char>     *pyr_IDes;
    
    vpMatrix                    H;
    vpMatrix                    Hdesire;
    vpMatrix                   *HdesirePyr;
    vpMatrix                    HLM;
    vpMatrix                    HLMdesire;
    vpMatrix                   *HLMdesirePyr;
    vpMatrix                    HLMdesireInverse;
    vpMatrix                   *HLMdesireInversePyr;
    vpColVector                 G;
    
    double                      gain;
    double                      thresholdGradient;
    bool                        costFunctionVerification;
    bool                        blur;
    bool                        useBrent;
    unsigned int                nbIterBrent;
    unsigned int                taillef;
    double                     *fgG;
    double                     *fgdG;
    double                      ratioPixelIn;
    int                         mod_i;
    int                         mod_j;//variable de sampling de zone de reference
    unsigned int                nbParam ;
    double                      lambdaDep ;
    unsigned int                iterationMax ;
    //pour BFGS
    unsigned int                iterationGlobale;
    //diverge is set to true if there is no more point in the tracked area
    bool                        diverge;
    unsigned int                nbIteration;
    bool                        useCompositionnal;
    bool                        useInverse;

    vpTemplateTrackerWarp      *Warp;
    //Parametre de deplacement
    vpColVector                 p;
    vpColVector                 dp;

    //temporary values for warping
    vpColVector                 X1;
    vpColVector                 X2;
    //temporary derivative matrix
    vpMatrix                    dW;

    vpImage<double>             BI;
    vpImage<double>             dIx ;
    vpImage<double>             dIy ;
    vpTemplateTrackerZone       zoneWarp_;
    
  public:
    vpTemplateTracker(vpTemplateTrackerWarp *_warp);
    virtual        ~vpTemplateTracker();
    
    void    display(const vpImage<unsigned char> &I, const vpColor& col = vpColor::green, const unsigned int thickness=3);
    void    display(const vpImage<vpRGBa> &I, const vpColor& col = vpColor::green, const unsigned int thickness=3);

    bool    getDiverge() const {return diverge;}
    vpColVector getdp(){ return dp; }
    vpColVector getG() const { return G; }
    vpMatrix    getH() const { return H; }
    unsigned int getNbParam() const { return nbParam ; }
    unsigned int getNbIteration() const { return nbIteration; }
    vpColVector getp() const { return p;}
    double  getRatioPixelIn() const {return ratioPixelIn;}

    /*!

       \return The pointer to the warper.
     */
    vpTemplateTrackerWarp *getWarp() const {return Warp;}

    /*!
     Return the reference template zone.
     */
    vpTemplateTrackerZone getZoneWarp() const { return zoneWarp_; }

    void    initClick(const vpImage<unsigned char> &I, bool delaunay=false);
    void    initFromPoints(const vpImage<unsigned char> &I, const std::vector< vpImagePoint > &v_ip, bool delaunay=false);
    void    initFromZone(const vpImage<unsigned char> &I, const vpTemplateTrackerZone& zone);
    
    void    resetTracker();
    
    void    setBlur(bool b){blur = b;}
    void    setCostFunctionVerification(bool b){costFunctionVerification = b;}
    void    setGain(double g){gain=g;}
    void    setGaussianFilterSize(unsigned int new_taill);
    void    setHDes(vpMatrix &tH){ Hdesire=tH; vpMatrix::computeHLM(Hdesire,lambdaDep,HLMdesire); HLMdesireInverse = HLMdesire.inverseByLU();}
    void    setIterationMax(const unsigned int &n) { iterationMax = n ; }
    void    setLambda(double l) { lambdaDep = l ; }
    void    setNbIterBrent(const unsigned int &b){nbIterBrent=b;}
    void    setp(const vpColVector &tp){ p=tp; diverge=false; iterationGlobale=0; }
    /*!
     Set the number of pyramid levels used in the multi-resolution scheme.
     If \e nlevels > 1, the tracker uses a pyramidal approach.
     */
    void    setPyramidal(unsigned int nlevels=2, unsigned int level_to_stop=1) {nbLvlPyr = nlevels; l0Pyr = level_to_stop; }
    void    setSampling(int _mod_i,int _mod_j){mod_i=_mod_i;mod_j=_mod_j;}
    void    setThresholdGradient(double threshold){thresholdGradient=threshold;}
    /*! By default Brent usage is disabled. */
    void    setUseBrent(bool b){useBrent = b;}
    
    void    track(const vpImage<unsigned char> &I);
    void    trackRobust(const vpImage<unsigned char> &I);
    
  protected:

    void            computeOptimalBrentGain(const vpImage<unsigned char> &I,vpColVector &tp,double tMI,vpColVector &direction,double &alpha);
    virtual double  getCost(const vpImage<unsigned char> &I, vpColVector &tp) = 0;
    void            getGaussianBluredImage(const vpImage<unsigned char> &I){ vpImageFilter::filter(I, BI,fgG,taillef); }
    void            initCompInverse(const vpImage<unsigned char> &I);
    virtual void    initCompInversePyr(const vpImage<unsigned char> &I);
    virtual void    initHessienDesired(const vpImage<unsigned char> &I)=0;
    virtual void    initHessienDesiredPyr(const vpImage<unsigned char> &I);
    virtual void    initPyramidal(unsigned int nbLvl,unsigned int l0);
    void            initTracking(const vpImage<unsigned char>& I,vpTemplateTrackerZone &zone);
    virtual void    initTrackingPyr(const vpImage<unsigned char>& I,vpTemplateTrackerZone &zone);
    virtual void    trackNoPyr(const vpImage<unsigned char> &I) = 0;
    virtual void    trackPyr(const vpImage<unsigned char> &I);
};
#endif

