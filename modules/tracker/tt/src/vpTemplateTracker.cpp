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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerBSpline.h>

vpTemplateTracker::vpTemplateTracker(vpTemplateTrackerWarp *_warp)
  : nbLvlPyr(1), l0Pyr(0), pyrInitialised(false), ptTemplate(NULL), ptTemplatePyr(NULL), ptTemplateInit(false),
    templateSize(0), templateSizePyr(NULL), ptTemplateSelect(NULL), ptTemplateSelectPyr(NULL),
    ptTemplateSelectInit(false), templateSelectSize(0), ptTemplateSupp(NULL), ptTemplateSuppPyr(NULL),
    ptTemplateCompo(NULL), ptTemplateCompoPyr(NULL), zoneTracked(NULL), zoneTrackedPyr(NULL), pyr_IDes(NULL), H(),
    Hdesire(), HdesirePyr(), HLM(), HLMdesire(), HLMdesirePyr(), HLMdesireInverse(), HLMdesireInversePyr(), G(),
    gain(1.), thresholdGradient(40), costFunctionVerification(false), blur(true), useBrent(false), nbIterBrent(3),
    taillef(7), fgG(NULL), fgdG(NULL), ratioPixelIn(0), mod_i(1), mod_j(1), nbParam(0), lambdaDep(0.001),
    iterationMax(30), iterationGlobale(0), diverge(false), nbIteration(0), useCompositionnal(true), useInverse(false),
    Warp(_warp), p(0), dp(), X1(), X2(), dW(), BI(), dIx(), dIy(), zoneRef_()
{
  nbParam = Warp->getNbParam();
  p.resize(nbParam);
  dp.resize(nbParam);

  fgG = new double[(taillef + 1) / 2];
  vpImageFilter::getGaussianKernel(fgG, taillef);

  fgdG = new double[(taillef + 1) / 2];
  vpImageFilter::getGaussianDerivativeKernel(fgdG, taillef);
}

void vpTemplateTracker::setGaussianFilterSize(unsigned int new_taill)
{
  taillef = new_taill;
  if (fgG)
    delete[] fgG;
  fgG = new double[taillef];
  vpImageFilter::getGaussianKernel(fgG, taillef);

  if (fgdG)
    delete[] fgdG;
  fgdG = new double[taillef];
  vpImageFilter::getGaussianDerivativeKernel(fgdG, taillef);
}

void vpTemplateTracker::initTracking(const vpImage<unsigned char> &I, vpTemplateTrackerZone &zone)
{
  // 	std::cout<<"\tInitialise reference..."<<std::endl;
  zoneTracked = &zone;

  int largeur_im = (int)I.getWidth();
  int hauteur_im = (int)I.getHeight();

  unsigned int NbPointDsZone = 0;
  // double xtotal=0,ytotal=0;
  int mod_fi, mod_fj;
  mod_fi = mod_i;
  mod_fj = mod_i;

  for (int i = 0; i < hauteur_im; i += mod_fi) {
    for (int j = 0; j < largeur_im; j += mod_fj) {
      if (zone.inZone(i, j)) {
        NbPointDsZone++;
        // xtotal+=j;
        // ytotal+=i;
      }
    }
  }

  // Warp->setCentre((double)xtotal/NbPointDsZone,(double)ytotal/NbPointDsZone);

  templateSize = NbPointDsZone;
  ptTemplate = new vpTemplateTrackerPoint[templateSize];
  ptTemplateInit = true;
  ptTemplateSelect = new bool[templateSize];
  ptTemplateSelectInit = true;

  Hdesire.resize(nbParam, nbParam);
  HLMdesire.resize(nbParam, nbParam);

  vpTemplateTrackerPoint pt;
  // vpTemplateTrackerZPoint ptZ;
  vpImage<double> GaussI;
  vpImageFilter::filter(I, GaussI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  unsigned int cpt_point = 0;
  templateSelectSize = 0;
  for (int i = 0; i < hauteur_im; i += mod_i) {
    // for(int  j=minx_t;j<maxx_t;j++)
    for (int j = 0; j < largeur_im; j += mod_j) {
      //      if(i%mod_i ==0 && j%mod_j ==0)
      if (zone.inZone(i, j)) {
        pt.x = j;
        pt.y = i;

        pt.dx = dIx[i][j];
        pt.dy = dIy[i][j];

        if (pt.dx * pt.dx + pt.dy * pt.dy > thresholdGradient) {
          ptTemplateSelect[cpt_point] = true;
          templateSelectSize++;
        } else
          ptTemplateSelect[cpt_point] = false;
        // ptTemplate_select[cpt_point]=true;

        /*if(blur)
        pt.val=GaussI[i][j];
      else
        pt.val=I[i][j];*/
        pt.val = vpTemplateTrackerBSpline::getSubPixBspline4(GaussI, i, j);
        // ptZone_pyr[NbLevelPyramid-cpt].push_back(pt);

        ptTemplate[cpt_point] = pt;
        cpt_point++;
      }
    }
  }

  // 	std::cout<<"\tNb pt template apres scale:"<<cpt_point<<std::endl;
  // 	std::cout<<"utilisation de
  // "<<taille_template_select<<"/"<<cpt_point<<" =
  // "<<100.*taille_template_select/cpt_point<<"% pour calcul
  // derivees"<<std::endl;

  templateSize = cpt_point;
  GaussI.destroy();
  // 	std::cout<<"\tEnd of reference initialisation ..."<<std::endl;
}

vpTemplateTracker::~vpTemplateTracker()
{
  // 	vpTRACE("destruction tracker");
  delete[] fgG;
  delete[] fgdG;

  resetTracker();
}

/*!
  Reset the tracker by freeing the memory allocated by the template tracker
  during the initialization.
 */
void vpTemplateTracker::resetTracker()
{
  // reset the tracker parameters
  p = 0;

  // 	vpTRACE("resetTracking");
  if (pyrInitialised) {
    if (ptTemplatePyr) {
      for (unsigned int i = 0; i < nbLvlPyr; i++) {
        if (ptTemplatePyr[i]) {
          for (unsigned int point = 0; point < templateSizePyr[i]; point++) {
            delete[] ptTemplatePyr[i][point].dW;
            delete[] ptTemplatePyr[i][point].HiG;
          }
          delete[] ptTemplatePyr[i];
        }
      }
      delete[] ptTemplatePyr;
      ptTemplatePyr = NULL;
    }

    if (ptTemplateCompoPyr) {
      for (unsigned int i = 0; i < nbLvlPyr; i++) {
        if (ptTemplateCompoPyr[i]) {
          for (unsigned int point = 0; point < templateSizePyr[i]; point++) {
            delete[] ptTemplateCompoPyr[i][point].dW;
          }
          delete[] ptTemplateCompoPyr[i];
        }
      }
      delete[] ptTemplateCompoPyr;
      ptTemplateCompoPyr = NULL;
    }

    if (ptTemplateSuppPyr) {
      for (unsigned int i = 0; i < nbLvlPyr; i++) {
        if (ptTemplateSuppPyr[i]) {
          for (unsigned int point = 0; point < templateSizePyr[i]; point++) {
            delete[] ptTemplateSuppPyr[i][point].Bt;
            delete[] ptTemplateSuppPyr[i][point].BtInit;
            delete[] ptTemplateSuppPyr[i][point].dBt;
            delete[] ptTemplateSuppPyr[i][point].d2W;
            delete[] ptTemplateSuppPyr[i][point].d2Wx;
            delete[] ptTemplateSuppPyr[i][point].d2Wy;
          }
          delete[] ptTemplateSuppPyr[i];
        }
      }
      delete[] ptTemplateSuppPyr;
      ptTemplateSuppPyr = NULL;
    }

    if (ptTemplateSelectPyr) {
      for (unsigned int i = 0; i < nbLvlPyr; i++) {
        if (ptTemplateSelectPyr[i])
          delete[] ptTemplateSelectPyr[i];
      }
      delete[] ptTemplateSelectPyr;
      ptTemplateSelectPyr = NULL;
    }

    if (templateSizePyr) {
      delete[] templateSizePyr;
      templateSizePyr = NULL;
    }

    if (HdesirePyr) {
      delete[] HdesirePyr;
      HdesirePyr = NULL;
    }

    if (HLMdesirePyr) {
      delete[] HLMdesirePyr;
      HLMdesirePyr = NULL;
    }

    if (HLMdesireInversePyr) {
      delete[] HLMdesireInversePyr;
      HLMdesireInversePyr = NULL;
    }

    if (zoneTrackedPyr) {
      delete[] zoneTrackedPyr;
      zoneTrackedPyr = NULL;
    }

    if (pyr_IDes) {
      delete[] pyr_IDes;
      pyr_IDes = NULL;
    }
  } else {
    if (ptTemplateInit) {
      for (unsigned int point = 0; point < templateSize; point++) {
        delete[] ptTemplate[point].dW;
        delete[] ptTemplate[point].HiG;
      }
      delete[] ptTemplate;
      ptTemplate = NULL;
      ptTemplateInit = false;
    }
    if (ptTemplateCompo) {
      for (unsigned int point = 0; point < templateSize; point++) {
        delete[] ptTemplateCompo[point].dW;
      }
      delete[] ptTemplateCompo;
      ptTemplateCompo = NULL;
    }
    if (ptTemplateSupp) {
      for (unsigned int point = 0; point < templateSize; point++) {
        delete[] ptTemplateSupp[point].Bt;
        delete[] ptTemplateSupp[point].BtInit;
        delete[] ptTemplateSupp[point].dBt;
        delete[] ptTemplateSupp[point].d2W;
        delete[] ptTemplateSupp[point].d2Wx;
        delete[] ptTemplateSupp[point].d2Wy;
      }
      delete[] ptTemplateSupp;
      ptTemplateSupp = NULL;
    }
    if (ptTemplateSelectInit) {
      if (ptTemplateSelect) {
        delete[] ptTemplateSelect;
        ptTemplateSelect = NULL;
      }
    }
  }
}

/*!
  Display the warped reference template in an image.

  \param I: Image in which the warped zone has to be displayed.
  \param col: Color used to draw the triangle edges.
  \param thickness: Thickness of the lines.

  The following code shows how to use display capabilities:
  \code
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>

int main()
{
  vpImage<unsigned char> I;
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional tracker(&warp);
  vpTemplateTrackerZone zoneRef, zoneWarped;

  // Display the warped zone
  tracker.display(I, vpColor::red);

  // Display the reference zone
  zoneRef = tracker.getZoneRef();
  zoneRef.display(I, vpColor::green);

  // Display the warped zone
  vpColVector p = tracker.getp();
  warp.warpZone(zoneRef, p, zoneWarped);
  zoneWarped.display(I, vpColor::blue);
}
  \endcode
 */
void vpTemplateTracker::display(const vpImage<unsigned char> &I, const vpColor &col, const unsigned int thickness)
{
  if (I.display) { // Only if a display is associated to the image
    vpTemplateTrackerZone zoneWarped;
    Warp->warpZone(*zoneTracked, p, zoneWarped);
    zoneWarped.display(I, col, thickness);
  }
}

/*!
  Display the warped reference template in an image.

  \param I: Image in which the warped zone has to be displayed.
  \param col: Color used to draw the triangle edges.
  \param thickness: Thickness of the lines.

  The following code shows how to use display capabilities:
  \code
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>

int main()
{
  vpImage<vpRGBa> I;
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional tracker(&warp);
  vpTemplateTrackerZone zoneRef, zoneWarped;

  // Display the warped zone
  tracker.display(I, vpColor::red);

  // Display the reference zone
  zoneRef = tracker.getZoneRef();
  zoneRef.display(I, vpColor::green);

  // Display the warped zone
  vpColVector p = tracker.getp();
  warp.warpZone(zoneRef, p, zoneWarped);
  zoneWarped.display(I, vpColor::blue);
}
  \endcode
 */
void vpTemplateTracker::display(const vpImage<vpRGBa> &I, const vpColor &col, const unsigned int thickness)
{
  if (I.display) { // Only if a display is associated to the image
    vpTemplateTrackerZone zoneWarped;
    Warp->warpZone(*zoneTracked, p, zoneWarped);
    zoneWarped.display(I, col, thickness);
  }
}

void vpTemplateTracker::computeOptimalBrentGain(const vpImage<unsigned char> &I, vpColVector &tp, double tMI,
                                                vpColVector &direction, double &alpha)
{
  vpColVector **ptp;
  ptp = new vpColVector *[4];
  vpColVector p0(nbParam);
  p0 = tp;

  // valeur necessaire si conditionnel
  vpColVector dpt(Warp->getNbParam());
  vpColVector adpt(Warp->getNbParam());

  vpColVector p1(nbParam);
  if (useCompositionnal) {
    if (useInverse)
      Warp->getParamInverse(direction, dpt);
    else
      dpt = direction;
    Warp->pRondp(tp, dpt, p1);
  } else {
    p1 = tp + direction;
  }

  vpColVector p2(nbParam);
  if (useCompositionnal) {
    adpt = alpha * direction;
    if (useInverse)
      Warp->getParamInverse(adpt, dpt);
    else
      dpt = adpt;
    Warp->pRondp(tp, dpt, p2);
  } else {
    p2 = tp + alpha * direction;
  }
  vpColVector p3(nbParam);
  ptp[0] = &p0;
  ptp[1] = &p1;
  ptp[2] = &p2;
  ptp[3] = &p3;

  double *Cost = new double[4];
  // Cost[0]=getCost(I,p0);
  Cost[0] = tMI;
  Cost[1] = getCost(I, p1);
  Cost[2] = getCost(I, p2);

  double *talpha = new double[4];
  talpha[0] = 0;
  talpha[1] = 1.;
  talpha[2] = alpha;

  // for(int i=0;i<3;i++)
  //	std::cout<<"alpha["<<i<<"] = "<<talpha[i]<<"   Cost["<<i<<"] =
  //"<<Cost[i]<<std::endl;

  // Utilise trois estimï¿œes de paraboles succesive ...
  // A changer pour rendre adaptable
  for (unsigned int opt = 0; opt < nbIterBrent; opt++) {
    // double a=talpha[0];
    // double b=talpha[1];
    // double c=talpha[2];
    // double Cost0=Cost[0];
    // double Cost1=Cost[1];
    // double Cost2=Cost[2];

    vpMatrix A(3, 3);
    for (unsigned int i = 0; i < 3; i++) {
      A[i][0] = talpha[i] * talpha[i];
      A[i][1] = talpha[i];
      A[i][2] = 1.;
    }
    // std::cout<<"A="<<A<<std::endl;
    vpColVector B(3);
    for (unsigned int i = 0; i < 3; i++)
      B[i] = Cost[i];
    vpColVector parabol(3);
    parabol = (A.t() * A).inverseByLU() * A.t() * B;
    // vpColVector parabol(3);parabol=A.pseudoInverse()*B;

    // si convexe
    if (parabol[0] > 0) {
      talpha[3] = -0.5 * parabol[1] / parabol[0];
      // std::cout<<"parabol = "<<parabol<<std::endl;
      // std::cout<<"convexe talpha = "<<talpha[3]<<std::endl;
    } else // si concave
    {
      int tindic_x_min = 0;
      int tindic_x_max = 0;
      for (int i = 1; i < 3; i++) {
        if (talpha[i] < talpha[tindic_x_min])
          tindic_x_min = i;
        if (talpha[i] > talpha[tindic_x_max])
          tindic_x_max = i;
      }

      if (Cost[tindic_x_max] < Cost[tindic_x_min]) {
        // talpha[3]=talpha[tindic_x_max]+1.;
        talpha[3] = talpha[tindic_x_max] + 1.;
        /*if(talpha[tindic_x_min]>talpha[tindic_x_max])
          talpha[3]=talpha[tindic_x_min]+1.;
        else
          talpha[3]=talpha[tindic_x_min]-1.;*/
      } else {
        // talpha[3]=talpha[tindic_x_min]-1.;
        talpha[3] = talpha[tindic_x_min] - 1.;
        /*if(talpha[tindic_x_min]<talpha[tindic_x_max])
          talpha[3]=talpha[tindic_x_max]+1.;
        else
          talpha[3]=talpha[tindic_x_max]-1.;*/
      }
      // std::cout<<"concave talpha="<<talpha[3]<<std::endl;
    }
    // std::cout<<"talpha="<<talpha[3]<<std::endl;
    int indic_x_min = 0;
    int indic_x_max = 0;
    for (int i = 1; i < 3; i++) {
      if (talpha[i] < talpha[indic_x_min])
        indic_x_min = i;
      if (talpha[i] > talpha[indic_x_max])
        indic_x_max = i;
    }
    // std::cout<<"talpha = "<<talpha[3]<<std::endl;
    if (talpha[3] > talpha[indic_x_max])
      if ((talpha[3] - talpha[indic_x_max]) > alpha)
        talpha[3] = talpha[indic_x_max] + 4.;
    if (talpha[3] < talpha[indic_x_min])
      if ((talpha[indic_x_min] - talpha[3]) > alpha)
        talpha[3] = talpha[indic_x_min] - 4.;

    /*if(((b-a)*(Cost1-Cost2)-(b-c)*(Cost1-Cost0))==0)
    {Cost[3]=1000;break;}

    //calcul du gain correspondant au minimum de la parabole estimï¿œe
    talpha[3]=b-0.5*((b-a)*(b-a)*(Cost1-Cost2)-(b-c)*(b-c)*(Cost1-Cost0))/((b-a)*(Cost1-Cost2)-(b-c)*(Cost1-Cost0));
    int indic_x_min=0;
    int indic_x_max=0;
    for(int i=1;i<3;i++)
    {
      if(talpha[i]<talpha[indic_x_min])
        indic_x_min=i;
      if(talpha[i]>talpha[indic_x_max])
        indic_x_max=i;
    }
    std::cout<<"talpha = "<<talpha[3]<<std::endl;
    if(talpha[3]>talpha[indic_x_max])
      if((talpha[3]-talpha[indic_x_max])>alpha)talpha[3]=talpha[indic_x_max]+alpha;
    if(talpha[3]<talpha[indic_x_min])
      if((talpha[indic_x_min]-talpha[3])>alpha)talpha[3]=talpha[indic_x_min]-alpha;*/

    // p3=tp-talpha[3]*direction;
    if (useCompositionnal) {
      adpt = talpha[3] * direction;
      if (useInverse)
        Warp->getParamInverse(adpt, dpt);
      else
        dpt = adpt;
      Warp->pRondp(tp, dpt, p3);
    } else {
      p3 = tp + talpha[3] * direction;
    }

    Cost[3] = getCost(I, p3);
    // std::cout<<"new cost="<<Cost[3]<<std::endl;

    int indice_f_max = 0;
    for (int i = 1; i < 4; i++)
      if (Cost[i] > Cost[indice_f_max])
        indice_f_max = i;
    if (indice_f_max != 3) {
      *ptp[indice_f_max] = *ptp[3];
      Cost[indice_f_max] = Cost[3];
      talpha[indice_f_max] = talpha[3];
    } else
      break;
  }

  int indice_f_min = 0;
  for (int i = 0; i < 4; i++)
    if (Cost[i] < Cost[indice_f_min])
      indice_f_min = i;

  alpha = talpha[indice_f_min];

  if (alpha < 1)
    alpha = 1.;

  delete[] ptp;
  delete[] Cost;
  delete[] talpha;
}

/*!
  \param nbLvl : Number of levels in the pyramid.
  \param l0 : Pyramid level where the tracking is stopped. The level with the
  highest resolution is 0.
 */
void vpTemplateTracker::initPyramidal(unsigned int nbLvl, unsigned int l0)
{
  // 	vpTRACE("init_pyramidal");
  nbLvlPyr = nbLvl;
  l0Pyr = l0;

  zoneTrackedPyr = new vpTemplateTrackerZone[nbLvlPyr];
  pyr_IDes = new vpImage<unsigned char>[nbLvlPyr];
  ptTemplatePyr = new vpTemplateTrackerPoint *[nbLvlPyr];
  ptTemplateSelectPyr = new bool *[nbLvlPyr];
  ptTemplateSuppPyr = new vpTemplateTrackerPointSuppMIInv *[nbLvlPyr];
  ptTemplateCompoPyr = new vpTemplateTrackerPointCompo *[nbLvlPyr];
  for (unsigned int i = 0; i < nbLvlPyr; i++) {
    ptTemplatePyr[i] = NULL;
    ptTemplateSuppPyr[i] = NULL;
    ptTemplateSelectPyr[i] = NULL;
    ptTemplateCompoPyr[i] = NULL;
  }
  templateSizePyr = new unsigned int[nbLvlPyr];
  HdesirePyr = new vpMatrix[nbLvlPyr];
  HLMdesirePyr = new vpMatrix[nbLvlPyr];
  HLMdesireInversePyr = new vpMatrix[nbLvlPyr];

  pyrInitialised = true;
  // 	vpTRACE("fin init_pyramidal");
}
void vpTemplateTracker::initTrackingPyr(const vpImage<unsigned char> &I, vpTemplateTrackerZone &zone)
{
  // 	vpTRACE("initTrackingPyr");
  zoneTrackedPyr[0].copy(zone);
  // vpTRACE("fin copy zone");

  pyr_IDes[0] = I;
  initTracking(pyr_IDes[0], zoneTrackedPyr[0]);
  ptTemplatePyr[0] = ptTemplate;
  ptTemplateSelectPyr[0] = ptTemplateSelect;
  templateSizePyr[0] = templateSize;

  // creationpyramide de zones et images desiree
  if (nbLvlPyr > 1) {
    for (unsigned int i = 1; i < nbLvlPyr; i++) {
      zoneTrackedPyr[i] = zoneTrackedPyr[i - 1].getPyramidDown();
      vpImageFilter::getGaussPyramidal(pyr_IDes[i - 1], pyr_IDes[i]);

      initTracking(pyr_IDes[i], zoneTrackedPyr[i]);
      ptTemplatePyr[i] = ptTemplate;
      ptTemplateSelectPyr[i] = ptTemplateSelect;
      templateSizePyr[i] = templateSize;
      // reste probleme avec le Hessien
    }
  }
  /*for(int i=0;i<nbLvlPyr;i++)
  {
    vpColVector ptemp(8);ptemp=0;
    zoneTracked=&zoneTrackedPyr[i];
    init_pos_evalRMS(ptemp);
  }*/
  zoneTracked = &zoneTrackedPyr[0];

  // 	vpTRACE("fin initTrackingPyr");
}

/*!
  Select the reference template in image \e I using mouse click.

  \param I: Image containing the reference template.
  \param delaunay: Flag used to enable Delaunay triangulation.
  - If true, from the image points selected by the user, a Delaunay
  triangulation is performed to initialize the reference template.
    - A left click select a image point;
    - A right click select the last image point and ends the initialisation
  stage.
  - If false, the user select directly points as successive triangle corners.
    The size of \e v_ip vector should be a multiple of 3. It is not mandatory
    that triangles have one edge in common; they can define a discontinued
  area.
    - A left click select a triangle corner;
    - A right click select the last triangle corner and ends the
  initialisation stage. For example, to select the reference template as two
  triangles, the user has to left click five times and finish the selection on
  the sixth corner with a right click.

 */
void vpTemplateTracker::initClick(const vpImage<unsigned char> &I, bool delaunay)
{
  zoneRef_.initClick(I, delaunay);

  if (nbLvlPyr > 1) {
    initPyramidal(nbLvlPyr, l0Pyr);
    initTrackingPyr(I, zoneRef_);
    initHessienDesiredPyr(I);
  } else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
    //    trackNoPyr(I);
  }
}

/*!
  Initialize the reference template from a vector of points.

  \param I: Image containing the reference template.
  \param v_ip: Vector of image points defining the reference template.
  \param delaunay:
  - If true, from the image points defining the reference template enable
  Delaunay triangulation.
  - If false, the vector of image points define the reference template as a
  list of triangles. The size of \e v_ip vector should be a multiple of 3.
 */
void vpTemplateTracker::initFromPoints(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &v_ip,
                                       bool delaunay)
{
  zoneRef_.initFromPoints(I, v_ip, delaunay);

  if (nbLvlPyr > 1) {
    initPyramidal(nbLvlPyr, l0Pyr);
    initTrackingPyr(I, zoneRef_);
    initHessienDesiredPyr(I);
  } else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
    // trackNoPyr(I);
  }
}

/*!
  Initialize the reference template from a vector of points.

  \param I: Image containing the reference template.
  \param zone: The zone that describes the reference template.
 */
void vpTemplateTracker::initFromZone(const vpImage<unsigned char> &I, const vpTemplateTrackerZone &zone)
{
  zoneRef_ = zone;

  if (nbLvlPyr > 1) {
    initPyramidal(nbLvlPyr, l0Pyr);
    initTrackingPyr(I, zoneRef_);
    initHessienDesiredPyr(I);
  } else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
    //    trackNoPyr(I);
  }
}

void vpTemplateTracker::initHessienDesiredPyr(const vpImage<unsigned char> &I)
{
  // 	vpTRACE("initHessienDesiredPyr");

  templateSize = templateSizePyr[0];
  // ptTemplateSupp=ptTemplateSuppPyr[0];
  // ptTemplateCompo=ptTemplateCompoPyr[0];
  ptTemplate = ptTemplatePyr[0];
  ptTemplateSelect = ptTemplateSelectPyr[0];
  //  ptTemplateSupp=new vpTemplateTrackerPointSuppMIInv[templateSize];
  try {
    initHessienDesired(I);
    ptTemplateSuppPyr[0] = ptTemplateSupp;
    ptTemplateCompoPyr[0] = ptTemplateCompo;
    HdesirePyr[0] = Hdesire;
    HLMdesirePyr[0] = HLMdesire;
    HLMdesireInversePyr[0] = HLMdesireInverse;
  } catch (const vpException &e) {
    ptTemplateSuppPyr[0] = ptTemplateSupp;
    ptTemplateCompoPyr[0] = ptTemplateCompo;
    HdesirePyr[0] = Hdesire;
    HLMdesirePyr[0] = HLMdesire;
    HLMdesireInversePyr[0] = HLMdesireInverse;
    throw(e);
  }

  if (nbLvlPyr > 1) {
    vpImage<unsigned char> Itemp;
    Itemp = I;
    for (unsigned int i = 1; i < nbLvlPyr; i++) {
      vpImageFilter::getGaussPyramidal(Itemp, Itemp);

      templateSize = templateSizePyr[i];
      ptTemplate = ptTemplatePyr[i];
      ptTemplateSelect = ptTemplateSelectPyr[i];
      // ptTemplateSupp=ptTemplateSuppPyr[i];
      // ptTemplateCompo=ptTemplateCompoPyr[i];
      try {
        initHessienDesired(Itemp);
        ptTemplateSuppPyr[i] = ptTemplateSupp;
        ptTemplateCompoPyr[i] = ptTemplateCompo;
        HdesirePyr[i] = Hdesire;
        HLMdesirePyr[i] = HLMdesire;
        HLMdesireInversePyr[i] = HLMdesireInverse;
      } catch (const vpException &e) {
        ptTemplateSuppPyr[i] = ptTemplateSupp;
        ptTemplateCompoPyr[i] = ptTemplateCompo;
        HdesirePyr[i] = Hdesire;
        HLMdesirePyr[i] = HLMdesire;
        HLMdesireInversePyr[i] = HLMdesireInverse;
        throw(e);
      }
    }
  }
  // 	vpTRACE("fin initHessienDesiredPyr");
}

/*!
   Track the template on image \e I.
   \param I: Image to process.
 */
void vpTemplateTracker::track(const vpImage<unsigned char> &I)
{
  if (nbLvlPyr > 1)
    trackPyr(I);
  else
    trackNoPyr(I);
}

void vpTemplateTracker::trackPyr(const vpImage<unsigned char> &I)
{
  // vpTRACE("trackPyr");
  vpImage<unsigned char> *pyr_I;
  //  pyr_I=new vpImage<unsigned char>[nbLvlPyr+1]; // Why +1 ?
  pyr_I = new vpImage<unsigned char>[nbLvlPyr]; // Why +1 ?
  pyr_I[0] = I;

  try {
    vpColVector ptemp(nbParam);
    if (nbLvlPyr > 1) {
      //    vpColVector *p_sauv=new vpColVector[nbLvlPyr];
      //    for(unsigned int i=0;i<nbLvlPyr;i++)p_sauv[i].resize(nbParam);

      //    p_sauv[0]=p;
      for (unsigned int i = 1; i < nbLvlPyr; i++) {
        vpImageFilter::getGaussPyramidal(pyr_I[i - 1], pyr_I[i]);
        // test getParamPyramidDown
        /*vpColVector vX_test(2);vX_test[0]=15.;vX_test[1]=30.;
        vpColVector vX_test2(2);
        Warp->computeCoeff(p);
        Warp->computeDenom(vX_test,p);
        Warp->warpX(vX_test,vX_test2,p);
        std::cout<<"p = "<<p.t()<<std::endl;*/
        // std::cout<<"get p down"<<std::endl;
        Warp->getParamPyramidDown(p, ptemp);
        p = ptemp;
        zoneTracked = &zoneTrackedPyr[i];

        //      p_sauv[i]=p;
        /*std::cout<<"p_down = "<<p.t()<<std::endl;

        vpColVector vX_testd(2);vX_testd[0]=15./2.;vX_testd[1]=30./2.;
        vpColVector vX_testd2(2);
        Warp->computeCoeff(p);
        Warp->computeDenom(vX_testd,p);
        Warp->warpX(vX_testd,vX_testd2,p);
        std::cout<<2.*vX_testd2[0]<<","<<2.*vX_testd2[1]<<" <=>
        "<<vX_test2[0]<<","<<vX_test2[1]<<std::endl;*/
      }

      for (int i = (int)nbLvlPyr - 1; i >= 0; i--) {
        if (i >= (int)l0Pyr) {
          templateSize = templateSizePyr[i];
          ptTemplate = ptTemplatePyr[i];
          ptTemplateSelect = ptTemplateSelectPyr[i];
          ptTemplateSupp = ptTemplateSuppPyr[i];
          ptTemplateCompo = ptTemplateCompoPyr[i];
          H = HdesirePyr[i];
          HLM = HLMdesirePyr[i];
          HLMdesireInverse = HLMdesireInversePyr[i];
          //        zoneTracked=&zoneTrackedPyr[i];
          trackRobust(pyr_I[i]);
        }
        // std::cout<<"get p up"<<std::endl;
        //      ptemp=p_sauv[i-1];
        if (i > 0) {
          Warp->getParamPyramidUp(p, ptemp);
          p = ptemp;
          zoneTracked = &zoneTrackedPyr[i - 1];
        }
      }
#if 0
        if(l0Pyr==0)
        {
          templateSize=templateSizePyr[0];
          ptTemplate=ptTemplatePyr[0];
          ptTemplateSelect=ptTemplateSelectPyr[0];
          ptTemplateSupp=ptTemplateSuppPyr[0];
          ptTemplateCompo=ptTemplateCompoPyr[0];
          H=HdesirePyr[0];
          HLM=HLMdesirePyr[0];
          HLMdesireInverse=HLMdesireInversePyr[0];
          zoneTracked=&zoneTrackedPyr[0];
          trackRobust(pyr_I[0]);
        }

        if (l0Pyr > 0) {
    //      for (int l=(int)l0Pyr; l >=0; l--) {
    //        Warp->getParamPyramidUp(p,ptemp);
    //        p=ptemp;
    //      }
          zoneTracked=&zoneTrackedPyr[0];
        }
#endif
      //    delete [] p_sauv;
    } else {
      // std::cout<<"reviens a tracker de base"<<std::endl;
      trackRobust(I);
    }
    delete[] pyr_I;
  } catch (const vpException &e) {
    delete[] pyr_I;
    throw(vpTrackingException(vpTrackingException::badValue, e.getMessage()));
  }
}

void vpTemplateTracker::trackRobust(const vpImage<unsigned char> &I)
{
  if (costFunctionVerification) {
    vpColVector p_pre_estimation;
    p_pre_estimation = p;
    getGaussianBluredImage(I);
    double pre_fcost = getCost(I, p);

    trackNoPyr(I);

    // std::cout<<"fct avant : "<<pre_fcost<<std::endl;
    double post_fcost = getCost(I, p);
    // std::cout<<"fct apres : "<<post_fcost<<std::endl<<std::endl;
    if (pre_fcost < post_fcost)
      p = p_pre_estimation;
  } else
    trackNoPyr(I);
}
