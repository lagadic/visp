/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 *
*****************************************************************************/

#include <visp3/tt/vpTemplateTracker.h>
#include <visp3/tt/vpTemplateTrackerBSpline.h>

BEGIN_VISP_NAMESPACE
vpTemplateTracker::vpTemplateTracker(vpTemplateTrackerWarp *_warp)
  : nbLvlPyr(1), l0Pyr(0), pyrInitialised(false), evolRMS(0), x_pos(), y_pos(), evolRMS_eps(1e-4), ptTemplate(nullptr),
  ptTemplatePyr(nullptr), ptTemplateInit(false), templateSize(0), templateSizePyr(nullptr), ptTemplateSelect(nullptr),
  ptTemplateSelectPyr(nullptr), ptTemplateSelectInit(false), templateSelectSize(0), ptTemplateSupp(nullptr),
  ptTemplateSuppPyr(nullptr), ptTemplateCompo(nullptr), ptTemplateCompoPyr(nullptr), zoneTracked(nullptr), zoneTrackedPyr(nullptr),
  pyr_IDes(nullptr), H(), Hdesire(), HdesirePyr(), HLM(), HLMdesire(), HLMdesirePyr(), HLMdesireInverse(),
  HLMdesireInversePyr(), G(), gain(1.), thresholdGradient(40), costFunctionVerification(false), blur(true),
  useBrent(false), nbIterBrent(3), taillef(7), fgG(nullptr), fgdG(nullptr), ratioPixelIn(0), mod_i(1), mod_j(1), nbParam(0),
  lambdaDep(0.001), iterationMax(30), iterationGlobale(0), diverge(false), nbIteration(0), useCompositionnal(true),
  useInverse(false), Warp(_warp), p(0), dp(), X1(), X2(), dW(), BI(), dIx(), dIy(), zoneRef_()
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
  zoneTracked = &zone;

  int largeur_im = (int)I.getWidth();
  int hauteur_im = (int)I.getHeight();

  unsigned int NbPointDsZone = 0;
  int mod_fi, mod_fj;
  mod_fi = mod_i;
  mod_fj = mod_i;

  for (int i = 0; i < hauteur_im; i += mod_fi) {
    for (int j = 0; j < largeur_im; j += mod_fj) {
      if (zone.inZone(i, j)) {
        NbPointDsZone++;
      }
    }
  }

  templateSize = NbPointDsZone;
  ptTemplate = new vpTemplateTrackerPoint[templateSize];
  ptTemplateInit = true;
  ptTemplateSelect = new bool[templateSize];
  ptTemplateSelectInit = true;

  Hdesire.resize(nbParam, nbParam);
  HLMdesire.resize(nbParam, nbParam);

  vpTemplateTrackerPoint pt;
  vpImage<double> GaussI;
  vpImageFilter::filter(I, GaussI, fgG, taillef);
  vpImageFilter::getGradXGauss2D(I, dIx, fgG, fgdG, taillef);
  vpImageFilter::getGradYGauss2D(I, dIy, fgG, fgdG, taillef);

  unsigned int cpt_point = 0;
  templateSelectSize = 0;
  for (int i = 0; i < hauteur_im; i += mod_i) {
    for (int j = 0; j < largeur_im; j += mod_j) {
      if (zone.inZone(i, j)) {
        pt.x = j;
        pt.y = i;

        pt.dx = dIx[i][j];
        pt.dy = dIy[i][j];

        if (pt.dx * pt.dx + pt.dy * pt.dy > thresholdGradient) {
          ptTemplateSelect[cpt_point] = true;
          templateSelectSize++;
        }
        else {
          ptTemplateSelect[cpt_point] = false;
        }
        pt.val = vpTemplateTrackerBSpline::getSubPixBspline4(GaussI, i, j);

        ptTemplate[cpt_point] = pt;
        cpt_point++;
      }
    }
  }

  templateSize = cpt_point;
  GaussI.destroy();
}

vpTemplateTracker::~vpTemplateTracker()
{
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
      ptTemplatePyr = nullptr;
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
      ptTemplateCompoPyr = nullptr;
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
      ptTemplateSuppPyr = nullptr;
    }

    if (ptTemplateSelectPyr) {
      for (unsigned int i = 0; i < nbLvlPyr; i++) {
        if (ptTemplateSelectPyr[i])
          delete[] ptTemplateSelectPyr[i];
      }
      delete[] ptTemplateSelectPyr;
      ptTemplateSelectPyr = nullptr;
    }

    if (templateSizePyr) {
      delete[] templateSizePyr;
      templateSizePyr = nullptr;
    }

    if (HdesirePyr) {
      delete[] HdesirePyr;
      HdesirePyr = nullptr;
    }

    if (HLMdesirePyr) {
      delete[] HLMdesirePyr;
      HLMdesirePyr = nullptr;
    }

    if (HLMdesireInversePyr) {
      delete[] HLMdesireInversePyr;
      HLMdesireInversePyr = nullptr;
    }

    if (zoneTrackedPyr) {
      delete[] zoneTrackedPyr;
      zoneTrackedPyr = nullptr;
    }

    if (pyr_IDes) {
      delete[] pyr_IDes;
      pyr_IDes = nullptr;
    }
  }
  else {
    if (ptTemplateInit) {
      for (unsigned int point = 0; point < templateSize; point++) {
        delete[] ptTemplate[point].dW;
        delete[] ptTemplate[point].HiG;
      }
      delete[] ptTemplate;
      ptTemplate = nullptr;
      ptTemplateInit = false;
    }
    if (ptTemplateCompo) {
      for (unsigned int point = 0; point < templateSize; point++) {
        delete[] ptTemplateCompo[point].dW;
      }
      delete[] ptTemplateCompo;
      ptTemplateCompo = nullptr;
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
      ptTemplateSupp = nullptr;
    }
    if (ptTemplateSelectInit) {
      if (ptTemplateSelect) {
        delete[] ptTemplateSelect;
        ptTemplateSelect = nullptr;
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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
void vpTemplateTracker::display(const vpImage<unsigned char> &I, const vpColor &col, unsigned int thickness)
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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
void vpTemplateTracker::display(const vpImage<vpRGBa> &I, const vpColor &col, unsigned int thickness)
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
  }
  else {
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
  }
  else {
    p2 = tp + alpha * direction;
  }
  vpColVector p3(nbParam);
  ptp[0] = &p0;
  ptp[1] = &p1;
  ptp[2] = &p2;
  ptp[3] = &p3;

  double *Cost = new double[4];
  Cost[0] = tMI;
  Cost[1] = getCost(I, p1);
  Cost[2] = getCost(I, p2);

  double *talpha = new double[4];
  talpha[0] = 0;
  talpha[1] = 1.;
  talpha[2] = alpha;

  // Utilise trois estimees de paraboles successive ...
  // A changer pour rendre adaptable
  for (unsigned int opt = 0; opt < nbIterBrent; opt++) {
    vpMatrix A(3, 3);
    for (unsigned int i = 0; i < 3; i++) {
      A[i][0] = talpha[i] * talpha[i];
      A[i][1] = talpha[i];
      A[i][2] = 1.;
    }
    vpColVector B(3);
    for (unsigned int i = 0; i < 3; i++)
      B[i] = Cost[i];
    vpColVector parabol(3);
    parabol = (A.t() * A).inverseByLU() * A.t() * B;

    // If convexe
    if (parabol[0] > 0) {
      talpha[3] = -0.5 * parabol[1] / parabol[0];
    }
    else { // If concave
      int tindic_x_min = 0;
      int tindic_x_max = 0;
      for (int i = 1; i < 3; i++) {
        if (talpha[i] < talpha[tindic_x_min])
          tindic_x_min = i;
        if (talpha[i] > talpha[tindic_x_max])
          tindic_x_max = i;
      }

      if (Cost[tindic_x_max] < Cost[tindic_x_min]) {
        talpha[3] = talpha[tindic_x_max] + 1.;
      }
      else {
        talpha[3] = talpha[tindic_x_min] - 1.;
      }
    }
    int indic_x_min = 0;
    int indic_x_max = 0;
    for (int i = 1; i < 3; i++) {
      if (talpha[i] < talpha[indic_x_min])
        indic_x_min = i;
      if (talpha[i] > talpha[indic_x_max])
        indic_x_max = i;
    }
    if (talpha[3] > talpha[indic_x_max])
      if ((talpha[3] - talpha[indic_x_max]) > alpha)
        talpha[3] = talpha[indic_x_max] + 4.;
    if (talpha[3] < talpha[indic_x_min])
      if ((talpha[indic_x_min] - talpha[3]) > alpha)
        talpha[3] = talpha[indic_x_min] - 4.;

    if (useCompositionnal) {
      adpt = talpha[3] * direction;
      if (useInverse)
        Warp->getParamInverse(adpt, dpt);
      else
        dpt = adpt;
      Warp->pRondp(tp, dpt, p3);
    }
    else {
      p3 = tp + talpha[3] * direction;
    }

    Cost[3] = getCost(I, p3);

    int indice_f_max = 0;
    for (int i = 1; i < 4; i++)
      if (Cost[i] > Cost[indice_f_max])
        indice_f_max = i;
    if (indice_f_max != 3) {
      *ptp[indice_f_max] = *ptp[3];
      Cost[indice_f_max] = Cost[3];
      talpha[indice_f_max] = talpha[3];
    }
    else
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
  nbLvlPyr = nbLvl;
  l0Pyr = l0;

  zoneTrackedPyr = new vpTemplateTrackerZone[nbLvlPyr];
  pyr_IDes = new vpImage<unsigned char>[nbLvlPyr];
  ptTemplatePyr = new vpTemplateTrackerPoint *[nbLvlPyr];
  ptTemplateSelectPyr = new bool *[nbLvlPyr];
  ptTemplateSuppPyr = new vpTemplateTrackerPointSuppMIInv *[nbLvlPyr];
  ptTemplateCompoPyr = new vpTemplateTrackerPointCompo *[nbLvlPyr];
  for (unsigned int i = 0; i < nbLvlPyr; i++) {
    ptTemplatePyr[i] = nullptr;
    ptTemplateSuppPyr[i] = nullptr;
    ptTemplateSelectPyr[i] = nullptr;
    ptTemplateCompoPyr[i] = nullptr;
  }
  templateSizePyr = new unsigned int[nbLvlPyr];
  HdesirePyr = new vpMatrix[nbLvlPyr];
  HLMdesirePyr = new vpMatrix[nbLvlPyr];
  HLMdesireInversePyr = new vpMatrix[nbLvlPyr];

  pyrInitialised = true;
}

void vpTemplateTracker::initTrackingPyr(const vpImage<unsigned char> &I, vpTemplateTrackerZone &zone)
{
  zoneTrackedPyr[0].copy(zone);

  pyr_IDes[0] = I;
  initTracking(pyr_IDes[0], zoneTrackedPyr[0]);
  ptTemplatePyr[0] = ptTemplate;
  ptTemplateSelectPyr[0] = ptTemplateSelect;
  templateSizePyr[0] = templateSize;

  // creation pyramide de zones et images desiree
  if (nbLvlPyr > 1) {
    for (unsigned int i = 1; i < nbLvlPyr; i++) {
      zoneTrackedPyr[i] = zoneTrackedPyr[i - 1].getPyramidDown();
      vpImageFilter::getGaussPyramidal(pyr_IDes[i - 1], pyr_IDes[i]);

      initTracking(pyr_IDes[i], zoneTrackedPyr[i]);
      ptTemplatePyr[i] = ptTemplate;
      ptTemplateSelectPyr[i] = ptTemplateSelect;
      templateSizePyr[i] = templateSize;
    }
  }
  zoneTracked = &zoneTrackedPyr[0];
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
  }
  else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
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
  }
  else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
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
  }
  else {
    initTracking(I, zoneRef_);
    initHessienDesired(I);
  }
}

void vpTemplateTracker::initHessienDesiredPyr(const vpImage<unsigned char> &I)
{
  templateSize = templateSizePyr[0];
  ptTemplate = ptTemplatePyr[0];
  ptTemplateSelect = ptTemplateSelectPyr[0];
  try {
    initHessienDesired(I);
    ptTemplateSuppPyr[0] = ptTemplateSupp;
    ptTemplateCompoPyr[0] = ptTemplateCompo;
    HdesirePyr[0] = Hdesire;
    HLMdesirePyr[0] = HLMdesire;
    HLMdesireInversePyr[0] = HLMdesireInverse;
  }
  catch (const vpException &e) {
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
      try {
        initHessienDesired(Itemp);
        ptTemplateSuppPyr[i] = ptTemplateSupp;
        ptTemplateCompoPyr[i] = ptTemplateCompo;
        HdesirePyr[i] = Hdesire;
        HLMdesirePyr[i] = HLMdesire;
        HLMdesireInversePyr[i] = HLMdesireInverse;
      }
      catch (const vpException &e) {
        ptTemplateSuppPyr[i] = ptTemplateSupp;
        ptTemplateCompoPyr[i] = ptTemplateCompo;
        HdesirePyr[i] = Hdesire;
        HLMdesirePyr[i] = HLMdesire;
        HLMdesireInversePyr[i] = HLMdesireInverse;
        throw(e);
      }
    }
  }
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
  vpImage<unsigned char> *pyr_I;
  pyr_I = new vpImage<unsigned char>[nbLvlPyr]; // Why +1 ?
  pyr_I[0] = I;

  try {
    vpColVector ptemp(nbParam);
    if (nbLvlPyr > 1) {
      for (unsigned int i = 1; i < nbLvlPyr; i++) {
        vpImageFilter::getGaussPyramidal(pyr_I[i - 1], pyr_I[i]);
        Warp->getParamPyramidDown(p, ptemp);
        p = ptemp;
        zoneTracked = &zoneTrackedPyr[i];
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
          trackRobust(pyr_I[i]);
        }
        if (i > 0) {
          Warp->getParamPyramidUp(p, ptemp);
          p = ptemp;
          zoneTracked = &zoneTrackedPyr[i - 1];
        }
      }
    }
    else {
      trackRobust(I);
    }
    delete[] pyr_I;
  }
  catch (const vpException &e) {
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

    double post_fcost = getCost(I, p);
    if (pre_fcost < post_fcost) {
      p = p_pre_estimation;
    }
  }
  else {
    trackNoPyr(I);
  }
}

/*!
  Compute residual. Before using this function you need to call initPosEvalRMS() once.
  \param[in] param : Warp function parameters.

  \sa initPosEvalRMS()
 */
void vpTemplateTracker::computeEvalRMS(const vpColVector &param)
{
  unsigned int nb_corners = zoneTracked->getNbTriangle() * 3;

  Warp->computeCoeff(param);
  evolRMS = 0;
  vpTemplateTrackerTriangle triangle;

  for (unsigned int i = 0; i < zoneTracked->getNbTriangle(); i++) {
    zoneTracked->getTriangle(i, triangle);
    for (unsigned int j = 0; j < 3; j++) {
      triangle.getCorner(j, X1[0], X1[1]);

      Warp->computeDenom(X1, param);
      Warp->warpX(X1, X2, param);

      unsigned int index = i * 3 + j;
      double x_ = x_pos[index] - X2[0];
      double y_ = y_pos[index] - X2[1];
      evolRMS += x_ * x_ + y_ * y_;
      x_pos[index] = X2[0];
      y_pos[index] = X2[1];
    }
  }
  evolRMS /= nb_corners;
}

/*!
  Initialize residual computed using computeEvalRMS().
  \param[in] param : Warp function parameters.

  \sa computeEvalRMS()
 */
void vpTemplateTracker::initPosEvalRMS(const vpColVector &param)
{
  unsigned int nb_corners = zoneTracked->getNbTriangle() * 3;
  x_pos.resize(nb_corners);
  y_pos.resize(nb_corners);

  Warp->computeCoeff(param);
  vpTemplateTrackerTriangle triangle;

  for (unsigned int i = 0; i < zoneTracked->getNbTriangle(); i++) {
    unsigned int i3 = i * 3;
    zoneTracked->getTriangle(i, triangle);
    for (unsigned int j = 0; j < 3; j++) {
      triangle.getCorner(j, X1[0], X1[1]);

      Warp->computeDenom(X1, param);
      Warp->warpX(X1, X2, param);
      x_pos[i3 + j] = X2[0];
      y_pos[i3 + j] = X2[1];
    }
  }
}
END_VISP_NAMESPACE
