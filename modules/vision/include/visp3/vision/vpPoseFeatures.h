/*
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
 * Pose computation from any features.
 */

/*!
  \file vpPose.h
  \brief Tools for pose computation from any feature.
*/

#ifndef vpPoseFeatures_HH
#define vpPoseFeatures_HH

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MODULE_VISUAL_FEATURES) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

#include <visp3/core/vpCircle.h>
#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpForwardProjection.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpSphere.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>
#include <visp3/visual_features/vpFeaturePoint.h>

#include <iostream>
#include <vector>
#include <tuple>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
//#################################################
//##  Call a function with a tuple as parameters
//#################################################
template <unsigned int N> struct vpDesiredFeatureBuilderWithTuple
{
  template <typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args>
  static void buildDesiredFeatureWithTuple(featureType &feature, RetType(*f)(ArgsF...), const std::tuple<ArgsT...> &t,
                                           Args &&...args)
  {
    vpDesiredFeatureBuilderWithTuple<N - 1>::buildDesiredFeatureWithTuple(feature, f, t, std::get<N - 1>(t), args...);
  }
};

template <> struct vpDesiredFeatureBuilderWithTuple<0>
{
  template <typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args>
  static void buildDesiredFeatureWithTuple(featureType & /* feature */, RetType(*f)(ArgsF...),
                                           const std::tuple<ArgsT...> & /* t */, Args &&...args)
  {
    f(args...);
  }
};

template <> struct vpDesiredFeatureBuilderWithTuple<1>
{
  template <typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args>
  static void buildDesiredFeatureWithTuple(featureType &feature, RetType(*f)(ArgsF...), const std::tuple<ArgsT...> &t,
                                           Args &&...args)
  {
    vpDesiredFeatureBuilderWithTuple<0>::buildDesiredFeatureWithTuple(feature, f, t, feature, args...);
  }
};

template <typename featureType, typename RetType, typename... Args, typename... ArgsFunc>
void buildDesiredFeatureWithTuple(featureType &feature, RetType(*f)(ArgsFunc...), std::tuple<Args...> const &t)
{
  vpDesiredFeatureBuilderWithTuple<sizeof...(Args)>::buildDesiredFeatureWithTuple(feature, f, t);
}

//#################################################
//##  Call a function with a tuple as parameters
//##  Object Mode
//#################################################

template <unsigned int N> struct vpDesiredFeatureBuilderObjectWithTuple
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT,
    typename... Args>
  static void buildDesiredFeatureObjectWithTuple(objType *obj, featureType &feature, RetType(objType:: *f)(ArgsF...),
                                                 const std::tuple<ArgsT...> &t, Args &&...args)
  {
    vpDesiredFeatureBuilderObjectWithTuple<N - 1>::buildDesiredFeatureObjectWithTuple(obj, feature, f, t,
                                                                                      std::get<N - 1>(t), args...);
  }
};

template <> struct vpDesiredFeatureBuilderObjectWithTuple<0>
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT,
    typename... Args>
  static void buildDesiredFeatureObjectWithTuple(objType *obj, featureType & /*feature*/,
                                                 RetType(objType:: *f)(ArgsF...), const std::tuple<ArgsT...> & /* t */,
                                                 Args &&...args)
  {
    (obj->*f)(args...);
  }
};

template <> struct vpDesiredFeatureBuilderObjectWithTuple<1>
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT,
    typename... Args>
  static void buildDesiredFeatureObjectWithTuple(objType *obj, featureType &feature, RetType(objType:: *f)(ArgsF...),
                                                 const std::tuple<ArgsT...> &t, Args &&...args)
  {
    vpDesiredFeatureBuilderObjectWithTuple<0>::buildDesiredFeatureObjectWithTuple(obj, feature, f, t, feature, args...);
  }
};

template <typename objType, typename featureType, typename RetType, typename... Args, typename... ArgsFunc>
void buildDesiredFeatureObjectWithTuple(objType *obj, featureType &feature, RetType(objType:: *f)(ArgsFunc...),
                                        std::tuple<Args...> const &t)
{
  vpDesiredFeatureBuilderObjectWithTuple<sizeof...(Args)>::buildDesiredFeatureObjectWithTuple(obj, feature, f, t);
}

//#####################################################
//##  Call un function with a tuple as parameters
//##  Track all the parameters with the cMo
//##  Except the first one (must be de "BasicFeature"
//#####################################################

template <unsigned int N> struct vpCurrentFeatureBuilderWithTuple
{
  template <typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureWithTuple(featureType &feature, const vpHomogeneousMatrix &cMo, RetType(*f)(ArgsF...),
                                           std::tuple<ArgsTuple...> &t, ArgsDecomposed &&...args)
  {
    auto proj = std::get<N - 1>(t);
    proj.track(cMo);
    vpCurrentFeatureBuilderWithTuple<N - 1>::buildCurrentFeatureWithTuple(feature, cMo, f, t, proj, args...);
  }
};

template <> struct vpCurrentFeatureBuilderWithTuple<0>
{
  template <typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureWithTuple(featureType & /*feature*/, const vpHomogeneousMatrix & /*cMo*/,
                                           RetType(*f)(ArgsF...), std::tuple<ArgsTuple...> &, ArgsDecomposed &&...args)
  {
    f(args...);
  }
};

template <> struct vpCurrentFeatureBuilderWithTuple<1>
{
  template <typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureWithTuple(featureType &feature, const vpHomogeneousMatrix &cMo, RetType(*f)(ArgsF...),
                                           std::tuple<ArgsTuple...> &t, ArgsDecomposed &&...args)
  {
    vpCurrentFeatureBuilderWithTuple<0>::buildCurrentFeatureWithTuple(feature, cMo, f, t, feature, args...);
  }
};

template <typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsFunc>
void buildCurrentFeatureWithTuple(featureType &feature, const vpHomogeneousMatrix &cMo, RetType(*f)(ArgsFunc...),
                                  std::tuple<ArgsTuple...> &t)
{
  vpCurrentFeatureBuilderWithTuple<sizeof...(ArgsTuple)>::buildCurrentFeatureWithTuple(feature, cMo, f, t);
}

//#####################################################
//##  Call un function with a tuple as parameters
//##  Track all the parameters with the cMo
//##  Except the first one (must be de "BasicFeature"
//##  Object Mode
//#####################################################

template <unsigned int N> struct vpCurrentFeatureBuilderObjectWithTuple
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureObjectWithTuple(objType *obj, featureType &feature, const vpHomogeneousMatrix &cMo,
                                                 RetType(objType:: *f)(ArgsF...), std::tuple<ArgsTuple...> &t,
                                                 ArgsDecomposed &&...args)
  {
    auto proj = std::get<N - 1>(t);
    proj.track(cMo);
    vpCurrentFeatureBuilderObjectWithTuple<N - 1>::buildCurrentFeatureObjectWithTuple(obj, feature, cMo, f, t, proj,
                                                                                      args...);
  }
};

template <> struct vpCurrentFeatureBuilderObjectWithTuple<0>
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureObjectWithTuple(objType *obj, featureType & /*feature*/,
                                                 const vpHomogeneousMatrix & /*cMo*/, RetType(objType:: *f)(ArgsF...),
                                                 std::tuple<ArgsTuple...> &, ArgsDecomposed &&...args)
  {
    (obj->*f)(args...);
  }
};

template <> struct vpCurrentFeatureBuilderObjectWithTuple<1>
{
  template <typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed,
    typename... ArgsF>
  static void buildCurrentFeatureObjectWithTuple(objType *obj, featureType &feature, const vpHomogeneousMatrix &cMo,
                                                 RetType(objType:: *f)(ArgsF...), std::tuple<ArgsTuple...> &t,
                                                 ArgsDecomposed &&...args)
  {
    vpCurrentFeatureBuilderObjectWithTuple<0>::buildCurrentFeatureObjectWithTuple(obj, feature, cMo, f, t, feature,
                                                                                  args...);
  }
};

template <typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsFunc>
void buildCurrentFeatureObjectWithTuple(objType *obj, featureType &feature, const vpHomogeneousMatrix &cMo,
                                        RetType(objType:: *f)(ArgsFunc...), std::tuple<ArgsTuple...> &t)
{
  vpCurrentFeatureBuilderObjectWithTuple<sizeof...(ArgsTuple)>::buildCurrentFeatureObjectWithTuple(obj, feature, cMo, f,
                                                                                                   t);
}

//#################################################
//##  Call that will be used in our vpPoseFeatures
//##  to store the specific features.
//#################################################
/*!
 * \class vpPoseSpecificFeature
 * \ingroup group_vision_pose
 * \brief Class used to define specific features that could be considered in
 * pose estimation from visual features implemented in vpPoseFeatures.
 */
class VISP_EXPORT vpPoseSpecificFeature
{
public:
  vpPoseSpecificFeature() { }
  virtual ~vpPoseSpecificFeature() { }

  virtual vpColVector error() = 0;
  virtual vpMatrix currentInteraction() = 0;
  virtual void createDesired() = 0;
  virtual void createCurrent(const vpHomogeneousMatrix &cMo) = 0;
};

//#################################################
//##  Template for all kind of specific features
//#################################################

/*!
 * \class vpPoseSpecificFeatureTemplate
 * \ingroup group_vision_pose
 * \brief Template class that allows to estimate a pose from all kind of
 * specific features if the compiler support C++ 11.
 */
template <typename featureType, typename RetType, typename... Args>
class vpPoseSpecificFeatureTemplate : public vpPoseSpecificFeature
{
private:
  featureType m_desiredFeature;
  featureType m_currentFeature;
  std::tuple<Args...> *m_tuple;
  RetType(*func_ptr)(Args...);

public:
  vpPoseSpecificFeatureTemplate(RetType(*f_ptr)(Args...), Args &&...args)
  {
    func_ptr = f_ptr; // std::move(f_ptr);
    m_tuple = new std::tuple<Args...>(args...);
  }
  virtual ~vpPoseSpecificFeatureTemplate() VP_OVERRIDE
  {
    delete m_tuple;
  }

  virtual void createDesired() VP_OVERRIDE
  {
    buildDesiredFeatureWithTuple(m_desiredFeature, func_ptr, *m_tuple);
  }

  virtual vpColVector error() VP_OVERRIDE
  {
    // std::cout << "Getting S... : " << std::get<0>(*tuple).get_s() <<
    // std::endl;
    return m_currentFeature.error(m_desiredFeature);
  }

  virtual vpMatrix currentInteraction() VP_OVERRIDE
  {
    return m_currentFeature.interaction();
  }

  virtual void createCurrent(const vpHomogeneousMatrix &cMo) VP_OVERRIDE
  {
    buildCurrentFeatureWithTuple(m_currentFeature, cMo, func_ptr, *m_tuple);
  }
};

//#################################################
//##  Template for all kind of specific features
//##  Object Mode
//#################################################

/*!
 * \class vpPoseSpecificFeatureTemplateObject
 * \ingroup group_vision_pose
 * \brief Template class that allows to estimate a pose from all kind of
 * specific features if the compiler support C++ 11.
 */
template <typename ObjectType, typename featureType, typename RetType, typename... Args>
class vpPoseSpecificFeatureTemplateObject : public vpPoseSpecificFeature
{
private:
  featureType m_desiredFeature;
  featureType m_currentFeature;
  std::tuple<Args...> *m_tuple;
  RetType(ObjectType:: *func_ptr)(Args...);
  ObjectType *m_obj;

public:
  vpPoseSpecificFeatureTemplateObject(ObjectType *o, RetType(ObjectType:: *f_ptr)(Args...), Args &&...args)
  {
    func_ptr = f_ptr; // std::move(f_ptr);
    m_tuple = new std::tuple<Args...>(args...);
    m_obj = o;
  }

  virtual ~vpPoseSpecificFeatureTemplateObject() VP_OVERRIDE
  {
    delete m_tuple;
  }

  virtual void createDesired() VP_OVERRIDE
  {
    buildDesiredFeatureObjectWithTuple(m_obj, m_desiredFeature, func_ptr, *m_tuple);
  }

  virtual vpColVector error() VP_OVERRIDE
  {
    return m_currentFeature.error(m_desiredFeature);
  }

  virtual vpMatrix currentInteraction() VP_OVERRIDE
  {
    return m_currentFeature.interaction();
  }

  virtual void createCurrent(const vpHomogeneousMatrix &cMo) VP_OVERRIDE
  {
    buildCurrentFeatureObjectWithTuple(m_obj, m_currentFeature, cMo, func_ptr, *m_tuple);
  }
};
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 * \class vpPoseFeatures
 * \brief Tools for pose computation from any feature.
 * \ingroup group_vision_pose
 *
 *  This class allows to estimate a pose by virtual visual servoing from visual
 * features. The features that are considered are points, segments, lines,
 * ellipses. If the compiler is compatible with C++ 11, it is possible to
 * introduce specific features that are not directly implemented in ViSP.
 */
class VISP_EXPORT vpPoseFeatures
{
public:
  /*!
   * Method that will be used to estimate the pose from visual features.
   */
  typedef enum
  {
    VIRTUAL_VS,       /*!< Virtual visual servoing approach. */
    ROBUST_VIRTUAL_VS /*!< Robust virtual visual servoing approach. */
  } vpPoseFeaturesMethodType;

  /*!
   * Default constructor.
   */
  vpPoseFeatures();

  /*!
   * Destructor that deletes the array of features and projections.
   */
  virtual ~vpPoseFeatures();

  /*!
   * Add a point feature to the list of features to be considered in the pose
   * computation.
   *
   * \param p : Point projection expressed as a vpPoint.
   */
  void addFeaturePoint(const vpPoint &p);

  /*!
   * Add a point 3D feature to the list of features to be considered in the pose
   * computation.
   *
   * \param p : Projection expressed as a vpPoint.
   */
  void addFeaturePoint3D(const vpPoint &p);

  /*!
   * Add a vanishing point feature to the list of features to be considered in
   * the pose computation.
   *
   * \param p : Projection expressed as a vpPoint.
   */
  void addFeatureVanishingPoint(const vpPoint &p);

  /*!
   * Add a vanishing point feature to the list of features to be considered in
   * the pose computation.
   *
   * \param l1 : First line used to create the feature.
   * \param l2 : Second line used to create the feature.
   */
  void addFeatureVanishingPoint(const vpLine &l1, const vpLine &l2);

  /*!
   * Add an ellipse feature to the list of features to be considered in the pose
   * computation.
   *
   * \param c : Ellipse projection expressed as a vpCircle.
   */
  void addFeatureEllipse(const vpCircle &);

  /*!
   * Add an ellipse feature to the list of features to be considered in the pose
   * computation.
   *
   * \param s : Ellipse projection expressed as a vpSphere.
   */
  void addFeatureEllipse(const vpSphere &);

  /*!
   * Add a line feature to the list of features to be considered in the pose
   * computation.
   *
   * \param l : Line projection expressed as a vpLine.
   */
  void addFeatureLine(const vpLine &);

  /*!
   * Add a line feature to the list of features to be considered in the pose
   * computation.
   *
   * \param c : Line projection expressed as a vpCylinder.
   * \param line : Integer id that indicates which limb of the cylinder is to
   * consider. It can be vpCylinder::line1 or vpCylinder::line2.
   */
  void addFeatureLine(const vpCylinder &, const int &line);

  /*!
   * Add a segment feature to the list of features to be considered in the pose
   * computation.
   *
   * \param P1 : First extremity projection.
   * \param P2 : Second extremity projection.
   */
  void addFeatureSegment(vpPoint &, vpPoint &);

  /*!
   * Add a specific feature for the pose computation.
   */
  template <typename RetType, typename... ArgsFunc, typename... Args>
  void addSpecificFeature(RetType(*fct_ptr)(ArgsFunc...), Args &&...args);

  /*!
   * Add a specific feature for the pose computation.
   */
  template <typename ObjType, typename RetType, typename... ArgsFunc, typename... Args>
  void addSpecificFeature(ObjType *obj, RetType(ObjType:: *fct_ptr)(ArgsFunc...), Args &&...args);

  /*!
   * Clear all the features
   */
  void clear();

  /*!
   * Compute the pose according to the desired method (virtual visual servoing,
   * or robust virtual visual servoing approach).
   *
   * \param cMo : Computed pose.
   *
   * \param type : Method to use for the pose computation.
   *
   * - The virtual visual servoing approach is described in \cite Marchand02c.
   *
   * - The robust virtual visual servoing approach is described in
   * \cite Comport06b.
   */
  void computePose(vpHomogeneousMatrix &cMo, const vpPoseFeaturesMethodType &type = VIRTUAL_VS);

  /*!
   * Get the covariance matrix of the pose parameters computed by virtual
   * visual servoing.
   *
   * \warning By default, the covariance matrix is not computed. To enable the
   * computation, use setCovarianceComputation().
   */
  vpMatrix getCovarianceMatrix() const
  {
    if (!m_computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See "
              "setCovarianceComputation() to do it.");

    return m_covarianceMatrix;
  }

  /*!
   * Get the gain that is used to compute the pose with the control law \f${\bf
   * v} = -\lambda {\bf L}^+ ({\bf s} - {\bf s}^*)\f$.
   *
   * \return Value of \f$\lambda\f$, the gain of the control law.
   */
  double getLambda() { return m_lambda; }

  /*!
   * Get the maximum number of iterations of the virtual visual servoing (VVS)
   * scheme implemented in computePose().
   *
   * \return Maximum number of iterations used during VVS minimization.
   */
  unsigned int getVVSIterMax() { return m_vvsIterMax; }

  /*!
   * Enable or disable covariance computation of the pose parameters.
   *
   * \param flag : True if the covariance has to be computed, false otherwise.
   */
  void setCovarianceComputation(const bool &flag) { m_computeCovariance = flag; }

  /*!
   * Set the gain used in the virtual visual servoing scheme : \f${\bf v} =
   * -\lambda {\bf L}^+ ({\bf s} - {\bf s}^*)\f$.
   *
   * \param val : Value of the gain \f$\lambda\f$.
   */
  void setLambda(const double &val) { m_lambda = val; }

  /*!
   * Set the maximum number of iterations used in computePose().
   *
   * \param val : Maximum number of iteration used in the VVS scheme.
   */
  void setVVSIterMax(const unsigned int &val) { m_vvsIterMax = val; }

  /*!
   * Turn the verbose mode ON / OFF.
   *
   * \param mode : new verbose state. True to turn ON, false otherwise.
   */
  void setVerbose(const bool &mode) { m_verbose = mode; }

private:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  template <typename FeatureType, typename FirstParamType> struct vpDuo
  {
    FeatureType *desiredFeature;
    FirstParamType firstParam;
    vpDuo() : desiredFeature(nullptr), firstParam() { }
  };

  template <typename FeatureType, typename FirstParamType, typename SecondParamType> struct vpTrio
  {
    FeatureType *desiredFeature;
    FirstParamType firstParam;
    SecondParamType secondParam;

    vpTrio() : desiredFeature(nullptr), firstParam(), secondParam() { }
  };
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS

  unsigned int m_maxSize;
  unsigned int m_totalSize;
  unsigned int m_vvsIterMax;
  double m_lambda;

  bool m_verbose;

  bool m_computeCovariance;
  vpMatrix m_covarianceMatrix;

  // vpFeaturePoint
  std::vector<vpDuo<vpFeaturePoint, vpPoint> > m_featurePoint_Point_list;
  // vpFeaturePoint3D
  std::vector<vpDuo<vpFeaturePoint3D, vpPoint> > m_featurePoint3D_Point_list;
  // vpFeatureVanishingPoint
  std::vector<vpDuo<vpFeatureVanishingPoint, vpPoint> > m_featureVanishingPoint_Point_list;
  std::vector<vpTrio<vpFeatureVanishingPoint, vpLine, vpLine> > m_featureVanishingPoint_DuoLine_list;
  // vpFeatureEllipse
  std::vector<vpDuo<vpFeatureEllipse, vpSphere> > m_featureEllipse_Sphere_list;
  std::vector<vpDuo<vpFeatureEllipse, vpCircle> > m_featureEllipse_Circle_list;
  // vpFeatureLine
  std::vector<vpDuo<vpFeatureLine, vpLine> > m_featureLine_Line_list;
  std::vector<vpTrio<vpFeatureLine, vpCylinder, int> > m_featureLine_DuoLineInt_List;
  // vpFeatureSegment
  std::vector<vpTrio<vpFeatureSegment, vpPoint, vpPoint> > m_featureSegment_DuoPoints_list;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  // Specific features
  std::vector<vpPoseSpecificFeature *> m_featureSpecific_list;
#endif

/*!
 * Get the error vector and L matrix from all the features.
 *
 * \param cMo : Current Pose.
 * \param err : Resulting error vector.
 * \param L : Resulting interaction matrix.
 */
  void error_and_interaction(vpHomogeneousMatrix &cMo, vpColVector &err, vpMatrix &L);

  /*!
   * Compute the pose thanks to the virtual visual servoing approach.
   *
   * This approach is described in \cite Marchand02c.
   *
   * \param cMo : Computed pose.
   */
  void computePoseVVS(vpHomogeneousMatrix &cMo);

  /*!
   * Compute the pose thanks to the robust virtual visual servoing approach
   * described in \cite Comport06b.
   *
   * \param cMo : Computed pose.
   */
  void computePoseRobustVVS(vpHomogeneousMatrix &cMo);
};

/*!
 * Add a specific feature for the pose computation.
 *
 * \param fct_ptr : pointer on the function used to create the feature.
 * \param args : List of function parameters;
 * First argument supposed to be derived from vpBasicFeature
 * (redefine interaction() and error() functions), others are supposed to be
 * derived from vpForwardProjection (redefine track() function)
 *
 * \warning This function is only available with C++11. It has to be activated
 * with USE_CPP11 option from CMake.
 *
 * \code
 * #include <visp3/vision/vpPoseFeatures.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * void vp_createPoint(vpFeaturePoint &fp,const vpPoint &p) {
 *   vpFeatureBuilder::create(fp,p);
 * }
 *
 * void vp_createTwoPoint(vpFeaturePoint &fp,const vpPoint &p, const vpPoint&p2) {
 *   vpFeatureBuilder::create(fp,p);
 *   vpFeatureBuilder::create(fp,p2);
 * }
 *
 * void vp_createLine(vpFeatureLine &fp,const vpLine &l) {
 *   vpFeatureBuilder::create(fp,l);
 * }
 *
 * int main()
 * {
 *   vpPoseFeatures pose;
 *
 *   vpPoint pts[4];
 *   vpLine line;
 *
 *   //... Projection of the points and line
 *
 *   vpFeaturePoint fp;
 *   vpFeatureLine fl;
 *   void (*ptr)(vpFeaturePoint&, const vpPoint&) = &vpFeatureBuilder::create;
 *
 *   pose.addSpecificFeature(ptr, fp, pts[0]);
 *   pose.addSpecificFeature(&vp_createPoint, fp, pts[1]);
 *   pose.addSpecificFeature(&vp_createTwoPoint, fp, pts[2], pts[3]);
 *   pose.addSpecificFeature(&vp_createLine, fl, line);
 *
 *   //... Pose Computation
 *
 *   return 0;
 * }
 * \endcode
 */
template <typename RetType, typename... ArgsFunc, typename... Args>
void vpPoseFeatures::addSpecificFeature(RetType(*fct_ptr)(ArgsFunc...), Args &&...args)
{
  typedef typename std::tuple_element<0, std::tuple<Args...> >::type featureTypeReference;
  typedef typename std::remove_reference<featureTypeReference>::type featureType;
  m_featureSpecific_list.push_back(
      new vpPoseSpecificFeatureTemplate<featureType, RetType, ArgsFunc...>(fct_ptr, std::forward<ArgsFunc>(args)...));

  m_featureSpecific_list.back()->createDesired();

  m_totalSize++;
  if (m_featureSpecific_list.size() > m_maxSize)
    m_maxSize = static_cast<unsigned int>(m_featureSpecific_list.size());
}

/*!
 * Add a specific feature for the pose computation.
 *
 * \param obj : object used to call the function defined by fct_ptr.
 * \param fct_ptr : pointer on the function used to create the feature.
 * \param args : List of function parameters;
 * First argument supposed to be derived from vpBasicFeature
 * (redefine interaction() and error() functions), others are supposed to be
 * derived from vpForwardProjection (redefine track() function)
 *
 * \warning This function is only available with C++11. It has to be activated
 * with USE_CPP11 option from CMake.
 *
 * \code
 * #include <visp3/vision/vpPoseFeatures.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * class vp_createClass{
 * public:
 *   vp_createClass(){}
 *
 *   int vp_createPoint(vpFeaturePoint &fp,const vpPoint &p) {
 *     vpFeatureBuilder::create(fp,p);
 *     return 2;
 *   }
 *
 *   void vp_createTwoPoint(vpFeaturePoint &fp,const vpPoint &p, const vpPoint &p2) {
 *     vpFeatureBuilder::create(fp,p); vpFeatureBuilder::create(fp,p2);
 *   }
 *
 *   void vp_createLine(vpFeatureLine &fp,const vpLine &l) {
 *     vpFeatureBuilder::create(fp,l);
 *   }
 * };
 *
 * int main()
 * {
 *   vpPoseFeatures pose;
 *
 *   vpPoint pts[3];
 *   vpLine line;
 *
 *   //... Projection of the points and line
 *
 *   vpFeaturePoint fp;
 *   vpFeatureLine fl;
 *
 *   vp_createClass cpClass;
 *   int (vp_createClass::*ptrClassPoint)(vpFeaturePoint&, const vpPoint&)
 *     = &vp_createClass::vp_createPoint;
 *   void (vp_createClass::*ptrClassTwoPoint)(vpFeaturePoint&, const vpPoint&, const vpPoint&)
 *     = &vp_createClass::vp_createTwoPoint;
 *   void (vp_createClass::*ptrClassLine)(vpFeatureLine &, const vpLine &)
 *     = &vp_createClass::vp_createLine;
 *
 *   pose.addSpecificFeature(&cpClass, ptrClassPoint, fp, pts[0]);
 *   pose.addSpecificFeature(&cpClass, ptrClassTwoPoint, fp, pts[1], pts[2]);
 *   pose.addSpecificFeature(&cpClass, ptrClassLine, fl, line);
 *
 *   //... Pose Computation
 *
 *   return 0;
 * }
 * \endcode
 */
template <typename ObjType, typename RetType, typename... ArgsFunc, typename... Args>
void vpPoseFeatures::addSpecificFeature(ObjType *obj, RetType(ObjType:: *fct_ptr)(ArgsFunc...), Args &&...args)
{
  typedef typename std::tuple_element<0, std::tuple<Args...> >::type featureTypeReference;
  typedef typename std::remove_reference<featureTypeReference>::type featureType;
  m_featureSpecific_list.push_back(new vpPoseSpecificFeatureTemplateObject<ObjType, featureType, RetType, ArgsFunc...>(
    obj, fct_ptr, std::forward<ArgsFunc>(args)...));

  m_featureSpecific_list.back()->createDesired();

  m_totalSize++;
  if (m_featureSpecific_list.size() > m_maxSize)
    m_maxSize = static_cast<unsigned int>(m_featureSpecific_list.size());
}

END_VISP_NAMESPACE
#endif

#endif
