/****************************************************************************
 *
 * $Id: vpPose.h 3530 2012-01-03 10:52:12Z fspindle $
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
 * Pose computation From any features.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \file vpPose.h
  \brief Tools for pose computation from any feature.

  \author Aurelien Yol
  \date   June, 5 2012
*/

#ifndef vpPoseFeatures_HH
#define vpPoseFeatures_HH

#include <visp/vpDebug.h>
#include <visp/vpException.h>
#include <visp/vpExponentialMap.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpBasicFeature.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpRobust.h>
#include <visp/vpForwardProjection.h>
#include <visp/vpPoint.h>
#include <visp/vpCircle.h>
#include <visp/vpSphere.h>
#include <visp/vpLine.h>
#include <visp/vpCylinder.h>

#include <vector>
#include <iostream>


#ifdef VISP_HAVE_C11_COMPATIBILITY
#include <tuple>
#ifndef DOXYGEN_SHOULD_SKIP_THIS
//#################################################
//##  Call a function with a tuple as parameters
//#################################################
template < unsigned int N >
struct vpDesiredFeatureBuilderWithTuple
{
  template < typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureWithTuple( featureType &feature, 
                           RetType (*f)( ArgsF... ), 
                           const std::tuple<ArgsT...>& t, 
                           Args &&... args )
  {
    vpDesiredFeatureBuilderWithTuple<N-1>::buildDesiredFeatureWithTuple( feature, f, t, std::get<N-1>( t ), args... );
  }
};

template <>
struct vpDesiredFeatureBuilderWithTuple<0>
{
  template < typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureWithTuple( featureType &/* feature */, 
                           RetType (*f)( ArgsF... ),
                           const std::tuple<ArgsT...>& /* t */,
                           Args&&... args )
  {
    f( args... );
  }
};

template <>
struct vpDesiredFeatureBuilderWithTuple<1>
{
  template < typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureWithTuple( featureType &feature, 
                           RetType (*f)( ArgsF... ),
                           const std::tuple<ArgsT...>& t,
                           Args&&... args )
  {
    vpDesiredFeatureBuilderWithTuple<0>::buildDesiredFeatureWithTuple( feature, f, t, feature, args... );
  }
};

template < typename featureType, typename RetType, typename... Args, typename... ArgsFunc >
void buildDesiredFeatureWithTuple( featureType &feature, 
                  RetType (*f)(ArgsFunc...), 
                  std::tuple<Args...> const& t )
{
   vpDesiredFeatureBuilderWithTuple<sizeof...(Args)>::buildDesiredFeatureWithTuple( feature, f, t );
}

//#################################################
//##  Call a function with a tuple as parameters
//##  Object Mode
//#################################################

template < unsigned int N >
struct vpDesiredFeatureBuilderObjectWithTuple
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureObjectWithTuple( objType *obj, featureType &feature, 
                           RetType (objType::*f)( ArgsF... ), 
                           const std::tuple<ArgsT...>& t, 
                           Args &&... args )
  {
    vpDesiredFeatureBuilderObjectWithTuple<N-1>::buildDesiredFeatureObjectWithTuple( obj, feature, f, t, std::get<N-1>( t ), args... );
  }
};

template <>
struct vpDesiredFeatureBuilderObjectWithTuple<0>
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureObjectWithTuple( objType *obj, featureType & /*feature*/, 
                           RetType (objType::*f)( ArgsF... ),
                           const std::tuple<ArgsT...>& /* t */,
                           Args&&... args )
  {
    (obj->*f)( args... );
  }
};

template <>
struct vpDesiredFeatureBuilderObjectWithTuple<1>
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsF, typename... ArgsT, typename... Args >
  static void buildDesiredFeatureObjectWithTuple( objType *obj, featureType &feature, 
                           RetType (objType::*f)( ArgsF... ),
                           const std::tuple<ArgsT...>& t,
                           Args&&... args )
  {
    vpDesiredFeatureBuilderObjectWithTuple<0>::buildDesiredFeatureObjectWithTuple( obj, feature, f, t, feature, args... );
  }
};

template < typename objType, typename featureType, typename RetType, typename... Args, typename... ArgsFunc >
void buildDesiredFeatureObjectWithTuple( objType *obj, featureType &feature, 
                  RetType (objType::*f)(ArgsFunc...), 
                  std::tuple<Args...> const& t )
{
   vpDesiredFeatureBuilderObjectWithTuple<sizeof...(Args)>::buildDesiredFeatureObjectWithTuple( obj, feature, f, t );
}

//#####################################################
//##  Call un function with a tuple as parameters
//##  Track all the parameters with the cMo
//##  Except the first one (must be de "BasicFeature"
//#####################################################

template < unsigned int N >
struct vpCurrentFeatureBuilderWithTuple
{
  template < typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureWithTuple( featureType &feature, 
                                  const vpHomogeneousMatrix &cMo, 
                                  RetType (*f)(ArgsF...), std::tuple<ArgsTuple...>& t, 
                                  ArgsDecomposed &&... args )
  {
    auto proj = std::get<N-1>( t );
    proj.track(cMo);
    vpCurrentFeatureBuilderWithTuple<N-1>::buildCurrentFeatureWithTuple( feature, cMo, f, t, proj, args... );
  }
};

template <>
struct vpCurrentFeatureBuilderWithTuple<0>
{
  template < typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureWithTuple( featureType &/*feature*/, 
                                  const vpHomogeneousMatrix & /*cMo*/, 
                                  RetType (*f)(ArgsF...), 
                                  std::tuple<ArgsTuple...>&, 
                                  ArgsDecomposed &&... args )
  {
    f( args... );
  }
};

template <>
struct vpCurrentFeatureBuilderWithTuple<1>
{
  template < typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureWithTuple( featureType &feature, 
                                  const vpHomogeneousMatrix &cMo, 
                                  RetType (*f)(ArgsF...), 
                                  std::tuple<ArgsTuple...>&t, 
                                  ArgsDecomposed &&... args )
  {
    vpCurrentFeatureBuilderWithTuple<0>::buildCurrentFeatureWithTuple( feature, cMo, f, t, feature, args... );
  }
};

template < typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsFunc >
void buildCurrentFeatureWithTuple( featureType &feature, 
                         const vpHomogeneousMatrix &cMo, 
                         RetType (*f)(ArgsFunc...), 
                         std::tuple<ArgsTuple...> &t )
{
  vpCurrentFeatureBuilderWithTuple<sizeof...(ArgsTuple)>::buildCurrentFeatureWithTuple( feature, cMo, f, t );
}

//#####################################################
//##  Call un function with a tuple as parameters
//##  Track all the parameters with the cMo
//##  Except the first one (must be de "BasicFeature"
//##  Object Mode
//#####################################################

template < unsigned int N >
struct vpCurrentFeatureBuilderObjectWithTuple
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureObjectWithTuple( objType *obj, featureType &feature, 
                                  const vpHomogeneousMatrix &cMo, 
                                  RetType (objType::*f)(ArgsF...), 
                                  std::tuple<ArgsTuple...>& t, 
                                  ArgsDecomposed &&... args )
  {
    auto proj = std::get<N-1>( t );
    proj.track(cMo);
    vpCurrentFeatureBuilderObjectWithTuple<N-1>::buildCurrentFeatureObjectWithTuple( obj, feature, cMo, f, t, proj, args... );
  }
};

template <>
struct vpCurrentFeatureBuilderObjectWithTuple<0>
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureObjectWithTuple( objType *obj, featureType &/*feature*/, 
                                  const vpHomogeneousMatrix &/*cMo*/, 
                                  RetType (objType::*f)(ArgsF...), 
                                  std::tuple<ArgsTuple...>&, 
                                  ArgsDecomposed &&... args )
  {
    (obj->*f)( args... );
  }
};

template <>
struct vpCurrentFeatureBuilderObjectWithTuple<1>
{
  template < typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsDecomposed, typename... ArgsF >
  static void buildCurrentFeatureObjectWithTuple( objType *obj, featureType &feature, 
                                  const vpHomogeneousMatrix &cMo, 
                                  RetType (objType::*f)(ArgsF...), 
                                  std::tuple<ArgsTuple...>&t, 
                                  ArgsDecomposed &&... args )
  {
    vpCurrentFeatureBuilderObjectWithTuple<0>::buildCurrentFeatureObjectWithTuple( obj, feature, cMo, f, t, feature, args... );
  }
};

template < typename objType, typename featureType, typename RetType, typename... ArgsTuple, typename... ArgsFunc >
void buildCurrentFeatureObjectWithTuple( objType *obj, featureType &feature, 
                         const vpHomogeneousMatrix &cMo, 
                         RetType (objType::*f)(ArgsFunc...), 
                         std::tuple<ArgsTuple...> &t )
{
  vpCurrentFeatureBuilderObjectWithTuple<sizeof...(ArgsTuple)>::buildCurrentFeatureObjectWithTuple( obj, feature, cMo, f, t );
}
#endif //#ifndef DOXYGEN_SHOULD_SKIP_THIS

//#################################################
//##  Call that will be used in our vpPoseFeatures
//##  to store the specific features.
//#################################################

class vpSpecificFeature
{
public: 
  vpSpecificFeature(){}
  virtual ~vpSpecificFeature(){};
  
  virtual vpColVector error() = 0;
  virtual vpMatrix currentInteraction() = 0;
  virtual void createDesired() = 0;
  virtual void createCurrent(const vpHomogeneousMatrix &cMo) = 0;
};

//#################################################
//##  Template for all kind of specific features
//#################################################

template< typename featureType, typename RetType, typename ...Args >
class vpSpecificFeatureTemplate : public vpSpecificFeature
{
private:
  featureType desiredFeature;
  featureType currentFeature;
  std::tuple<Args...> *tuple;
  RetType (*func_ptr)(Args...);
  
public:  
  vpSpecificFeatureTemplate(RetType (*f_ptr)(Args...), Args &&...args)
  {
    func_ptr = f_ptr; //std::move(f_ptr);
    tuple = new std::tuple<Args...>(args...);
  }
  
  virtual ~vpSpecificFeatureTemplate()
  {
    delete tuple;
  };
  
  virtual void createDesired(){
    buildDesiredFeatureWithTuple(desiredFeature, func_ptr, *tuple);
  }
  
  virtual vpColVector error(){
    //std::cout << "Getting S... : " << std::get<0>(*tuple).get_s() << std::endl;
    return currentFeature.error(desiredFeature);
  }
  
  virtual vpMatrix currentInteraction(){
    return currentFeature.interaction();
  }
  
  virtual void createCurrent(const vpHomogeneousMatrix &cMo){
    buildCurrentFeatureWithTuple(currentFeature, cMo, func_ptr, *tuple);
  }
};

//#################################################
//##  Template for all kind of specific features
//##  Object Mode
//#################################################

template< typename ObjectType, typename featureType, typename RetType, typename ...Args >
class vpSpecificFeatureTemplateObject : public vpSpecificFeature
{
private:
  featureType desiredFeature;
  featureType currentFeature;
  std::tuple<Args...> *tuple;
  RetType (ObjectType::*func_ptr)(Args...);
  ObjectType* obj;
  
public:  
  vpSpecificFeatureTemplateObject(ObjectType *o, RetType (ObjectType::*f_ptr)(Args...), Args &&...args)
  {
    func_ptr = f_ptr; //std::move(f_ptr);
    tuple = new std::tuple<Args...>(args...);
    obj = o;
  }
  
  virtual ~vpSpecificFeatureTemplateObject()
  {
    delete tuple;
  };
  
  virtual void createDesired(){
    buildDesiredFeatureObjectWithTuple(obj, desiredFeature, func_ptr, *tuple);
  }
  
  virtual vpColVector error(){
    return currentFeature.error(desiredFeature);
  }
  
  virtual vpMatrix currentInteraction(){
    return currentFeature.interaction();
  }
  
  virtual void createCurrent(const vpHomogeneousMatrix &cMo){
    buildCurrentFeatureObjectWithTuple(obj, currentFeature, cMo, func_ptr, *tuple);
  }
};
#endif //VISP_HAVE_C11_COMPATIBILITY

//######################################
//#
//######################################

class VISP_EXPORT vpPoseFeatures
{
public:
  typedef enum
    {
      VIRTUAL_VS       ,
			ROBUST_VIRTUAL_VS
    } vpPoseFeaturesMethodType;
		
private:
  
  template<typename FeatureType, typename FirstParamType>
  struct vpDuo{
    FeatureType    *desiredFeature;
    FirstParamType firstParam;
  };
  
  template<typename FeatureType, typename FirstParamType, typename SecondParamType>
  struct vpTrio{
    FeatureType    *desiredFeature;
    FirstParamType  firstParam;
    SecondParamType secondParam;
  };
  
  unsigned int                        maxSize;
  unsigned int                        totalSize;
  unsigned int                        vvsIterMax;
  double                              lambda;
  
  bool                                verbose;
  
  bool                                computeCovariance;
  vpMatrix                            covarianceMatrix;
  
  //vpFeaturePoint
  std::vector<vpDuo<vpFeaturePoint,vpPoint> >                   featurePoint_Point_list;
  //vpFeaturePoint3D
  std::vector<vpDuo<vpFeaturePoint3D,vpPoint> >                 featurePoint3D_Point_list;
  //vpFeatureVanishingPoint
  std::vector<vpDuo<vpFeatureVanishingPoint,vpPoint> >          featureVanishingPoint_Point_list;
  std::vector<vpTrio<vpFeatureVanishingPoint,vpLine,vpLine> >   featureVanishingPoint_DuoLine_list;
  //vpFeatureEllipse
  std::vector<vpDuo<vpFeatureEllipse,vpSphere> >                featureEllipse_Sphere_list;
  std::vector<vpDuo<vpFeatureEllipse,vpCircle> >                featureEllipse_Circle_list;
  //vpFeatureLine
  std::vector<vpDuo<vpFeatureLine,vpLine> >                     featureLine_Line_list;
  std::vector<vpTrio<vpFeatureLine,vpCylinder,int> >            featureLine_DuoLineInt_List;
	//vpFeatureSegment
  std::vector<vpTrio<vpFeatureSegment,vpPoint,vpPoint> >        featureSegment_DuoPoints_list;
  
#ifdef VISP_HAVE_C11_COMPATIBILITY
  //Specific features
  std::vector<vpSpecificFeature*>                               featureSpecific_list;
#endif
  
public:
  
	vpPoseFeatures();
	virtual ~vpPoseFeatures();
	
	// ! Features addition
	void addFeaturePoint(const vpPoint&);
	
  void addFeaturePoint3D(const vpPoint&);
	
	void addFeatureVanishingPoint(const vpPoint&);
  void addFeatureVanishingPoint(const vpLine&, const vpLine&);
	
	void addFeatureEllipse(const vpCircle&);
	void addFeatureEllipse(const vpSphere&);
	
	void addFeatureLine(const vpLine&);
	void addFeatureLine(const vpCylinder&, const int &line);
  
  void addFeatureSegment(vpPoint &, vpPoint&);

  
#ifdef VISP_HAVE_C11_COMPATIBILITY
  template<typename RetType, typename ...ArgsFunc, typename ...Args>
	void addSpecificFeature(RetType (*fct_ptr)(ArgsFunc ...), Args &&...args);
  
  template<typename ObjType, typename RetType, typename ...ArgsFunc, typename ...Args>
  void addSpecificFeature(ObjType *obj, RetType (ObjType::*fct_ptr)(ArgsFunc ...), Args &&...args);
#endif
	
	// ! Pose computation
	void computePose(vpHomogeneousMatrix & cMo, const vpPoseFeaturesMethodType &type = VIRTUAL_VS);
  
  /*!
    Get the covariance matrix computed in the Virtual Visual Servoing approach.
    
    \warning The compute covariance flag has to be true if you want to compute the covariance matrix.
    
    \sa setCovarianceComputation
  */
  vpMatrix getCovarianceMatrix() const { 
    if(!computeCovariance)
      vpTRACE("Warning : The covariance matrix has not been computed. See setCovarianceComputation() to do it.");
    
    return covarianceMatrix; 
  }
  
  /*!
  Get the lambda from the command law : v = -lambda L+ (s - s*)

  \return value of lambda
  */
  double getLambda(){ return lambda; }
  
  /*!
  Get the maximum number of iteration in computePoseVVS and computePoseRobustVVS

  \return value of vvsIterMax
  */
  double getVVSIterMax(){ return vvsIterMax; }
  
  /*!
    Set if the covaraince matrix has to be computed in the Virtual Visual Servoing approach.

    \param flag : True if the covariance has to be computed, false otherwise.
  */
  void setCovarianceComputation(const bool& flag) { computeCovariance = flag; }
  
  /*!
  Set the lambda from the command law : v = -lambda L+ (s - s*)

  \param val : value of lambda
  */
  void setLambda(const double &val){ lambda = val; }
  
  /*!
  Set the maximum number of iteration in computePoseVVS and computePoseRobustVVS

  \param val : maximum iteration value
  */
  void setVVSIterMax(const int &val){ vvsIterMax = val; }
  
  /*!
   Turn the verbose mode ON / OFF
   
   \param mode : new verbose state. True to turn ON, false otherwise
  */
  void setVerbose(const bool &mode){ verbose = mode; }
 
  
private:
  void error_and_interaction(vpHomogeneousMatrix & cMo, vpColVector &err, vpMatrix &L);
  
	void computePoseVVS(vpHomogeneousMatrix & cMo);
	void computePoseRobustVVS(vpHomogeneousMatrix & cMo);
};

#ifdef VISP_HAVE_C11_COMPATIBILITY
template< typename RetType, typename ...ArgsFunc, typename ...Args>
void vpPoseFeatures::addSpecificFeature(RetType (*fct_ptr)(ArgsFunc ...), Args &&...args)
{
  typedef typename std::tuple_element<0, std::tuple<Args...> >::type featureTypeReference;
  typedef typename std::remove_reference<featureTypeReference>::type featureType; 
  featureSpecific_list.push_back(
    new vpSpecificFeatureTemplate< featureType, RetType, ArgsFunc... >(fct_ptr,std::forward<ArgsFunc>(args)...)
  );
  
  featureSpecific_list.back()->createDesired();
  
  totalSize++;
  if(featureSpecific_list.size() > maxSize)
    maxSize = featureSpecific_list.size();
}

template< typename ObjType, typename RetType, typename ...ArgsFunc, typename ...Args>
void vpPoseFeatures::addSpecificFeature(ObjType *obj, RetType (ObjType::*fct_ptr)(ArgsFunc ...), Args &&...args)
{
  typedef typename std::tuple_element<0, std::tuple<Args...> >::type featureTypeReference;
  typedef typename std::remove_reference<featureTypeReference>::type featureType; 
  featureSpecific_list.push_back(
    new vpSpecificFeatureTemplateObject< ObjType, featureType, RetType, ArgsFunc... >(obj, fct_ptr,std::forward<ArgsFunc>(args)...)
  );
  
  featureSpecific_list.back()->createDesired();
  
  totalSize++;
  if(featureSpecific_list.size() > maxSize)
    maxSize = featureSpecific_list.size();
}
#endif


#endif
