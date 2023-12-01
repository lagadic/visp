MbGenericTracker
================

.. currentmodule:: visp.mbt

.. autoclass:: MbGenericTracker
   :members:
   :show-inheritance:
   :member-order: groupwise
   :inherited-members: pybind11_builtins.pybind11_object
   :special-members:

   
   
   .. rubric:: Methods

   .. autosummary::
      :nosignatures:
   
      ~MbGenericTracker.computeCurrentProjectionError
      ~MbGenericTracker.display
      ~MbGenericTracker.getCameraNames
      ~MbGenericTracker.getCameraParameters
      ~MbGenericTracker.getCameraTrackerTypes
      ~MbGenericTracker.getClipping
      ~MbGenericTracker.getError
      ~MbGenericTracker.getFaces
      ~MbGenericTracker.getFeaturesCircle
      ~MbGenericTracker.getFeaturesForDisplay
      ~MbGenericTracker.getFeaturesKlt
      ~MbGenericTracker.getFeaturesKltCylinder
      ~MbGenericTracker.getGoodMovingEdgesRatioThreshold
      ~MbGenericTracker.getKltImagePoints
      ~MbGenericTracker.getKltImagePointsWithId
      ~MbGenericTracker.getKltMaskBorder
      ~MbGenericTracker.getKltNbPoints
      ~MbGenericTracker.getKltOpencv
      ~MbGenericTracker.getKltPoints
      ~MbGenericTracker.getKltThresholdAcceptation
      ~MbGenericTracker.getLcircle
      ~MbGenericTracker.getLcylinder
      ~MbGenericTracker.getLline
      ~MbGenericTracker.getModelForDisplay
      ~MbGenericTracker.getMovingEdge
      ~MbGenericTracker.getNbFeaturesDepthDense
      ~MbGenericTracker.getNbFeaturesDepthNormal
      ~MbGenericTracker.getNbFeaturesEdge
      ~MbGenericTracker.getNbFeaturesKlt
      ~MbGenericTracker.getNbPoints
      ~MbGenericTracker.getNbPolygon
      ~MbGenericTracker.getPolygonFaces
      ~MbGenericTracker.getPose
      ~MbGenericTracker.getReferenceCameraName
      ~MbGenericTracker.getRobustWeights
      ~MbGenericTracker.getTrackerType
      ~MbGenericTracker.init
      ~MbGenericTracker.initClick
      ~MbGenericTracker.initFromPoints
      ~MbGenericTracker.initFromPose
      ~MbGenericTracker.loadConfigFile
      ~MbGenericTracker.loadModel
      ~MbGenericTracker.reInitModel
      ~MbGenericTracker.resetTracker
      ~MbGenericTracker.saveConfigFile
      ~MbGenericTracker.setAngleAppear
      ~MbGenericTracker.setAngleDisappear
      ~MbGenericTracker.setCameraParameters
      ~MbGenericTracker.setCameraTransformationMatrix
      ~MbGenericTracker.setClipping
      ~MbGenericTracker.setDepthDenseFilteringMaxDistance
      ~MbGenericTracker.setDepthDenseFilteringMethod
      ~MbGenericTracker.setDepthDenseFilteringMinDistance
      ~MbGenericTracker.setDepthDenseFilteringOccupancyRatio
      ~MbGenericTracker.setDepthDenseSamplingStep
      ~MbGenericTracker.setDepthNormalFaceCentroidMethod
      ~MbGenericTracker.setDepthNormalFeatureEstimationMethod
      ~MbGenericTracker.setDepthNormalPclPlaneEstimationMethod
      ~MbGenericTracker.setDepthNormalPclPlaneEstimationRansacMaxIter
      ~MbGenericTracker.setDepthNormalPclPlaneEstimationRansacThreshold
      ~MbGenericTracker.setDepthNormalSamplingStep
      ~MbGenericTracker.setDisplayFeatures
      ~MbGenericTracker.setFarClippingDistance
      ~MbGenericTracker.setFeatureFactors
      ~MbGenericTracker.setGoodMovingEdgesRatioThreshold
      ~MbGenericTracker.setKltMaskBorder
      ~MbGenericTracker.setKltOpencv
      ~MbGenericTracker.setKltThresholdAcceptation
      ~MbGenericTracker.setLod
      ~MbGenericTracker.setMask
      ~MbGenericTracker.setMinLineLengthThresh
      ~MbGenericTracker.setMinPolygonAreaThresh
      ~MbGenericTracker.setMovingEdge
      ~MbGenericTracker.setNearClippingDistance
      ~MbGenericTracker.setOgreShowConfigDialog
      ~MbGenericTracker.setOgreVisibilityTest
      ~MbGenericTracker.setOptimizationMethod
      ~MbGenericTracker.setPose
      ~MbGenericTracker.setProjectionErrorComputation
      ~MbGenericTracker.setProjectionErrorDisplay
      ~MbGenericTracker.setProjectionErrorDisplayArrowLength
      ~MbGenericTracker.setProjectionErrorDisplayArrowThickness
      ~MbGenericTracker.setReferenceCameraName
      ~MbGenericTracker.setScanLineVisibilityTest
      ~MbGenericTracker.setTrackerType
      ~MbGenericTracker.setUseDepthDenseTracking
      ~MbGenericTracker.setUseDepthNormalTracking
      ~MbGenericTracker.setUseEdgeTracking
      ~MbGenericTracker.setUseKltTracking
      ~MbGenericTracker.testTracking
      ~MbGenericTracker.track
   
   

   
   
   .. rubric:: Inherited Methods

   .. autosummary::
      :nosignatures:
   
      ~MbGenericTracker.getInitialMu
      ~MbGenericTracker.getEstimatedDoF
      ~MbGenericTracker.setLambda
      ~MbGenericTracker.setCovarianceComputation
      ~MbGenericTracker.setProjectionErrorMovingEdge
      ~MbGenericTracker.getMaxIter
      ~MbGenericTracker.getOptimizationMethod
      ~MbGenericTracker.setInitialMu
      ~MbGenericTracker.setMaxIter
      ~MbGenericTracker.getAngleAppear
      ~MbGenericTracker.GAUSS_NEWTON_OPT
      ~MbGenericTracker.setPoseSavingFilename
      ~MbGenericTracker.MbtOptimizationMethod
      ~MbGenericTracker.getFarClippingDistance
      ~MbGenericTracker.getAngleDisappear
      ~MbGenericTracker.setStopCriteriaEpsilon
      ~MbGenericTracker.setProjectionErrorKernelSize
      ~MbGenericTracker.getLambda
      ~MbGenericTracker.getCovarianceMatrix
      ~MbGenericTracker.getNearClippingDistance
      ~MbGenericTracker.getProjectionError
      ~MbGenericTracker.LEVENBERG_MARQUARDT_OPT
      ~MbGenericTracker.getStopCriteriaEpsilon
      ~MbGenericTracker.setEstimatedDoF
      ~MbGenericTracker.savePose
   
   

   
   
   .. rubric:: Operators

   .. autosummary::
      :nosignatures:
   
      ~MbGenericTracker.__doc__
      ~MbGenericTracker.__init__
      ~MbGenericTracker.__module__
   
   

   
   
   .. rubric:: Attributes

   .. autosummary::
   
      ~MbGenericTracker.DEPTH_DENSE_TRACKER
      ~MbGenericTracker.DEPTH_NORMAL_TRACKER
      ~MbGenericTracker.EDGE_TRACKER
      ~MbGenericTracker.GAUSS_NEWTON_OPT
      ~MbGenericTracker.KLT_TRACKER
      ~MbGenericTracker.LEVENBERG_MARQUARDT_OPT
   
   