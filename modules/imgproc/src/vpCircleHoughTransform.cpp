#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpImageMorphology.h>

#include <visp3/imgproc/vpCircleHoughTransform.h>

vpCircleHoughTransform::vpCircleHoughTransform()
  : m_algoParams()
{  
  initGaussianFilters();
}

vpCircleHoughTransform::vpCircleHoughTransform(const CHTransformParameters &algoParams)
  : m_algoParams(algoParams)
{  
  initGaussianFilters();
}

void 
vpCircleHoughTransform::init(const CHTransformParameters &algoParams)
{
  m_algoParams = algoParams;
  initGaussianFilters();
}

vpCircleHoughTransform::~vpCircleHoughTransform()
{
}

#ifdef VISP_HAVE_NLOHMANN_JSON
vpCircleHoughTransform::vpCircleHoughTransform(const std::string &jsonPath)
{
  initFromJSON(jsonPath);
}

void 
vpCircleHoughTransform::initFromJSON(const std::string &jsonPath)
{
  std::ifstream file(jsonPath);
  if (!file.good()) {
    std::stringstream ss;
    ss << "Problem opening file " << jsonPath << ". Make sure it exists and is readable" << std::endl;
    throw vpException(vpException::ioError, ss.str());
  }
  json j;
  try {
    j = json::parse(file);
  }
  catch (json::parse_error &e) {
    std::stringstream msg;
    msg << "Could not parse JSON file : \n";

    msg << e.what() << std::endl;
    msg << "Byte position of error: " << e.byte;
    throw vpException(vpException::ioError, msg.str());
  }
  m_algoParams = j; // Call from_json(const json& j, vpDetectorDNN& *this) to read json
  file.close();
  initGaussianFilters();
}

void 
vpCircleHoughTransform::saveConfigurationInJSON(const std::string &jsonPath) const
{
  m_algoParams.saveConfigurationInJSON(jsonPath);
}
#endif

void 
vpCircleHoughTransform::initGaussianFilters()
{
  m_fg.resize(1,(m_algoParams.m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianKernel(m_fg.data, m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev, true);
  m_fgDg.resize(1,(m_algoParams.m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianDerivativeKernel(m_fgDg.data, m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev, true);
}

std::vector<vpCircleHoughTransform::vpCircle2D> 
vpCircleHoughTransform::detect(const vpImage<vpRGBa> &I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I, I_gray);
  return detect(I_gray);
}

std::vector<vpCircleHoughTransform::vpCircle2D> 
vpCircleHoughTransform::detect(const cv::Mat &cv_I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(cv_I, I_gray);
  return detect(I_gray);
}

std::vector<vpCircleHoughTransform::vpCircle2D> 
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I, const int &nbCircles)
{
  std::vector<vpCircleHoughTransform::vpCircle2D> detections = detect(I);
  size_t nbDetections = detections.size();
  std::vector<vpCircleHoughTransform::vpCircle2D> bestCircles;
  std::vector<std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int>> detectionsWithVotes;
  for(size_t i = 0; i < nbDetections; i++)
  {
    std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int> detectionWithVote(detections[i], m_circleCandidatesVotes[i]);
    detectionsWithVotes.push_back(detectionWithVote);
  }

  bool (*hasMoreVotes)(std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int>, std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int>)
    = [](std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int> a, std::pair<vpCircleHoughTransform::vpCircle2D, unsigned int> b)
  {
    return (a.second > b.second);
  };

  std::sort(detectionsWithVotes.begin(), detectionsWithVotes.end(), hasMoreVotes);

  size_t limitMin;
  if(nbCircles < 0)
  {
    limitMin = nbDetections;
  }
  else
  {
    limitMin = std::min(nbDetections, (size_t) nbCircles);
  } 
  for(size_t i = 0; i < limitMin; i++)
  {
    bestCircles.push_back(detectionsWithVotes[i].first);
  }

  return bestCircles;
}

std::vector<vpCircleHoughTransform::vpCircle2D> 
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I)
{
  // Cleaning results of potential previous detection
  m_centerCandidatesList.clear();
  m_edgePointsList.clear();
  m_circleCandidates.clear();
  m_circleCandidatesVotes.clear();
  m_finalCircles.clear();

  // First thing, we need to apply a Gaussian filter on the image to remove some spurious noise
  // Then, we need to compute the image gradients in order to be able to perform edge detection
  computeGradientsAfterGaussianSmoothing(I);

  // Using the gradients, it is now possible to perform edge detection
  // We rely on the Canny edge detector
  // It will also give us the connected edged points
  edgeDetection(I);

  // From the edge map and gradient information, it is possible to compute
  // the center point candidates
  computeCenterCandidates();

  // From the edge map and center point candidates, we can compute candidate
  // circles. These candidate circles are circles whose center belong to
  // the center point candidates and whose radius is a "radius bin" that got
  // enough votes by computing the distance between each point of the edge map
  // and the center point candidate
  computeCircleCandidates();

  // Finally, we perform a merging operation that permits to merge circles
  // respecting similarity criteria (distance between centers and similar radius)
  mergeCircleCandidates();

  return m_finalCircles;
}

void 
vpCircleHoughTransform::computeGradientsAfterGaussianSmoothing(const vpImage<unsigned char> &I)
{
  vpImageFilter::getGradXGauss2D 	( 	I,
		m_dIx,
		m_fg.data,
		m_fgDg.data,
		m_algoParams.m_gaussianKernelSize
	);
  vpImageFilter::getGradYGauss2D 	( 	I,
		m_dIy,
		m_fg.data,
		m_fgDg.data,
		m_algoParams.m_gaussianKernelSize
	);
}

void 
vpCircleHoughTransform::edgeDetection(const vpImage<unsigned char> &I)
{
  #if VISP_HAVE_OPENCV_VERSION >= 0x020100 // Canny uses OpenCV >=2.1.0
    //Apply the Canny edge operator to compute the edge map
    // The canny method performs Gaussian blurr and gradient computation
    if(m_algoParams.m_cannyThresh < 0.)
    {
      m_algoParams.m_cannyThresh = ImageFilter::computeCannyThreshold(I);
    }
    vpImageFilter::canny(I, m_edgeMap, m_algoParams.m_gaussianKernelSize, m_algoParams.m_cannyThresh, m_algoParams.m_sobelKernelSize);
  #else
    throw(vpException(vpException::functionNotImplementedError, "Canny edge detection has not been implemented yet !"));
  #endif
}

void 
vpCircleHoughTransform::computeCenterCandidates()
{
  // For each edge point EP_i, check the image gradient at EP_i
  // Then, for each image point in the direction of the gradient,
  // increment the accumulator
  // We can perform bilinear interpolation in order not to vote for a "line" of
  // points, but for an "area" of points
  unsigned int nbRows = m_edgeMap.getRows();
  unsigned int nbCols = m_edgeMap.getCols();
  vpImage<double> centersAccum(nbRows, nbCols + 1, 0.); /*!< Matrix that contains the votes for the center candidates.*/

  m_algoParams.m_maxRadius = std::min(m_algoParams.m_maxRadius, (unsigned int)(std::min(nbCols, nbRows)*0.5));

  for (unsigned int r = 0; r < nbRows; r++)
  {
    for (unsigned int c = 0; c < nbCols; c++)
    {
      if (m_edgeMap[r][c] == 255)
      {
        // Saving the edge point for further use
        m_edgePointsList.push_back(std::pair<unsigned int, unsigned int>(r, c));

        // Voting for points in both direction of the gradient
        // Step from min_radius to max_radius in both directions of the gradient
        double mag = std::sqrt(m_dIx[r][c] * m_dIx[r][c] + m_dIy[r][c] * m_dIy[r][c]);

        double sx = m_dIx[r][c] / mag;
        double sy = m_dIy[r][c] / mag;

        for (int k1 = 0; k1 < 2; k1++)
        {
          for (int rad = m_algoParams.m_minRadius; rad <= m_algoParams.m_maxRadius; rad++)
          {
            double x1 = (double)c + (double) rad * sx;
            double y1 = (double)r + (double) rad * sy;

            unsigned int x_low = std::floor(x1);
            unsigned int y_low = std::floor(y1);

            unsigned int x_high = std::ceil(x1);
            unsigned int y_high = std::ceil(y1);

            if( (unsigned)x_low >= nbCols ||
                (unsigned)y_low >= nbRows )
                            break;
            centersAccum[y_low][x_low] += std::sqrt(std::pow(x1 - (double)x_low,2) + std::pow(y1 - (double)y_low,2));

            if( (unsigned)x_high >= nbCols ||
                (unsigned)y_high >= nbRows )
                            break;
            centersAccum[y_high][x_high] += std::sqrt(std::pow(x1 - (double)x_high,2) + std::pow(y1 - (double)y_high,2));
          }

          sx = -sx;
          sy = -sy;
        }
      }
    }
  }

  // Use dilatation with large kernel in order to determine the
  // accumulator maxima
  vpImage<double> centerCandidatesMaxima = centersAccum;
  int niters = std::max(m_algoParams.m_dilatationNbIter, 1);
  for (int i = 0; i < niters; i++)
  {
    vpImageMorphology::dilatation<double>(centerCandidatesMaxima, vpImageMorphology::CONNEXITY_4);
  }

  // Look for the image points that correspond to the accumulator maxima
  // These points will become the center candidates
  // find the possible circle centers
  for (unsigned int y = 0; y < nbRows; y++)
  {
    int left = -1;
    for (unsigned int x = 0; x < nbCols; x++)
    {
      if (  centersAccum[y][x] >= m_algoParams.m_centerThresh
         && centersAccum[y][x] == centerCandidatesMaxima[y][x] 
         && centersAccum[y][x] >  centersAccum[y][x + 1]
         )
      {
        if (left < 0)
          left = x;
      }
      else if (left >= 0)
      {
        unsigned int cx = (unsigned int)((left + x - 1) * 0.5f);
        m_centerCandidatesList.push_back(std::pair<unsigned int, unsigned int>(y, cx));
        left = -1;
      }
    }
  }
}

void 
vpCircleHoughTransform::computeCircleCandidates()
{
  size_t nbCenterCandidates = m_centerCandidatesList.size();
  unsigned int nbBins = (m_algoParams.m_maxRadius - m_algoParams.m_minRadius + 1)/ m_algoParams.m_centerMinDist;
  nbBins = std::max((unsigned int)1, nbBins); // Avoid having 0 bins, which causes segfault
  std::vector<unsigned int> radiusAccumList; /*!< Radius accumulator for each center candidates.*/
  std::vector<double> radiusActualValueList; /*!< Vector that contains the actual distance between the edge points and the center candidates.*/ 

  unsigned int rmin2 = m_algoParams.m_minRadius * m_algoParams.m_minRadius;
  unsigned int rmax2 = m_algoParams.m_maxRadius * m_algoParams.m_maxRadius;
  int circlePerfectness2 = m_algoParams.m_circlePerfectness  * m_algoParams.m_circlePerfectness;

  for(size_t i = 0; i < nbCenterCandidates; i++)
  {
    std::pair<unsigned int, unsigned int> centerCandidate = m_centerCandidatesList[i];
    // Initialize the radius accumulator of the candidate with 0s
    radiusAccumList.clear();
    radiusAccumList.resize(nbBins, 0);
    radiusActualValueList.clear();
    radiusActualValueList.resize(nbBins, 0.);

    for(auto edgePoint : m_edgePointsList)
    {
      // For each center candidate CeC_i, compute the distance with each edge point EP_j d_ij = dist(CeC_i; EP_j)  
      unsigned int rx = edgePoint.first  - centerCandidate.first;
      unsigned int ry = edgePoint.second - centerCandidate.second;
      unsigned int r2 = rx * rx + ry * ry;

      if((r2 > rmin2) && (r2 < rmax2))
      {
        double r = std::sqrt(r2);

        double gx = m_dIx[edgePoint.first][edgePoint.second];
        double gy = m_dIy[edgePoint.first][edgePoint.second];
        double grad2 = gx * gx + gy * gy;
      
        int scalProd = rx * gx + ry * gy;
        int scalProd2 = scalProd * scalProd;
        if(scalProd2 >= circlePerfectness2 * r2 * grad2)
        {
          // Look for the Radius Candidate Bin RCB_k to which d_ij is "the closest" will have an additionnal vote
          unsigned int r_bin = std::ceil((r - m_algoParams.m_minRadius)/ m_algoParams.m_centerMinDist);
          r_bin = std::min(r_bin, nbBins - 1);
          radiusAccumList[r_bin]++;
          radiusActualValueList[r_bin] +=r;
        }
      }
    }
    
    for(unsigned int idBin = 0; idBin < nbBins; idBin++)
    {
      // If the circle of center CeC_i  and radius RCB_k has enough votes, it is added to the list 
      // of Circle Candidates
      double r_effective = radiusActualValueList[idBin] / (double)radiusAccumList[idBin];
      if((double)radiusAccumList[idBin] / r_effective > m_algoParams.m_radiusRatioThresh )
      {
        m_circleCandidates.push_back(vpCircleHoughTransform::vpCircle2D( vpImagePoint(centerCandidate.first, centerCandidate.second)
                                               , r_effective 
                                               )
                                    );
        m_circleCandidatesVotes.push_back(radiusAccumList[idBin]);
      }
    }
  }
}

void 
vpCircleHoughTransform::mergeCircleCandidates()
{  
  // For each circle candidate CiC_i do:
  size_t nbCandidates = m_circleCandidates.size();
  for(size_t i = 0; i < nbCandidates; i++)
  {
    vpCircleHoughTransform::vpCircle2D cic_i = m_circleCandidates[i];
    // // For each other circle candidate CiC_j do:
    for(size_t j = i + 1; j < nbCandidates; j++)
    {
      vpCircleHoughTransform::vpCircle2D cic_j = m_circleCandidates[j];
      // // // Compute the similarity between CiC_i and CiC_j
      double distanceBetweenCenters = vpImagePoint::distance(cic_i.getCenter(), cic_j.getCenter());
      double radiusDifference = std::abs(cic_i.getRadius() - cic_j.getRadius());
      bool areCirclesSimilar = (  distanceBetweenCenters < m_algoParams.m_centerMinDist
                               && radiusDifference  < m_algoParams.m_mergingRadiusDiffThresh
                               );
      
      if( areCirclesSimilar )
      {
        // // // If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
        unsigned int totalVotes = m_circleCandidatesVotes[i] + m_circleCandidatesVotes[j];
        cic_i = vpCircleHoughTransform::vpCircle2D( (cic_i.getCenter() * m_circleCandidatesVotes[i]+ cic_j.getCenter()  * m_circleCandidatesVotes[j]) / totalVotes
                          , (cic_i.getRadius() * m_circleCandidatesVotes[i] + cic_j.getRadius() * m_circleCandidatesVotes[j]) / totalVotes
                          );
        m_circleCandidates[j] = m_circleCandidates[nbCandidates - 1];
        m_circleCandidatesVotes[i] = totalVotes;
        m_circleCandidatesVotes[j] = m_circleCandidatesVotes[nbCandidates - 1];
        m_circleCandidates.pop_back();
        m_circleCandidatesVotes.pop_back();
        nbCandidates--;
        j--;
      }
    }
    // // Add the circle candidate CiC_i, potentially merged with other circle candidates, to the final list of detected circles
    m_finalCircles.push_back(cic_i);
  }

  nbCandidates = m_finalCircles.size();
  for(size_t i = 0; i < nbCandidates; i++)
  {
    vpCircleHoughTransform::vpCircle2D cic_i = m_finalCircles[i];
    // // For each other circle candidate CiC_j do:
    for(size_t j = i + 1; j < nbCandidates; j++)
    {
      vpCircleHoughTransform::vpCircle2D cic_j = m_finalCircles[j];
      // // // Compute the similarity between CiC_i and CiC_j
      double distanceBetweenCenters = vpImagePoint::distance(cic_i.getCenter(), cic_j.getCenter());
      double radiusDifference = std::abs(cic_i.getRadius() - cic_j.getRadius());
      bool areCirclesSimilar = (  distanceBetweenCenters < m_algoParams.m_centerMinDist
                               && radiusDifference  < m_algoParams.m_mergingRadiusDiffThresh
                               );
      
      if( areCirclesSimilar )
      {
        // // // If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
        unsigned int totalVotes = m_circleCandidatesVotes[i] + m_circleCandidatesVotes[j];
        cic_i = vpCircleHoughTransform::vpCircle2D( (cic_i.getCenter() * m_circleCandidatesVotes[i]+ cic_j.getCenter()  * m_circleCandidatesVotes[j]) / totalVotes
                          , (cic_i.getRadius() * m_circleCandidatesVotes[i] + cic_j.getRadius() * m_circleCandidatesVotes[j]) / totalVotes
                          );
        m_finalCircles[j] = m_finalCircles[nbCandidates - 1];
        m_circleCandidatesVotes[i] = totalVotes;
        m_circleCandidatesVotes[j] = m_circleCandidatesVotes[nbCandidates - 1];
        m_finalCircles.pop_back();
        m_circleCandidatesVotes.pop_back();
        nbCandidates--;
        j--;
      }
    }
    // // Add the circle candidate CiC_i, potentially merged with other circle candidates, to the final list of detected circles
    m_finalCircles[i] = cic_i;
  }
}

std::string
vpCircleHoughTransform::toString() const
{
  return m_algoParams.toString();
}

std::ostream &operator<<(std::ostream &os, const vpCircleHoughTransform &detector)
{
  os << detector.toString();
  return os;
}