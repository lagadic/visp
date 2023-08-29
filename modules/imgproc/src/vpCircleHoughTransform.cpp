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
{ }

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
  m_fg.resize(1, (m_algoParams.m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianKernel(m_fg.data, m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev, false);
  m_fgDg.resize(1, (m_algoParams.m_gaussianKernelSize + 1)/2);
  vpImageFilter::getGaussianDerivativeKernel(m_fgDg.data, m_algoParams.m_gaussianKernelSize, m_algoParams.m_gaussianStdev, false);
}

std::vector<vpCircleHoughTransform::vpCircle2D>
vpCircleHoughTransform::detect(const vpImage<vpRGBa> &I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(I, I_gray);
  return detect(I_gray);
}

#ifdef HAVE_OPENCV_CORE
std::vector<vpCircleHoughTransform::vpCircle2D>
vpCircleHoughTransform::detect(const cv::Mat &cv_I)
{
  vpImage<unsigned char> I_gray;
  vpImageConvert::convert(cv_I, I_gray);
  return detect(I_gray);
}
#endif

std::vector<vpCircleHoughTransform::vpCircle2D>
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I, const int &nbCircles)
{
  std::vector<vpCircle2D> detections = detect(I);
  size_t nbDetections = detections.size();
  std::vector<vpCircle2D> bestCircles;
  std::vector<std::pair<vpCircle2D, unsigned int>> detectionsWithVotes;
  for (size_t i = 0; i < nbDetections; i++) {
    std::pair<vpCircle2D, unsigned int> detectionWithVote(detections[i], m_finalCircleVotes[i]);
    detectionsWithVotes.push_back(detectionWithVote);
  }

  bool (*hasMoreVotes)(std::pair<vpCircle2D, unsigned int>, std::pair<vpCircle2D, unsigned int>)
    = [](std::pair<vpCircle2D, unsigned int> a, std::pair<vpCircle2D, unsigned int> b)
    {
      // We divide the number of votes by the radius to avoid to favour big circles
      return (a.second / a.first.getRadius() > b.second / b.first.getRadius());
    };

  std::sort(detectionsWithVotes.begin(), detectionsWithVotes.end(), hasMoreVotes);

  size_t limitMin;
  if (nbCircles < 0) {
    limitMin = nbDetections;
  }
  else {
    limitMin = std::min(nbDetections, (size_t)nbCircles);
  }
  for (size_t i = 0; i < limitMin; i++) {
    bestCircles.push_back(detectionsWithVotes[i].first);
  }

  return bestCircles;
}

std::vector<vpCircleHoughTransform::vpCircle2D>
vpCircleHoughTransform::detect(const vpImage<unsigned char> &I)
{
  // Cleaning results of potential previous detection
  m_centerCandidatesList.clear();
  m_centerVotes.clear();
  m_edgePointsList.clear();
  m_circleCandidates.clear();
  m_circleCandidatesVotes.clear();
  m_finalCircles.clear();
  m_finalCircleVotes.clear();

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
  vpImageFilter::getGradXGauss2D(I,
    m_dIx,
    m_fg.data,
    m_fgDg.data,
    m_algoParams.m_gaussianKernelSize
  );
  vpImageFilter::getGradYGauss2D(I,
    m_dIy,
    m_fg.data,
    m_fgDg.data,
    m_algoParams.m_gaussianKernelSize
  );
}

void
vpCircleHoughTransform::edgeDetection(const vpImage<unsigned char> &I)
{
#if defined(HAVE_OPENCV_IMGPROC)
  int cannyThresh = m_algoParams.m_cannyThresh;
  // Apply the Canny edge operator to compute the edge map
  // The canny method performs Gaussian blur and gradient computation
  if (m_algoParams.m_cannyThresh < 0.) {
    cannyThresh = ImageFilter::computeCannyThreshold(I);
  }
  vpImageFilter::canny(I, m_edgeMap, m_algoParams.m_gaussianKernelSize, cannyThresh, m_algoParams.m_sobelKernelSize);
#else
  throw(vpException(vpException::functionNotImplementedError, "Canny edge detection has not been implemented yet !"));
#endif

  for (int i = 0; i < m_algoParams.m_edgeMapFilteringNbIter; i++) {
    filterEdgeMap();
  }
}

void
vpCircleHoughTransform::filterEdgeMap()
{
  vpImage<unsigned char> J = m_edgeMap;

  for (unsigned int i = 1; i < J.getHeight() - 1; i++) {
    for (unsigned int j = 1; j < J.getWidth() - 1; j++) {
      if (J[i][j] == 255) {
        // Consider 8 neighbors
        int topLeftPixel = (int)J[i - 1][j - 1];
        int topPixel = (int)J[i - 1][j];
        int topRightPixel = (int)J[i - 1][j + 1];
        int botLeftPixel = (int)J[i + 1][j - 1];
        int bottomPixel = (int)J[i + 1][j];
        int botRightPixel = (int)J[i + 1][j + 1];
        int leftPixel = (int)J[i][j - 1];
        int rightPixel = (int)J[i][j + 1];
        if ((topLeftPixel + topPixel + topRightPixel
             + botLeftPixel + bottomPixel + botRightPixel
             + leftPixel + rightPixel
             ) >= 2 * 255) {
          // At least 2 of the 8-neighbor points are also an edge point
          // so we keep the edge point
          m_edgeMap[i][j] = 255;
        }
        else {
          // The edge point is isolated => we erase it
          m_edgeMap[i][j] = 0;
        }
      }
    }
  }
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

  m_algoParams.m_maxRadius = std::min(m_algoParams.m_maxRadius, std::min(nbCols, nbRows));

  // Computing the minimum and maximum horizontal position of the center candidates
  // The miminum horizontal position of the center is at worst -maxRadius outside the image
  // The maxinum horizontal position of the center is at worst +maxRadiusoutside the image
  // The width of the accumulator is the difference between the max and the min
  int minimumXposition = std::max(m_algoParams.m_centerXlimits.first, -1 * (int)m_algoParams.m_maxRadius);
  int maximumXposition = std::min(m_algoParams.m_centerXlimits.second, (int)(m_algoParams.m_maxRadius + nbCols));
  minimumXposition = std::min(minimumXposition, maximumXposition - 1);
  float minimumXpositionDouble = minimumXposition;
  int offsetX = minimumXposition;
  int accumulatorWidth = maximumXposition - minimumXposition + 1;
  if (accumulatorWidth <= 0) {
    std::cout << "Width <= 0 !" << std::endl << std::flush;
  }

  // Computing the minimum and maximum vertical position of the center candidates
  // The miminum vertical position of the center is at worst -maxRadius outside the image
  // The maxinum vertical position of the center is at worst +maxRadiusoutside the image
  // The height of the accumulator is the difference between the max and the min
  int minimumYposition = std::max(m_algoParams.m_centerYlimits.first, -1 * (int)m_algoParams.m_maxRadius);
  int maximumYposition = std::min(m_algoParams.m_centerYlimits.second, (int)(m_algoParams.m_maxRadius + nbRows));
  minimumYposition = std::min(minimumYposition, maximumYposition - 1);
  float minimumYpositionDouble = minimumYposition;
  int offsetY = minimumYposition;
  int accumulatorHeight = maximumYposition - minimumYposition + 1;
  if (accumulatorHeight <= 0) {
    std::cout << "accumulatorHeight <= 0 !" << std::endl << std::flush;
  }

  vpImage<float> centersAccum(accumulatorHeight, accumulatorWidth + 1, 0.); /*!< Matrix that contains the votes for the center candidates.*/

  for (unsigned int r = 0; r < nbRows; r++) {
    for (unsigned int c = 0; c < nbCols; c++) {
      if (m_edgeMap[r][c] == 255) {
        // Saving the edge point for further use
        m_edgePointsList.push_back(std::pair<unsigned int, unsigned int>(r, c));

        // Voting for points in both direction of the gradient
        // Step from min_radius to max_radius in both directions of the gradient
        float mag = std::sqrt(m_dIx[r][c] * m_dIx[r][c] + m_dIy[r][c] * m_dIy[r][c]);

        float sx = m_dIx[r][c] / mag;
        float sy = m_dIy[r][c] / mag;

        for (int k1 = 0; k1 < 2; k1++) {
          bool hasToStopLoop = false;
          for (int rad = m_algoParams.m_minRadius; rad <= m_algoParams.m_maxRadius && !hasToStopLoop; rad++) {
            float x1 = (float)c + (float)rad * sx;
            float y1 = (float)r + (float)rad * sy;

            if (x1 < minimumXpositionDouble || y1 < minimumYpositionDouble) {
              continue; // If either value is lower than maxRadius, it means that the center is outside the search region.
            }

            int x_low, x_high;
            int y_low, y_high;

            if (x1 > 0.) {
              x_low = std::floor(x1);
              x_high = std::ceil(x1);
            }
            else {
              x_low = -1 * std::ceil(-1. * x1);
              x_high = -1 * std::floor(-1. * x1);
            }

            if (y1 > 0.) {
              y_low = std::floor(y1);
              y_high = std::ceil(y1);
            }
            else {
              y_low = -1 * std::ceil(-1. * y1);
              y_high = -1 * std::floor(-1. * y1);
            }

            auto updateAccumulator =
              [](const float &x_orig, const float &y_orig,
                  const unsigned int &x, const unsigned int &y,
                  const int &offsetX, const int &offsetY,
                  const unsigned int &nbCols, const unsigned int &nbRows,
                  vpImage<float> &accum, bool &hasToStop)
              {
                if (x - offsetX >= nbCols ||
                    y - offsetY >= nbRows
                  ) {
                  hasToStop = true;
                }
                else {
                  float dx = (x_orig - (float)x);
                  float dy = (y_orig - (float)y);
                  accum[y - offsetY][x - offsetX] += std::abs(dx) + std::abs(dy);
                }
              };

            updateAccumulator(x1, y1, x_low, y_low,
                              offsetX, offsetY,
                              accumulatorWidth, accumulatorHeight,
                              centersAccum, hasToStopLoop
            );

            updateAccumulator(x1, y1, x_high, y_high,
                              offsetX, offsetY,
                              accumulatorWidth, accumulatorHeight,
                              centersAccum, hasToStopLoop
            );
          }

          sx = -sx;
          sy = -sy;
        }
      }
    }
  }

  // Use dilatation with large kernel in order to determine the
  // accumulator maxima
  vpImage<float> centerCandidatesMaxima = centersAccum;
  int niters = std::max(m_algoParams.m_dilatationNbIter, 1); // Ensure at least one dilatation operation
  for (int i = 0; i < niters; i++) {
    vpImageMorphology::dilatation(centerCandidatesMaxima, vpImageMorphology::CONNEXITY_4);
  }

  // Look for the image points that correspond to the accumulator maxima
  // These points will become the center candidates
  // find the possible circle centers
  int nbColsAccum = centersAccum.getCols();
  int nbRowsAccum = centersAccum.getRows();
  int nbVotes = -1;
  for (int y = 0; y < nbRowsAccum; y++) {
    int left = -1;
    for (int x = 0; x < nbColsAccum; x++) {
      if (centersAccum[y][x] >= m_algoParams.m_centerThresh
         && centersAccum[y][x] == centerCandidatesMaxima[y][x]
         && centersAccum[y][x] >  centersAccum[y][x + 1]
         ) {
        if (left < 0)
          left = x;
        nbVotes = std::max(nbVotes, (int)centersAccum[y][x]);
      }
      else if (left >= 0) {
        int cx = (int)((left + x - 1) * 0.5f);
        m_centerCandidatesList.push_back(std::pair<int, int>(y + offsetY, cx + offsetX));
        m_centerVotes.push_back(nbVotes);
        left = -1;
        nbVotes = -1;
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
  std::vector<float> radiusActualValueList; /*!< Vector that contains the actual distance between the edge points and the center candidates.*/

  unsigned int rmin2 = m_algoParams.m_minRadius * m_algoParams.m_minRadius;
  unsigned int rmax2 = m_algoParams.m_maxRadius * m_algoParams.m_maxRadius;
  int circlePerfectness2 = m_algoParams.m_circlePerfectness  * m_algoParams.m_circlePerfectness;

  for (size_t i = 0; i < nbCenterCandidates; i++) {
    std::pair<int, int> centerCandidate = m_centerCandidatesList[i];
    // Initialize the radius accumulator of the candidate with 0s
    radiusAccumList.clear();
    radiusAccumList.resize(nbBins, 0);
    radiusActualValueList.clear();
    radiusActualValueList.resize(nbBins, 0.);

    for (auto edgePoint : m_edgePointsList) {
      // For each center candidate CeC_i, compute the distance with each edge point EP_j d_ij = dist(CeC_i; EP_j)
      unsigned int rx = edgePoint.first  - centerCandidate.first;
      unsigned int ry = edgePoint.second - centerCandidate.second;
      unsigned int r2 = rx * rx + ry * ry;

      if ((r2 > rmin2) && (r2 < rmax2)) {
        float gx = m_dIx[edgePoint.first][edgePoint.second];
        float gy = m_dIy[edgePoint.first][edgePoint.second];
        float grad2 = gx * gx + gy * gy;

        int scalProd = rx * gx + ry * gy;
        int scalProd2 = scalProd * scalProd;
        if (scalProd2 >= circlePerfectness2 * r2 * grad2) {
          // Look for the Radius Candidate Bin RCB_k to which d_ij is "the closest" will have an additionnal vote
          float r = std::sqrt(r2);
          unsigned int r_bin = std::ceil((r - m_algoParams.m_minRadius)/ m_algoParams.m_centerMinDist);
          r_bin = std::min(r_bin, nbBins - 1);
          radiusAccumList[r_bin]++;
          radiusActualValueList[r_bin] += r;
        }
      }
    }

    for (unsigned int idBin = 0; idBin < nbBins; idBin++) {
      // If the circle of center CeC_i  and radius RCB_k has enough votes, it is added to the list
      // of Circle Candidates
      float r_effective = radiusActualValueList[idBin] / (float)radiusAccumList[idBin];
      if ((float)radiusAccumList[idBin] / r_effective > m_algoParams.m_radiusRatioThresh) {
        m_circleCandidates.push_back(vpCircle2D(vpImagePoint(centerCandidate.first, centerCandidate.second)
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
  std::vector<vpCircle2D> circleCandidates = m_circleCandidates;
  std::vector<unsigned int> circleCandidatesVotes = m_circleCandidatesVotes;
  size_t nbCandidates = m_circleCandidates.size();
  for (size_t i = 0; i < nbCandidates; i++) {
    vpCircle2D cic_i = circleCandidates[i];
    // // For each other circle candidate CiC_j do:
    std::cout << "< CiC_" << i << " >" << std::endl;
    for (size_t j = i + 1; j < nbCandidates; j++) {
      vpCircle2D cic_j = circleCandidates[j];
      // // // Compute the similarity between CiC_i and CiC_j
      double distanceBetweenCenters = vpImagePoint::distance(cic_i.getCenter(), cic_j.getCenter());
      double radiusDifference = std::abs(cic_i.getRadius() - cic_j.getRadius());
      bool areCirclesSimilar = (distanceBetweenCenters < m_algoParams.m_centerMinDist
                               && radiusDifference  < m_algoParams.m_mergingRadiusDiffThresh
                               );

      if (areCirclesSimilar) {
        // // // If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
        unsigned int totalVotes = circleCandidatesVotes[i] + circleCandidatesVotes[j];
        float newRadius = (cic_i.getRadius() * circleCandidatesVotes[i] + cic_j.getRadius() * circleCandidatesVotes[j]) / totalVotes;
        vpImagePoint newCenter = (cic_i.getCenter() * circleCandidatesVotes[i]+ cic_j.getCenter()  * circleCandidatesVotes[j]) / totalVotes;
        std::cout << "\tPrev centers =\n\t" << cic_i.getCenter() << "\n\t" << cic_j.getCenter() << std::endl;
        std::cout << "\tNew center = " << newCenter << std::endl;
        std::cout << "\tPrev radii =\n\t" << cic_i.getRadius() << "\n\t" << cic_j.getRadius() << std::endl;
        std::cout << "\tNew radius = " << newRadius << std::endl;
        std::cout << "\tVotes =\n\t" << circleCandidatesVotes[i] << "\n\t" << circleCandidatesVotes[j] << std::endl;
        cic_i = vpCircle2D(newCenter, newRadius);
        circleCandidates[j] = circleCandidates[nbCandidates - 1];
        circleCandidatesVotes[i] = totalVotes;
        circleCandidatesVotes[j] = circleCandidatesVotes[nbCandidates - 1];
        circleCandidates.pop_back();
        circleCandidatesVotes.pop_back();
        nbCandidates--;
        j--;
      }
    }
    // // Add the circle candidate CiC_i, potentially merged with other circle candidates, to the final list of detected circles
    m_finalCircles.push_back(cic_i);
    std::cout << "</ CiC_" << i << " >" << std::endl;
  }

  nbCandidates = m_finalCircles.size();
  for (size_t i = 0; i < nbCandidates; i++) {
    vpCircle2D cic_i = m_finalCircles[i];
    // // For each other circle candidate CiC_j do:
    std::cout << "< CiC_" << i << " >" << std::endl;
    for (size_t j = i + 1; j < nbCandidates; j++) {
      vpCircle2D cic_j = m_finalCircles[j];
      // // // Compute the similarity between CiC_i and CiC_j
      double distanceBetweenCenters = vpImagePoint::distance(cic_i.getCenter(), cic_j.getCenter());
      double radiusDifference = std::abs(cic_i.getRadius() - cic_j.getRadius());
      bool areCirclesSimilar = (distanceBetweenCenters < m_algoParams.m_centerMinDist
                               && radiusDifference  < m_algoParams.m_mergingRadiusDiffThresh
                               );

      if (areCirclesSimilar) {
        // // // If the similarity exceeds a threshold, merge the circle candidates CiC_i and CiC_j and remove CiC_j of the list
        unsigned int totalVotes = circleCandidatesVotes[i] + circleCandidatesVotes[j];
        vpImagePoint newCenter = (cic_i.getCenter() * circleCandidatesVotes[i]+ cic_j.getCenter()  * circleCandidatesVotes[j]) / totalVotes;
        float newRadius = (cic_i.getRadius() * circleCandidatesVotes[i] + cic_j.getRadius() * circleCandidatesVotes[j]) / totalVotes;
        std::cout << "Prev centers =\n\t" << cic_i.getCenter() << "\n\t" << cic_j.getCenter() << std::endl;
        std::cout << "New center = " << newCenter << std::endl;
        std::cout << "Prev radii =\n\t" << cic_i.getRadius() << "\n\t" << cic_j.getRadius() << std::endl;
        std::cout << "New radius = " << newRadius << std::endl;
        cic_i = vpCircle2D(newCenter, newRadius);
        m_finalCircles[j] = m_finalCircles[nbCandidates - 1];
        circleCandidatesVotes[i] = totalVotes;
        circleCandidatesVotes[j] = circleCandidatesVotes[nbCandidates - 1];
        m_finalCircles.pop_back();
        circleCandidatesVotes.pop_back();
        nbCandidates--;
        j--;
      }
    }
    // // Add the circle candidate CiC_i, potentially merged with other circle candidates, to the final list of detected circles
    m_finalCircles[i] = cic_i;
    std::cout << "</ CiC_" << i << " >" << std::endl;
  }
  m_finalCircleVotes = circleCandidatesVotes;
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