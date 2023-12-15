#include <visp3/visual_features/vpFeatureLuminancePCA.h>

vpLuminancePCA::vpLuminancePCA(std::shared_ptr<vpMatrix> basis, std::shared_ptr<vpColVector> mean)
{

}

vpLuminancePCA vpLuminancePCA::load(const std::string &basisFilename, const std::string &meanFilename)
{
  std::shared_ptr<vpMatrix> basis = std::make_shared<vpMatrix>();
  std::shared_ptr<vpColVector> mean = std::make_shared<vpColVector>();
  vpMatrix::loadMatrix(basisFilename, *basis, true);
  vpColVector::load(meanFilename, *mean);

  if (mean->getCols() > 1) {
    throw vpException(vpException::badValue,
    "Read something that was not a column vector when trying to load the PCA mean vector");
  }
  if (basis->getCols() != mean->getRows()) {
    std::stringstream ss;
    ss << "Error when loading PCA from binary files";
    ss << "The basis matrix had dimensions (" << basis->getRows() << ", " << basis->getCols() << ")";
    ss << " and the mean vector had size " << mean->getRows() << ".";
    ss << "You may be loading data from two different PCAs";
    throw vpException(vpException::dimensionError, ss.str());
  }

  return vpLuminancePCA(basis, mean);
}

void vpLuminancePCA::save(const std::string &basisFilename, const std::string &meanFilename) const
{
  if (m_basis.get() == nullptr || m_mean.get() == nullptr) {
    throw vpException(vpException::notInitialized,
    "Tried to save a PCA projection that was uninitialized");
  }
  if (m_basis->size() == 0 || m_mean->getCols() == 0 || m_basis->getCols() != m_mean->getRows()) {
    throw vpException(vpException::dimensionError,
    "Tried to save a PCA projection but there are issues with the basis and mean dimensions");
  }
  vpMatrix::saveMatrix(basisFilename, *m_basis, true);
  vpColVector::save(meanFilename, *m_mean, true);
}

static vpLuminancePCA learn(std::vector<vpImage<unsigned char>> &images, const unsigned int projectionSize, const unsigned int imageBorder)
{

}

vpFeatureLuminancePCA::vpFeatureLuminancePCA(const vpCameraParameters &cam,
unsigned int h, unsigned int w, double Z, const vpLuminancePCA &pca)
{
  init(cam, h, w, Z, pca);
}

vpFeatureLuminancePCA::vpFeatureLuminancePCA(const vpFeatureLuminance &luminance, const vpLuminancePCA &pca)
{
  init(luminance, pca);
}
vpFeatureLuminancePCA::vpFeatureLuminancePCA(const vpFeatureLuminancePCA &f)
{
  *this = f;
}

void vpFeatureLuminancePCA::init()
{
  dim_s = 0;
  m_featI.init(0, 0, 0.0);
}

void vpFeatureLuminancePCA::init(
  const vpCameraParameters &cam, unsigned int h, unsigned int w, double Z,
  const vpLuminancePCA &pca)
{
  m_featI.init(h, w, Z);
  m_featI.setCameraParameters(cam);
  m_pca = pca;
  dim_s = pca.getProjectionSize();
  s.resize(dim_s, true);
}
void vpFeatureLuminancePCA::init(const vpFeatureLuminance &luminance, const vpLuminancePCA &pca)
{
  m_featI = luminance;
  m_pca = pca;
  dim_s = pca.getProjectionSize();
  s.resize(dim_s, true);
}


vpFeatureLuminancePCA &vpFeatureLuminancePCA::operator=(const vpFeatureLuminancePCA &f)
{
  dim_s = f.dim_s;
  s = f.s;
  m_pca = f.m_pca;
  m_featI = f.m_featI;
}
vpFeatureLuminancePCA *vpFeatureLuminancePCA::duplicate() const
{
  return new vpFeatureLuminancePCA(*this);
}


void vpFeatureLuminancePCA::buildFrom(vpImage<unsigned char> &I) { }

void vpFeatureLuminancePCA::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color,
              unsigned int thickness) const
{ }
void vpFeatureLuminancePCA::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
              unsigned int thickness) const
{ }


vpColVector vpFeatureLuminancePCA::error(const vpBasicFeature &s_star, unsigned int select)
{
  if (select != FEATURE_ALL) {
    throw vpException(vpException::notImplementedError, "cannot compute error on subset of PCA features");
  }
  vpColVector e(dim_s);
  error(s_star, e);
  return e;
}

void vpFeatureLuminancePCA::error(const vpBasicFeature &s_star, vpColVector &e)
{
  // if (dim_s != s_star.dimension_s()) {
  //   throw vpException(vpException::dimensionError, "Wrong dimensions when computing error in PCA features");
  // }
  e.resize(dim_s, false);
  for (unsigned int i = 0; i < dim_s; i++) {
    e[i] = s[i] - s_star[i];
  }
}

vpMatrix vpFeatureLuminancePCA::interaction(unsigned int select)
{
  if (select != FEATURE_ALL) {
    throw vpException(vpException::notImplementedError, "cannot compute interaction matrix for a subset of PCA features");
  }
  vpMatrix dWdr(dim_s, 6);
  interaction(dWdr);
  return dWdr;
}
void vpFeatureLuminancePCA::interaction(vpMatrix &L)
{
  L.resize(dim_s, 6, false, false);
  m_featI.interaction(m_LI);
}


void vpFeatureLuminancePCA::print(unsigned int select) const
{
  if (select != FEATURE_ALL) {
    throw vpException(vpException::notImplementedError, "cannot print subset of PCA features");
  }

}
