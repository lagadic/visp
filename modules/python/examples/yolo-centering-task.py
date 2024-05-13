

from visp.core import ColVector
from visp.visual_features import BasicFeature

class FeatureCNNRep(BasicFeature):
  def __init__(self):
    BasicFeature.__init__(self)

  def error(self, s_star: 'FeatureCNNRep', select: int) -> ColVector:
    return ColVector(0)


if __name__ == '__main__':
  s = FeatureCNNRep()
  sd = FeatureCNNRep()
  e = s.error(sd, 0)
