from pytest import approx
from visp.core import Math, ImagePoint, ColVector, ThetaUVector, Point
def test_tuple_return_basic_values_only_output():
  '''
  Test that a function that was with signature
  double lineFitting(const std::vector<...>& imPts, double& a, double& b, double& c)
  is now (with custom configuration) with a signature lineFitting(imPts) -> tuple[double * 4]

  all the reference parameters are used as outputs but not as inputs

  '''
  a = 45.0
  b = 52.0
  points = [
    ImagePoint(a*i+b, i) for i in range(3)
  ]
  res =  Math.lineFitting(points)
  print(res)
  values = (res[0], -res[1] / res[2], res[3] / res[2])
  expected = (0, a, b)
  for i, (val, exp) in enumerate(zip(values, expected)):
    print(i)
    assert val == approx(exp, 0.0001)

def test_tuple_return_basic_values_only_output_2():
  theta = 3.14
  vec = [0.0, 0.0, 1.0]
  thetau = ThetaUVector(*[v * theta for v in vec])
  theta_out, vec_out = thetau.extract()
  assert theta_out == theta
  assert vec_out == ColVector(vec)


def test_pass_by_ref():
  values = [0, 1, 2]
  p = Point(values)
  vec_ref = ColVector()
  p.getWorldCoordinates(vec_ref)
  assert vec_ref == p.getWorldCoordinates()


def test_tuple_return_basic_values_mixed():
  pass
