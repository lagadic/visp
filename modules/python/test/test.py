import visp
from visp.core import TranslationVector, ThetaUVector, Matrix
from visp.core import HomogeneousMatrix, ExponentialMap, UniRand, CameraParameters
from visp.core import ColVector

if __name__ == '__main__':
  T = HomogeneousMatrix(TranslationVector(1, 0, 1), ThetaUVector(0.0, 0.0, 0.0))
  print(T)
  print(f'Homogeneous matrix inverse \n{T.inverse()}')
  v = ExponentialMap.inverse(M=T, delta_t=1.0)
  print(f'Velocity = {v}')
  TT = ExponentialMap.direct(v)
  print(f'Original T = \n{T},\nExponentialMap inverse-direct: \n{TT}')

  v = ColVector([i ** 2 for i in range(5)])
  print(f'Vector of size {v.size()} with squared indices is equal to \n{v}')
  vt = v.transpose()
  print(f'Transpose of col vector is a row vector: {isinstance(vt, visp.core.RowVector)}')
  random_gen = UniRand()

  mat = Matrix(5, 5, random_gen.uniform(0.0, 1.0))
  print(f'Matrix = \n{mat}')
  print(f'Condition = {mat.cond(1e-5)}')

  print(f'diagonal = \n{mat.getDiag()}')
  print(f'Submatrix = \n{mat.extract(0, 0, 2, 2)}')

  cam = CameraParameters(600, 600, 240, 320)
  print(cam)
  print(cam.get_K())
