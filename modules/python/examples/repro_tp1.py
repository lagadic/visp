# Copyright 2023 Eric Marchand, Eric.Marchand@irisa.fr
#                Fabien Spindler, Fabien.Spindler@inria.fr
#                Samuel Felton, Samuel.Felton@inria.fr

import argparse
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import image
from pathlib import Path

from visp.core import CameraParameters, HomogeneousMatrix, TranslationVector, ThetaUVector, ImagePoint
from visp.core import ImageGray
from visp.io import ImageIo

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='The script corresponding to TP 1.')
    parser.add_argument('--use-case', type=int, default=1, dest='use_case', help='Use case value 1, 2 or 3')
    parser.add_argument('--data', type=str, required=True, help='Path to data')

    args, unknown_args = parser.parse_known_args()
    if unknown_args:
        print("The following args are not recognized and will not be used: %s" % unknown_args)

    data_path = Path(args.data)
    assert data_path.exists()
    print("Use case %s" % args.use_case)
    assert 0 < args.use_case < 4, 'use case should be between 1 and 3'

    # Position of the reference in the camera 2 frame
    translation = TranslationVector(0, 0, 2)
    thetau = ThetaUVector(0, 0, 0)
    c2Tw = HomogeneousMatrix(translation, thetau)
    print(f'c2Tw = \n{c2Tw}')

    print('-----------------')
    c1_cases = [
      HomogeneousMatrix(TranslationVector(-0.1, 0, 2), ThetaUVector(0, 0, 0)),
      HomogeneousMatrix(TranslationVector(0, 0, 1.8), ThetaUVector(0, 0, 0)),
      HomogeneousMatrix(TranslationVector(0.1, 0, 1.9), ThetaUVector(*[np.radians(5) for _ in range(3)]))
    ]
    c1Tw = c1_cases[args.use_case - 1]
    # Position of the reference in the camera 1 frame
    print(f'c1Tw = \n{c1Tw}')

    print('-----------------')
    cam = CameraParameters(800, 800, 200, 150)
    K = cam.get_K()


    print(f"cam = \n{cam}\nK =\n {K}")
    print('-----------------')
    x2 = np.array([
        [273, 166, 1],
        [112,  84, 1],
        [ 90, 196, 1],
        [321, 123, 1],
        [206,  	7, 1]
        ])
    plt.figure()
    plt.ion()

    #image1 = image.imread(f"data/I1{args.use_case}.png")
    image1, image2 = ImageGray(), ImageGray()
    ImageIo.read(image1, str(data_path / f'tp1-I1{args.use_case}.png'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)
    ImageIo.read(image2, str(data_path / f'tp1-I2.png'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)


    plt1 = plt.subplot(2,1,1)
    plt2 = plt.subplot(2,1,2)

    color = ['r','y','b','g','c','m','y','k']

    # Get image size
    img_rows = image1.getHeight()
    img_cols = image1.getWidth()
    print(f"Image size: {img_rows} x {img_cols}")

    # BEGIN TO COMPLETE
    #   Calculate the geometric location of a point x1 as a function of x2
    wTc2 = c2Tw.inverse()
    c1Tc2: HomogeneousMatrix = c1Tw * wTc2
    print(f'c1Tc2 = \n {c1Tc2}')

    # Option 1: acces numpy
    # c1tc2 = c1Tc2[:3,3:]
    # c1Rc2 = c1Tc2[:3,:3]

    c1tc2 = c1Tc2.getTranslationVector()
    c1Rc2 = c1Tc2.getRotationMatrix()

    Kinv = cam.get_K_inverse()

    A = c1tc2.skew() * c1Rc2 * Kinv
    print("A=", A)
    print("Kinv=", Kinv)

    Kinvtranspose = Kinv.transpose()
    print("Kinv.t=", Kinvtranspose)

    # Compute fundamental matrix F
    # On peut passer une matrice a une fonction numpy
    F = np.matmul(Kinvtranspose, A)
    print("F=", F)
    # END TO COMPLETE

    for i in range(len(x2)):
        plt2.plot(x2[i][0], x2[i][1], color[i]+'o')

        # BEGIN TO COMPLETE
        #   Calculate the geometric location of a point x1 as a function of x2
        De1 = np.matmul(F, x2[i])
        # END TO COMPLETE

        print(De1)

        # BEGIN TO COMPLETE
        #   Draw geometric location in image 1
        x = np.array([0, img_cols])
        y = np.array([-De1[2]/De1[1],(-De1[2]-img_cols*De1[0])/De1[1]])

        print('x = ',x)
        print('y = ',y)

        plt1.plot(x, y, color=color[i], linewidth=1)
        # END TO COMPLETE

    plt1.imshow(image1, cmap='gray')
    plt2.imshow(image2, cmap='gray')

    plt.waitforbuttonpress()
    plt.show()
