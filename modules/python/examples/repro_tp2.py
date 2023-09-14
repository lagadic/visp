# Copyright 2023 Eric Marchand, Eric.Marchand@irisa.fr
#                Fabien Spindler, Fabien.Spindler@inria.fr
#                Samuel Felton, Samuel.Felton@inria.fr

import argparse
import numpy as np

from matplotlib import pyplot as plt
from matplotlib import image
from pathlib import Path

from visp.core import CameraParameters, HomogeneousMatrix, TranslationVector, ThetaUVector, ImagePoint, Matrix
from visp.core import ImageGray, ImageTools
from visp.vision import Homography
from visp.io import ImageIo
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='The script corresponding to TP 2.')
    parser.add_argument('--data', type=str, required=True, help='Path to data')
    args = parser.parse_args()
    data_path = Path(args.data)
    assert data_path.exists()
    image1, image2 = ImageGray(), ImageGray()
    ImageIo.read(image1, str(data_path / 'tp2-I1.jpg'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)
    ImageIo.read(image2, str(data_path / 'tp2-I2.jpg'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)

    # Pretty print of numpy arrays
    np.set_printoptions(precision=7, suppress=True)

    plt.figure(figsize=(15,15))
    plt.ion()

    plt1 = plt.subplot(2,2,1)
    plt2 = plt.subplot(2,2,2)
    plt3 = plt.subplot(2,2,4)

    plt1.imshow(image1, cmap='gray')
    plt2.imshow(image2, cmap='gray')

    # Coordinates of the 5 points in image 1
    u1 = np.array([ 135.0517282, 464.3374805, 432.1843943, 49.75437317, 298.9792208])
    v1 = np.array([44.30715709, 185.9258178, 422.7760738, 339.1144011, 236.5545455])

    # Coordinates of the 5 points in image 2
    u2 = np.array([ 122.9683498, 453.4691964, 521.5894161, 199.0225322, 336.3672109])
    v2 = np.array([ 126.6271928, 61.28444847, 278.4105839, 388.944206, 198.3472826 ])

    color = ['r','y','b','g','c','m','y','k']

    for i in range(len(u1)):
        plt1.plot(u1[i], v1[i],color[i]+'o')
        plt2.plot(u2[i], v2[i],color[i]+'o')

        plt1.text(u1[i]+10, v1[i]-10, i, color=color[i])
        plt2.text(u2[i]+10, v2[i]-10, i, color=color[i])

    # Compute matrix c2Hc1 such as x2 = c2Hc1 * x1
    c2Hc1 = Homography()
    Homography.DLT(u1, v1, u2, v2, c2Hc1, False)

    print("c2Hc1= \n", c2Hc1)

    residual = 0
    for i in range(len(u1)):
        print('point ', i)
        x1 = np.array([u1[i], v1[i], 1])
        # BEGIN TO COMPLETE
        x2 = np.matmul(c2Hc1, x1)
        x2 = x2/x2[2]
        print("x2 = ", x2)
        # END TO COMPLETE

        # BEGIN TO COMPLETE
        error = np.linalg.norm(np.array([u2[i], v2[i], 1]) - x2)
        print("Error for point ", i, ":", error)
        # END TO COMPLETE

        residual += error*error

        plt2.plot(u2[i], v2[i],color[i+1]+'+')
        plt2.add_artist(plt.Circle((u2[i], v2[i]), error*100, fill=False, color=color[i+1]))

    residual = np.sqrt(residual / len(u1))
    print("Mean residual: ", residual)

    h, w = image1.getRows(), image1.getCols()

    hbl = (int)(1.5*h)
    wbl = (int)(1.5*w)
    blendedImage = ImageGray(height=hbl, width=wbl, value=0)

    ImageTools.warpImage(image1, Matrix(c2Hc1), blendedImage, ImageTools.ImageInterpolationType.INTERPOLATION_LINEAR, False, True)

    plt3.imshow(blendedImage, cmap='gray')

    plt.waitforbuttonpress()
    plt.show()

    # Final homography matrix H21 should be
    # c2Hc1=
    #  [[-0.0040643 -0.0031677 -0.0435478]
    #  [ 0.0034882 -0.0051718 -0.9989996]
    #  [ 0.000002  -0.0000017 -0.0061552]]
