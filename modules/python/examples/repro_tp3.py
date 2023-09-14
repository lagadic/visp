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
    ImageIo.read(image1, str(data_path / 'tp3-I1.jpg'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)
    ImageIo.read(image2, str(data_path / 'tp3-I2.jpg'), ImageIo.ImageIoBackendType.IO_DEFAULT_BACKEND)
    # Pretty print of numpy arrays
    np.set_printoptions(precision=7, suppress=True)

    plt.figure(figsize=(15,15))
    plt.ion()

    plt1 = plt.subplot(2, 2, 1) # Image 1
    plt2 = plt.subplot(2, 2, 2) # Image 2
    plt3 = plt.subplot(212)     # Image 1 besides Image 2

    plt1.imshow(image1, cmap='gray')
    plt2.imshow(image2, cmap='gray')

    imageAppariement = np.hstack((image1, image2))
    plt3.imshow(imageAppariement, interpolation='bilinear', cmap='gray')

    # Matrix of coordinates of the points in image
    # (u1,v1) are in image1 and (u2,v2) in image2
    point = np.array(
    [    #   u1,          v1,            u2,           v2
        [117.5130997,  62.34123611,  202.841095,   36.29648209],
        [84.06044006,  67.55551147,  169.5350189,  26.80556679],
        [80.27194214,  111.0672302,  147.9641113,  64.5475769],
        [342.6855164,  199.8661346,  63.4621048,   68.28819275],
        [302.6676636,  226.6687317,  300.4017639,  263.6835022],
        [101.5870972,  63.0242424,   187.8421478,  29.56011963],
        [153.4119415,  91.05652618,  222.968277,   77.2434845],
        [190.6780548,  110.7231598,  247.8312683,  110.4263763],
        [302.8087463,  133.9337616,  339.9194641,  178.880661],
        [162.7279968,  276.4970398,  152.7050171,  248.9367065],
        [151.0850067,  36.12360764,  244.672287,   25.44586563],
        [171.7740173,  53.67162704,  256.0083618,  49.99362183],
        [116.7895355,  74.19098663,  196.8202972,  45.97808456],
        [104.2023163,  83.85998535,  181.4200439,  50.26084518],
        [84.71365356,  190.8507233,  300.4017639,  263.6835022],
        [138.8526764,  273.5761719,  131.6974182,  236.8515778],
        [167.2081451,  96.59983063,  233.1238556,  88.96112061],
    ]
    )

    # Plot corresponding points
    u1 = point[:,0]
    v1 = point[:,1]
    u2 = point[:,2]
    v2 = point[:,3]

    # Get image size
    img_rows = image1.getHeight()
    img_cols = image1.getWidth()
    print("Image size:", img_cols, "x", img_rows)

    # Total number of points
    nbPoints = len(u1)

    # BEGIN TO COMPLETE
    #   Plot lines between matches
    for i in range(nbPoints):
        plt1.plot(u1[i], v1[i],'r+')
        plt2.plot(u2[i], v2[i],'r+')
        x = np.array([u1[i], u2[i] + img_cols])
        y = np.array([v1[i], v2[i]])
        plt3.plot(x, y, linewidth=1)

        plt1.text(u1[i]+3, v1[i]-3, i, color='red')
        plt2.text(u2[i]+3, v2[i]-3, i, color='red')
        plt3.text(u1[i]+3, v1[i]-3, i, color='red')
        plt3.text(u2[i]+img_cols-3, v2[i]-3, i, color='red')
    # END TO COMPLETE

    # Compute homography c2Hc1 such as x2 = c2Hc1 * x1 without RANSAC
    c2Hc1 = Homography()
    Homography.DLT(u1, v1, u2, v2, c2Hc1, False)

    print("c2Hc1= \n", c2Hc1)

    for i in range(nbPoints):
        # BEGIN TO COMPLETE
        #   Compute the position of (u2',v2') function of (u1,v1)
        print('point',i)
        x1 = np.array([u1[i], v1[i], 1])
        x2 = np.matmul(c2Hc1,x1)
        x2 = x2/x2[2]
        print("x2 = ", x2)

        #   Compute the error between (u2',v2') and (u2,v2)
        error = np.linalg.norm(np.array([u2[i], v2[i], 1]) - x2)
        print("error = ", error)
        # END TO COMPLETE

        plt2.plot(u2[i], v2[i],'+')
        plt2.add_artist(plt.Circle((u2[i], v2[i]), error, fill=False, color='g'))

    # Wait for a mouse click in the image
    print("Use a mouse click to continue..")
    plt.waitforbuttonpress()

    # Erase image 2 and circle drawings
    plt2.cla()
    plt2.imshow(image2, cmap='gray')

    # Threshold to detect outliers
    seuilOutlier = 10

    indexInliers = np.zeros(nbPoints)
    inliers = [False for _ in range(len(u1))]
    residual = 0.0 # TODO: this should be returned
    assert Homography.ransac(u1, v1, u2, v2, c2Hc1, inliers, residual, 12, seuilOutlier, False)
    print(inliers) # TODO: inliers isn't modified by the call to ransac
    # Compute the homography with all inliers
    u1r = u1[inliers]
    v1r = v1[inliers]
    u2r = u2[inliers]
    v2r = v2[inliers]

    print('Inliers = ', u1r, v1r, u2r, v2r)


    # Compute the error for all the points
    for j in range(nbPoints):
        # BEGIN TO COMPLETE
        #   Compute the position of (u2',v2') function of (u1,v1)
        x1 = np.array([u1[j], v1[j], 1])
        x2 = np.matmul(c2Hc1,x1)
        x2 = x2/x2[2]

        #   Compute the error between (u2',v2') and (u2,v2)
        error = np.linalg.norm(np.array([u2[j], v2[j], 1]) - x2)
        print("error[",j,"] = ", error)
        # END TO COMPLETE

        if error < seuilOutlier:
            plt2.plot(x2[0], x2[1],'g+')
            plt2.add_artist(plt.Circle((x2[0], x2[1]), error*10, fill=False, color='g'))
        else:
            plt2.plot(x2[0], x2[1],'r+')
            plt2.add_artist(plt.Circle((x2[0], x2[1]), error, fill=False, color='r'))

    plt.waitforbuttonpress()
    plt.show()
