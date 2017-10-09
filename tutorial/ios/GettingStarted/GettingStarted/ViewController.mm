/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

#import "ViewController.h"
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface ViewController ()
@end

@implementation ViewController
#pragma mark - Example of a function that uses ViSP
- (void)processViSPHomography{
    
    std::vector<vpPoint> oP(4), aP(4), bP(4);
    double L = 0.1;
    
    oP[0].setWorldCoordinates( -L,-L,   0);
    oP[1].setWorldCoordinates(2*L,-L,   0);
    oP[2].setWorldCoordinates(  L, 3*L, 0);
    oP[3].setWorldCoordinates( -L, 4*L, 0);
    
    vpHomogeneousMatrix bMo(0,0, 1, 0, 0, 0) ;
    vpHomogeneousMatrix aMb(0.2, 0, 0.1, 0,vpMath::rad(20), 0);
    vpHomogeneousMatrix aMo = aMb*bMo ;
    
    // Normalized coordinates of points in the image frame
    std::vector<double> xa(4), ya(4), xb(4), yb(4);
    
    for(int i=0 ; i < 4; i++){
        oP[i].project(aMo);
        xa[i] = oP[i].get_x();
        ya[i] = oP[i].get_y();
        oP[i].project(bMo);
        xb[i] = oP[i].get_x();
        yb[i] = oP[i].get_y();
    }
    
    vpHomography aHb ;
    
    // Compute the homography
    vpHomography::DLT(xb, yb, xa, ya, aHb, true);
    
    std::cout << "Homography:\n" << aHb << std::endl;
    
    vpRotationMatrix aRb;
    vpTranslationVector atb;
    vpColVector n;
    
    // Compute the 3D transformation
    aHb.computeDisplacement(aRb, atb, n);
    
    std::cout << "atb: " << atb.t() << std::endl;
    
    // Compute coordinates in pixels of point 3
    vpImagePoint iPa, iPb;
    vpCameraParameters cam;
    vpMeterPixelConversion::convertPoint(cam, xb[3], yb[3], iPb);
    vpMeterPixelConversion::convertPoint(cam, xa[3], ya[3], iPa);
    
    std::cout << "Ground truth:" << std::endl;
    std::cout << "  Point 3 in pixels in frame b: " << iPb << std::endl;
    std::cout << "  Point 3 in pixels in frame a: " << iPa << std::endl;
    
    // Estimate the position in pixel of point 3 from the homography
    vpMatrix H = cam.get_K() * aHb * cam.get_K_inverse();
    
    // Project the position in pixel of point 3 from the homography
    std::cout << "Estimation from homography:" << std::endl;
    std::cout << "  Point 3 in pixels in frame a: " << vpHomography::project(cam, aHb, iPb) << std::endl;
}
- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view, typically from a nib.
    [self processViSPHomography];
}
- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}
@end
