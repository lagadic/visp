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
#import "ImageConversion.h"
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
@interface ViewController ()
@end

@implementation ViewController

// Define the different process we want to apply to the input image
NSArray *process = [[NSArray alloc]initWithObjects:@"load image", @"convert to gray", @"compute gradient",
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
                    @"canny detector",
#endif
                    nil];

@synthesize myImageView;
#endif

- (void)viewDidLoad {
  
  [super viewDidLoad];
  
  // create an image
  UIImage *myScreenShot = [UIImage imageNamed:@"monkey.png"];
  
  // image view instance to display the image
  self.myImageView = [[UIImageView alloc] initWithImage:myScreenShot];
  
  // set the frame for the image view
  CGRect myFrame = CGRectMake(0.0f, 0.0f, self.myImageView.frame.size.width*2, self.myImageView.frame.size.height*2);
  [self.myImageView setFrame:myFrame];
  
  // add the image view to the current view
  [self.view addSubview:self.myImageView];
  
  // create buttons
  CGFloat posx=140, posy=350;
  CGFloat padding = 50;
  CGSize button_size = CGSizeMake( 150, 25 );
  for (int i=0; i<[process count]; i++) {
    UIButton *button = [UIButton buttonWithType:UIButtonTypeRoundedRect];
    [button addTarget:self action:@selector(checkButtonClick:) forControlEvents:UIControlEventTouchUpInside];
    [button setTitle:[process objectAtIndex: i] forState:UIControlStateNormal];
    
    button.frame = CGRectMake(posx, posy+i*padding, button_size.width, button_size.height);
    [button setBackgroundColor:[UIColor blueColor]];
    [button setTitleColor:[UIColor whiteColor] forState:UIControlStateNormal];
    button.layer.cornerRadius = 10;
    [self.view addSubview:button];
  }
}

- (void) checkButtonClick:(UIButton *)paramSender{
  
  UIButton *myButton = paramSender;
  
  //check which button was tapped
  if([myButton.currentTitle isEqualToString:[process objectAtIndex: 0]]){
    // load image
    NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 0]);
    
    [myImageView setImage:[UIImage imageNamed:@"monkey.png"]];
  }
  else if([myButton.currentTitle isEqualToString:[process objectAtIndex: 1]]){
    // convert to gray
    NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 1]);
    
    UIImage *img = [UIImage imageNamed:@"monkey.png"];
    vpImage<unsigned char> gray = [ImageConversion vpImageGrayFromUIImage:img];
    [myImageView setImage:[ImageConversion UIImageFromVpImageGray:gray]];
  }
  else if([myButton.currentTitle isEqualToString:[process objectAtIndex: 2]]){
    // compute gradient
    NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 2]);
    
    UIImage *img = [UIImage imageNamed:@"monkey.png"];
    vpImage<unsigned char> gray = [ImageConversion vpImageGrayFromUIImage:img];
    vpImage<double> dIx;
    vpImageFilter::getGradX(gray, dIx);
    vpImageConvert::convert(dIx, gray);
    
    [myImageView setImage:[ImageConversion UIImageFromVpImageGray:gray]];
  }
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  else if([myButton.currentTitle isEqualToString:[process objectAtIndex: 3]]){
    // canny detector
    NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 3]);
    
    UIImage *img = [UIImage imageNamed:@"monkey.png"];
    vpImage<unsigned char> gray = [ImageConversion vpImageGrayFromUIImage:img];
    vpImage<unsigned char> canny;
    vpImageFilter::canny(gray, canny, 5, 15, 3);
    [myImageView setImage:[ImageConversion UIImageFromVpImageGray:canny]];
  }
#endif
}

- (void)didReceiveMemoryWarning {
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

@end
