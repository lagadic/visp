#import "ViewController.h"
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
        vpImage<unsigned char> gray = [self vpImageGrayFromUIImage:img];
        [myImageView setImage:[self UIImageFromVpImageGray:gray]];
    }
    else if([myButton.currentTitle isEqualToString:[process objectAtIndex: 2]]){
        // compute gradient
        NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 2]);
        
        UIImage *img = [UIImage imageNamed:@"monkey.png"];
        vpImage<unsigned char> gray = [self vpImageGrayFromUIImage:img];
        vpImage<double> dIx;
        vpImageFilter::getGradX(gray, dIx);
        vpImageConvert::convert(dIx, gray);

        [myImageView setImage:[self UIImageFromVpImageGray:gray]];
    }
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
    else if([myButton.currentTitle isEqualToString:[process objectAtIndex: 3]]){
        // canny detector
        NSLog(@"Clicked on \"%@\" button ", [process objectAtIndex: 3]);

        UIImage *img = [UIImage imageNamed:@"monkey.png"];
        vpImage<unsigned char> gray = [self vpImageGrayFromUIImage:img];
        vpImage<unsigned char> canny;
        vpImageFilter::canny(gray, canny, 5, 15, 3);
        [myImageView setImage:[self UIImageFromVpImageGray:canny]];
    }
#endif
}

//! [vpImageColorFromUIImage]
// Converts an UIImage that could be in gray or color into a ViSP color image
- (vpImage<vpRGBa>)vpImageColorFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    
    if (CGColorSpaceGetModel(colorSpace) == kCGColorSpaceModelMonochrome) {
        NSLog(@"Input UIImage is grayscale");
        vpImage<unsigned char> gray(image.size.height, image.size.width); // 8 bits per component, 1 channel
        
        CGContextRef contextRef = CGBitmapContextCreate(gray.bitmap,                // pointer to  data
                                                        image.size.width,           // width of bitmap
                                                        image.size.height,          // height of bitmap
                                                        8,                          // bits per component
                                                        image.size.width,           // bytes per row
                                                        colorSpace,                 // colorspace
                                                        kCGImageAlphaNone |
                                                        kCGBitmapByteOrderDefault); // bitmap info flags
        
        CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
        CGContextRelease(contextRef);
        
        vpImage<vpRGBa> color;
        vpImageConvert::convert(gray, color);
        
        return color;
    }
    else {
        NSLog(@"Input UIImage is color");
        vpImage<vpRGBa> color(image.size.height, image.size.width); // 8 bits per component, 4 channels
        
        colorSpace = CGColorSpaceCreateDeviceRGB();
        
        CGContextRef contextRef = CGBitmapContextCreate(color.bitmap,               // pointer to  data
                                                        image.size.width,           // width of bitmap
                                                        image.size.height,          // height of bitmap
                                                        8,                          // bits per component
                                                        4 * image.size.width,       // bytes per row
                                                        colorSpace,                 // colorspace
                                                        kCGImageAlphaNoneSkipLast |
                                                        kCGBitmapByteOrderDefault); // bitmap info flags
        
        CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
        CGContextRelease(contextRef);
        
        return color;
    }
}
//! [vpImageColorFromUIImage]

//! [vpImageGrayFromUIImage]
// Converts an UIImage that could be in gray or color into a ViSP gray image
- (vpImage<unsigned char>)vpImageGrayFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    
    if (CGColorSpaceGetModel(colorSpace) == kCGColorSpaceModelMonochrome) {
        NSLog(@"Input UIImage is grayscale");
        vpImage<unsigned char> gray(image.size.height, image.size.width); // 8 bits per component, 1 channel
        
        CGContextRef contextRef = CGBitmapContextCreate(gray.bitmap,                // pointer to  data
                                                        image.size.width,           // width of bitmap
                                                        image.size.height,          // height of bitmap
                                                        8,                          // bits per component
                                                        image.size.width,           // bytes per row
                                                        colorSpace,                 // colorspace
                                                        kCGImageAlphaNone |
                                                        kCGBitmapByteOrderDefault); // bitmap info flags
        
        CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
        CGContextRelease(contextRef);
        
        return gray;
    } else {
        NSLog(@"Input UIImage is color");
        vpImage<vpRGBa> color(image.size.height, image.size.width); // 8 bits per component, 4 channels (color channels + alpha)
        
        colorSpace = CGColorSpaceCreateDeviceRGB();
        
        CGContextRef contextRef = CGBitmapContextCreate(color.bitmap,               // pointer to  data
                                                        image.size.width,           // width of bitmap
                                                        image.size.height,          // height of bitmap
                                                        8,                          // bits per component
                                                        4 * image.size.width,       // bytes per row
                                                        colorSpace,                 // colorspace
                                                        kCGImageAlphaNoneSkipLast |
                                                        kCGBitmapByteOrderDefault); // bitmap info flags
        
        CGContextDrawImage(contextRef, CGRectMake(0, 0, image.size.width, image.size.height), image.CGImage);
        CGContextRelease(contextRef);
        
        vpImage<unsigned char> gray;
        vpImageConvert::convert(color, gray);
        
        return gray;
    }
}
//! [vpImageGrayFromUIImage]

//! [UIImageFromVpImageColor]
// Converts a color ViSP image into a color UIImage
-(UIImage *)UIImageFromVpImageColor:(vpImage<vpRGBa>)I
{
    NSData *data = [NSData dataWithBytes:I.bitmap length:I.getSize()*4];
    CGColorSpaceRef colorSpace;
    
    colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    // Creating CGImage from vpImage
    CGImageRef imageRef = CGImageCreate(I.getWidth(),                               // width
                                        I.getHeight(),                              // height
                                        8,                                          // bits per component
                                        8 * 4,                                      // bits per pixel
                                        4 * I.getWidth(),                           // bytesPerRow
                                        colorSpace,                                 // colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   // CGDataProviderRef
                                        NULL,                                       // decode
                                        false,                                      // should interpolate
                                        kCGRenderingIntentDefault                   // intent
                                        );
    
    
    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return finalImage;
}
//! [UIImageFromVpImageColor]

//! [UIImageFromVpImageGray]
// Converts a gray level ViSP image into a gray level UIImage
-(UIImage *)UIImageFromVpImageGray:(vpImage<unsigned char>)I
{
    NSData *data = [NSData dataWithBytes:I.bitmap length:I.getSize()];
    CGColorSpaceRef colorSpace;
    
    colorSpace = CGColorSpaceCreateDeviceGray();
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    // Creating CGImage from vpImage
    CGImageRef imageRef = CGImageCreate(I.getWidth(),                               // width
                                        I.getHeight(),                              // height
                                        8,                                          // bits per component
                                        8,                                          // bits per pixel
                                        I.getWidth(),                               // bytesPerRow
                                        colorSpace,                                 // colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   // CGDataProviderRef
                                        NULL,                                       // decode
                                        false,                                      // should interpolate
                                        kCGRenderingIntentDefault                   // intent
                                        );
    
    
    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return finalImage;
}
//! [UIImageFromVpImageGray]

- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

@end
