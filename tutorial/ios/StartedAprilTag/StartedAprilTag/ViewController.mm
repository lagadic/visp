#import "ViewController.h"
#import "ImageConversion.h"
#import "ImageDisplay.h"
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
@interface ViewController ()
@end

@implementation ViewController

@synthesize myImageView;
#endif


- (void)viewDidLoad {
  
  [super viewDidLoad];
  
  // Load an image
  UIImage *img = [UIImage imageNamed:@"AprilTag.png"];
  
  // Image view instance to display the image
  self.myImageView = [[UIImageView alloc] initWithImage:img];
  
  // Set the frame for the image view
  CGRect myFrame = CGRectMake(0.0f, 0.0f, self.myImageView.frame.size.width, self.myImageView.frame.size.height);
  [self.myImageView setFrame:myFrame];
  
  // Add the image view to the current view
  [self.view addSubview:self.myImageView];
  
  // View image
  [myImageView setImage:img];
  
  // Convert image to visp
  vpImage<unsigned char> I = [ImageConversion vpImageGrayFromUIImage:img];
  
  // Detect AprilTag
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.053;
  float quad_decimate = 3.0;
  int nThreads = 1;
  std::vector<vpHomogeneousMatrix> cMo_vec;
  
  // Set camera parameters
  vpCameraParameters cam;
  cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, 312.1889954, 243.4373779);
  
  // Initialize apriltag detector
  vpDetectorAprilTag detector(tagFamily);
  detector.setAprilTagQuadDecimate(quad_decimate);
  detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
  detector.setAprilTagNbThreads(nThreads);
  
  // Detect all the tags in the image
  detector.detect(I, tagSize, cam, cMo_vec);
  
  // Parse detected tags for printings
  std::cout << "Number of tags in the image: " << detector.getNbObjects() << std::endl;
  for(size_t i=0; i < detector.getNbObjects(); i++) {
    std::cout << "- Detected tag: " << detector.getMessage(i) << std::endl;
    std::cout << "  pose: " << cMo_vec[i] << std::endl;
  }
  
  // Draw red lines arround each tag
  for (size_t i = 0; i < detector.getNbObjects() ; i++) {
    std::vector<vpImagePoint> polygon = detector.getPolygon(i);
    for (size_t j = 0; j < polygon.size(); j++) {
      img = [ImageDisplay displayLine:img :polygon[j] :polygon[(j+1)%polygon.size()] :[UIColor redColor] :2];
    }
  }
  
  // Draw a frame corresponding to the tag 3D location
  for(size_t i=0; i < detector.getNbObjects(); i++) {
    img = [ImageDisplay displayFrame:img :cMo_vec[i] :cam :0.025 :2];
  }
  [myImageView setImage:img];
}

- (void)didReceiveMemoryWarning {
  [super didReceiveMemoryWarning];
  // Dispose of any resources that can be recreated.
}

@end
