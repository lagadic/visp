import AVFoundation

class ViewController: UIViewController, VideoCaptureDelegate {
    
    @IBOutlet weak var imageView: UIImageView!
    
    let videoCapture = VideoCapture()
    private let visp = VispDetector()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.videoCapture.delegate = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        self.videoCapture.startCapturing()
    }
    
    // pass image to detector.
    //! [imageDidCapture]
    func imageDidCapture(_ uiImage: UIImage, with px: Float, and py: Float) {
        self.imageView.image = self.visp.detectAprilTag(uiImage, px:px, py:py)
    }
    //! [imageDidCapture]
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        self.videoCapture.stopCapturing()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        self.videoCapture.stopCapturing()
    }
    
}

