import AVFoundation

class ViewController: UIViewController, VideoCaptureDelegate {
    
    @IBOutlet weak var imageView: UIImageView!
    
    let videoCapture = VideoCapture()
    let visp = VispDetector()
    
    override func viewDidLoad() {
        super.viewDidLoad()
        videoCapture.delegate = self
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        videoCapture.startCapturing()
    }
    
    // pass image to detector.
    func imageDidCapture(_ uiImage: UIImage, with px: Float, and py: Float) {
        imageView.image = visp.detectAprilTag(uiImage, px:px, py:py)
    }
    
    
    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        videoCapture.stopCapturing()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        videoCapture.stopCapturing()
    }
    
}

