// This file is copied and modified from original files, which are distributed under
// MIT License (https://github.com/shu223/iOS-Depth-Sampler/blob/master/LICENSE)
// and are copyrighted as follows.
//
//  VideoCapture.swift
//  Created by Shuichi Tsutsumi on 4/3/16.
//  Copyright © 2016 Shuichi Tsutsumi. All rights reserved.
//
//  RealtimeDepthViewController.swift
//  Created by Shuichi Tsutsumi on 2018/08/20.
//  Copyright © 2018 Shuichi Tsutsumi. All rights reserved.
//
//  UIImage+CVPixelBuffer.swift
//  Created by Shuichi Tsutsumi on 2018/08/28.
//  Copyright © 2018 Shuichi Tsutsumi. All rights reserved.

import AVFoundation

protocol VideoCaptureDelegate {
    func imageDidCapture(_ uiImage: UIImage, withIntrinsicParam px: Float, and py: Float)
}

class VideoCapture:NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    
    private let captureSession = AVCaptureSession()
    private var videoDevice: AVCaptureDevice!
    private var videoConnection: AVCaptureConnection!

    var count = 0
    var dismissTiming = 3
    var delegate: VideoCaptureDelegate?
    
    override init(){
        
        super.init()
        
        captureSession.beginConfiguration()
        captureSession.sessionPreset = .high
        
        // Input
        let videoDevice = AVCaptureDevice.default(for: .video)
        let videoDeviceInput = try! AVCaptureDeviceInput(device: videoDevice!)
        guard captureSession.canAddInput(videoDeviceInput) else { fatalError() }
        captureSession.addInput(videoDeviceInput)
        
        // Output
        let videoDataOutput = AVCaptureVideoDataOutput()
        do {
            videoDataOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32BGRA)]
            videoDataOutput.alwaysDiscardsLateVideoFrames = true
            videoDataOutput.setSampleBufferDelegate(self, queue: DispatchQueue.main)
            guard captureSession.canAddOutput(videoDataOutput) else { fatalError() }
            captureSession.addOutput(videoDataOutput)
        }
        
        // Connection
        videoConnection = videoDataOutput.connection(with: .video)!
        videoConnection.videoOrientation = .portrait
        if videoConnection.isCameraIntrinsicMatrixDeliverySupported {
            // Enable Intrinsic parameter
            videoConnection.isCameraIntrinsicMatrixDeliveryEnabled = true
            print("Intrinsic Matrix is supported on this device :)" )
        }
        
        captureSession.commitConfiguration()
    }
    
    func captureOutput(_ captureOutput: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        
        // skip image process some times to reduece CPU processing power.
        count += 1
        guard count % dismissTiming == 0 else { if(count > 10000){ count = 0 }; return}
        
        guard let imagePixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { fatalError() }
        
        // get intrinsic matrix
        var matrix = matrix_float3x3.init()
        if let camData = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil) as? Data {
            matrix = camData.withUnsafeBytes { $0.pointee }
        }
        let px = matrix.columns.0.x
        let py = matrix.columns.1.y
        
        // get UIImage
        guard let uiImage = imageFromCVPixelBuffer(pixelBuffer: imagePixelBuffer) else { fatalError() }
        
        // image process
        delegate?.imageDidCapture(uiImage, withIntrinsicParam: px, and: py)
    }
    
    func imageFromCVPixelBuffer(pixelBuffer :CVPixelBuffer) -> UIImage? {
        
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer)
        let pixelBufferWidth = CGFloat(CVPixelBufferGetWidth(pixelBuffer))
        let pixelBufferHeight = CGFloat(CVPixelBufferGetHeight(pixelBuffer))
        let imageRect:CGRect = CGRect(x: 0, y: 0, width: pixelBufferWidth, height: pixelBufferHeight)
        let ciContext = CIContext.init()
        guard let cgImage = ciContext.createCGImage(ciImage, from: imageRect) else {
            return nil
        }
        return UIImage(cgImage: cgImage, scale: 1.0, orientation:.up)
    }
    
    func startCapturing(){
        if(!self.captureSession.isRunning){
            self.captureSession.startRunning()
        }
    }
    
    func stopCapturing(){
        if(self.captureSession.isRunning){
            self.captureSession.stopRunning()
        }
    }
}
