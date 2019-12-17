//
// VideoCapture.swift
// Copyright © 2019 Aplix and/or its affiliates. All rights reserved.
// This software is released under the MIT License.
// http://opensource.org/licenses/mit-license.php
//
// This file is copied and modified from original files, which are distributed under
// MIT License (https://github.com/shu223/iOS-Depth-Sampler/blob/master/LICENSE)
// and are copyrighted as follows.
//
//  VideoCapture.swift
//  Created by Shuichi Tsutsumi on 4/3/16.
//  Copyright © 2016 Shuichi Tsutsumi. All rights reserved.
//
//  UIImage+CVPixelBuffer.swift
//  Created by Shuichi Tsutsumi on 2018/08/28.
//  Copyright © 2018 Shuichi Tsutsumi. All rights reserved.

import AVFoundation

protocol VideoCaptureDelegate {
    func imageDidCapture(_ uiImage: UIImage, with px: Float, and py: Float)
}

class VideoCapture:NSObject, AVCaptureVideoDataOutputSampleBufferDelegate {
    
    private let captureSession = AVCaptureSession()
    var delegate: VideoCaptureDelegate?
    
    // thread settings
    private let dataOutputQueue = DispatchQueue(label: "dataOutputQueue")
    private var isImgProcessing:Bool = false
    
    override init(){
        
        super.init()
        
        captureSession.beginConfiguration()
        
        // Session
        if self.captureSession.canSetSessionPreset(.high) {
            self.captureSession.sessionPreset = .high
        } else {
            fatalError()
        }
        
        // Input
        guard let videoDevice = AVCaptureDevice.default(for: .video) else {
            fatalError()
        }
        guard
            let videoDeviceInput = try? AVCaptureDeviceInput(device: videoDevice),
            self.captureSession.canAddInput(videoDeviceInput)
            else { fatalError() }
        self.captureSession.addInput(videoDeviceInput)
        
        // Output
        let videoDataOutput = AVCaptureVideoDataOutput()
        videoDataOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32BGRA)]
        videoDataOutput.alwaysDiscardsLateVideoFrames = true
        videoDataOutput.setSampleBufferDelegate(self, queue: dataOutputQueue)
        if !self.captureSession.canAddOutput(videoDataOutput) {
            fatalError()
        }
        self.captureSession.addOutput(videoDataOutput)
        
        // Connection
        guard let videoConnection = videoDataOutput.connection(with: .video) else {
            fatalError()
        }
        videoConnection.videoOrientation = .portrait
	//! [camera parameters]
        if videoConnection.isCameraIntrinsicMatrixDeliverySupported {
            // Enable Intrinsic parameter
            videoConnection.isCameraIntrinsicMatrixDeliveryEnabled = true
            print("Intrinsic Matrix is supported on this device :)" )
        } else {
            print("Intrinsic Matrix is NOT supported on this device :(" )
        }
        //! [camera parameters]
        captureSession.commitConfiguration()
    }

    //! [captureOutput]
    func captureOutput(_ captureOutput: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        
        // check image processing flag.
        if self.isImgProcessing { return }
        
        // make Pixel Buffer
        guard let imagePixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer) else { fatalError() }
        
        
        CVPixelBufferLockBaseAddress(imagePixelBuffer, [])
        
        // get intrinsic matrix
        var matrix = matrix_float3x3.init()
        if let camData = CMGetAttachment(sampleBuffer, key: kCMSampleBufferAttachmentKey_CameraIntrinsicMatrix, attachmentModeOut: nil) as? Data {
            matrix = camData.withUnsafeBytes { $0.pointee }
        }
        let px = matrix.columns.0.x
        let py = matrix.columns.1.y
        
        // get UIImage
        guard let uiImage = imageFromCVPixelBuffer(pixelBuffer: imagePixelBuffer) else { fatalError() }
        
        CVPixelBufferUnlockBaseAddress(imagePixelBuffer,[])
        
        // process image in main threads
        self.isImgProcessing = true
        DispatchQueue.main.async {

            self.delegate?.imageDidCapture(uiImage, with:px, and: py)
            
            // clear processing flag
            self.dataOutputQueue.async {
                self.isImgProcessing = false
            }
        }
    }
    //! [captureOutput]
    
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
}
