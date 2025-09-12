#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point

class DepthAIDetector(Node):
    def __init__(self):
        super().__init__('depthai_detector')
        
        # ROS2 parameters
        self.declare_parameter('nnBlobPath', str((Path(__file__).parent / Path('../models/mobilenet-ssd_openvino_2021.4_6shave.blob')).resolve().absolute()))
        self.declare_parameter('syncNN', True)
        self.declare_parameter('confidenceThreshold', 0.5)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        
        # Get parameters
        nnBlobPath = self.get_parameter('nnBlobPath').value
        syncNN = self.get_parameter('syncNN').value
        confidenceThreshold = self.get_parameter('confidenceThreshold').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # Verify blob path
        if not Path(nnBlobPath).exists():
            self.get_logger().error(f'Blob file not found at {nnBlobPath}')
            raise FileNotFoundError(f'Required file not found: {nnBlobPath}')
        
        # MobilenetSSD label texts
        self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                        "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        
        # ROS2 publishers
        self.bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.detections_pub = self.create_publisher(Detection2DArray, 'detections/camera_frame', 10)
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        xoutNN = self.pipeline.create(dai.node.XLinkOut)
        xoutDepth = self.pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")

        # Properties
        camRgb.setPreviewSize(300, 300)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setSubpixel(True)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        spatialDetectionNetwork.setBlobPath(nnBlobPath)
        spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
        spatialDetectionNetwork.input.setBlocking(False)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb.preview.link(spatialDetectionNetwork.input)
        if syncNN:
            spatialDetectionNetwork.passthrough.link(xoutRgb.input)
        else:
            camRgb.preview.link(xoutRgb.input)

        spatialDetectionNetwork.out.link(xoutNN.input)

        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        # Connect to device
        self.device = dai.Device(self.pipeline)
        
        # Output queues
        self.previewQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        
        # Timer for processing frames
        self.timer = self.create_timer(0.001, self.process_frames)
        
        # FPS calculation
        self.counter = 0
        self.fps = 0
        self.startTime = time.monotonic()
        
        self.get_logger().info("DepthAI Detector Node initialized")

    def process_frames(self):
        # Get frames from queues
        inPreview = self.previewQueue.tryGet()
        inDet = self.detectionNNQueue.tryGet()
        depth = self.depthQueue.tryGet()
        
        if inPreview is None or inDet is None or depth is None:
            return
        
        # FPS calculation
        self.counter += 1
        current_time = time.monotonic()
        if (current_time - self.startTime) > 1:
            self.fps = self.counter / (current_time - self.startTime)
            self.counter = 0
            self.startTime = current_time
            self.get_logger().info(f"Processing at {self.fps:.2f} fps")

        # Get frames
        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame()  # depthFrame values are in millimeters

        # Convert depth frame to color map
        depth_downscaled = depthFrame[::4]
        if np.all(depth_downscaled == 0):
            min_depth = 0  # Set a default minimum depth value when all elements are zero
        else:
            min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
        max_depth = np.percentile(depth_downscaled, 99)
        depthFrameColor = np.interp(depthFrame, (min_depth, max_depth), (0, 255)).astype(np.uint8)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        # Prepare detection message
        detections_msg = Detection2DArray()
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = self.camera_frame

        # Process detections
        height = frame.shape[0]
        width = frame.shape[1]
        
        for detection in inDet.detections:
            # Create Detection2D message
            detection_msg = Detection2D()
            detection_msg.header = detections_msg.header
            
            # Bounding box
            detection_msg.bbox.center.x = (detection.xmin + detection.xmax) / 2 * width
            detection_msg.bbox.center.y = (detection.ymin + detection.ymax) / 2 * height
            detection_msg.bbox.size_x = (detection.xmax - detection.xmin) * width
            detection_msg.bbox.size_y = (detection.ymax - detection.ymin) * height
            
            # Hypothesis
            hypothesis = ObjectHypothesisWithPose()
            try:
                hypothesis.id = str(detection.label)
                hypothesis.score = detection.confidence
                hypothesis.class_id = self.labelMap[detection.label]
            except:
                hypothesis.id = str(detection.label)
                hypothesis.score = detection.confidence
                hypothesis.class_id = "unknown"
            
            # Spatial coordinates (convert mm to meters)
            point = Point()
            point.x = float(detection.spatialCoordinates.x) / 1000.0
            point.y = float(detection.spatialCoordinates.y) / 1000.0
            point.z = float(detection.spatialCoordinates.z) / 1000.0
            hypothesis.pose.pose.position = point
            
            detection_msg.results.append(hypothesis)
            detections_msg.detections.append(detection_msg)

        # Publish messages
        self.rgb_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        self.depth_pub.publiss(self.bridge.cv2_to_imgmsg(depthFrameColor, "bgr8"))
        self.detections_pub.publish(detections_msg)

    def destroy_node(self):
        if hasattr(self, 'device'):
            self.device.close()
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthAIDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in DepthAIDetector: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
