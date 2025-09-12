#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class DepthAndFusionNode(Node):
    def __init__(self):
        super().__init__('localisation_node')

        # Publishers
        self.depth_pub = self.create_publisher(Image, '/oak/stereo/depth', 10)
        self.detections_with_depth_pub = self.create_publisher(Detection2DArray, '/detections_with_depth', 10)

        # Subscribers
        self.create_subscription(Detection2DArray, '/detections', self.detections_callback, 10)

        self.bridge = CvBridge()
        self.depth_image = None
        self.depth_width = None
        self.depth_height = None

        # Initialize DepthAI pipeline
        self.pipeline = dai.Pipeline()
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.setConfidenceThreshold(200)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setSubpixel(True)  # Higher accuracy

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        # Connect to device
        self.device = dai.Device(self.pipeline)
        self.q_depth = self.device.getOutputQueue('depth', maxSize=4, blocking=False)

        # Timer to grab depth frames from device
        self.timer = self.create_timer(0.03, self.publish_depth_frame)
        self.get_logger().info('Depth and Fusion Node started')

    def publish_depth_frame(self):
        depth_packet = self.q_depth.tryGet()
        if depth_packet:
            depth_frame = depth_packet.getFrame()  # 16-bit, mm
            self.depth_image = depth_frame.astype(np.float32) / 1000.0  # convert to meters
            self.depth_height, self.depth_width = depth_frame.shape

            # Publish depth as ROS image for debugging/visualization
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono16')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = 'oak_rgb_camera'
            self.depth_pub.publish(depth_msg)

    def detections_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn('No depth frame available yet')
            return

        detections_with_depth = Detection2DArray()
        detections_with_depth.header = msg.header

        for detection in msg.detections:
            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)

            if cx < 0 or cy < 0 or cx >= self.depth_width or cy >= self.depth_height:
                self.get_logger().warn(f"Bounding box center out of depth bounds: ({cx}, {cy})")
                continue

            # Take median of 5x5 ROI for stability
            roi_size = 5
            x1 = max(0, cx - roi_size)
            x2 = min(self.depth_width, cx + roi_size)
            y1 = max(0, cy - roi_size)
            y2 = min(self.depth_height, cy + roi_size)

            roi = self.depth_image[y1:y2, x1:x2]
            valid_depths = roi[np.isfinite(roi) & (roi > 0)]
            depth_value = float(np.median(valid_depths)) if valid_depths.size > 0 else float('nan')

            detection.pose.pose.position.z = depth_value
            detections_with_depth.detections.append(detection)

            self.get_logger().info(f"Object: {detection.results[0].hypothesis.class_id}, Depth: {depth_value:.2f} m")

        if detections_with_depth.detections:
            self.detections_with_depth_pub.publish(detections_with_depth)

def main(args=None):
    rclpy.init(args=args)
    node = DepthAndFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '_main_':
    main()