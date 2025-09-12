#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import numpy as np

class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        self.depth_pub = self.create_publisher(Image, '/oak/d/stereo/depth', 10)
        self.bridge = CvBridge()

        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()

        # Mono cameras
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Stereo depth
        stereo = self.pipeline.create(dai.node.StereoDepth)
        stereo.setConfidenceThreshold(200)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # Align depth to RGB
        stereo.setSubpixel(True)  # Better accuracy
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # XLinkOut for depth
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName('depth')
        stereo.depth.link(xout_depth.input)

        # Connect to device
        self.device = dai.Device(self.pipeline)
        self.q_depth = self.device.getOutputQueue('depth', maxSize=4, blocking=False)

        self.timer = self.create_timer(0.03, self.publish_depth_frame)
        self.get_logger().info('Stereo Depth Node started, publishing /oak/d/stereo/depth')

    def publish_depth_frame(self):
        depth_packet = self.q_depth.tryGet()
        if depth_packet:
            depth_frame = depth_packet.getFrame()  # 16-bit, in millimeters
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='mono16')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = 'oak_rgb_camera'
            self.depth_pub.publish(depth_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()