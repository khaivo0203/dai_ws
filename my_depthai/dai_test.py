import rclpy
from rclpy.node import Node
import depthai as dai
import cv2

from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point

class DirectOAKNode(Node):
    def __init__(self):
        super().__init__('direct_oak_node')
        self.pipeline = dai.Pipeline()

        cam_rgb = self.pipeline.createColorCamera()
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.RGB)

        xout = self.pipeline.createXLinkOut()
        xout.setStreamName("video")
        cam_rgb.preview.link(xout.input)

        self.device = dai.Device(self.pipeline)
        self.q = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

        self.timer = self.create_timer(0.03, self.process_frame)

    def process_frame(self):
        in_frame = self.q.tryGet()
        if in_frame is not None:
            frame = in_frame.getCvFrame()

            # Example processing: show frame
            cv2.imshow("OAK Video", frame)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DirectOAKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    

if __name__ == '__main__':
    main()

