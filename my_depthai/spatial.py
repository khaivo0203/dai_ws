import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from detection_msgs.msg import Detection2DArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import json
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')

        # Subscriptions
        self.create_subscription(Detection2DArray, '/detections', self.detection_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera_info', self.camera_info_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(String, '/object_position', 10)

        # Internal state
        self.bridge = CvBridge()
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("Localization Node initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_callback(self, msg: Detection2DArray):
        if self.depth_image is None or self.fx is None:
            self.get_logger().warn("Depth image or camera intrinsics not ready.")
            return

        positions = []
        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id
            confidence = det.results[0].hypothesis.score

            # Get bbox center in pixels
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            # Validate bounds
            if u < 0 or v < 0 or u >= self.depth_image.shape[1] or v >= self.depth_image.shape[0]:
                continue

            # Depth at center
            Z = float(self.depth_image[v, u])
            if Z <= 0 or np.isnan(Z):
                continue

            # Compute 3D in camera frame
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy

            # Transform to map frame
            try:
                point_cam = PointStamped()
                point_cam.header.stamp = self.get_clock().now().to_msg()
                point_cam.header.frame_id = "camera_link"
                point_cam.point.x = X
                point_cam.point.y = Y
                point_cam.point.z = Z

                transform = self.tf_buffer.lookup_transform('map', 'camera_link', rclpy.time.Time())
                point_map = tf2_geometry_msgs.do_transform_point(point_cam, transform)

                obj_data = {
                    "id": f"{label}_{u}_{v}",
                    "type": label,
                    "confidence": round(confidence, 2),
                    "x": round(point_map.point.x, 2),
                    "y": round(point_map.point.y, 2),
                    "z": round(point_map.point.z, 2)
                }
                positions.append(obj_data)
            except Exception as e:
                self.get_logger().warn(f"Transform failed: {e}")
                continue

        if positions:
            msg_out = String()
            msg_out.data = json.dumps(positions)
            self.publisher.publish(msg_out)
            self.get_logger().info(f"Published {len(positions)} localized objects.")

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()