import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point, PointStamped
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class TransformExample(Node):

    TF_BUFFER_DURATION = 10  # seconds

    def __init__(self, agent_id="oak"):
        super().__init__('transform_example')

        # Set frames
        self.cam_frame = f"{agent_id}/camera_color_optical_frame"  # matches TF tree
        self.target_frame = "map"  # an existing frame, can change to "map" if static transform is added

        # Initialize TF buffer and listener
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=self.TF_BUFFER_DURATION))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribe to detection topic
        self.create_subscription(
            Detection3DArray,
            '/oak/nn/spatial_detections',
            self.detection_callback,
            qos_profile_sensor_data
        )


        self.marker_pub = self.create_publisher(Marker, 'dog_markers', 10)
        self.dog_marked = False

        self.get_logger().info("Node ready, looking for dogs")

    def detection_callback(self, msg):
        if self.dog_marked:
            return

        for detection in msg.detections:
            try:
                if detection.results:
                    result = detection.results[0]
                    label = result.hypothesis.class_id
                
                if label.lower() != '12':
                    continue

                pos = result.pose.pose.position            

                source_point = Point(x=pos.x, y=pos.y, z=pos.z)

                transformed = self.transform_point(self.cam_frame, self.target_frame,  source_point)
                # transformed = self.send_request_transform(source_point, self.cam_frame, self.target_frame)
                self.get_logger().info(
                        f"[CAMERA FRAME] detected dog at : x={source_point.x:.2f}, y={source_point.y:.2f}, z={source_point.z:.2f}"
                    )

                if transformed:
                    self.get_logger().info(
                            f"[MAP FRAME] 'Dog' at x={transformed.x:.2f}, y={transformed.y:.2f}, z={transformed.z:.2f}"
                        )
                    self.publish_marker(transformed)
                    self.dog_marked = True
                    return

                #else:
                #    self.get_logger().warn("Detection had no result entries.")

            except Exception as e:
                self.get_logger().error(f"Error processing detection: {e}")


    def transform_point(self, target_frame, source_frame, source_point):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return None

        try:
            stamped_point = PointStamped()
            stamped_point.header.frame_id = source_frame
            stamped_point.header.stamp = self.get_clock().now().to_msg()
            stamped_point.point = source_point

            return do_transform_point(stamped_point, transform).point
        except Exception as e:
            self.get_logger().error(f"Point transformation failed: {e}")
            return None
    def publish_marker(self, point):
            # First detection - create the marker
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "dog"
            marker.id = 0  # Constant ID for the single marker
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.2
            marker.color.a = 1.0
            marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # Persistent marker
            
            self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = TransformExample()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '_main_':
    main()