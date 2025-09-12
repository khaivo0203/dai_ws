import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_point

class TransformExample(Node):
    # Class constants
    WAIT_MAP_TIMEOUT = 500
    WAIT_MAP_LOG_INTERVAL = 50
    TF_BUFFER_DURATION = 10  # seconds

    def __init__(self, agent_id="dcmdbot8"):  # Default agent_id provided
        super().__init__('transform_example')
        
        # Store agent_id
        self.agent_id = agent_id
        
        # transform service stuff
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.wait_map_count = 0
        self.cam_frame = f"{self.agent_id}/camera_rgb_optical_frame"
        self.map_frame = "map"
        
        # test the transform
        test_point = Point(x=1.0, y=2.0, z=3.0)
        tf_res = None
        while tf_res is None:
            tf_res = self.send_request_transform(test_point, self.cam_frame, self.map_frame)
        
            # transforms can take a long time to be valid
            # usually it seems to be a time synchronisation issue
            # one way to overcome this is to simply loop over and over until the transformation is successful
            # once you get one valid transformation, usually future transforms work without any issues
            if self.wait_map_count == 1:
                self.get_logger().info("waiting for map")

            self.wait_map_count += 1

            if self.wait_map_count % self.WAIT_MAP_LOG_INTERVAL == 0:
                self.get_logger().info("...")

            if self.wait_map_count > self.WAIT_MAP_TIMEOUT:
                self.get_logger().error("Could not get map frame, exiting...")
                exit()
                
        self.get_logger().info(f"Transformed point: {tf_res}")
    
    def transform_point(self, target_frame, source_frame, source_point):
        try:
            transformation_matrix = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            print(f"tranformation matrix: {transformation_matrix}")
        except TransformException as e:
            self.get_logger().error(f"Transform lookup failed: {e}")
            self.get_logger().error(f"Available frames: \n{self.tf_buffer.all_frames_as_string()}")
            return None

        try: 
            target_point = do_transform_point(PointStamped(point=source_point), transformation_matrix).point
        except Exception as e:
            self.get_logger().error(f"Point transform failed: {e}")
            return None

        return target_point

    def send_request_transform(self, source_point, source_frame, target_frame): 
        try:                                       
            tp = self.transform_point(target_frame, source_frame, source_point)
            
            if self.wait_map_count > 0:
                self.get_logger().info(f"waited for map on {self.wait_map_count} attempts")
                self.wait_map_count = 0

            return tp
        
        except Exception as e:
            self.get_logger().error(f"ERROR in request transform: {e}")
            return None
        
    def destroy_node(self):
        self.tf_listener.unregister()
        super().destroy_node()
        
def main():
    rclpy.init()

    tf_example = TransformExample()

    try:
        rclpy.spin(tf_example)
    except Exception as e:
        print(e)
    finally:
        if tf_example:
            tf_example.destroy_node()
        rclpy.shutdown()
       
if __name__ == '__main__':
    main()
