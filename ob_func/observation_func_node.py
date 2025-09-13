import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String
import json
import uuid

class observation_function(Node):
    def __init__(self):
        super().__init__('observation_func_node')

        # Subscriptions
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10

        )
        self.subscription = self.create_subscription(
            PointStamped,
            '/object_depth',
            self.depth_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(String, '/observation', 10)

        # Ontology based
        self.knowledge_base = {
            "objects": {},
            "room": {}
        }
        self.depth_point = None
        self.get_logger().info("Observation Function Node Started1")
    def depth_callback(self, msg: PointStamped):
        self.depth_point = msg.point
    def detection_callback(self, msg: Detection2DArray):
        stimulus_event = []

        for det in msg.detections:
            if not det.results:
                continue

            label = det.results[0].hypothesis.class_id.lower() # class name from detection node
            score = det.results[0].hypothesis.score

            
            obj_type, attributes = self.parse_label(label)
            attributes["confidence"] = score
            
            # fire_presense = True if "fire" in label else False
            # env_info = {"fire_presense": fire_presense}

            observation = {
                "Type": obj_type,
                "Status": attributes,
                # "Aenv": env_info,
                "position": {
                    "x": self.depth_point.x,
                    "y": self.depth_point.y,
                    "z": self.depth_point.z
            
                } if self.depth_point else "unknown"
            }
            stimulus_event.append(observation)

        if stimulus_event:
            msg_out = String()
            msg_out.data = json.dumps({"objects":stimulus_event})
            self.publisher.publish(msg_out)
            self.get_logger().info(f"published {len(stimulus_event)} object obsevations")
            

    def parse_label(self, label):
        attributes = {}
        obj_type = None

        if "dog" in label:
            obj_type = "dog"
            if "toy" in label: attributes["size"] = "toy"
            elif "small" in label: attributes["size"] = "small"
            elif "medium" in label: attributes["size"] = "medium"
            elif "large" in label: attributes["size"] = "large"

        elif "human" in label:
            obj_type = "human"
            if "unconscious" in label: attributes["status"] = "unconscious"
            elif "conscious" in label: attributes["status"] = "conscious"

        elif "cat" in label:
            obj_type = "cat"
        elif "fire" in label:
            obj_type = "fire"

        return obj_type, attributes

    
def main(args=None):
    rclpy.init(args=args)
    node = observation_function()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                
                