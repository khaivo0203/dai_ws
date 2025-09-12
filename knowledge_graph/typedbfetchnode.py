import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions
import json

# TypeDB configuration
DB_NAME = "dcmdbot_database"
ADDRESS = "typedb-server:1729"
CREDENTIALS = Credentials("admin", "password")
OPTIONS = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)

class TypeDBFetcherNode(Node):
    def __init__(self):
        super().__init__('typedb_fetcher_node')
        
        # Publishers
        self.data_view_publisher = self.create_publisher(String, '/typedb_view', 10)
        
        # TypeDB driver
        self.driver = TypeDB.driver(ADDRESS, CREDENTIALS, OPTIONS)
        
        # Timer to fetch data every 10 seconds
        self.timer = self.create_timer(1.0, self.match_and_fetch)
        
        self.get_logger().info("TypeDB Fetcher Node started")

    def match_and_fetch(self):
        try:
            with self.driver.transaction(DB_NAME, TransactionType.READ) as tx:
                fetch_query = """
                match
                $obj isa $obj_type, has confidence $conf;
                $room isa room, has room_name $room_name, has fire_status $fire;
                located_in (subject: $obj, container: $room);
                fetch {
                    "object_type": $obj_type,
                    "confidence": $conf,
                    "room_name": $room_name,
                    "fire_status": $fire
                };
                """
                result = tx.query(fetch_query).resolve()
                
                results_list = []
                for answer in result:
                    results_list.append({
                        "object_type": str(answer["object_type"]),
                        "confidence": float(answer["confidence"]),
                        "room_name": str(answer["room_name"]),
                        "fire_status": bool(answer["fire_status"])
                    })
                
                if results_list:
                    msg_out = String()
                    msg_out.data = json.dumps(results_list)
                    self.data_view_publisher.publish(msg_out)
                    self.get_logger().info(f"Published {len(results_list)} objects from TypeDB.")
                    
        except Exception as e:
            self.get_logger().error(f"Error fetching data from TypeDB: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = TypeDBFetcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()