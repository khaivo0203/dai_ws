# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions
# import json
# import time


# # TypeDB configuration
# DB_NAME = "dcmdbot_database2"
# ADDRESS = "typedb-server:1729"
# CREDENTIALS = Credentials("admin", "password")
# OPTIONS = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)


# class KnowledgeGraphNode(Node):
#     def __init__(self):
#         super().__init__('knowledge_graph_node')

#         # ROS 2 topics
#         self.create_subscription(String, '/observation', self.stimulus_callback, 10)
#         self.priority_assign = self.create_publisher(String, '/priority_list', 10)
#         self.data_view_publisher = self.create_publisher(String, '/typedb_view', 10)

#         # TypeDB driver
#         self.driver = TypeDB.driver(ADDRESS, CREDENTIALS, OPTIONS)

#         # Check/create database
#         if not self.driver.databases.contains(DB_NAME):
#             self.driver.databases.create(DB_NAME)
#             self.get_logger().info(f"Database 1'{DB_NAME}' created")

#             # # Wait a bit for DB creation to propagate
#             # retries = 5
#             # for i in range(retries):
#             #     if self.driver.databases.contains(DB_NAME):
#             #         break
#             #     self.get_logger().info(f"Waiting for database '{DB_NAME}' to be ready... retry {i+1}")
#             #     time.sleep(1)
#             # else:
#             #     self.get_logger().error(f"Database '{DB_NAME}' not ready after {retries} seconds.")
#             #     raise RuntimeError(f"Database '{DB_NAME}' creation failed or delayed.")

#             # # Load schema on fresh DB creation
#             # self.db_schema_setup(self.driver, DB_NAME)
#         else:
#             self.get_logger().info(f"Database '{DB_NAME}' exists, skipping schema load")

#         # Timer to fetch periodically
#         self.create_timer(10.0, self.match_and_fetch)  # every 10 seconds
#         self.create_timer(30.0, self.cleanup_old_objects)  # clean up every 30 seconds
    
    
#     def db_schema_setup(self, driver, DB_NAME, schema_file='/home/dcmdbot9/dai_ws/src/knowledge_graph/Database/rescue_schema.tql'):
#         try:
#             with open(schema_file, 'r') as data:
#                 define_query = data.read()
            
#             # Add timestamp attribute if not already in schema
#             if "timestamp sub attribute" not in define_query:
#                 define_query += "\n\ntimestamp sub attribute, value long;"
            
#             print("Schema being loaded:\n", define_query)
#             with driver.transaction(DB_NAME, TransactionType.SCHEMA) as tx:
#                 print("Defining schema", end="...")
#                 tx.query(define_query).resolve()
#                 tx.commit()
#                 self.get_logger().info("Schema defined successfully.")
#         except Exception as e:
#             self.get_logger().error(f"Failed to load schema: {e}")

#     def stimulus_callback(self, msg):
#         try:
#             data = json.loads(msg.data)
#             objects = data.get("objects", [])
#             if not objects:
#                 self.get_logger().info("No objects found.")
#                 return

#             current_time = int(time.time())
#             inserted_count = 0
#             skipped_count = 0
            
#             with self.driver.transaction(DB_NAME, TransactionType.WRITE) as tx:
#                 for obj in objects:
#                     obj_type = obj["Type"].lower()
#                     confidence = obj["Status"].get("confidence", 0.0)
                    
#                     # Check if similar object was inserted very recently (within last 2 seconds)
#                     check_query = f'''
#                     match
#                         $x isa {obj_type},
#                         has confidence $conf;
#                     get $id;
#                     count;
#                     '''
                    
#                     try:
#                         recent_count_result = list(tx.query(check_query).resolve())
#                         if recent_count_result and recent_count_result[0].number() > 0:
#                             skipped_count += 1
#                             continue  # Skip insertion - object already exists recently
#                     except Exception as e:
#                         self.get_logger().warn(f"Check query failed: {e}, proceeding with insert")
                    
#                     # Insert new object with current timestamp
#                     insert_query = f'''
#                     insert $x isa {obj_type}, 
#                         has confidence {confidence},
#                         has timestamp {current_time};
#                     '''
                    
#                     # Add coordinates if available
#                     if "position" in obj and isinstance(obj["position"], dict):
#                         x_pos = obj["position"].get("x")
#                         y_pos = obj["position"].get("y")
#                         if x_pos is not None and y_pos is not None:
#                             insert_query += f'''
#                             $coord isa coordinates,
#                                 has x_pos {x_pos},
#                                 has y_pos {y_pos};
#                             (subject: $x, location: $coord) isa located_at;
#                             '''
                    
#                     tx.query(insert_query).resolve()
#                     inserted_count += 1
                
#                 tx.commit()
            
#             if inserted_count > 0 or skipped_count > 0:
#                 self.get_logger().info(f"Inserted {inserted_count} objects, skipped {skipped_count} recent duplicates")

#         except Exception as e:
#             self.get_logger().error(f"Error parsing or inserting observation: {str(e)}")

#     def match_and_fetch(self):
#         try:
#             with self.driver.transaction(DB_NAME, TransactionType.READ) as tx:
#                 # Get all objects with their latest information
#                 query = """
#                 match
#                     $obj isa object, has confidence $conf;
#                 fetch {
#                     "object_type": $obj,
#                     "confidence": $conf,
#                 };
#                 """
                
#                 result = tx.query(query).resolve()
#                 results_list = []
                
#                 for answer in result:
#                     results_list.append({
#                         "object_type": str(answer["object_type"]),
#                         "confidence": float(answer["confidence"]),
#                         "timestamp": int(answer["timestamp"])
#                     })
                
#                 if results_list:
#                     msg_out = String()
#                     msg_out.data = json.dumps(results_list)
#                     self.data_view_publisher.publish(msg_out)
#                     self.get_logger().info(f"Published {len(results_list)} objects from TypeDB")
#                 else:
#                     self.get_logger().info("No objects found in TypeDB")

#         except Exception as e:
#             self.get_logger().error(f"Error fetching data from TypeDB: {str(e)}")

#     def cleanup_old_objects(self, max_age_seconds=60):
#         """Remove objects older than the specified age"""
#         try:
#             current_time = int(time.time())
#             with self.driver.transaction(DB_NAME, TransactionType.WRITE) as tx:
#                 # Delete objects older than max_age_seconds
#                 delete_query = f'''
#                 match
#                     $obj isa $type,
#                     has timestamp $ts;
#                     $ts < {current_time - max_age_seconds};
#                 delete $obj isa $type;
#                 '''
                
#                 delete_result = list(tx.query(delete_query).resolve())
#                 tx.commit()
                
#                 if delete_result:
#                     self.get_logger().info(f"Cleaned up {len(delete_result)} old objects")
                
#         except Exception as e:
#             self.get_logger().error(f"Error cleaning up objects: {str(e)}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = KnowledgeGraphNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions
import json
import time
import uuid


# TypeDB configuration
DB_NAME = "dcmdbot_database9"
ADDRESS = "typedb-server:1729"
CREDENTIALS = Credentials("admin", "password")
OPTIONS = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)


class KnowledgeGraphNode(Node):
    def __init__(self):
        super().__init__('knowledge_graph_node')

        # ROS 2 topics
        self.create_subscription(String, '/observation', self.observation_callback, 10)
        self.priority_assign = self.create_publisher(String, '/priority_list', 10)
        self.data_view_publisher = self.create_publisher(String, '/typedb_view', 10)

        # TypeDB driver
        self.driver = TypeDB.driver(ADDRESS, CREDENTIALS, OPTIONS)
        self.latest_observation = None
        # Check/create database
        if not self.driver.databases.contains(DB_NAME):
            self.driver.databases.create(DB_NAME)
            self.get_logger().info(f"Database '{DB_NAME}' created")

            # Load schema on fresh DB creation
            
        else:
            self.get_logger().info(f"Database '{DB_NAME}' exists, reload the schema")
        self.db_schema_setup(self.driver, DB_NAME)
        # Timer to fetch periodically
        self.process_timer = self.create_timer(10.0, self.insert_lastest_observation)
        self.timer = self.create_timer(10.0, self.match_and_fetch)  # every 10 seconds


    
    def db_schema_setup(self, driver, DB_NAME, schema_file='/home/dcmdbot9/dai_ws/src/knowledge_graph/Database/rescue_schema.tql'):
        # In your db_schema_setup method:
        try:
            with open(schema_file, 'r') as data:
                define_query = data.read()
                print ("Schema being loaded 1:\n", define_query)
            with driver.transaction(DB_NAME, TransactionType.SCHEMA) as tx:
                print("Defining schema", end="...")
                tx.query(define_query).resolve()
                tx.commit()
                self.get_logger().info("Schema defined successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load schema: {e}")
    def observation_callback(self, msg):
        self.latest_observation = msg
    def insert_lastest_observation(self):
        if self.latest_observation:
            self.insert(self.latest_observation)
            self.latest_observation = None
    def insert(self, msg):
        try:
            data = json.loads(msg.data)
            objects = data.get("objects", [])
            robot_id = "dcmdbot9"
            mission_description = "searching"


            if not objects:
                self.get_logger().info("No objects found.")
                return

            with self.driver.transaction(DB_NAME, TransactionType.WRITE) as tx:
                # robot_insert = f'''
                # insert $robot isa dcmdbot, has name "{robot_id}";
                # '''
                # tx.query(robot_insert).resolve()

                # mission_insert = f'''
                # insert 
                #     $mission isa mission,
                #     has description "{mission_description}";
                # '''
                # tx.query(mission_insert).resolve()
                for obj in objects:
                    # obj_id = obj.get("object_id", str(uuid.uuid4()))
                    obj_type = obj["Type"].lower()

                    if obj_type == "dog":
                        size = obj["Status"].get("size")
                        confidence = obj["Status"].get("confidence")
                        object_insert = f'''
                        insert 
                        $dog isa {obj_type},
                        has confidence {confidence},
                        has size "{size}";
                        '''
                    elif obj_type == "human":
                        status = obj["Status"].get("status")
                        confidence = obj["Status"].get("confidence")
                        object_insert = f'''
                        insert 
                        $human isa human,
                        has confidence {confidence},
                        has status "{status}";

                        '''
                    elif obj_type == "fire":
                        confidence = obj["Status"].get("confidence")
                        fire_status = "dangerous"
                        object_insert = f'''
                        insert
                        $fire isa fire,
                        has confidence {confidence},
                        has fire_status "{fire_status}";
                    '''
                    else :
                        continue
                        
                    # check_query = f"""
                    # match
                    #     $obj isa {obj_type}, has confidence $conf;
                    # fetch {{
                    # "confidence": $conf       
                    # }};

                    # """
                    # is_duplicate = False

                    # try:
                    #     results = tx.query(check_query).resolve()
                    #     for res in results:
                    #         existing_conf = float(res["confidence"])
                    #         if abs(existing_conf - new_confidence) < 0.1 * new_confidence:
                    #             is_duplicate = True
                    #             break
                    # except Exception as e:
                    #     self.get_logger().warn(f"Query failed during duplicate check: {e}")

                    # if is_duplicate:
                    #     self.get_logger().info(f"Duplicate {obj_type} in '{room_name}' skipped (conf ≈ {new_confidence})")
                    #     continue
                    # insert_query = f"""
                    # insert
                    #     ${obj_type} isa {obj_type},
                    #         has confidence {new_confidence};

                    # """

                    # # Room
                    # if room_name != "unknown":
                    #     insert_query += f"""
                    #     $room isa room,
                    #         has room_name "{room_name}",
                    #         has fire_status {"true" if fire_status else "false"};

                    #     located_in (subject: ${obj_type}, container: $room);
                    #     """

                    
                    if "position" in obj and isinstance(obj["position"], dict):
                        x_pos = obj["position"].get("x")
                        y_pos = obj["position"].get("y")
                        z_pos = obj["position"].get("z")
                        if x_pos is not None and y_pos is not None and z_pos is not None:
                            object_insert += f"""
                            $coord isa coordinates,
                                has x_pos {x_pos},
                                has y_pos {y_pos},
                                has z_pos {z_pos};
                            located_at (subject: ${obj_type}, location: $coord);
                            """

                    tx.query(object_insert).resolve()

                tx.commit()
                self.get_logger().info(f"Inserted {len(objects)} objects into TypeDB.")

        except Exception as e:
            self.get_logger().error(f"Error parsing or inserting observation: {str(e)}")

    def match_and_fetch(self):
        try:
            with self.driver.transaction(DB_NAME, TransactionType.READ) as tx:
                query = """
                match
                $obj isa $obj_type;
                { 
                $obj isa dog;
                $obj has confidence $conf;
                $obj has size $size;
                } or {
                $obj isa human;
                $obj has confidence $conf;
                $obj has status $human_status;

                } or {
                $obj isa fire;
                $obj has confidence $conf;
                $obj has fire_status $fire_bool;

                };

                $loc_at isa located_at, links (subject: $obj, location: $coord);
                $coord has x_pos $x_pos;
                $coord has y_pos $y_pos;
                $coord has z_pos $z_pos; 
                limit 10;
                fetch {
                "object_type": (match
                { $ty label human; } or { $ty label dog; } or { $ty label cat; } or { $ty label fire; };
                $obj isa $ty;
                return first $ty;
                ),
                "confidence": $conf,
                "coordinate": {
                "x" : $coord.x_pos,
                "y" : $coord.y_pos,
                "z" : $coord.z_pos
                }
                };
                """
                result = tx.query(query).resolve()

                results_list = []
                for answer in result:
                    results_list.append({
                        "object_type": str(answer["object_type"]),
                        "confidence": float(answer["confidence"]),
                        # "status": str(answer["status"]),  
                        #"coordinate": str(answer["coordinate"])
                        "x": float(answer["x"]),
                        "y": float(answer["y"]),
                        "z": float(answer["z"])
                        # "room_name": str(answer["room_name"]),
                        # "fire_status": bool(answer["fire_status"])
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
    node = KnowledgeGraphNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# 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
# 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
