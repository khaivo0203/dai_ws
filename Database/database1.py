from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions

# Configuration
DB_NAME = "dcmdbot_database"
address = "typedb-server:1729"
credentials = Credentials("admin", "password")
options = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)

# Connect to TypeDB
with TypeDB.driver(address, credentials, options) as driver:
    # Create database
    try:
        driver.databases.create(DB_NAME)
        print(f"Database '{DB_NAME}' created")
    except Exception:
        print(f"Database '{DB_NAME}' already exists")
    
    # Load schema
    with driver.transaction(DB_NAME, TransactionType.SCHEMA) as tx:
        schema_query = """
define

  # Attributes
  attribute type, value string;
  attribute name, value string;
  attribute robot_id , value string;
  attribute mission_id, value string;
  attribute object_id, value string;
  attribute room_id, value string;
  attribute room_name, value string;
  attribute discription value string;
  attribute priority value integer;
  attribute size, value string;
  attribute status, value string;
  attribute confidence, value double;
  attribute fire_status, value boolean;
  attribute x_pos, value double;
  attribute y_pos, value double;
  # attribute timestamp, value integer;


  # Base entity
  entity robot,
    owns name,
    owns robot_id @key,
    plays located_at:subject,
    plays assigned_to:robot,
    plays exchange:exchange_to;

  entity mission,
    owns mission_id @key,
    owns discription,
    plays exchange:info,
    plays assigned_to:mission;

  entity coordinates,
    owns x_pos,
    owns y_pos,
    plays in_room:coordinate,
    plays located_at:location;

  entity room,
    owns room_name,
    owns room_id,
    owns fire_status,
    plays exchange:info,
    plays in_room:room,
    plays located_in:container;

  # Object of interest entity
  entity object,
    owns type,
    owns confidence,
    # owns timestamp,
    plays located_at:subject,
    plays exchange:info,
	  plays located_in:subject;

  # Subtypes of object
  entity dog sub object,
    owns size,
    plays assign_priority:patient;

  entity human sub object,
    owns status,
    plays assign_priority:patient;

  entity cat sub object,
    plays assign_priority:patient;

  entity fire sub object,
    owns fire_status;


  # Relations
  relation assign_priority,
    owns priority,
    relates patient;
  relation assigned_to,
    relates robot,
    relates mission;
  relation located_at,
    relates subject,
    relates location;
  relation located_in,
    relates subject,
    relates container;
  relation exchange,
    relates exchange_to,
    relates info;
  relation in_room,
    relates coordinate,
    relates room;

        """
        # tx.query(schema_query).resolve()
        # tx.commit()
        # print("Schema loaded successfully1")
        # with driver.transaction(DB_NAME, TransactionType.WRITE) as tx: 
        #   check_query = f"""
        #   match
        #       $obj isa $obj_type, has confidence $conf;
        #   fetch {{
        #   "confidence": $conf;       
        #   }};

        #   """
        #   results = tx.query(check_query).resolve()
        #   for res in results:
        #       existing_conf = float(res["confidence"])
        #       if abs(existing_conf - new_confidence) < 0.1 * new_confidence:
        #           is_duplicate = True
        #           break
    # Insert data
#     with driver.transaction(DB_NAME, TransactionType.WRITE) as tx:
#         insert_query = """
# insert
#     $dog isa dog,
#         has confidence 0.95,
#         has type "dog";
#         """
#         tx.query(insert_query).resolve()
#         tx.commit()
#         print("Data inserted successfully1")
    with driver.transaction(DB_NAME, TransactionType.READ) as tx:
      query = """
      match
        $obj isa $obj_type, has confidence $conf;
      get $conf;
      """
      result = tx.query(query).resolve()
      for answer in result:
          obj = answer.get("obj_type")
          conf_attr = answer.get("conf")
          confidence_value = conf_attr.get_value()
          print(f"Object ID: {obj.get_iid()}, Confidence: {confidence_value}")
    
    # Fetch data
    with driver.transaction(DB_NAME, TransactionType.READ) as tx:
        fetch_query = """
        match
        $obj isa $obj_type, has confidence $conf;
        fetch {
          "object_type": $obj_type,
          "confidence": $conf
        };
        """
        result = tx.query(fetch_query).resolve()
        
        print("\nFetched camera detection data:")
        for answer in result:
            print(answer)



# Username:
# khaiv
# Password:
# 4719
