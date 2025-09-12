from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions

# Configuration for external access
DB_NAME = "dcmdbot_database9"
address = "typedb-server:1729"  # Same server address
credentials = Credentials("admin", "password")  # Your credentials
options = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)

# Connect and fetch data
with TypeDB.driver(address, credentials, options) as driver:
    with driver.transaction(DB_NAME, TransactionType.READ) as tx:
        fetch_query = """
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
                },
                "x" : $coord.x_pos,
                "y" : $coord.y_pos,
                "z" : $coord.z_pos

                };
        """
        result = tx.query(fetch_query).resolve()
        
        print("Fetched data from external connection:")
        for answer in result:
            print(answer)