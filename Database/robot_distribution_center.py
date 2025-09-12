from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions

# Configuration
DB_NAME = "distribution_center_robot"
address = "typedb-server:1729"
credentials = Credentials("admin", "password")
options = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)

def list_databases():
    with TypeDB.driver(address, credentials, options) as driver:
        dbs = driver.databases.all()
        if not dbs:
            print("No databases found.")
        else:
            print("Available databases:")
            for db in dbs:
                print(f" - {db.name}")

def create_database(driver, db_name) -> bool:
    print("Creating a new database", end="...")
    driver.databases.create(db_name)
    print("OK")
    db_schema_setup(driver, db_name)
    return True	

def replace_database(driver, db_name) -> bool:
    print("Deleting an existing database", end="...")
    driver.databases.get(db_name).delete()  # Delete the database if it exists already
    print("OK")
    if create_database(driver, db_name):
        return True
    else:
        print("Failed to create a new database. Terminating...")
        return False

def db_schema_setup(driver, db_name, schema_file='/home/dcmdbot9/dai_ws/src/knowledge_graph/Database/rescue_schema.tql'):
    with open(schema_file, 'r') as data:
        define_query = data.read()
        print("Schema being loaded:\n", define_query)
    with driver.transaction(db_name, TransactionType.SCHEMA) as tx:
        print("Defining schema", end="...")
        tx.query(define_query).resolve()
        tx.commit()
        print("OK")	
def db_dataset_setup(driver, db_name, data_file='/home/dcmdbot9/dai_ws/src/knowledge_graph/rescue_schema.tql'):
    with open(data_file, 'r') as data:
        insert_query = data.read()
    with driver.transaction(db_name, TransactionType.WRITE) as tx:
        print("Loading data", end="...")
        tx.query(insert_query).resolve()
        tx.commit()
        print("OK")
        
def db_setup(driver, db_name, db_reset=False) -> bool:
    print(f"Setting up the database: {db_name}")
    if driver.databases.contains(db_name):
        if db_reset or (input("Found a pre-existing database. Do you want to replace it? (Y/N) ").lower() == "y"):
            if not replace_database(driver, db_name):
                return False
        else:
            print("Reusing an existing database.")
    else:  # No such database found on the server
        if not create_database(driver, db_name):
            print("Failed to create a new database. Terminating...")
            return False
        return True
                
def main():
    with TypeDB.driver(address, credentials, options) as driver:
        db_setup(driver, DB_NAME, db_reset=False)

if __name__ == "__main__":
    list_databases()
    main()