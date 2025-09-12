from typedb.driver import TypeDB, Credentials, DriverOptions, TransactionType

SERVER_ADDR = "https://bfzgdz-0.cluster.typedb.com:80"
USERNAME = "admin"
PASSWORD = "Khai@vo0603"
DB_NAME = "dcmdbot_database"
    
def list_databases():
    # Use False for TLS (encryption), and None for custom CA cert
    options = DriverOptions(True, None)
    with TypeDB.driver(SERVER_ADDR, Credentials(USERNAME, PASSWORD), options) as driver:
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

def driver_connect(uri, username=USERNAME, password=PASSWORD):
    # tag::driver_new[]
    driver = TypeDB.driver(uri, Credentials(username, password), DriverOptions(True, None))
    # end::driver_new[]
    return driver

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
    # if driver.databases.contains(db_name):
    #     return validate_data(driver, db_name)
    # else:
    #     print("Database not found. Terminating...")
    #     return False
# end::db-setup[]

def replace_database(driver, db_name) -> bool:
    print("Deleting an existing database", end="...")
    driver.databases.get(db_name).delete()  # Delete the database if it exists already
    print("OK")
    if create_database(driver, db_name):
        return True
    else:
        print("Failed to create a new database. Terminating...")
        return False
    
def validate_data(driver, db_name) -> bool:
    with driver.transaction(db_name, TransactionType.READ) as tx:
        count_query = "match $x isa human; reduce $count = count;"
        print("Testing the dataset", end="...")
        count = next(tx.query(count_query).resolve().as_concept_rows()).get("count").try_get_integer()
        if count == 3:
            print("Passed")
            return True
        else:
            print("Validation failed, unexpected number of users:", count, "\n Expected result: 3. Terminating...")
            return False


def main():
    with driver_connect(SERVER_ADDR) as driver:
        db_setup(driver, DB_NAME, db_reset=False)

if __name__ == "__main__":
    list_databases()
    main()

