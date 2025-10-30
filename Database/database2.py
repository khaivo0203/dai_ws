from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions

# Configuration for external access
DB_NAME = "dcmdbot_database25" 
address = "localhost:1729"  # Same server address
credentials = Credentials("admin", "password")  # Your credentials
options = DriverOptions(is_tls_enabled=False, tls_root_ca_path=None)

# Connect and fetch data
with TypeDB.driver(address, credentials, options) as driver:
    try:
        driver.databases.create(DB_NAME)
        print(f"Database '{DB_NAME}' created")
    except Exception:
        print(f"Database '{DB_NAME}' already exists")
    with driver.transaction(DB_NAME, TransactionType.READ) as tx:
        print
        fetch_query = """
              match
                $obj isa $obj_type;
                { 
                $obj isa dog;
                $obj has confidence $conf;
                $obj has size $status;
                } or {
                $obj isa human;
                $obj has confidence $conf;
                $obj has status $status;
                };

                $loc_at isa located_at, links (subject: $obj, location: $coord);
                $coord has x_pos $x_pos;
                $coord has y_pos $y_pos;
                $coord has z_pos $z_pos; 
                


                $fire isa fire;
                $fire_coord isa coordinates, has x_pos $x_fire, has y_pos $y_fire, has z_pos $z_fire;
                $loc isa located_at, links (subject: $fire, location: $fire_coord);
                


                
                $loc1 isa located_at, links (subject:$obj, location:$obj_coord);
                $obj_coord isa coordinates, has x_pos $x_pos, has y_pos $y_pos, has z_pos $z_pos;

                let $dist = distance($fire_coord, $obj_coord);
                let $priority_score = priority_score($obj, $fire_coord);
                
                


                fetch {
                "object_type": (match
                { $ty label human; } or { $ty label dog; } or { $ty label fire; };
                $obj isa $ty;
                return first $ty;
                ),
                "status": $status,
                "confidence": $conf,
                "coordinate": {
                "x" : $coord.x_pos,
                "y" : $coord.y_pos,
                "z" : $coord.z_pos
                },
                "priority": $priority_score,
                "distance from fire": $dist

                };
        """
        result = tx.query(fetch_query).resolve()
        print("Fetched data from external connection", DB_NAME)
        for answer in result:
            print(answer)

# import time

# from pyvis.network import Network

# import networkx as nx

# from typedb.driver import TypeDB, TransactionType, Credentials, DriverOptions
 
# DB_NAME = "dcmdbot_database25"

# address = "localhost:1729"

# credentials = Credentials("admin", "password")

# options = DriverOptions(is_tls_enabled=False)
 
# def fetch_data():

#     with TypeDB.driver(address, credentials, options) as driver:

#         with driver.transaction(DB_NAME, TransactionType.READ) as tx:

#             fetch_query = """

#                 match

#                   $obj isa $obj_type;

#                   {

#                     $obj isa dog;

#                     $obj has confidence $conf;

#                     $obj has size $status;

#                   } or {

#                     $obj isa human;

#                     $obj has confidence $conf;

#                     $obj has status $status;

#                   };
 
#                   $loc_at isa located_at, links (subject: $obj, location: $coord);

#                   $coord has x_pos $x_pos;

#                   $coord has y_pos $y_pos;

#                   $coord has z_pos $z_pos;
 
#                   $fire isa fire;

#                   $fire_coord isa coordinates, has x_pos $x_fire, has y_pos $y_fire, has z_pos $z_fire;

#                   $loc isa located_at, links (subject: $fire, location: $fire_coord);
 
#                     let $dist = 1;

#                     let $priority_score = priority_score($obj, $fire_coord);
 
#                   fetch {

#                     "object_type": (match

#                       { $ty label human; } or { $ty label dog; } or { $ty label fire; };

#                       $obj isa $ty;

#                       return first $ty;

#                     ),

#                     "status": $status,

#                     "confidence": $conf,

#                     "coordinate": {

#                       "x" : $coord.x_pos,

#                       "y" : $coord.y_pos,

#                       "z" : $coord.z_pos

#                     },

#                     "priority": $priority_score,

#                     "distance from fire": $dist

#                   };

#             """

#             result = tx.query(fetch_query).resolve()

#             parsed_data = []
 
#             for answer in result:

#                 obj_type = answer.get("object_type")["label"]

#                 status = answer.get("status")

#                 conf = float(answer.get("confidence"))
 
#                 coord = answer.get("coordinate")

#                 x = float(coord["x"])

#                 y = float(coord["y"])

#                 z = float(coord["z"])
 
#                 priority = float(answer.get("priority"))

#                 dist = float(answer.get("distance from fire"))
 
#                 parsed_data.append({

#                     "type": obj_type,

#                     "status": status,

#                     "confidence": conf,

#                     "x": x,

#                     "y": y,

#                     "z": z,

#                     "priority": priority,

#                     "distance": dist

#                 })
 
#             return parsed_data
 
 
# def create_graph(data):

#     G = nx.Graph()
 
#     # Add nodes with attributes

#     for i, obj in enumerate(data):

#         label = f'{obj["type"]}\nPrio: {obj["priority"]:.2f}'

#         size = obj["confidence"] * 30 + 10

#         color = "blue" if obj["type"] == "human" else "red" if obj["type"] == "dog" else "orange"
 
#         G.add_node(i, label=label, size=size, color=color, x=obj["x"], y=obj["y"], z=obj["z"])
 
#     # Add edges based on distance threshold (example: connect nodes within 10 units distance)

#     for i in range(len(data)):

#         for j in range(i + 1, len(data)):

#             dist = ((data[i]["x"] - data[j]["x"])**2 + (data[i]["y"] - data[j]["y"])**2 + (data[i]["z"] - data[j]["z"])**2) ** 0.5

#             if dist < 10:

#                 G.add_edge(i, j, value=1)
 
#     return G
 
# def visualize_graph(G, filename="graph.html"):

#     net = Network(height="750px", width="100%", bgcolor="#222222", font_color="white")
 
#     # Set physics for nicer layout

#     net.barnes_hut()
 
#     net.from_nx(G)
 
#     # Customize node sizes and colors

#     for node in net.nodes:

#         node['size'] = G.nodes[node['id']]['size']

#         node['color'] = G.nodes[node['id']]['color']

#         node['title'] = G.nodes[node['id']]['label']

#         node['label'] = G.nodes[node['id']]['label']
 
#     net.show(filename, notebook=False)
 
# def main(update_interval=5, cycles=10):

#     for i in range(cycles):

#         print(f"Fetching data, cycle {i+1}/{cycles}...")

#         data = fetch_data()

#         print(f"Fetched {len(data)} objects")

#         G = create_graph(data)

#         visualize_graph(G)

#         print(f"Graph updated: open 'graph.html' in your browser to see it.")

#         time.sleep(update_interval)
 
# if __name__ == "__main__":

#     main()

 
