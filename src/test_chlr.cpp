#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/min_max.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/label.h>
#include <routingkit/io_helper.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
    auto tail = invert_inverse_vector(graph.first_out);

    auto ch = ContractionHierarchy::build(graph.node_count(), tail, graph.head, graph.geo_distance, graph.label);

    auto geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);
    RoutingRequest request = parse_routing_request(argc, argv, geo_position_to_node);

    if(!request.is_valid){
        cout << "Invalid routing request. Please provide valid coordinates." << endl;
        return 1;
    }

    ContractionHierarchyQuery ch_query(ch);

    auto time = -get_micro_time();
    ch_query.reset()
        .add_source(request.from_node)
        .set_profile(request.profile)
        .add_target(request.to_node)
        .run();

    auto path = ch_query.get_arc_path();

    time += get_micro_time();

    //cout << "Query time: " << time << " microseconds" << endl;

    cout << request.from_latitude << " " << request.from_longitude << " "
			<< request.to_latitude << " " << request.to_longitude << endl;

	for(auto x:path)
		cout << graph.latitude[graph.head[x]] << " " << graph.longitude[graph.head[x]] << " "
				<< human_readable_label(graph.label[x]) << endl;
    
}

