#include <routingkit/osm_simple.h>
#include <routingkit/dijkstra.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/io_helper.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_label_decoder.h>
#include <iostream>
using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
	auto log_message = [](const std::string&msg){
		cout << msg << endl;
	};

	auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

	// Build the index to quickly map latitudes and longitudes
	GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);
	Dijkstra dij(graph.first_out, tail, graph.head);

	auto request = parse_routing_request(argc, argv, map_geo_position);

	dij.reset().add_source(request.from_node).set_labels(graph.label).set_profile(request.profile);
	while(!dij.is_finished()){
		auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
			return graph.geo_distance[arc];
		});
		if(settle_result.node == request.to_node){
			break;
		}
	}
	if(!dij.was_node_reached(request.to_node)){
		cout << "No path found from source to target" << endl;
		return 1;
	}

	long long start_time = get_micro_time();
	auto path = dij.get_node_path_to(request.to_node);
	long long end_time = get_micro_time();

	cout << request.from_latitude << " " << request.from_longitude << " "
			<< request.to_latitude << " " << request.to_longitude << endl;

	for(auto x:path)
		cout << graph.latitude[x] << " " << graph.longitude[x] << " " << human_readable_label(graph.label[x]) << endl;
}
