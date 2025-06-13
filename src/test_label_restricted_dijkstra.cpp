#include <routingkit/osm_simple.h>
#include <routingkit/dijkstra.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_label_decoder.h>
#include <iostream>
using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
	auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
	auto carGraph = simple_load_osm_car_routing_graph_from_pbf("map.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

	// Build the index to quickly map latitudes and longitudes
	GeoPositionToNode map_geo_position_car(carGraph.latitude, carGraph.longitude);
	GeoPositionToNode map_geo_position(graph.latitude, graph.longitude);
	Dijkstra dij(graph.first_out, tail, graph.head);

	Label profile = Label(0);
	profile.set_bit(false, CAR);
	profile.set_bit(true, BICYCLE); // Use bicycle profile
	profile.set_bit(false, PEDESTRIAN);

	float from_latitude, from_longitude, to_latitude, to_longitude;
	if (argc != 5){
        cout << "Usage: " << argv[0] << " from_latitude from_longitude to_latitude to_longitude" << endl;
        return 1;
    } else {
        from_latitude = atof(argv[1]);
        from_longitude = atof(argv[2]);
        to_latitude = atof(argv[3]);
        to_longitude = atof(argv[4]);
    }

	// Snap to input coordinates to the car graph
	unsigned fromIdCar = map_geo_position_car.find_nearest_neighbor_within_radius(from_latitude, from_longitude, 1000).id;
	from_latitude = carGraph.latitude[fromIdCar];
	from_longitude = carGraph.longitude[fromIdCar];
	unsigned toIdCar = map_geo_position_car.find_nearest_neighbor_within_radius(to_latitude, to_longitude, 1000).id;
	to_latitude = carGraph.latitude[toIdCar];
	to_longitude = carGraph.longitude[toIdCar];

    unsigned from = map_geo_position.find_nearest_neighbor_within_radius(from_latitude, from_longitude, 1000).id;
		if(from == invalid_id){
			cout << "No node within 1000m from source position" << endl;
			return 1;
		}
		unsigned to = map_geo_position.find_nearest_neighbor_within_radius(to_latitude, to_longitude, 1000).id;
		if(to == invalid_id){
			cout << "No node within 1000m from target position" << endl;
			return 1;
		}

		bool from_is_allowed = false;
		for(unsigned arc = graph.first_out[from]; arc < graph.first_out[from+1]; ++arc) {
			if(graph.label[arc].is_allowed(profile.get_label())) {
				from_is_allowed = true;
				break;
			}
		}

		if(!from_is_allowed){
			cout << "Source node is not allowed for the given profile" << endl;
			return 1;
		}

		dij.reset().add_source(from).set_labels(graph.label).set_profile(profile);
		while(!dij.is_finished()){
			auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
				return graph.geo_distance[arc];
			});
			if(settle_result.node == to){
				break;
			}
		}
		if(!dij.was_node_reached(to)){
			cout << "No path found from source to target" << endl;
			return 1;
		}

		long long start_time = get_micro_time();
		auto path = dij.get_node_path_to(to);
		long long end_time = get_micro_time();

		cout << from_latitude << " " << from_longitude << " "
             << to_latitude << " " << to_longitude << endl;

		for(auto x:path)
			cout << graph.latitude[x] << " " << graph.longitude[x] << " " << human_readable_label(graph.label[x]) << endl;
}
