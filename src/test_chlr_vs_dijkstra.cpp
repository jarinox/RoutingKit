#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/min_max.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/label.h>
#include <routingkit/io_helper.h>
#include <routingkit/dijkstra.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    cout << "Loading OSM data..." << endl;
    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

    cout << "Building Contraction Hierarchy..." << endl;
    auto ch = ContractionHierarchy::build(graph.node_count(), tail, graph.head, graph.travel_time, graph.label);
    //check_contraction_hierarchy_for_errors(ch);

    auto geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);
    std::vector<RoutingRequest> requests;

    RoutingRequest reqBuilder = {true, 49.499352, 8.472701, 49.491333, 8.467192, 0, 0, Label()};
    reqBuilder.from_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.from_latitude, reqBuilder.from_longitude, 1000).id;
    reqBuilder.to_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.to_latitude, reqBuilder.to_longitude, 1000).id;
    reqBuilder.profile.set_bit(true, CAR);
    requests.push_back(reqBuilder);

    reqBuilder.profile.set_bit(false, CAR);
    reqBuilder.profile.set_bit(true, BICYCLE);
    requests.push_back(reqBuilder);

    reqBuilder.profile.set_bit(false, BICYCLE);
    reqBuilder.profile.set_bit(true, PEDESTRIAN);
    requests.push_back(reqBuilder);

    ContractionHierarchyQuery ch_query(ch);
    Dijkstra dij(graph.first_out, tail, graph.head);

    cout << "Calculating paths..." << endl;
    for(auto& request : requests){
        ch_query.reset()
            .add_source(request.from_node)
            .set_profile(request.profile)
            .add_target(request.to_node)
            .run();

        auto ch_path = ch_query.get_arc_path();

        dij.reset().add_source(request.from_node).set_labels(graph.label).set_profile(request.profile);
        while(!dij.is_finished()){
            auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
                return graph.geo_distance[arc];
            });
            if(settle_result.node == request.to_node){
                break;
            }
        }

        auto dij_path = dij.get_arc_path_to(request.to_node);

        if(ch_path.size() != dij_path.size()){
            cout << "Path size mismatch for request from (" << request.from_latitude << ", " << request.from_longitude
                 << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                 << human_readable_label(request.profile.invert()) << endl;
            cout << "CH path size: " << ch_path.size() << ", Dijkstra path size: " << dij_path.size() << endl;
            return 1;
        }

        for(unsigned i = 0; i < ch_path.size(); ++i){
            if(ch_path[i] != dij_path[i]){
                cout << "Path mismatch at index " << i << " for request from (" << request.from_latitude << ", " << request.from_longitude
                     << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                     << human_readable_label(request.profile.invert()) << endl;
                cout << "CH path arc: " << ch_path[i] << ", Dijkstra path arc: " << dij_path[i] << endl;
                return 1;
            }
        }

        if(ch_path.empty()){
            cout << "Empty path found for request from (" << request.from_latitude << ", " << request.from_longitude
                 << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                 << human_readable_label(request.profile.invert()) << endl;
            return 1;
        }
    }


}

