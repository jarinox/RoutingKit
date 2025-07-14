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
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

unsigned path_length(const std::vector<unsigned>& path, function<unsigned(unsigned)> travel_time) {
    unsigned length = 0;
    for(unsigned arc : path){
        length += travel_time(arc);
    }
    return length;
}

void add_for_all_profiles(RoutingRequest& req, std::vector<RoutingRequest>& requests) {
    req.profile.set_bit(true, PEDESTRIAN);
    req.profile.set_bit(false, BICYCLE);
    req.profile.set_bit(false, CAR);
    requests.push_back(req);

    req.profile.set_bit(false, PEDESTRIAN);
    req.profile.set_bit(true, BICYCLE);
    req.profile.set_bit(false, CAR);
    requests.push_back(req);

    req.profile.set_bit(false, BICYCLE);
    req.profile.set_bit(true, CAR);
    requests.push_back(req);
}

TEST(CHLR, ch_vs_dijkstra) {
    cout << "Loading OSM data..." << endl;
    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("../map.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

    cout << "Building Contraction Hierarchy..." << endl;
    auto ch = ContractionHierarchy::build(graph.node_count(), tail, graph.head, graph.geo_distance, graph.label);
    //check_contraction_hierarchy_for_errors(ch);

    auto geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);
    std::vector<RoutingRequest> requests;

    RoutingRequest reqBuilder = {true, 49.499352, 8.472701, 49.491333, 8.467192, 0, 0, Label()};
    reqBuilder.from_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.from_latitude, reqBuilder.from_longitude, 1000).id;
    reqBuilder.to_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.to_latitude, reqBuilder.to_longitude, 1000).id;

    //add_for_all_profiles(reqBuilder, requests);

    reqBuilder.from_node = geo_position_to_node.find_nearest_neighbor_within_radius(49.497794, 8.475011, 1000).id;
    reqBuilder.to_node = geo_position_to_node.find_nearest_neighbor_within_radius(49.497204, 8.470293, 1000).id;

    add_for_all_profiles(reqBuilder, requests);


    ContractionHierarchyQuery ch_query(ch);
    Dijkstra dij(graph.first_out, tail, graph.head);

    cout << "Calculating paths..." << endl;
    for(auto& request : requests){
        long long start_time_ch = get_micro_time();

        ch_query.reset()
            .add_source(request.from_node)
            .set_profile(request.profile)
            .add_target(request.to_node)
            .run();

        auto ch_path = ch_query.get_arc_path();

        long long end_time_ch = get_micro_time();
        long long ch_time = end_time_ch - start_time_ch;

        dij.reset().add_source(request.from_node).set_labels(graph.label).set_profile(request.profile);
        long long start_time_dij = get_micro_time();

        while(!dij.is_finished()){
            auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
                return graph.geo_distance[arc];
            });
            if(settle_result.node == request.to_node){
                break;
            }
        }

        auto dij_path = dij.get_arc_path_to(request.to_node);

        long long end_time_dij = get_micro_time();
        long long dij_time = end_time_dij - start_time_dij;

        if(ch_path.empty() || dij_path.empty()){
            FAIL() << "Empty path found for request from (" << request.from_latitude << ", " << request.from_longitude
                   << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                   << human_readable_label(request.profile.invert()) << endl;
        }

        unsigned ch_length = path_length(ch_path, [&](unsigned arc){ return graph.geo_distance[arc]; });
        unsigned dij_length = path_length(dij_path, [&](unsigned arc){ return graph.geo_distance[arc]; });

        if(dij_length != ch_length){
            FAIL() << "Path length mismatch for request from (" << request.from_latitude << ", " << request.from_longitude
                   << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                   << human_readable_label(request.profile.invert()) << "\nCH path length: " << ch_length << ", Dijkstra path length: " << dij_length << endl;
        }

        if(ch_time > dij_time){
            cout << "CH query took longer than Dijkstra for request from (" << request.from_latitude << ", " << request.from_longitude
                 << ") to (" << request.to_latitude << ", " << request.to_longitude << ") with profile "
                 << human_readable_label(request.profile.invert()) << endl;
            cout << "CH time: " << ch_time << " microseconds, Dijkstra time: " << dij_time << " microseconds" << endl;
            FAIL() << "CH query took longer than Dijkstra";
        }
    }
}

