#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/chlr.h>
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

unsigned path_length(const std::vector<unsigned>& path, function<unsigned(unsigned)> weight) {
    unsigned length = 0;
    for(unsigned arc : path){
        length += weight(arc);
    }
    return length;
}

bool file_exists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

TEST(CHLR, ch_vs_dijkstra) {
    cout << "Loading OSM data..." << endl;
    std::string pbf_file = "b.osm.pbf";
    if(!file_exists("b.osm.pbf")) {
        pbf_file = "../" + pbf_file;
    }

    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf(pbf_file);
	auto tail = invert_inverse_vector(graph.first_out);

    cout << "Building Contraction Hierarchy..." << endl;
    auto chg = CHLRGraph(graph.node_count(), tail, graph.head, graph.geo_distance, graph.latitude, graph.longitude, graph.label);
    auto ch = CHLR(chg);
    ch.build();
    return;

    auto geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);
    std::vector<RoutingRequest> requests;

    RoutingRequest reqBuilder = {true, 47.629668, 7.908236, 47.631892, 7.912852, 0, 0, Label()};
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
}

