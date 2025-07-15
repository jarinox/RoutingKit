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


bool file_exists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

TEST(CHLR_alt, witness_search) {
    std::string pbf_file = "b.osm.pbf";
    if(!file_exists("b.osm.pbf")) {
        pbf_file = "../" + pbf_file;
    }

    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf(pbf_file);
	auto tail = invert_inverse_vector(graph.first_out);

    auto geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);

    RoutingRequest reqBuilder = {true, 47.629668, 7.908236, 47.631892, 7.912852, 0, 0, Label()};
    reqBuilder.from_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.from_latitude, reqBuilder.from_longitude, 1000).id;
    reqBuilder.to_node = geo_position_to_node.find_nearest_neighbor_within_radius(reqBuilder.to_latitude, reqBuilder.to_longitude, 1000).id;
    reqBuilder.profile.set_bit(true, CAR);
    auto restriction = reqBuilder.profile;

    CHLRGraph chg = CHLRGraph(graph.node_count(), tail, graph.head, graph.geo_distance, graph.latitude, graph.longitude, graph.label);
    DijkstraLR dijkstra(chg, reqBuilder.from_node);

    unsigned distance = dijkstra.witness_search(reqBuilder.to_node, restriction, [&](unsigned node) {
        return true; // All nodes are valid for this test
    });

    Dijkstra dij(graph.first_out, tail, graph.head);
    dij.reset().add_source(reqBuilder.from_node).set_profile(reqBuilder.profile);
    while(!dij.is_finished()){
        auto settle_result = dij.settle([&](unsigned arc, unsigned distance){
            return graph.geo_distance[arc];
        });
        if(settle_result.node == reqBuilder.to_node){
            break;
        }
    }

    unsigned reference_distance = dij.get_distance_to(reqBuilder.to_node);

    EXPECT_EQ(distance, reference_distance) << "Witness search distance does not match Dijkstra's distance.";
}

TEST(CHLR_alt, add_arc) {
    CHLRGraph graph;
    graph.nodes.resize(3);

    graph.add_arc(0, invalid_id, 1, 10, Label());
    graph.add_arc(1, invalid_id, 2, 11, Label());
    graph.add_arc(2, invalid_id, 0, 12, Label());

    ASSERT_EQ(graph.nodes[0].out_arcs[0].weight, 10);
    ASSERT_EQ(graph.nodes[1].in_arcs[0].weight, 10);
    ASSERT_EQ(graph.nodes[1].out_arcs[0].weight, 11);
    ASSERT_EQ(graph.nodes[2].in_arcs[0].weight, 11);
    ASSERT_EQ(graph.nodes[2].out_arcs[0].weight, 12);
    ASSERT_EQ(graph.nodes[0].in_arcs[0].weight, 12);
}


TEST(CHLR_alt, remove_incident_arc) {
    CHLRGraph graph;
    graph.nodes.resize(3);

    graph.add_arc(0, invalid_id, 1, 10, Label());
    graph.add_arc(1, invalid_id, 2, 11, Label());
    graph.add_arc(2, invalid_id, 0, 12, Label());
    graph.remove_incident_arcs(1);

    ASSERT_EQ(graph.nodes[0].in_arcs.size(), 1); // incoming from 2
    ASSERT_EQ(graph.nodes[0].out_arcs.size(), 0);
    ASSERT_EQ(graph.nodes[1].in_arcs.size(), 0);
    ASSERT_EQ(graph.nodes[1].out_arcs.size(), 0);;
    ASSERT_EQ(graph.nodes[2].in_arcs.size(), 0);
    ASSERT_EQ(graph.nodes[2].out_arcs.size(), 1); // outgoing to 0
}

TEST(CHLR_alt, building_order_and_shortcuts){
    CHLRGraph graph;
    graph.nodes.resize(5);

    Label car_label = Label();
    car_label.set_bit(true, CAR);

    Label pedestrian_car_label = Label();
    pedestrian_car_label.set_bit(true, PEDESTRIAN);
    pedestrian_car_label.set_bit(true, CAR);

    graph.add_arc(0, invalid_id, 1, 10, car_label); 
    graph.add_arc(1, invalid_id, 2, 20, car_label);
    graph.add_arc(2, invalid_id, 3, 30, pedestrian_car_label); 
    graph.add_arc(3, invalid_id, 4, 40, pedestrian_car_label); 
    graph.add_arc(4, invalid_id, 0, 50, car_label);

    CHLR chlr(graph);
    chlr.build();

    ASSERT_EQ(chlr.order.size(), 5);
    for (unsigned i = 0; i < chlr.order.size(); ++i) {
        ASSERT_EQ(graph.nodes[chlr.order[i]].rank, i + 1);
    }

    ASSERT_EQ(graph.nodes[0].rank, 1);
    ASSERT_EQ(graph.nodes[2].rank, 2);
    ASSERT_EQ(graph.nodes[3].rank, 3);
    ASSERT_EQ(graph.nodes[4].rank, 4);
    ASSERT_EQ(graph.nodes[1].rank, 5);

    ASSERT_EQ(graph.nodes[4].out_arcs[1].mid_node, 0);
    ASSERT_EQ(graph.nodes[1].in_arcs[1].mid_node, 0);
    ASSERT_EQ(graph.nodes[1].in_arcs[1].label, car_label);
    ASSERT_EQ(graph.nodes[1].out_arcs[1].mid_node, 2);
    ASSERT_EQ(graph.nodes[3].in_arcs[1].mid_node,  2);
    ASSERT_EQ(graph.nodes[1].out_arcs[2].mid_node, 3);
    ASSERT_EQ(graph.nodes[4].in_arcs[1].mid_node, 3);
    ASSERT_EQ(graph.nodes[4].in_arcs[1].label, pedestrian_car_label);

    ASSERT_EQ(graph.nodes[0].in_arcs.size(), 1);
    ASSERT_EQ(graph.nodes[0].out_arcs.size(), 1);
    ASSERT_EQ(graph.nodes[1].in_arcs.size(), 2);
    ASSERT_EQ(graph.nodes[1].out_arcs.size(), 3);
    ASSERT_EQ(graph.nodes[2].in_arcs.size(), 1);
    ASSERT_EQ(graph.nodes[2].out_arcs.size(), 1);
    ASSERT_EQ(graph.nodes[3].in_arcs.size(), 2);
    ASSERT_EQ(graph.nodes[3].out_arcs.size(), 1);
    ASSERT_EQ(graph.nodes[4].in_arcs.size(), 2);
    ASSERT_EQ(graph.nodes[4].out_arcs.size(), 2);
}


TEST(CHLR_alt, extract_forward_and_backward_graphs) {
    CHLRGraph graph;
    graph.nodes.resize(5);

    Label car_label = Label();
    car_label.set_bit(true, CAR);

    Label pedestrian_car_label = Label();
    pedestrian_car_label.set_bit(true, PEDESTRIAN);
    pedestrian_car_label.set_bit(true, CAR);

    graph.add_arc(0, invalid_id, 1, 10, car_label); 
    graph.add_arc(1, invalid_id, 2, 20, car_label);
    graph.add_arc(2, invalid_id, 3, 30, pedestrian_car_label); 
    graph.add_arc(3, invalid_id, 4, 40, pedestrian_car_label); 
    graph.add_arc(4, invalid_id, 0, 50, car_label);

    CHLR chlr(graph);
    chlr.build();

    CHLRQuery query(chlr.graph);
    for(unsigned i = 0; i < chlr.graph.nodes.size(); ++i) {
        ASSERT_EQ(query.forward.nodes[i].rank, chlr.graph.nodes[i].rank);
        ASSERT_EQ(query.backward.nodes[i].rank, chlr.graph.nodes[i].rank);
        ASSERT_EQ(query.forward.nodes[i].lat, chlr.graph.nodes[i].lat);
        ASSERT_EQ(query.backward.nodes[i].lat, chlr.graph.nodes[i].lat);
        ASSERT_EQ(query.forward.nodes[i].lon, chlr.graph.nodes[i].lon);
        ASSERT_EQ(query.backward.nodes[i].lon, chlr.graph.nodes[i].lon);

        for(auto& arc : query.forward.nodes[i].out_arcs) {
            ASSERT_LT(query.forward.nodes[i].rank, query.forward.nodes[arc.other_node].rank);
        }

        for(auto& arc : query.backward.nodes[i].out_arcs) {
            ASSERT_LT(query.backward.nodes[i].rank, query.backward.nodes[arc.other_node].rank);
        }
    }
}
