#ifndef ROUTINGKIT_TEST_BUILDER_H
#define ROUTINGKIT_TEST_BUILDER_H

#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/chlr.h>
#include <routingkit/min_max.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/label.h>
#include <routingkit/io_helper.h>
#include <routingkit/dijkstra.h>

#include <gtest/gtest.h>

void _add_for_all_profiles(std::vector<RoutingRequest>& requests, float from_lat, float from_long,
                           float to_lat, float to_long, GeoPositionToNode& geo_position_to_node) {
    RoutingRequest req;
    req.from_latitude = from_lat;
    req.from_longitude = from_long;
    req.to_latitude = to_lat;
    req.to_longitude = to_long;

    req.from_node = geo_position_to_node.find_nearest_neighbor_within_radius(req.from_latitude, req.from_longitude, 1000).id;
    req.to_node = geo_position_to_node.find_nearest_neighbor_within_radius(req.to_latitude, req.to_longitude, 1000).id;

    RoutingRequest req_rev = req;
    std::swap(req_rev.from_node, req_rev.to_node);
    std::swap(req_rev.from_latitude, req_rev.to_latitude);
    std::swap(req_rev.from_longitude, req_rev.to_longitude);

    req.profile.set_bit(true, PEDESTRIAN);
    req.profile.set_bit(false, BICYCLE);
    req.profile.set_bit(false, CAR);
    requests.push_back(req);

    req_rev.profile = req.profile;
    requests.push_back(req_rev);

    req.profile.set_bit(false, PEDESTRIAN);
    req.profile.set_bit(true, BICYCLE);
    req.profile.set_bit(false, CAR);
    requests.push_back(req);

    req_rev.profile = req.profile;
    requests.push_back(req_rev);

    req.profile.set_bit(false, BICYCLE);
    req.profile.set_bit(true, CAR);
    requests.push_back(req);

    req_rev.profile = req.profile;
    requests.push_back(req_rev);
}

class ResultArc {
public:
    float from_latitude;
    float from_longitude;
    float to_latitude;
    float to_longitude;

    unsigned weight;
    Label label;
};

class RoutingResult {
public:
    std::vector<ResultArc> path;
    unsigned total_weight;
    unsigned time_taken;
};


class TestSetup {
public:
    SimpleOSMMultiProfileRoutingGraph graph;
    std::vector<unsigned> tail;
    ContractionHierarchy ch;
    CHLRGraph chg;
    CHLR chlr;
    Dijkstra dij;

    GeoPositionToNode geo_position_to_node;
    std::vector<RoutingRequest> requests;

    TestSetup(std::string osm_file) : chg(), chlr(chg) {
        std::string path = "graphs/" + osm_file;
        std::ifstream file(path);
        if(!file.good()){
            path = "../" + path;
        }

        std::cout << "Loading graph from " << path << std::endl;
        graph = simple_load_osm_multi_profile_routing_graph_from_pbf(path);
        tail = invert_inverse_vector(graph.first_out);

        std::cout << "Building geo position to node mapping..." << std::endl;
        geo_position_to_node = GeoPositionToNode(graph.latitude, graph.longitude);

        std::cout << "Initializing algorithms..." << std::endl;
        chg = CHLRGraph(graph.node_count(), tail, graph.head, graph.geo_distance, graph.latitude, graph.longitude, graph.label);
        chlr.order.resize(graph.node_count());
        chlr.graph = chg;

        dij = Dijkstra(graph.first_out, tail, graph.head);

        if(osm_file == "rippo.osm.pbf") {
            _add_for_all_profiles(requests, 47.575653, 7.990065, 47.578838, 7.988847, geo_position_to_node);
            _add_for_all_profiles(requests, 47.578415, 7.991132, 47.576568, 7.985245, geo_position_to_node);
            _add_for_all_profiles(requests, 47.575654, 7.984099, 47.575508, 7.991384, geo_position_to_node);
        }

        if(osm_file == "ma_alter_messplatz.osm.pbf" || osm_file == "ma_min_messplatz.osm.pbf") {
            _add_for_all_profiles(requests, 49.499352, 8.472701, 49.491333, 8.467192, geo_position_to_node);
            _add_for_all_profiles(requests, 49.497794, 8.475011, 49.497204, 8.470293, geo_position_to_node);
        }

        if(osm_file == "hd_west.osm.pbf") {
            _add_for_all_profiles(requests, 49.403746, 8.693190, 49.404382, 8.688909, geo_position_to_node);
            _add_for_all_profiles(requests, 49.403362, 8.690068, 49.402307, 8.692675, geo_position_to_node);
        }

        if(osm_file == "hd_neuenheim_min.osm.pbf" || osm_file == "hd_neuenheim.osm.pbf") {
            _add_for_all_profiles(requests, 49.422691, 8.686860, 49.423005, 8.680583, geo_position_to_node);
            _add_for_all_profiles(requests, 49.422852, 8.681796, 49.421650, 8.685851, geo_position_to_node);
        }

        if(osm_file == "hd_neuenheim.osm.pbf" || osm_file == "heidelberg.osm.pbf") {
            _add_for_all_profiles(requests, 49.419547, 8.674956, 49.415857, 8.690261, geo_position_to_node);
            _add_for_all_profiles(requests, 49.423458, 8.686463, 49.414355, 8.677912, geo_position_to_node);
            _add_for_all_profiles(requests, 49.414511, 8.690186, 49.423600, 8.682632, geo_position_to_node);
        }

        if(osm_file == "andorra.osm.pbf") {
            _add_for_all_profiles(requests, 42.558387, 1.687437, 42.632901, 1.490997, geo_position_to_node);
            _add_for_all_profiles(requests, 42.572491, 1.471320, 42.551935, 1.454219, geo_position_to_node);
        }
    }

    RoutingResult run_dijkstra(RoutingRequest request){
        RoutingResult result;

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

        auto path = dij.get_arc_path_to(request.to_node);

        long long end_time_dij = get_micro_time();

        result.time_taken = end_time_dij - start_time_dij;
        result.total_weight = dij.get_distance_to(request.to_node);

        for(unsigned arc : path){
            ResultArc result_arc;
            result_arc.from_latitude = graph.latitude[tail[arc]];
            result_arc.from_longitude = graph.longitude[tail[arc]];
            result_arc.to_latitude = graph.latitude[graph.head[arc]];
            result_arc.to_longitude = graph.longitude[graph.head[arc]];
            result_arc.weight = graph.geo_distance[arc];
            result_arc.label = graph.label[arc];
            result.path.push_back(result_arc);
        }

        return result;
    }

    void build_ch() {
        ch = ContractionHierarchy::build(graph.node_count(), tail, graph.head, graph.geo_distance, graph.label);
    }

    RoutingResult run_chlr(RoutingRequest request){
        RoutingResult result;

        CHLRQuery query(chlr.graph);
        query.set(request.from_node, request.to_node, request.profile);
        long long start_time_ch = get_micro_time();
        query.run();
        auto path = query.get_arc_path();
        long long end_time_ch = get_micro_time();

        result.time_taken = end_time_ch - start_time_ch;
        result.total_weight = 0;
        unsigned previous_node = request.from_node;

        for(const auto& arc : path){
            ResultArc result_arc;
            result_arc.from_latitude = chlr.graph.nodes[previous_node].lat;
            result_arc.from_longitude = chlr.graph.nodes[previous_node].lon;
            result_arc.to_latitude = chlr.graph.nodes[arc.other_node].lat;
            result_arc.to_longitude = chlr.graph.nodes[arc.other_node].lon;
            result_arc.weight = arc.weight;
            result_arc.label = arc.label;
            result.path.push_back(result_arc);

            result.total_weight += arc.weight;
            previous_node = arc.other_node;
        }

        return result;
    }

    RoutingResult run_ch(RoutingRequest request) {
        RoutingResult result;

        ContractionHierarchyQuery ch_query(ch);
        ch_query.reset()
            .add_source(request.from_node)
            .set_profile(request.profile)
            .add_target(request.to_node);

        long long start_time_ch = get_micro_time();
        ch_query.run();
        auto path = ch_query.get_arc_path();
        long long end_time_ch = get_micro_time();
        
        result.time_taken = end_time_ch - start_time_ch;
        result.total_weight = 0;

        for(const auto& arc : path) {
            ResultArc result_arc;
            result_arc.from_latitude = graph.latitude[tail[arc]];
            result_arc.from_longitude = graph.longitude[tail[arc]];
            result_arc.to_latitude = graph.latitude[graph.head[arc]];
            result_arc.to_longitude = graph.longitude[graph.head[arc]];
            result_arc.weight = graph.geo_distance[arc];
            result_arc.label = graph.label[arc];
            result.path.push_back(result_arc);
            
            result.total_weight += result_arc.weight;
        }

        return result;
    }

    void assert_same_path_length(const RoutingResult& ch_result, const RoutingResult& dij_result) {
        ASSERT_TRUE((ch_result.total_weight == dij_result.total_weight) || (dij_result.total_weight == inf_weight && ch_result.total_weight == 0))
            << "Path length mismatch: CH result total weight = " << ch_result.total_weight
            << ", Dijkstra result total weight = " << dij_result.total_weight;
    }

    void assert_ch_faster_than_dijkstra(const RoutingResult& ch_result, const RoutingResult& dij_result) {
        ASSERT_LE(ch_result.time_taken, dij_result.time_taken)
            << "CH query took longer than Dijkstra: CH time = " << ch_result.time_taken
            << ", Dijkstra time = " << dij_result.time_taken;
    }

    void assert_same_path(const RoutingResult& ch_result, const RoutingResult& dij_result) {
        ASSERT_EQ(ch_result.path.size(), dij_result.path.size())
            << "Path size mismatch: CH result path size = " << ch_result.path.size()
            << ", Dijkstra result path size = " << dij_result.path.size();

        for(size_t i = 0; i < ch_result.path.size(); ++i) {
            const auto& ch_arc = ch_result.path[i];
            const auto& dij_arc = dij_result.path[i];

            ASSERT_EQ(ch_arc.from_latitude, dij_arc.from_latitude)
                << "Latitude mismatch at arc " << i;
            ASSERT_EQ(ch_arc.from_longitude, dij_arc.from_longitude)
                << "Longitude mismatch at arc " << i;
            ASSERT_EQ(ch_arc.to_latitude, dij_arc.to_latitude)
                << "To latitude mismatch at arc " << i;
            ASSERT_EQ(ch_arc.to_longitude, dij_arc.to_longitude)
                << "To longitude mismatch at arc " << i;
            ASSERT_EQ(ch_arc.weight, dij_arc.weight)
                << "Weight mismatch at arc " << i;
            ASSERT_EQ(ch_arc.label.get_label(), dij_arc.label.get_label())
                << "Label mismatch at arc " << i;
        }
    }

    void assert_all(const RoutingResult& ch_result, const RoutingResult& dij_result) {
        assert_same_path_length(ch_result, dij_result);
        //assert_ch_faster_than_dijkstra(ch_result, dij_result);
        //assert_same_path(ch_result, dij_result);
    }

    void print_path(RoutingRequest& request, RoutingResult& result) {
        std::cout << request.from_latitude << " " << request.from_longitude << " "
                  << request.to_latitude << " " << request.to_longitude << std::endl;
        for(const auto& arc : result.path)
            std::cout << arc.to_latitude << " " << arc.to_longitude << " " << human_readable_label(arc.label) << std::endl;
    }
};



#endif // ROUTINGKIT_TEST_BUILDER_H
