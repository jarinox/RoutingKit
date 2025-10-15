#include <routingkit/test_builder.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


void run_random_queries(TestSetup& setup) {
    unsigned runs = 100;
    unsigned seed = 42;

    for (unsigned run = 0; run < runs; ++run) {
        unsigned from = rand_r(&seed) % setup.graph.node_count();
        unsigned to = rand_r(&seed) % setup.graph.node_count();
        if(from == to) continue;

        Label profile = Label(rand_r(&seed) % 4);

        RoutingRequest req;
        req.from_node = from;
        req.to_node = to;
        req.profile = profile;

        auto chlr_result = setup.run_chlr(req);
        unsigned dij_result = setup.chlr.AStar(from, to, profile);
        if(dij_result == inf_weight) dij_result = 0;

        EXPECT_EQ(dij_result, chlr_result.total_weight);
    }
}

TEST(CHLR, ch_vs_dijkstra) {
    return;
    std::vector<std::string> osm_files = {
        //"heidelberg.osm.pbf",
        "ma_min_messplatz.osm.pbf",
        "rippo.osm.pbf",
        "hd_west.osm.pbf",
        "hd_neuenheim.osm.pbf",
    };

    for (const auto& osm_file : osm_files) {
        TestSetup setup = TestSetup(osm_file);
        std::cout << "Building CH for " << osm_file << std::endl;
        setup.chlr.build();

        std::cout << "Running requests..." << std::endl;
        for(unsigned i = 0; i < setup.requests.size(); ++i) {
            RoutingRequest& req = setup.requests[i];
            Label pr = req.profile;
            std::cout << "Running request " << i + 1 << "/" << setup.requests.size() << ": " << human_readable_label(pr.invert()) << std::endl;
            auto chlr_result = setup.run_chlr(req);
            auto dij_result = setup.run_dijkstra(req);

            setup.assert_same_path_length(chlr_result, dij_result);
        }
    }
}

TEST(CHLR, rebuild_using_order) {
    unsigned seed = 42;
    std::vector<std::string> osm_files = {
        //"heidelberg.osm.pbf",
        "ma_min_messplatz.osm.pbf",
        "rippo.osm.pbf",
        "hd_west.osm.pbf",
        "hd_neuenheim.osm.pbf",
    };

    for (const auto& osm_file : osm_files) {
        TestSetup setup = TestSetup(osm_file);
        std::cout << "Building CH for " << osm_file << std::endl;

        long long before_build = get_micro_time();
        setup.chlr.build();
        long long after_build = get_micro_time();

        std::cout << "Build in " << after_build-before_build << " microseconds" << std::endl;
        run_random_queries(setup);

        auto& graph = setup.chlr.graph;

        // Apply random changes
        for (unsigned i = 0; i < graph.nodes.size(); i = i + 10) {
            unsigned node = rand_r(&seed) % graph.nodes.size();
            if(graph.nodes[node].out_arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % graph.nodes[node].out_arcs.size();
            auto& out_arc = graph.nodes[node].out_arcs[arc];
            auto& in_arc = graph.get_reverse_arc(out_arc, node);

            assert(in_arc.other_node == node);

            if(out_arc.mid_node != invalid_id) continue;

            unsigned old_weight = in_arc.weight;
            int mod = (int)old_weight + (rand_r(&seed) % 200) - 50;
            unsigned weight = (unsigned)std::max(1, mod); // random new weight
            assert(weight > 0);

            in_arc.weight = weight;
            out_arc.weight = weight;
        }

        std::cout << "Rebuilding CH with existing order..." << std::endl;
        long long before_rebuild = get_micro_time();
        setup.chlr.rebuild_with_order(nullptr, inf_weight);
        long long after_rebuild = get_micro_time();
        std::cout << "Rebuild in " << after_rebuild-before_rebuild << " microseconds" << std::endl;

        run_random_queries(setup);
    }
}
