#include <routingkit/test_builder.h>
#include <routingkit/chlrm.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


TEST(CHLR, ch_vs_dijkstra) {
    std::vector<std::string> osm_files = {
        "ma_min_messplatz.osm.pbf",
        "rippo.osm.pbf",
        "hd_west.osm.pbf",
        "hd_neuenheim.osm.pbf",
    };

    for (const auto& osm_file : osm_files) {
        TestSetup setup = TestSetup(osm_file);
        std::cout << "Building CH for " << osm_file << std::endl;
        setup.chlr.build();

        CHLRQuery query(setup.chlr.graph);

        CHLRGraph combined_graph;
        combined_graph.nodes.reserve(setup.chlr.graph.nodes.size());

        for(auto node : query.forward.nodes) {
            combined_graph.nodes.push_back(node);
            combined_graph.nodes.back().in_arcs.clear();
        }
        
        for(unsigned i = 0; i < query.backward.nodes.size(); ++i) {
            for(unsigned j = 0; j < query.backward.nodes[i].out_arcs.size(); ++j) {
                auto& arc = query.backward.nodes[i].out_arcs[j];

                // Reverse reversed arcs to match the forward direction
                combined_graph.nodes[arc.other_node].out_arcs.push_back(CHLRArc{
                    i, arc.mid_node, arc.weight, arc.label
                });
            }
        }

        for(auto& node : combined_graph.nodes) {
            node.sort_arcs_for_weight();
        }

        CHLRMGraph chlrm_graph(combined_graph);
    }
}

