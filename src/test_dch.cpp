#include <routingkit/test_builder.h>
#include <routingkit/dch.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#define DEBUG

using namespace RoutingKit;
using namespace std;

CHLRGraph synthetic(unsigned node_count, bool build = true, unsigned seed = 42) {
    CHLRGraph graph;
    graph.nodes.resize(node_count);

    for (unsigned i = 0; i < node_count; ++i) {
        std::set<unsigned> targets;
        for (int j = 0; j < rand_r(&seed) % 5 + 1; ++j) { // Random number of edges per node
            unsigned target = rand_r(&seed) % node_count;
            if(targets.find(target) != targets.end()) continue;
            if (target != i) {
                unsigned new_weight = rand_r(&seed) % 20 + 5;
                assert(new_weight > 4);
                assert(new_weight < 30);
                graph.add_arc(i, invalid_id, target, new_weight, Label());
                targets.insert(target);
            }
        }
    }

    for(unsigned i = 0; i < node_count; ++i) {
        graph.nodes[i].sort_arcs_for_weight();
    }

    if(!build) return graph;

    CHLR chlr = CHLR(graph);
    chlr.build();

    return chlr.graph;
}

unsigned path_length(const std::vector<CHLRArc>& path) {
    unsigned length = 0;

    if(path.empty()) return inf_weight;

    for (const auto& arc : path) {
        length += arc.weight;
    }

    return length;
}

void print_graph_to_file(DCHGraph& graph, std::string path = "debug_graph.txt") {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing" << std::endl;
        return;
    }

    file << graph.nodes.size() << " nodes" << std::endl;
    for (const auto& node : graph.nodes) {
        file << "Node " << node.node_index << " rank " << node.rank << std::endl;
    }

    file << "Edges:" << std::endl;
    for (const auto& node : graph.nodes) {
        for (const auto& arc : node.arcs) {
            file << arc.from << " -> " << arc.to << " (" << arc.weight << "," << arc.mid_node << ")" << std::endl;
        }
    }

    file.close();
}

void test_dch_vs_dijkstra(DCHGraph& dch) {
    CHLRGraph chlr = dch.to_chlr();
    unsigned node_count = chlr.nodes.size();

    for(unsigned query = 0; query < min(30u, node_count); query += 2) {
        CHLRQuery q(chlr);
        q.set(0, query, Label());
        q.run();

        auto [dij, dij_path] = dch.dijkstra(0, query, Label());
        auto chlr_path = q.get_arc_path();
        unsigned chlr_len = path_length(chlr_path);
        
        EXPECT_EQ(dij, chlr_len);

        if(dij != chlr_len) {
            CHLRQuery q2(chlr);
            q2.set(0, query, Label());
            q2.run();

            auto [dij, dij_path] = dch.dijkstra(0, query, Label());
            auto chlr_path = q2.get_arc_path();
            unsigned chlr_len = path_length(chlr_path);
        }
    }
}

TEST(CHM, reduce_weight) {
    unsigned runs = 5000;
    unsigned seed = 42;
    unsigned node_count = 12;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCH- test " << run << std::endl;

        if (run == 316) {
            std::cout << "Stop here" << std::endl;
        }

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);
        DCHGraph dch = DCHGraph(chlr);

        // Prechange check
        test_dch_vs_dijkstra(dch);

        std::vector<std::pair<CHMArc, CHMArc>> changes;
        print_graph_to_file(dch, "generated/debug_graph_before.txt");

        // Apply random changes
        for (unsigned i = 0; i < 1; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            unsigned weight = rand_r(&seed) % (old_weight - 1) + 1; // decrease weight

            CHMArc before = dch.nodes[from].arcs[arc];
            dch.DCHMinus(dch.nodes[from].arcs[arc].get_pos(), weight);
            CHMArc after = dch.nodes[from].arcs[arc];

            changes.push_back({before, after});
        }

        print_graph_to_file(dch, "generated/debug_graph_after.txt");

        // Postchange check
        test_dch_vs_dijkstra(dch);
    }
}
