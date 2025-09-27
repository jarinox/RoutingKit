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
        std::vector<CHMArc> targets;
        for (unsigned j = 0; j < rand_r(&seed) % 5 + 1; ++j) { // Random number of edges per node
            unsigned target = rand_r(&seed) % node_count;
            if (target == i) continue; // No self-loops

            Label label = Label(rand_r(&seed) % 8);
            unsigned new_weight = rand_r(&seed) % 20 + 5;
            CHMArc arc(i, invalid_id, target, new_weight, label);

            bool skip = false;
            for(const auto& existing : targets) {
                if(existing.to != arc.to) continue;
                if(existing.label.is_subset_of(arc.label) && existing.weight <= arc.weight) {
                    skip = true;
                    break;
                }

                if(arc.label.is_subset_of(existing.label) && arc.weight <= existing.weight) {
                    skip = true;
                    break;
                }
            }

            if(!skip) {
                targets.push_back(arc);
                graph.add_arc(i, invalid_id, target, new_weight, label);
            }

        }

        if(targets.size() > 1) {
            std::sort(targets.begin(), targets.end(), [](const CHMArc& a, const CHMArc& b) {
                return a.weight != b.weight ? a.weight < b.weight : !a.label.is_superset_of(b.label);
            });
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

std::string readable_label(Label label) {
    std::string s = "";
    for(unsigned i = 0; i < 16; ++i) {
        if(label.get_bit(i)) {
            char bit = 'a' + i;
            s = s + bit;
        }
    }
    return s;
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
            file << arc.from << " -> " << arc.to << " (" << arc.weight << "," << arc.mid_node << ",\"" << readable_label(arc.label) << "\")" << std::endl;
        }
    }

    file.close();
}

void test_dch_vs_dijkstra(DCHGraph& dch, unsigned seed) {
    CHLRGraph chlr = dch.to_chlr();
    unsigned node_count = chlr.nodes.size();

    for(unsigned query = 0; query < min(30u, node_count); query += 2) {
        Label label = Label(rand_r(&seed) % 8);
        CHLRQuery q(chlr);
        q.set(0, query, label);
        q.run();

        auto [dij, dij_path] = dch.dijkstra(0, query, label);
        auto chlr_path = q.get_arc_path();
        unsigned chlr_len = path_length(chlr_path);
        
        EXPECT_EQ(dij, chlr_len);

        #ifdef DEBUG
        if(dij != chlr_len) {
            CHLRQuery q2(chlr);
            q2.set(0, query, label);
            q2.run();

            auto [dij, dij_path] = dch.dijkstra(0, query, label);
            auto chlr_path = q2.get_arc_path();
            unsigned chlr_len = path_length(chlr_path);
        }
        #endif
    }
}

TEST(CHM, reduce_weight) {
    return;
    unsigned runs = 10000;
    unsigned seed = 42;
    unsigned node_count = 12;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCH- test " << run << std::endl;

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);
        DCHGraph dch = DCHGraph(chlr);

        // Prechange check
        test_dch_vs_dijkstra(dch, seed);

        std::vector<std::pair<CHMArc, CHMArc>> changes;
        print_graph_to_file(dch, "generated/debug_graph_before.txt");

        // Apply random changes
        for (unsigned i = 0; i < node_count; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            unsigned weight = rand_r(&seed) % (old_weight - 1) + 1; // decrease weight

            CHMArc before = dch.nodes[from].arcs[arc];
            dch.DCHAlt(dch.nodes[from].arcs[arc].get_pos(), weight);
            CHMArc after = dch.nodes[from].arcs[arc];

            changes.push_back({before, after});
        }

        print_graph_to_file(dch, "generated/debug_graph_after.txt");

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);
    }
}

TEST(CHM, increase_weight) {
    return;
    unsigned runs = 20000;
    unsigned seed = 42;
    unsigned node_count = 9;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCH+ test " << run << std::endl;

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);
        DCHGraph dch = DCHGraph(chlr);

        // Prechange check
        test_dch_vs_dijkstra(dch, seed);

        std::vector<std::pair<CHMArc, CHMArc>> changes;

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_before.txt");
        #endif

        // Apply random changes
        for (unsigned i = 0; i < node_count; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            unsigned weight = rand_r(&seed) % 20 + old_weight + 1; // increase weight

            CHMArc before = dch.nodes[from].arcs[arc];
            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), weight);
            CHMArc after = dch.nodes[from].arcs[arc];
            assert(after.weight >= before.weight);
            assert(after.weight == weight);

            changes.push_back({before, after});
        }

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_after.txt");
        #endif

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);
    }
}

TEST(CHM, remove_labels) {
    unsigned runs = 10000;
    unsigned seed = 42;
    unsigned node_count = 5;

    unsigned ssc = 0;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCHLabel- test " << run << std::endl;

        if(run == 2049) {
            std::cout << "Debug run" << std::endl;
        }

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);

        #ifdef DEBUG_INFO
        unsigned original_arcs = 0;
        unsigned shortcut_arcs = 0;
        for(const auto& node : chlr.nodes) {
            for (const auto& arc : node.out_arcs) {
                if(arc.is_shortcut()) shortcut_arcs++;
                else original_arcs++;
            }
        }
        if(shortcut_arcs > ssc) ssc = shortcut_arcs;
        std::cout << "Original arcs: " << original_arcs << " Shortcut arcs: " << shortcut_arcs << " max: " << ssc << std::endl;
        #endif

        DCHGraph dch = DCHGraph(chlr);

        // Prechange check
        test_dch_vs_dijkstra(dch, seed);

        std::vector<std::pair<CHMArc, CHMArc>> changes;

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_before.txt");
        #endif

        // Apply random changes
        for (unsigned i = 0; i < 1; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            Label old_label = dch.nodes[from].arcs[arc].label;
            Label new_label = old_label;
            
            // Remove one label
            for(unsigned b = 0; b < 16; ++b) {
                if(new_label.get_bit(b)) {
                    new_label.set_bit(false, b);
                    break;
                }
            }

            CHMArc before = dch.nodes[from].arcs[arc];

            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            dch.DCHMinus(dch.nodes[from].arcs[arc].get_pos(), before.weight);

            CHMArc after = dch.nodes[from].arcs[arc];
            EXPECT_TRUE(after.label.is_subset_of(before.label));
            EXPECT_EQ(after.weight, before.weight);
            EXPECT_EQ(after.label.get_label(), new_label.get_label());

            changes.push_back({before, after});
        }

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_after.txt");
        #endif

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);
    }

    std::cout << "Max shortcut arcs in any graph: " << ssc << std::endl;
}

TEST(CHM, add_labels) {
    return;
    unsigned runs = 10000;
    unsigned seed = 42;
    unsigned node_count = 9;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCHLabel+ test " << run << std::endl;

        if(run == 27) {
            std::cout << "Debug run" << std::endl;
        }

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);
        DCHGraph dch = DCHGraph(chlr);

        // Prechange check
        test_dch_vs_dijkstra(dch, seed);

        std::vector<std::pair<CHMArc, CHMArc>> changes;

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_before.txt");
        #endif

        // Apply random changes
        for (unsigned i = 0; i < node_count; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            Label old_label = dch.nodes[from].arcs[arc].label;
            Label new_label = old_label;
            
            // Add one label
            for(unsigned b = 0; b < 16; ++b) {
                if(!new_label.get_bit(b)) {
                    new_label.set_bit(true, b);
                    break;
                }
            }

            CHMArc before = dch.nodes[from].arcs[arc];
            
            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            dch.DCHMinus(dch.nodes[from].arcs[arc].get_pos(), before.weight);

            CHMArc after = dch.nodes[from].arcs[arc];
            EXPECT_TRUE(after.label.is_superset_of(before.label));
            EXPECT_EQ(after.weight, before.weight);
            EXPECT_EQ(after.label.get_label(), new_label.get_label());

            changes.push_back({before, after});
        }

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_after.txt");
        #endif

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);
    }
}
