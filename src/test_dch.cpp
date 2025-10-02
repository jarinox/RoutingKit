#include <routingkit/test_builder.h>
#include <routingkit/dch.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>

//#define DEBUG

using namespace RoutingKit;
using namespace std;

TEST(DCHGraph, garbage_collection) {
    unsigned graph_size = 5;
    DCHGraph graph = DCHGraph();
    graph.nodes.resize(graph_size);
    graph.garbage.resize(graph_size);
    graph.backward_garbage.resize(graph_size);

    for(int i = 0; i < graph_size; ++i) {
        graph.add_arc(i, invalid_id, (i + 1) % graph_size, i, Label());
        graph.add_arc((i + 1) % graph_size, invalid_id, i, i, Label());
    }

    for(const auto& node : graph.nodes) {
        EXPECT_EQ(node.arcs.size(), 2);
        EXPECT_EQ(node.in_arcs.size(), 2);

        for(const auto& arc : node.arcs) {
            auto twin = graph.nodes[arc.twin.node_index].in_arcs[arc.twin.arc_index];
            EXPECT_EQ(twin.twin.arc_index, arc.arc_index);
            EXPECT_EQ(twin.twin.node_index, arc.from);
            EXPECT_EQ(twin.from, arc.from);
            EXPECT_EQ(twin.to, arc.to);
        }
    }

    graph.invalidate(graph.nodes[1].arcs[1]);
    EXPECT_EQ(graph.garbage[1].size(), 1);
    EXPECT_EQ(graph.backward_garbage[2].size(), 1);
    
    graph.invalidate(graph.nodes[2].arcs[1]);
    EXPECT_EQ(graph.backward_garbage[3].size(), 1);

    graph.add_arc(1, invalid_id, 3, 100, Label());

    EXPECT_EQ(graph.garbage[1].size(), 0);
    EXPECT_EQ(graph.backward_garbage[3].size(), 0);

    EXPECT_EQ(graph.nodes[1].arcs[1].weight, 100);
    EXPECT_EQ(graph.nodes[3].in_arcs[0].weight, 100);
}

CHLRGraph synthetic_realistic(unsigned node_count, bool build = true, unsigned seed = 42) {
    CHLRGraph graph;
    graph.nodes.resize(node_count);

    std::vector<Label> possible_labels = {
        Label(0b0000),
        Label(0b0000),
        Label(0b0001),
        Label(0b0010),
        Label(0b0100),
    };

    std::pair<unsigned, unsigned> range_of_range = {1, 3};

    // add arcs to the n previous and m next nodes
    for(unsigned i = 0; i < node_count; ++i) {
        unsigned n = rand_r(&seed) % (range_of_range.second - range_of_range.first + 1) + range_of_range.first;
        unsigned m = rand_r(&seed) % (range_of_range.second - range_of_range.first + 1) + range_of_range.first;

        for(unsigned j = 1; j <= n; ++j) {
            unsigned target = (i + node_count - j) % node_count;
            Label label = possible_labels[rand_r(&seed) % possible_labels.size()];
            unsigned new_weight = rand_r(&seed) % 50 + 5;
            graph.add_arc(i, invalid_id, target, new_weight, label);
        }

        for(unsigned j = 1; j <= m; ++j) {
            unsigned target = (i + j) % node_count;
            Label label = possible_labels[rand_r(&seed) % possible_labels.size()];
            unsigned new_weight = rand_r(&seed) % 50 + 5;
            graph.add_arc(i, invalid_id, target, new_weight, label);
        }
    }

    // add random arcs to roughly 1% of the nodes
    unsigned extra_arcs = 4 + (rand_r(&seed) % ((node_count / 100) + 1));

    for(unsigned i = 0; i < extra_arcs; ++i) {
        unsigned from = rand_r(&seed) % node_count;
        unsigned to = rand_r(&seed) % node_count;
        if(from == to) continue;

        Label label = possible_labels[rand_r(&seed) % possible_labels.size()];
        unsigned new_weight = rand_r(&seed) % 50 + 5;
        graph.add_arc(from, invalid_id, to, new_weight, label);
    }

    for(unsigned i = 0; i < node_count; ++i) {
        graph.nodes[i].sort_arcs_for_weight();
    }

    if(!build) return graph;

    CHLR chlr = CHLR(graph);

    long long before = get_micro_time();
    chlr.build();
    long long after = get_micro_time();
    std::cout << chlr.graph.nodes.size() << " nodes in " << (after - before) << " microseconds" << std::endl;

    return chlr.graph;
}

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

    for(unsigned query = 0; query < min(40u, node_count); query += 2) {
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
    return;
    unsigned runs = 10000;
    unsigned seed = 42;
    unsigned node_count = 12;

    unsigned ssc = 0;

    for (unsigned run = 0; run < runs; ++run) {
        std::cout << "Running DCHLabel- test " << run << std::endl;

        if(run == 148) {
            std::cout << "Debug run" << std::endl;
        }

        CHLRGraph chlr = synthetic(node_count, true, run*seed+1);
        std::cout << "Graph has " << chlr.nodes.size() << " nodes." << std::endl;

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

        std::cout << "Precheck done." << std::endl;

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
            
            // Remove one label
            for(unsigned b = 0; b < 16; ++b) {
                if(new_label.get_bit(b)) {
                    new_label.set_bit(false, b);
                    break;
                }
            }

            CHMArc before = dch.nodes[from].arcs[arc];
            if(before.weight == inf_weight) continue;

            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            CHMArc new_arc = dch.add_arc(dch.nodes[from].arcs[arc]);

            dch.DCHMinus(new_arc.get_pos(), before.weight);

            CHMArc after = dch.ref(new_arc);
            EXPECT_TRUE(after.label.is_subset_of(before.label));
            EXPECT_EQ(after.weight, before.weight);
            EXPECT_EQ(after.label.get_label(), new_label.get_label());

            changes.push_back({before, after});
        }

        std::cout << "Changes applied" << std::endl;

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_after.txt");
        #endif

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);
    }

    std::cout << "Max shortcut arcs in any graph: " << ssc << std::endl;
}

TEST(CHM, add_labels) {
    unsigned runs = 10000;
    unsigned seed = 42;
    unsigned node_count = 7;

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
            if(before.weight == inf_weight) continue;
            
            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            CHMArc new_arc = dch.add_arc(dch.nodes[from].arcs[arc]);
            dch.DCHMinus(new_arc.get_pos(), before.weight);

            CHMArc after = dch.ref(new_arc);
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

TEST(CHM, dch_on_real_road_network) {
    return;
    unsigned seed = 42;
    std::vector<std::string> osm_files = {
        //"heidelberg.osm.pbf",
        //"ma_min_messplatz.osm.pbf",
        //"rippo.osm.pbf",
        //"hd_west.osm.pbf",
        "hd_neuenheim.osm.pbf",
    };

    Label profiles[4] = {
        Label(0b000),
        Label(0b001),
        Label(0b010),
        Label(0b100),
    };

    for (const auto& osm_file : osm_files) {
        TestSetup setup = TestSetup(osm_file);
        unsigned node_count = setup.chlr.graph.nodes.size();
        std::cout << "Graph has " << node_count << " nodes." << std::endl;

        std::cout << "Building CH for " << osm_file << std::endl;
        setup.chlr.build();
        
        DCHGraph dch = DCHGraph(setup.chlr.graph);

        std::cout << "Apply random changes" << std::endl;

        // Apply random changes
        for (unsigned i = 0; i < node_count; i = i + 10) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;  

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            unsigned weight = rand_r(&seed) % 600 + 1; // random new weight

            if(weight < old_weight) {
                dch.DCHMinus(dch.nodes[from].arcs[arc].get_pos(), weight);
            } else if(weight > old_weight) {
                dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), weight);
            }
        }

        for (unsigned i = 1; i < node_count; i = i + 10) {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) continue;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) continue;
            if(dch.nodes[from].arcs[arc].weight < 2) continue;  

            Label new_label = profiles[rand_r(&seed) % 4];
            if(new_label.get_label() == dch.nodes[from].arcs[arc].label.get_label()) continue;

            CHMArc before = dch.nodes[from].arcs[arc];
            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            CHMArc new_arc = dch.add_arc(dch.nodes[from].arcs[arc]);
            dch.DCHMinus(new_arc.get_pos(), before.weight);
        }

        std::cout << "Running requests..." << std::endl;
        for(unsigned i = 0; i < 100; ++i) {
            unsigned from = rand_r(&seed) % node_count;
            unsigned to = rand_r(&seed) % node_count;
            if(from == to) continue;
            
            Label profile = profiles[rand_r(&seed) % 4];
            CHLRGraph chlr = dch.to_chlr();
            CHLRQuery q(chlr);
            q.set(from, to, profile);
            q.run();

            auto [dij, dij_path] = dch.dijkstra(from, to, profile);
            auto chlr_path = q.get_arc_path();
            unsigned chlr_len = path_length(chlr_path);

            EXPECT_EQ(dij, chlr_len);
        }
    }
}

TEST(CHM, benchmarking) {
    return;
    unsigned runs = 990;
    unsigned seed = 42;
    unsigned node_count = 10;

    std::vector<Label> possible_labels = {
        Label(0b0000),
        Label(0b0000),
        Label(0b0001),
        Label(0b0010),
        Label(0b0100),
    };

    for (unsigned run = 0; run < runs; ++run) {
        CHLRGraph chlr = synthetic_realistic(node_count, true, run*seed+1);
        DCHGraph dch = DCHGraph(chlr);

        unsigned eo = 0;
        unsigned shortcuts = 0;

        for(const auto& node : dch.nodes) {
            for (const auto& arc : node.arcs) {
                if(arc.is_shortcut()) shortcuts++;
                else eo++;
            }
        }

        std::cout << eo << " edges and " << shortcuts << " shortcuts" << std::endl;

        // Prechange check
        test_dch_vs_dijkstra(dch, seed);

        std::vector<std::pair<CHMArc, CHMArc>> changes;

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_before.txt");
        #endif

        auto update_fn = [&]() {
            unsigned from = rand_r(&seed) % node_count;
            if(dch.nodes[from].arcs.empty()) return false;
            unsigned arc = rand_r(&seed) % dch.nodes[from].arcs.size();
            if(dch.nodes[from].arcs[arc].mid_node != invalid_id) return false;
            if(dch.nodes[from].arcs[arc].weight < 2) return false;

            unsigned old_weight = dch.nodes[from].arcs[arc].weight;
            Label old_label = dch.nodes[from].arcs[arc].label;
            Label new_label = old_label;
            
            while(new_label == old_label) {
                new_label = possible_labels[rand_r(&seed) % possible_labels.size()];
            }

            CHMArc before = dch.nodes[from].arcs[arc];
            if(before.weight == inf_weight) return false;
            
            dch.DCHPlus(dch.nodes[from].arcs[arc].get_pos(), inf_weight);
            dch.nodes[from].arcs[arc].label = new_label;
            CHMArc new_arc = dch.add_arc(dch.nodes[from].arcs[arc]);
            dch.DCHMinus(new_arc.get_pos(), before.weight);

            CHMArc after = dch.ref(new_arc);
            EXPECT_EQ(after.weight, before.weight);

            return true;
        };

        for (unsigned mode = 0; mode < 6; ++mode) {
            // Apply random changes

            unsigned changes = 1;
            if(mode == 1) changes = 1 + (rand_r(&seed) % ((node_count / 100) + 1)); // 1% => 1%
            if(mode == 2) changes = 1 + (rand_r(&seed) % ((node_count / 25) + 1)); // 4% => 5%
            if(mode == 3) changes = 1 + (rand_r(&seed) % ((node_count / 20) + 1)); // 5% => 10%
            if(mode == 4) changes = 1 + (rand_r(&seed) % ((node_count / 10) + 1)); // 10% => 20%
            if(mode == 5) changes = 1 + (rand_r(&seed) % ((node_count / 5) + 1)); // 20% => 40%

            long long before = get_micro_time();
            for (unsigned i = 0; i < node_count; ++i) {
                if (!update_fn()) {
                    --i; // try again
                }
            }
            long long after = get_micro_time();
            std::cout << "Mode " << mode << ": " << changes << " changes in " << (after - before) << " microseconds" << std::endl;
        }

        #ifdef DEBUG
        print_graph_to_file(dch, "generated/debug_graph_after.txt");
        #endif

        // Postchange check
        test_dch_vs_dijkstra(dch, seed);

        std::cout << "===== Run " << run << " done." << std::endl;
        node_count++;
    }
}
