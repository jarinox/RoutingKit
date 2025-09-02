#include <routingkit/test_builder.h>
#include <routingkit/chm.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

CHLRGraph synthetic(unsigned node_count, bool build = true, unsigned seed = 42) {
    CHLRGraph graph;
    graph.nodes.resize(node_count);

    for (unsigned i = 0; i < node_count; ++i) {
        for (int j = 0; j < rand_r(&seed) % 5 + 1; ++j) { // Random number of edges per node
            unsigned target = rand_r(&seed) % node_count;
            if (target != i) {
                graph.add_arc(i, invalid_id, target, rand_r(&seed) % 20 + 1, Label());
            }
        }
    }

    if(!build) return graph;

    CHLR chlr = CHLR(graph);
    chlr.build();

    return chlr.graph;
}

CHLRGraph circular(unsigned node_count, bool build = true) {
    CHLRGraph graph;
    graph.nodes.resize(node_count);

    for (unsigned i = 0; i < node_count; ++i) {
        graph.add_arc(i, invalid_id, (i + 1) % node_count, 1, Label());
        graph.add_arc((i + 1) % node_count, invalid_id, i, 1, Label());
    }

    if(!build) return graph;

    CHLR chlr = CHLR(graph);
    chlr.build();

    return chlr.graph;
}

unsigned path_length(std::vector<CHLRArc> arcs) {
    unsigned length = 0;
    for (const auto& arc : arcs) {
        length += arc.weight;
    }
    if(length == 0) {
        return inf_weight;
    }

    return length;
}

TEST(CHM, test_convert_chm_chlr) {
    CHLRGraph original = synthetic(500);

    CHMGraph to_chm = CHMGraph(original);
    CHLRGraph to_chlr = to_chm.to_chlr();

    EXPECT_EQ(original.nodes.size(), to_chm.nodes.size());
    EXPECT_EQ(original.nodes.size(), to_chlr.nodes.size());
    EXPECT_NE(original.nodes.size(), 0u);

    for (unsigned i = 0; i < original.nodes.size(); ++i) {
        EXPECT_EQ(original.nodes[i].rank, to_chm.nodes[i].rank);
        EXPECT_EQ(original.nodes[i].lat, to_chm.nodes[i].lat);
        EXPECT_EQ(original.nodes[i].lon, to_chm.nodes[i].lon);
        EXPECT_EQ(original.nodes[i].out_arcs.size(), to_chm.nodes[i].arcs.size());
        EXPECT_EQ(to_chm.nodes[i].node_index, i);

        EXPECT_EQ(to_chlr.nodes[i].lat, original.nodes[i].lat);
        EXPECT_EQ(to_chlr.nodes[i].lon, original.nodes[i].lon);
        EXPECT_EQ(to_chlr.nodes[i].out_arcs.size(), original.nodes[i].out_arcs.size());

        for (unsigned j = 0; j < original.nodes[i].out_arcs.size(); ++j) {
            EXPECT_EQ(to_chlr.nodes[i].out_arcs[j].label, original.nodes[i].out_arcs[j].label);
            EXPECT_EQ(to_chlr.nodes[i].out_arcs[j].weight, original.nodes[i].out_arcs[j].weight);
            EXPECT_EQ(to_chlr.nodes[i].out_arcs[j].other_node, original.nodes[i].out_arcs[j].other_node);

            EXPECT_EQ(to_chm.nodes[i].arcs[j].label, to_chlr.nodes[i].out_arcs[j].label);
            EXPECT_EQ(to_chm.nodes[i].arcs[j].weight, to_chlr.nodes[i].out_arcs[j].weight);
            EXPECT_EQ(to_chm.nodes[i].arcs[j].to, to_chlr.nodes[i].out_arcs[j].other_node);

            EXPECT_EQ(original.nodes[i].out_arcs[j].mid_node, to_chlr.nodes[i].out_arcs[j].mid_node);
            EXPECT_EQ(to_chm.nodes[i].arcs[j].mid_node, to_chlr.nodes[i].out_arcs[j].mid_node);
            EXPECT_EQ(to_chm.nodes[i].arcs[j].arc_index, j);
            EXPECT_EQ(to_chm.nodes[i].arcs[j].from, i);
        }
    }
}

TEST(CHM, test_maintenance_on_circular_graphs) {
    for (unsigned node_count = 5; node_count < 25; ++node_count) {
        CHLRGraph chlr = circular(node_count);
        CHMGraph chm = CHMGraph(chlr);
        
        CHLRQuery q1(chlr);
        q1.set(0, 2, Label());
        q1.run();

        CHLRGraph chlr1 = chm.to_chlr();
        CHLRQuery q1b(chlr1);
        q1b.set(0, 2, Label());
        q1b.run();

        EXPECT_EQ(q1.get_arc_path().size(), q1b.get_arc_path().size());
        EXPECT_EQ(q1.get_arc_path().size(), 2);

        // Increase weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 2*node_count, Label());

        CHLRGraph chlr2 = chm.to_chlr();
        CHLRQuery q2(chlr2);
        q2.set(0, 2, Label());
        q2.run();

        EXPECT_EQ(q2.get_arc_path().size(), node_count - 2);

        // Decrease weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label());

        CHLRGraph chlr3 = chm.to_chlr();
        CHLRQuery q3(chlr3);
        q3.set(0, 2, Label());
        q3.run();

        EXPECT_EQ(q3.get_arc_path().size(), 2);

        // Update label
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label(1));

        CHLRGraph chlr4 = chm.to_chlr();
        CHLRQuery q4(chlr4);
        q4.set(0, 2, Label());
        q4.run();

        EXPECT_EQ(q4.get_arc_path().size(), node_count - 2);

        // Restore original arc
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label());

        CHLRGraph chlr5 = chm.to_chlr();
        CHLRQuery q5(chlr5);
        q5.set(0, 2, Label());
        q5.run();

        EXPECT_EQ(q5.get_arc_path().size(), 2);

        // Change label and weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), node_count*2, Label(1));

        CHLRGraph chlr6 = chm.to_chlr();
        CHLRQuery q6(chlr6);
        q6.set(0, 2, Label());
        q6.run();

        EXPECT_EQ(q6.get_arc_path().size(), node_count - 2);
    }
}

TEST(CHM, test_maintenance_on_synthetic_graph) {
    unsigned correct = 0;
    unsigned wrong = 0;
    unsigned mismatches = 0;
    unsigned accepted = 0;

    for (unsigned run = 0; run < 10; ++run) {
        std::cout << "Running synthetic test " << run << std::endl;
        CHLRGraph chlr = synthetic(200, true, run);
        CHMGraph chm = CHMGraph(chlr);

        std::vector<unsigned> original_distances;

        // Prechange
        for(unsigned query = 50; query < 100; ++query) {
            CHLRQuery q(chlr);
            q.set(0, query, Label());
            q.run();

            unsigned dij = chm.dijkstra(0, query, Label());
            EXPECT_EQ(dij, path_length(q.get_arc_path()));
            original_distances.push_back(dij);
        }

        // Apply random changes
        for(unsigned i = 0; i < 40; ++i) {
            unsigned from = rand() % 200;
            if(chm.nodes[from].arcs.empty()) continue;
            unsigned arc = rand() % chm.nodes[from].arcs.size();
            unsigned weight = rand() % 1000 + 1;
            Label label(rand() % 5);

            chm.maintenance(chm.nodes[from].arcs[arc].get_pos(), weight, label);
        }

        // Postchange
        CHLRGraph chlr_changed = chm.to_chlr();
        for(unsigned query = 50; query < 100; ++query) {
            CHLRQuery q(chlr_changed);
            q.set(0, query, Label());
            q.run();

            unsigned dij = chm.dijkstra(0, query, Label());
            unsigned chlr_len = path_length(q.get_arc_path());
            EXPECT_EQ(dij, chlr_len);

            if(dij != original_distances[query - 50]){
                if(dij == chlr_len) {
                    correct++;
                } else {
                    wrong++;
                }
            } else {
               if(dij != chlr_len) {
                    mismatches++;
                } else {
                    accepted++;
                }
            }
        }
    }

    std::cout << "Correct: " << correct << " Wrong: " << wrong << " Mismatches: " << mismatches << " Accepted: " << accepted << std::endl;

}
