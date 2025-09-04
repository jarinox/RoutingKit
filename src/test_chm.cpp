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
                unsigned new_weight = rand_r(&seed) % 20 + 1;
                assert(new_weight > 0);
                assert(new_weight < 22);
                graph.add_arc(i, invalid_id, target, new_weight, Label());
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

void print_graph_to_file(CHMGraph& graph) {
    std::ofstream file("debug_graph.txt");
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing" << std::endl;
        return;
    }

    file << graph.nodes.size() << " nodes" << std::endl;
    for (const auto& node : graph.nodes) {
        for (const auto& arc : node.arcs) {
            file << arc.from << " -> " << arc.to << " (" << arc.weight << "," << arc.mid_node << ")" << std::endl;
        }
    }

    file.close();
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

TEST(CHM, test_maintenance_paper_example) {
    CHMGraph graph;
    Label a, b, c;
    a.set_bit(true, 0);
    b.set_bit(true, 1);
    c.set_bit(true, 2);

    Label ab = a.unite(b);
    Label bc = b.unite(c);
    Label abc = ab.unite(c);

    {
        graph.nodes.resize(12);
        graph.nodes[0].rank = 6;
        graph.nodes[1].rank = 3;
        graph.nodes[2].rank = 2;
        graph.nodes[3].rank = 4;
        graph.nodes[4].rank = 8;
        graph.nodes[5].rank = 11;
        graph.nodes[6].rank = 9;
        graph.nodes[7].rank = 7;
        graph.nodes[8].rank = 10;
        graph.nodes[9].rank = 5;
        graph.nodes[10].rank = 12;
        graph.nodes[11].rank = 1;

        graph.add_arc(0, invalid_id, 2, 2, a);
        graph.add_arc(0, invalid_id, 8, 5, b);
        graph.add_arc(1, invalid_id, 7, 2, a);
        graph.add_arc(2, invalid_id, 8, 2, bc);
        graph.add_arc(3, invalid_id, 5, 5, b);
        graph.add_arc(4, invalid_id, 0, 6, b);
        graph.add_arc(4, invalid_id, 3, 2, b);
        graph.add_arc(4, invalid_id, 11, 3, b);
        graph.add_arc(6, invalid_id, 3, 5, a);
        graph.add_arc(6, invalid_id, 5, 7, ab);
        graph.add_arc(7, invalid_id, 8, 2, a);
        graph.add_arc(9, invalid_id, 8, 5, b);
        graph.add_arc(10, invalid_id, 9, 2, ab);
        graph.add_arc(10, invalid_id, 6, 4, b);
        graph.add_arc(10, invalid_id, 1, 3, b);
        graph.add_arc(11, invalid_id, 0, 3, b);
        graph.add_arc(11, invalid_id, 5, 4, b);

        graph.add_arc(0, 2, 8, 4, abc);
        graph.add_arc(4, 0, 8, 11, b);
        graph.add_arc(4, 0, 8, 10, abc);
        graph.add_arc(4, 11, 5, 7, b);
        graph.add_arc(10, 6, 5, 11, ab);
        graph.add_arc(10, 1, 7, 5, ab);
        graph.add_arc(10, 9, 8, 7, ab);
    }

    graph.maintenance(graph.nodes[3].arcs[0].get_pos(), 7, c);

    EXPECT_EQ(graph.w(3, 5, c), 7);
}

TEST(CHM, test_maintenance_on_circular_graphs) {
    for (unsigned node_count = 5; node_count < 100; ++node_count) {
        CHLRGraph chlr = circular(node_count);
        CHMGraph chm = CHMGraph(chlr);
        
        CHLRQuery q1(chlr);
        q1.set(0, 2, Label(1));
        q1.run();

        CHLRGraph chlr1 = chm.to_chlr();
        CHLRQuery q1b(chlr1);
        q1b.set(0, 2, Label(1));
        q1b.run();

        EXPECT_EQ(q1.get_arc_path().size(), q1b.get_arc_path().size());
        EXPECT_EQ(q1.get_arc_path().size(), 2);

        // Increase weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 2*node_count, Label());

        CHLRGraph chlr2 = chm.to_chlr();
        CHLRQuery q2(chlr2);
        q2.set(0, 2, Label(1));
        q2.run();

        auto path = q2.get_arc_path();
        EXPECT_EQ(path.size(), node_count - 2);

        // Decrease weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label());

        CHLRGraph chlr3 = chm.to_chlr();
        CHLRQuery q3(chlr3);
        q3.set(0, 2, Label(1));
        q3.run();

        EXPECT_EQ(q3.get_arc_path().size(), 2);

        // Update label
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label(1));

        CHLRGraph chlr4 = chm.to_chlr();
        CHLRQuery q4(chlr4);
        q4.set(0, 2, Label(1));
        q4.run();

        path = q4.get_arc_path();

        EXPECT_EQ(path.size(), node_count - 2);

        // Restore original arc
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), 1, Label());

        CHLRGraph chlr5 = chm.to_chlr();
        CHLRQuery q5(chlr5);
        q5.set(0, 2, Label(1));
        q5.run();

        EXPECT_EQ(q5.get_arc_path().size(), 2);

        // Change label and weight
        chm.maintenance(chm.nodes[0].arcs[0].get_pos(), node_count*2, Label(1));

        CHLRGraph chlr6 = chm.to_chlr();
        CHLRQuery q6(chlr6);
        q6.set(0, 2, Label(1));
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
        unsigned node_cnt = 150;
        CHLRGraph chlr = synthetic(node_cnt, true, run);
        CHMGraph chm = CHMGraph(chlr);

        std::vector<unsigned> original_distances;

        // Prechange
        for(unsigned query = 50; query < 120; query += 2) {
            CHLRQuery q(chlr);
            q.set(0, query, Label(3));
            q.run();

            unsigned dij = chm.dijkstra(0, query, Label());
            EXPECT_EQ(dij, path_length(q.get_arc_path()));
            original_distances.push_back(dij);
        }

        // Apply random changes
        for(unsigned i = 0; i < 50; ++i) {
            unsigned from = rand() % node_cnt;
            if(chm.nodes[from].arcs.empty()) continue;
            unsigned arc = rand() % chm.nodes[from].arcs.size();
            unsigned weight = rand() % 1000 + 1;
            Label label(rand() % 5);

            chm.maintenance(chm.nodes[from].arcs[arc].get_pos(), weight, label);
        }

        // Postchange
        CHLRGraph chlr_changed = chm.to_chlr();
        for(unsigned query = 50; query < 120; query += 2) {
            CHLRQuery q(chlr_changed);
            q.set(0, query, Label(3));
            q.run();

            unsigned dij = chm.dijkstra(0, query, Label());
            unsigned chlr_len = path_length(q.get_arc_path());
            if(chlr_len > inf_weight / 2 && dij < inf_weight / 2) {
                std::cout << "Weird flip detected: CHLR length = " << chlr_len << ", Dijkstra length = " << dij << std::endl;
                std::cout << q.get_arc_path().size() << std::endl;
            }
            EXPECT_EQ(dij, chlr_len);

            if(dij != original_distances[(query / 2) - 25]){
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
        std::cout << "Correct: " << correct << " Wrong: " << wrong << " Mismatches: " << mismatches << " Accepted: " << accepted << std::endl;
    }

    std::cout << "===> Correct: " << correct << " Wrong: " << wrong << " Mismatches: " << mismatches << " Accepted: " << accepted << std::endl;

}
