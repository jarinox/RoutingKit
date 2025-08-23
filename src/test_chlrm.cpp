#include <routingkit/test_builder.h>
#include <routingkit/chlrm.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

TEST(CHLRM, test_min_rank_queue) {
    CHLRGraph graph;
    graph.nodes.resize(10);
    graph.add_arc(0, invalid_id, 1, 10, Label());
    graph.add_arc(0, invalid_id, 1, 20, Label());

    CHLR chlr(graph);
    chlr.build();

    CHLRMGraph g(chlr.graph);
    MinRankQueue queue(g.nodes.size()*100+64);
    
    queue.push(g.nodes[0].arcs[0], true, g);
    queue.push(g.nodes[0].arcs[1], false, g);

    CHLRMArc e = g.nodes[0].arcs[0];
    ASSERT_TRUE(queue.contains(e, true));
    queue.push(e, true, g);

    ASSERT_FALSE(queue.empty());
    auto arc_pair = queue.pop();
    ASSERT_TRUE(arc_pair.first);
    ASSERT_EQ(arc_pair.second.from, 0);
    ASSERT_EQ(arc_pair.second.to, 1);
    ASSERT_EQ(arc_pair.second.weight, 10);

    ASSERT_FALSE(queue.empty());
    arc_pair = queue.pop();
    ASSERT_FALSE(arc_pair.first);
    ASSERT_EQ(arc_pair.second.from, 0);
    ASSERT_EQ(arc_pair.second.to, 1);
    ASSERT_EQ(arc_pair.second.weight, 20);

    ASSERT_TRUE(queue.empty());
}

TEST(CHLRM, test_convert_and_build_neighbour_index) {
    return;
    std::vector<std::string> osm_files = {
        "ma_min_messplatz.osm.pbf",
        "rippo.osm.pbf",
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

        for(auto n : chlrm_graph.N_minus) {
            auto& child = chlrm_graph.nodes[n.first.node_index].arcs[n.first.arc_index];
            auto& u = chlrm_graph.nodes[child.from];
            auto& v = chlrm_graph.nodes[child.to];
            
            for(auto parent_pair : n.second) {
                auto& parent1 = chlrm_graph.nodes[parent_pair.first.node_index].arcs[parent_pair.first.arc_index];
                auto& parent2 = chlrm_graph.nodes[parent_pair.second.node_index].arcs[parent_pair.second.arc_index];

                EXPECT_EQ(parent1.to, parent2.to) << "Parent arcs do not have the same target node: " 
                    << parent1.from << " -> " << parent1.to << " with mid node " << parent1.mid_node
                    << " and " << parent2.from << " -> " << parent2.to << " with mid node " << parent2.mid_node;
                
                EXPECT_EQ(child.label, parent1.label.unite(parent2.label)) << "Child arc label does not match parent arcs' united label: "
                    << child.label.get_label() << " != " << parent1.label.unite(parent2.label).get_label();
                
                auto& q = chlrm_graph.nodes[parent1.to];

                ASSERT_TRUE(child.is_shortcut()) << "Child arc is not a shortcut: " << child.from << " -> " << child.to << " with mid node " << child.mid_node;
                ASSERT_TRUE(parent1.is_shortcut() && parent2.is_shortcut()) << "Parent arcs are not shortcuts: " << parent1.from << " -> " << parent1.to << " with mid node " << parent1.mid_node
                                                                            << " and " << parent2.from << " -> " << parent2.to << " with mid node " << parent2.mid_node;
                ASSERT_LE(q.rank, u.rank) << "Child arc's target node rank is greater than parent arc's source node rank: " 
                    << q.rank << " > " << u.rank;
                ASSERT_LE(q.rank, v.rank) << "Child arc's target node rank is greater than parent arc's target node rank: "
                    << q.rank << " > " << v.rank;
            }
        }

        auto back_to_chlr = chlrm_graph.to_chlr();

        for(unsigned i = 0; i < back_to_chlr.nodes.size(); ++i) {
            auto& node = back_to_chlr.nodes[i];

            ASSERT_EQ(node.rank, combined_graph.nodes[i].rank) << "Node rank does not match: " 
                << i << " -> " << node.rank << " vs " << combined_graph.nodes[i].rank;
            
            ASSERT_EQ(node.lat, combined_graph.nodes[i].lat) << "Node latitude does not match: "
                << i << " -> " << node.lat << " vs " << combined_graph.nodes[i].lat;
            
            ASSERT_EQ(node.lon, combined_graph.nodes[i].lon) << "Node longitude does not match: "
                << i << " -> " << node.lon << " vs " << combined_graph.nodes[i].lon;
            
            ASSERT_EQ(node.out_arcs.size(), combined_graph.nodes[i].out_arcs.size()) << "Node out arcs size does not match: "
                << i << " -> " << node.out_arcs.size() << " vs " << combined_graph.nodes[i].out_arcs.size();
            
            ASSERT_EQ(node.out_arcs.size(), chlrm_graph.nodes[i].arcs.size()) << "Node out arcs size does not match in CHLRM graph: "
                << i << " -> " << node.out_arcs.size() << " vs " << chlrm_graph.nodes[i].arcs.size();
            

            for(unsigned j = 0; j < node.out_arcs.size(); ++j) {
                auto& arc_back = node.out_arcs[j]; 
                auto& arc_chlrm = chlrm_graph.get_arc(CHLRMArcPos(j, i));
                auto& arc_original = combined_graph.nodes[i].out_arcs[j];

                ASSERT_EQ(arc_back.other_node, arc_chlrm.to) << "Arc's other node does not match: " 
                    << i << " -> " << arc_back.other_node << " vs " 
                    << arc_chlrm.from << " -> " << arc_chlrm.to;
                
                ASSERT_EQ(arc_back.mid_node, arc_chlrm.mid_node) << "Arc's mid node does not match: "
                    << arc_back.mid_node << " vs " << arc_chlrm.mid_node;
                
                ASSERT_EQ(arc_back.weight, arc_chlrm.weight) << "Arc's weight does not match: "
                    << arc_back.weight << " vs " << arc_chlrm.weight;
                
                ASSERT_EQ(arc_back.label, arc_chlrm.label) << "Arc's label does not match";

                ASSERT_EQ(arc_back.other_node, arc_original.other_node) << "Arc's other node does not match original graph: "
                    << i << " -> " << arc_back.other_node << " vs " 
                    << i << " -> " << arc_original.other_node;
                
                ASSERT_EQ(arc_back.mid_node, arc_original.mid_node) << "Arc's mid node does not match original graph: "
                    << arc_back.mid_node << " vs " << arc_original.mid_node;
                
                ASSERT_EQ(arc_back.weight, arc_original.weight) << "Arc's weight does not match original graph: "
                    << arc_back.weight << " vs " << arc_original.weight;
                
                ASSERT_EQ(arc_back.label, arc_original.label) << "Arc's label does not match original graph";
            }
        }
    }

}


TEST(CHLRM, test_neighbour_relationships) {
    CHLRGraph graph;
    graph.nodes.resize(25);

    for(unsigned i = 0; i < graph.nodes.size(); ++i) {
        graph.add_arc(i, invalid_id, (i + 1) % graph.nodes.size(), 1, Label());

        if (i % 5 < 2) {
            graph.add_arc((i + 1) % graph.nodes.size(), invalid_id, i, 1, Label());
        }
    }

    CHLR chlr(graph);
    chlr.build();
    
    CHLRMGraph chlrm_graph(chlr.graph);

    bool found_ne = false;
    bool found_nm = false;
    bool found_np = false;

    for(auto node : chlrm_graph.nodes) {
        for(auto e2 : node.arcs) {
            auto ne = chlrm_graph.Ne(e2); // partners that have a child

            for(auto e1 : ne) {
                found_ne = true;

                ASSERT_EQ(e1.to, e2.from);
                ASSERT_LE(chlrm_graph.nodes[e1.to].rank, chlrm_graph.nodes[e1.from].rank);
                ASSERT_LE(chlrm_graph.nodes[e1.to].rank, chlrm_graph.nodes[e2.to].rank);

                auto child = chlrm_graph.Np(e1, e2);
                ASSERT_EQ(child.from, e1.from);
                ASSERT_EQ(child.to, e2.to);
                ASSERT_EQ(child.label, e2.label.unite(e1.label));

                found_np = true;
            }

            auto nm = chlrm_graph.Nm(e2);
            auto& child = e2;
            for(auto& parents : nm) {
                auto& e1 = parents.first;
                auto& e2_ = parents.second;

                found_nm = true;

                ASSERT_EQ(e1.to, e2_.from);
                ASSERT_EQ(e1.label.unite(e2_.label), child.label);
                ASSERT_EQ(e1.from, child.from);
                ASSERT_EQ(e2_.to, child.to);
                ASSERT_LE(chlrm_graph.nodes[e1.to].rank, chlrm_graph.nodes[child.from].rank);
                ASSERT_LE(chlrm_graph.nodes[e2_.from].rank, chlrm_graph.nodes[child.to].rank);
            }
        }
    }

    ASSERT_TRUE(found_ne);
    ASSERT_TRUE(found_nm);
    ASSERT_TRUE(found_np);
}

TEST(CHLRM, test_maintenance_on_paper_example_graph) {
    CHLRGraph graph;
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
    
    CHLRMGraph chlrm_graph(graph);

    CHLRMArc& e = chlrm_graph.nodes[3].arcs[0];

    chlrm_graph.maintenance(e.get_pos(chlrm_graph), 7, c);
}

TEST(CHLRM, test_graph_maintenance_synthetic) {
    CHLRGraph graph;
    graph.nodes.resize(12);

    for(unsigned i = 0; i < graph.nodes.size(); ++i) {
        graph.add_arc(i, invalid_id, (i + 1) % graph.nodes.size(), 1, Label());
        graph.add_arc((i + 1) % graph.nodes.size(), invalid_id, i, 1, Label());
    }

    auto chlr = CHLR(graph);
    chlr.build();

    CHLRQuery q1(chlr.graph);
    q1.set(0, 2, Label());
    q1.run();

    auto path = q1.get_arc_path();
    ASSERT_EQ(path.size(), 2) << "Wrong path length on unmaintained graph. Expected 2 arcs in the path, got " << path.size();

    auto chlrmg = CHLRMGraph(chlr.graph);
    auto chlr_conv = chlrmg.to_chlr();

    CHLRQuery q(chlr_conv);
    q.set(0, 2, Label());
    q.run();

    auto path1 = q.get_arc_path();
    ASSERT_EQ(path1.size(), 2) << "Wrong path length after converting from CHLRM to CHLR. Expected 2 arcs in the path, got " << path1.size();

    
    auto arc1 = chlrmg.nodes[1].arcs[0]; // Paths through node 1 will be very long
    auto arc2 = chlrmg.nodes[1].arcs[1]; // This results in the shortest path being 0 -> 9 -> ... -> 3 -> 2

    auto arc1_pos = arc1.get_pos(chlrmg);
    auto arc2_pos = arc2.get_pos(chlrmg);

    chlrmg.maintenance(arc1_pos, 100, Label());
    chlrmg.maintenance(arc2_pos, 100, Label());

    CHLRGraph g = chlrmg.to_chlr();
    CHLRQuery q2(g);
    q2.set(0, 2, Label());
    q2.run();

    auto path2 = q2.get_arc_path();
    ASSERT_EQ(path2.size(), 3) << "Wrong path length after maintenance. Expected 3 arcs in the path, got " << path2.size();

    chlrmg.maintenance(arc1_pos, 1, Label::fully_restricted());
    chlrmg.maintenance(arc2_pos, 1, Label::fully_restricted());

    g = chlrmg.to_chlr();
    CHLRQuery q3(g);
    q3.set(0, 2, Label::fully_restricted());
    q3.run();

    auto path3 = q3.get_arc_path();
    ASSERT_EQ(path3.size(), 3) << "Wrong path length after maintenance. Expected 3 arcs in the path, got " << path3.size();

    chlrmg.maintenance(arc1_pos, 1, Label());
    chlrmg.maintenance(arc2_pos, 1, Label());

    g = chlrmg.to_chlr();
    CHLRQuery q4(g);
    q4.set(0, 2, Label());
    q4.run();

    auto path4 = q4.get_arc_path();
    ASSERT_EQ(path4.size(), 2) << "Wrong path length after maintenance. Expected 2 arcs in the path, got " << path4.size();
}
