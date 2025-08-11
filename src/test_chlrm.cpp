#include <routingkit/test_builder.h>
#include <routingkit/chlrm.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


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
            if(!e2.is_shortcut()) continue;
            auto ne = chlrm_graph.Ne(e2); // partners that have a child

            for(auto e1 : ne) {
                found_ne = true;

                ASSERT_EQ(e1->to, e2.from);
                ASSERT_LE(chlrm_graph.nodes[e1->to].rank, chlrm_graph.nodes[e1->from].rank);
                ASSERT_LE(chlrm_graph.nodes[e1->to].rank, chlrm_graph.nodes[e2.to].rank);

                auto child = chlrm_graph.Np(*e1, e2);
                ASSERT_EQ(child.from, e1->from);
                ASSERT_EQ(child.to, e2.to);
                ASSERT_EQ(child.label, e2.label.unite(e1->label));

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

TEST(CHLRM, test_graph_maintenance_synthetic) {
    return;
    CHLRGraph graph;
    graph.nodes.resize(5);

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

    chlrmg.maintenance(arc1, 100, Label());
    chlrmg.maintenance(arc2, 100, Label());

    CHLRGraph g = chlrmg.to_chlr();
    CHLRQuery q2(g);
    q2.set(0, 2, Label());
    q2.run();

    auto path2 = q2.get_arc_path();
    ASSERT_EQ(path2.size(), 3) << "Wrong path length after maintenance. Expected 3 arcs in the path, got " << path2.size();

    chlrmg.maintenance(arc1, 1, Label::fully_restricted()); // Rese
    chlrmg.maintenance(arc2, 1, Label::fully_restricted());

    g = chlrmg.to_chlr();
    CHLRQuery q3(g);
    q3.set(0, 2, Label::fully_restricted());
    q3.run();

    auto path3 = q3.get_arc_path();
    ASSERT_EQ(path3.size(), 3) << "Wrong path length after maintenance. Expected 3 arcs in the path, got " << path3.size();

    chlrmg.maintenance(arc1, 1, Label());
    chlrmg.maintenance(arc2, 1, Label());

    g = chlrmg.to_chlr();
    CHLRQuery q4(g);
    q4.set(0, 2, Label());
    q4.run();

    auto path4 = q4.get_arc_path();
    ASSERT_EQ(path4.size(), 2) << "Wrong path length after maintenance. Expected 2 arcs in the path, got " << path4.size();
}
