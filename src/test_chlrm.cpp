#include <routingkit/test_builder.h>
#include <routingkit/chlrm.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


TEST(CHLRM, test_build_neighbour_index) {
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
    }
}

