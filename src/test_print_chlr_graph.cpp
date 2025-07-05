#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/contraction_hierarchy.h>
#include <routingkit/min_max.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/label.h>
#include <routingkit/io_helper.h>
#include <routingkit/dijkstra.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

    auto ch = ContractionHierarchy::build(graph.node_count(), tail, graph.head, graph.travel_time, graph.label);

    unsigned arcs_count[2] = {0, 0};
    
    for(unsigned i = 0; i < 2; ++i) {
        auto side = (i == 0) ? ch.forward : ch.backward;
        for (unsigned tail_node = 0; tail_node + 1 < side.first_out.size(); ++tail_node) {
            for (unsigned arc = side.first_out[tail_node]; arc < side.first_out[tail_node + 1]; ++arc) {
                arcs_count[i]++;
                unsigned h = side.head[arc];
                bool is_original = side.is_shortcut_an_original_arc.is_set(arc);

                cout << graph.latitude[ch.order[h]] << " " << graph.longitude[ch.order[h]] << " "
                    << graph.latitude[ch.order[tail_node]] << " " << graph.longitude[ch.order[tail_node]] << " "
                    << (is_original ? "O" : "S") << " "
                    << (i == 0 ? "F" : "B") << " "
                    << ch.rank[ch.order[tail_node]] << "-" << ch.rank[ch.order[h]] << " "
                    << side.weight[arc] << " "
                    << human_readable_label(side.label[arc]) << endl;
            }
        }
    }
}

