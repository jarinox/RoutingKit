#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>
#include <routingkit/timer.h>
#include <routingkit/chlr.h>
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
    auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("graphs/ma_alter_messplatz.osm.pbf");
	auto tail = invert_inverse_vector(graph.first_out);

    auto chg = CHLRGraph(graph.node_count(), tail, graph.head, graph.geo_distance, graph.latitude, graph.longitude, graph.label);
    auto ch = CHLR(chg);
    ch.build();

    auto chc = CHLRQuery(ch.graph);

    unsigned arcs_count[2] = {0, 0};            

    cout << "BEGIN NODES" << endl;
    for(unsigned i = 0; i < ch.graph.nodes.size(); ++i) {
        cout << ch.graph.nodes[i].lat << " " << ch.graph.nodes[i].lon << " "
             << ch.graph.nodes[i].rank << "_" << i << endl;
    }

    
    cout << "BEGIN EDGES" << endl;
    for(unsigned i = 0; i < 2; ++i) {
        auto side = (i == 0) ? chc.forward : chc.backward;
        unsigned j = 0;
        for (auto& node : side.nodes) {
            for (auto& arc : node.out_arcs) {
                arcs_count[i]++;
                
                bool is_original = arc.is_shortcut() == false;

                unsigned from = j;
                unsigned to = arc.other_node;

                assert(side.nodes[from].rank < side.nodes[to].rank);
                
                cout << side.nodes[from].lat << " " << side.nodes[from].lon << " "
                    << side.nodes[to].lat << " " << side.nodes[to].lon << " "
                    << (is_original ? "O" : "S") << " "
                    << (i == 0 ? "F" : "B") << " "
                    << from << "-" << to << " "
                    << arc.weight << " "
                    << human_readable_label(arc.label) << endl;
            }
            j++;
        }
    }
}
