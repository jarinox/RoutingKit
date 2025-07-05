#include <routingkit/contraction_hierarchy.h>
#include <routingkit/inverse_vector.h>
#include <routingkit/timer.h>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_label_decoder.h>
#include <routingkit/osm_simple.h>
#include <routingkit/vector_io.h>

#include <iostream>
#include <vector>
#include <random>
#include <stdexcept>
#include <iomanip>

using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
    try{
        auto graph = simple_load_osm_multi_profile_routing_graph_from_pbf("map.osm.pbf");
        //auto graph = simple_load_osm_car_routing_graph_from_pbf("map.osm.pbf");

        assert(graph.label.size() == graph.arc_count());

        const auto& first_out = graph.first_out;
        const auto& head = graph.head;
        const auto& latitude = graph.latitude;
        const auto& longitude = graph.longitude;


        unsigned node_count = first_out.size() - 1;
        unsigned arc_count = head.size();

        // Set precision for coordinate output
        cout << fixed << setprecision(6);

        auto car_label = Label();
        car_label.set_bit(false, CAR);
        car_label.set_bit(false, PEDESTRIAN);
        car_label.set_bit(false, BICYCLE);

        // Iterate through all nodes and their outgoing edges
        for(unsigned x = 0; x < node_count; ++x){
            for(unsigned xy = first_out[x]; xy < first_out[x+1]; ++xy){
                unsigned y = head[xy];
                if(graph.label[xy].is_allowed(car_label)) {
                    // Print: lat_start lon_start lat_end lon_end
                    cout << latitude[x] << " " << longitude[x] << " " 
                     << latitude[y] << " " << longitude[y] << " A B " << human_readable_label(graph.label[xy]) << endl;
                }
            }
        }

    }catch(exception&err){
        cerr << "Stopped on exception : " << err.what() << endl;
        return 1;
    }
    
    return 0;
}
