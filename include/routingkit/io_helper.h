#ifndef ROUTINGKIT_IO_HELPER_H
#define ROUTINGKIT_IO_HELPER_H

#include <iostream>
#include <routingkit/geo_position_to_node.h>
#include <routingkit/osm_label_decoder.h>
#include <routingkit/label.h>

using namespace RoutingKit;
using namespace std;

struct RoutingRequest {
    bool is_valid = false;
    float from_latitude;
    float from_longitude;
    float to_latitude;
    float to_longitude;
    unsigned from_node;
    unsigned to_node;
    Label profile;
};

RoutingRequest parse_routing_request(int argc, char* argv[], const GeoPositionToNode& map_geo_position) {
    RoutingRequest request = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0, Label(0)};
    if (argc != 5 && argc != 6){
        cout << "Usage: " << argv[0] << " from_latitude from_longitude to_latitude to_longitude" << endl;
        return request;
    } else {
        request.from_latitude = atof(argv[1]);
        request.from_longitude = atof(argv[2]);
        request.to_latitude = atof(argv[3]);
        request.to_longitude = atof(argv[4]);

    if(argc == 6) {
        switch (argv[5][0])  // Assuming argv[5] is a single character for profile selection
        {
        case 'c':
            request.profile.set_bit(true, CAR);
            break;
        case 'b':
            request.profile.set_bit(true, BICYCLE);
            break;
        case 'p':
            request.profile.set_bit(true, PEDESTRIAN);
            break;
        default:
            cout << "Invalid profile selection. Use 'c' for car, 'b' for bicycle, or 'p' for pedestrian." << endl;
            return request;
        }
    }
    }
    unsigned from = map_geo_position.find_nearest_neighbor_within_radius(request.from_latitude, request.from_longitude, 1000).id;
    if(from == invalid_id){
        cout << "No node within 1000m from source position" << endl;
        return request;
    }
    unsigned to = map_geo_position.find_nearest_neighbor_within_radius(request.to_latitude, request.to_longitude, 1000).id;
    if(to == invalid_id){
        cout << "No node within 1000m from target position" << endl;
        return request;
    }

    request.from_node = from;
    request.to_node = to;
    
    request.is_valid = true;
    return request;
}
#endif // ROUTINGKIT_IO_HELPER_H
