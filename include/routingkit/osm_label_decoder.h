#ifndef OSM_LABEL_DECODER_H
#define OSM_LABEL_DECODER_H

#include <routingkit/label.h>
#include <routingkit/tag_map.h>
#include <routingkit/osm_profile.h>

using namespace RoutingKit;

#ifdef CAR
#error "CAR is defined as a macro!"
#endif
enum LabelNames : unsigned int {
    PEDESTRIAN = 0,
    BICYCLE = 1,
    CAR = 2,
}; 

Label extract_label_from_osm_way(
    const TagMap&way_tags
);

std::string human_readable_label(Label&label);

#endif // OSM_LABEL_DECODER_H
