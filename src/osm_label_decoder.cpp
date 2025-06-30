#include <routingkit/osm_label_decoder.h>


bool str_eq(const char*l, const char*r){
    return !strcmp(l, r);
}

Label extract_label_from_osm_way(
    const TagMap&way_tags
) {
    Label label = Label::fully_restricted();
    label.set_bit(!is_osm_way_used_by_pedestrians(0, way_tags, nullptr), PEDESTRIAN);
    label.set_bit(!is_osm_way_used_by_bicycles(0, way_tags, nullptr), BICYCLE);
    label.set_bit(!is_osm_way_used_by_cars(0, way_tags, nullptr), CAR);
    return label;
}

std::string human_readable_label(Label& label) {
    std::string result;
    if(!label.get_bit(PEDESTRIAN)) {
        result += "P ";
    }
    if(!label.get_bit(BICYCLE)) {
        result += "B ";
    }
    if(!label.get_bit(CAR)) {
        result += "C ";
    }

    return result.empty() ? "No access" : result;
}
