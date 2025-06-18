#include <routingkit/osm_graph_builder.h>
#include <routingkit/osm_profile.h>
#include <routingkit/osm_simple.h>
#include <routingkit/osm_label_decoder.h>

#include <vector>
#include <stdint.h>
#include <string>

namespace RoutingKit{

SimpleOSMCarRoutingGraph simple_load_osm_car_routing_graph_from_pbf(
	const std::string&pbf_file,
	const std::function<void(const std::string&)>&log_message,
	bool all_modelling_nodes_are_routing_nodes,
	bool file_is_ordered_even_though_file_header_says_that_it_is_unordered
){
	auto mapping = load_osm_id_mapping_from_pbf(
		pbf_file,
		nullptr,
		[&](uint64_t osm_way_id, const TagMap&tags){
			return is_osm_way_used_by_cars(osm_way_id, tags, log_message);
		},
		log_message,
		all_modelling_nodes_are_routing_nodes
	);

	unsigned routing_way_count = mapping.is_routing_way.population_count();
	std::vector<unsigned>way_speed(routing_way_count);

	auto routing_graph = load_osm_routing_graph_from_pbf(
		pbf_file,
		mapping,
		[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
			way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, way_tags, log_message);
			return get_osm_car_direction_category(osm_way_id, way_tags, log_message);
		},
		[&](uint64_t osm_relation_id, const std::vector<OSMRelationMember>&member_list, const TagMap&tags, std::function<void(OSMTurnRestriction)>on_new_restriction){
			return decode_osm_car_turn_restrictions(osm_relation_id, member_list, tags, on_new_restriction, log_message);
		},
		extract_label_from_osm_way,
		log_message
	);

	mapping = OSMRoutingIDMapping(); // release memory

	SimpleOSMCarRoutingGraph ret;
	ret.first_out = std::move(routing_graph.first_out);
	ret.head = std::move(routing_graph.head);
	ret.geo_distance = std::move(routing_graph.geo_distance);
	ret.latitude = std::move(routing_graph.latitude);
	ret.longitude = std::move(routing_graph.longitude);
	ret.label = std::move(routing_graph.labels);

	ret.travel_time = ret.geo_distance;
	for(unsigned a=0; a<ret.travel_time.size(); ++a){
		ret.travel_time[a] *= 18000;
		ret.travel_time[a] /= way_speed[routing_graph.way[a]];
		ret.travel_time[a] /= 5;
	}

	ret.forbidden_turn_from_arc = std::move(routing_graph.forbidden_turn_from_arc);
	assert(is_sorted_using_less(ret.forbidden_turn_from_arc));
	ret.forbidden_turn_to_arc = std::move(routing_graph.forbidden_turn_to_arc);

	return ret;
}

SimpleOSMPedestrianRoutingGraph simple_load_osm_pedestrian_routing_graph_from_pbf(
	const std::string&pbf_file,
	const std::function<void(const std::string&)>&log_message,
	bool all_modelling_nodes_are_routing_nodes,
	bool file_is_ordered_even_though_file_header_says_that_it_is_unordered
){
	auto mapping = load_osm_id_mapping_from_pbf(
		pbf_file,
		nullptr,
		[&](uint64_t osm_way_id, const TagMap&tags){
			return is_osm_way_used_by_pedestrians(osm_way_id, tags, log_message);
		},
		log_message,
		all_modelling_nodes_are_routing_nodes
	);

	auto routing_graph = load_osm_routing_graph_from_pbf(
		pbf_file,
		mapping,
		[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
			return OSMWayDirectionCategory::open_in_both;
		},
		nullptr,
		nullptr,
		log_message
	);

	mapping = OSMRoutingIDMapping(); // release memory

	SimpleOSMPedestrianRoutingGraph ret;
	ret.first_out = std::move(routing_graph.first_out);
	ret.head = std::move(routing_graph.head);
	ret.geo_distance = std::move(routing_graph.geo_distance);
	ret.latitude = std::move(routing_graph.latitude);
	ret.longitude = std::move(routing_graph.longitude);

	return ret;
}


SimpleOSMBicycleRoutingGraph simple_load_osm_bicycle_routing_graph_from_pbf(
	const std::string&pbf_file,
	const std::function<void(const std::string&)>&log_message,
	bool all_modelling_nodes_are_routing_nodes,
	bool file_is_ordered_even_though_file_header_says_that_it_is_unordered
){
	auto mapping = load_osm_id_mapping_from_pbf(
		pbf_file,
		nullptr,
		[&](uint64_t osm_way_id, const TagMap&tags){
			return is_osm_way_used_by_bicycles(osm_way_id, tags, log_message);
		},
		log_message,
		all_modelling_nodes_are_routing_nodes
	);

	unsigned routing_way_count = mapping.is_routing_way.population_count();

	std::vector<unsigned char> comfort_level(routing_way_count, false);

	auto routing_graph = load_osm_routing_graph_from_pbf(
		pbf_file,
		mapping,
		[&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&way_tags){
			comfort_level[routing_way_id] = get_osm_way_bicycle_comfort_level(osm_way_id, way_tags, log_message);
			return get_osm_bicycle_direction_category(osm_way_id, way_tags, log_message);
		},
		nullptr,
		nullptr,
		log_message
	);

	unsigned arc_count = routing_graph.head.size();

	mapping = OSMRoutingIDMapping(); // release memory

	SimpleOSMBicycleRoutingGraph ret;
	ret.first_out = std::move(routing_graph.first_out);
	ret.head = std::move(routing_graph.head);
	ret.geo_distance = std::move(routing_graph.geo_distance);
	ret.latitude = std::move(routing_graph.latitude);
	ret.longitude = std::move(routing_graph.longitude);

	ret.arc_comfort_level.resize(arc_count);
	for(unsigned a=0; a<arc_count; ++a)
		ret.arc_comfort_level[a] = comfort_level[routing_graph.way[a]];

	return ret;
}

SimpleOSMMultiProfileRoutingGraph simple_load_osm_multi_profile_routing_graph_from_pbf(
	const std::string&pbf_file,
	const std::function<void(const std::string&)>&log_message,
	bool all_modelling_nodes_are_routing_nodes,
	bool file_is_ordered_even_though_file_header_says_that_it_is_unordered
){
	// Accepts all three vehicle types
	auto combined_way_filter = [&](uint64_t osm_way_id, const TagMap&tags) -> bool {
		return is_osm_way_used_by_cars(osm_way_id, tags, log_message) ||
		       is_osm_way_used_by_bicycles(osm_way_id, tags, log_message) ||
		       is_osm_way_used_by_pedestrians(osm_way_id, tags, log_message);
	};

	auto mapping = load_osm_id_mapping_from_pbf(
		pbf_file,
		nullptr,
		combined_way_filter,
		log_message,
		all_modelling_nodes_are_routing_nodes
	);

	unsigned routing_way_count = mapping.is_routing_way.population_count();
	
	std::vector<unsigned>way_speed(routing_way_count);
	std::vector<unsigned char>way_bicycle_comfort(routing_way_count);


	auto multi_profile_way_callback = [&](uint64_t osm_way_id, unsigned routing_way_id, const TagMap&tags) -> OSMLabelRestrictedDirections {
		// Vehicle permissions for this way

		Label label = extract_label_from_osm_way(tags);
		bool is_car_allowed = !label.get_bit(CAR);
		bool is_bicycle_allowed = !label.get_bit(BICYCLE);
		bool is_pedestrian_allowed = !label.get_bit(PEDESTRIAN);

		OSMLabelRestrictedDirections restricted_directions = OSMLabelRestrictedDirections();
		// Default to disallow all vehicles
		// Labels are restrictions, so we set the bits to true for all vehicles
		restricted_directions.forward.set_bit(CAR, true);
		restricted_directions.backward.set_bit(CAR, true);
		restricted_directions.forward.set_bit(BICYCLE, true);
		restricted_directions.backward.set_bit(BICYCLE, true);
		restricted_directions.forward.set_bit(PEDESTRIAN, true);
		restricted_directions.backward.set_bit(PEDESTRIAN, true);

		if(is_car_allowed) {
			auto direction_category = get_osm_car_direction_category(osm_way_id, tags, log_message);
			bool car_allowed_in_direction = (direction_category == OSMWayDirectionCategory::open_in_both ||
				direction_category == OSMWayDirectionCategory::only_open_forwards);
			restricted_directions.forward.set_bit(CAR, !car_allowed_in_direction); // If cars are allowed, disable the car restriction in the forward direction

			car_allowed_in_direction = (direction_category == OSMWayDirectionCategory::open_in_both ||
				direction_category == OSMWayDirectionCategory::only_open_backwards);
			restricted_directions.backward.set_bit(CAR, !car_allowed_in_direction);
		}

		if(is_bicycle_allowed) {
			auto direction_category = get_osm_bicycle_direction_category(osm_way_id, tags, log_message);
			bool bicycle_allowed_in_direction = (direction_category == OSMWayDirectionCategory::open_in_both ||
				direction_category == OSMWayDirectionCategory::only_open_forwards);
			restricted_directions.forward.set_bit(BICYCLE, !bicycle_allowed_in_direction);
			bicycle_allowed_in_direction = (direction_category == OSMWayDirectionCategory::open_in_both ||
				direction_category == OSMWayDirectionCategory::only_open_backwards);
			restricted_directions.backward.set_bit(BICYCLE, !bicycle_allowed_in_direction);
		}

		if(is_pedestrian_allowed) {
			restricted_directions.forward.set_bit(PEDESTRIAN, false);
			restricted_directions.backward.set_bit(PEDESTRIAN, false);
		}

		// Store car-specific data
		if (is_car_allowed) {
			way_speed[routing_way_id] = get_osm_way_speed(osm_way_id, tags, log_message);
		} else {
			way_speed[routing_way_id] = 5; // Default walking speed for non-car ways
		}
		
		// Store bicycle-specific data
		if (is_bicycle_allowed) {
			way_bicycle_comfort[routing_way_id] = get_osm_way_bicycle_comfort_level(osm_way_id, tags, log_message);
		} else {
			way_bicycle_comfort[routing_way_id] = 0; // Lowest comfort for non-bicycle ways
		}
		
		return restricted_directions;
	};

	auto routing_graph = load_osm_routing_graph_from_pbf(
		pbf_file,
		mapping,
		multi_profile_way_callback,
		[&](uint64_t osm_relation_id, const std::vector<OSMRelationMember>&member_list, const TagMap&tags, std::function<void(OSMTurnRestriction)>on_new_restriction){
			// Only decode turn restrictions for cars (most restrictive)
			return decode_osm_car_turn_restrictions(osm_relation_id, member_list, tags, on_new_restriction, log_message);
		},
		extract_label_from_osm_way,
		log_message,
		file_is_ordered_even_though_file_header_says_that_it_is_unordered
	);

	mapping = OSMRoutingIDMapping(); // release memory

	SimpleOSMMultiProfileRoutingGraph ret;
	ret.first_out = std::move(routing_graph.first_out);
	ret.head = std::move(routing_graph.head);
	ret.geo_distance = std::move(routing_graph.geo_distance);
	ret.latitude = std::move(routing_graph.latitude);
	ret.longitude = std::move(routing_graph.longitude);
	ret.label = std::move(routing_graph.labels);
	ret.forbidden_turn_from_arc = std::move(routing_graph.forbidden_turn_from_arc);
	ret.forbidden_turn_to_arc = std::move(routing_graph.forbidden_turn_to_arc);

	unsigned arc_count = ret.head.size();
	ret.travel_time.resize(arc_count);
	ret.bicycle_comfort_level.resize(arc_count);

	for(unsigned a = 0; a < arc_count; ++a) {
		unsigned way_id = routing_graph.way[a];
		ret.bicycle_comfort_level[a] = way_bicycle_comfort[way_id];
		
		// Calculate travel time (primarily for cars)
		ret.travel_time[a] = ret.geo_distance[a];
		ret.travel_time[a] *= 18000; // Convert to travel time units
		ret.travel_time[a] /= way_speed[way_id];
		ret.travel_time[a] /= 5;
	}

	if(log_message) {
		unsigned car_arc_count = 0, bicycle_arc_count = 0, pedestrian_arc_count = 0;
		for(unsigned a = 0; a < arc_count; ++a) {
			if(!ret.label[a].get_bit(CAR)) car_arc_count++;
			if(!ret.label[a].get_bit(BICYCLE)) bicycle_arc_count++;
			if(!ret.label[a].get_bit(PEDESTRIAN)) pedestrian_arc_count++;
		}
		log_message("Multi-profile graph contains " + std::to_string(arc_count) + " total arcs");
		log_message("  - " + std::to_string(car_arc_count) + " arcs allow cars");
		log_message("  - " + std::to_string(bicycle_arc_count) + " arcs allow bicycles");
		log_message("  - " + std::to_string(pedestrian_arc_count) + " arcs allow pedestrians");
	}

	return ret;
}

} // RoutingKit

