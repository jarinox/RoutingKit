#include <routingkit/geotiff_reader.h>
#include <routingkit/test_builder.h>
#include <iostream>
#include <gtest/gtest.h>

using namespace RoutingKit;

TEST(GeoTiffReader, get_value) {
    GeoTiffReader reader("graphs/hq100_heitersheim.tif");

    // Replace with the coordinates you want to query
    double lon = 7.6401165;
    double lat = 47.8793155;

    double value = reader.get_value(lon, lat);

    EXPECT_EQ(value, 1);
}

TEST(GeoTiffReader, line_intersects) {
    GeoTiffReader reader("graphs/hq100_heitersheim.tif");

    double lon1 = 7.6401165;
    double lat1 = 47.8793155;
    double lon2 = 7.6411165;
    double lat2 = 47.8803155;

    bool intersects = reader.line_intersects(lon1, lat1, lon2, lat2);
    EXPECT_TRUE(intersects);

    lon1 = 7.631889;
    lat1 = 47.873823;
    lon2 = 7.623679;
    lat2 = 47.871618;

    intersects = reader.line_intersects(lon1, lat1, lon2, lat2);
    EXPECT_FALSE(intersects);
}


TEST(GeoTiffReader, find_affected_arcs) {
    GeoTiffReader reader("graphs/hq100_freiburg.tif");
    TestSetup setup = TestSetup("freiburg-regbez.osm.pbf");

    auto graph = setup.chg;

    unsigned affected_arcs = 0;
    unsigned total_arcs = 0;

    unsigned max_length = 0;

    for(const auto& node : graph.nodes) {
        double from_lon = node.lon;
        double from_lat = node.lat;

        for(const auto& arc : node.out_arcs) {            
            total_arcs++;

            if(arc.weight > max_length)
                max_length = arc.weight;

            double to_lon = graph.nodes[arc.other_node].lon;
            double to_lat = graph.nodes[arc.other_node].lat;

            if(reader.line_intersects(from_lon, from_lat, to_lon, to_lat)) {
                affected_arcs++;
            }
        }
    }

    std::cout << "Affected arcs: " << affected_arcs << std::endl;
    std::cout << "Total arcs: " << total_arcs << std::endl;
    std::cout << "Node count: " << graph.nodes.size() << std::endl;
    std::cout << "Max arc length: " << max_length << std::endl;
}
