#include <routingkit/geotiff_reader.h>
#include <iostream>
#include <gtest/gtest.h>

TEST(GeoTiffReaderTest, GetValue) {
    RoutingKit::GeoTiffReader reader("graphs/hq100_heitersheim.tif");

    // Replace with the coordinates you want to query
    double lon = 7.6401165;
    double lat = 47.8793155;

    double value = reader.get_value(lon, lat);

    EXPECT_EQ(value, 1);
}
