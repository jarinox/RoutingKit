#include <routingkit/test_builder.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


TEST(CHLR, ch_vs_dijkstra) {
    std::vector<std::string> osm_files = {
        "rippo.osm.pbf",
        "hd_west.osm.pbf",
        "hd_neuenheim.osm.pbf"
    };

    for (const auto& osm_file : osm_files) {
        TestSetup setup = TestSetup(osm_file);
        std::cout << "Building CH for " << osm_file << std::endl;
        setup.chlr.build();

        std::cout << "Running requests..." << std::endl;
        for(auto& req : setup.requests) {
            auto chlr_result = setup.run_chlr(req);
            auto dij_result = setup.run_dijkstra(req);

            setup.assert_all(chlr_result, dij_result);
        }
    }
}

