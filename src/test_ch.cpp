#include <routingkit/test_builder.h>
#include <gtest/gtest.h>

#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


TEST(CHLR, ch_vs_dijkstra) {
    TestSetup setup = TestSetup("rippo.osm.pbf");
    std::cout << "Building Contraction Hierarchy..." << std::endl;
    setup.build_ch();

    std::cout << "Running requests..." << std::endl;
    for(auto& req : setup.requests) {
        auto chlr_result = setup.run_ch(req);
        auto dij_result = setup.run_dijkstra(req);

        setup.assert_all(chlr_result, dij_result);
    }
}

