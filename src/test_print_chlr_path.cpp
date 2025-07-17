#include <routingkit/test_builder.h>


#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


int main() {
    TestSetup setup = TestSetup("rippo.osm.pbf");
    setup.chlr.build();

    auto result = setup.run_chlr(setup.requests[0]);
    setup.print_path(setup.requests[0], result);
}

