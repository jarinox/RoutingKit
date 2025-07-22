#include <routingkit/test_builder.h>


#include "verify.h"

#include <iostream>
#include <stdexcept>
#include <vector>

using namespace RoutingKit;
using namespace std;


int main() {
    TestSetup setup = TestSetup("ma_alter_messplatz.osm.pbf");
    setup.chlr.build();

    auto result = setup.run_chlr(setup.requests[2]);
    setup.print_path(setup.requests[2], result);
}

