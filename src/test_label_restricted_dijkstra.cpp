#include <routingkit/test_builder.h>
#include <iostream>
using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
	TestSetup setup = TestSetup("ma_alter_messplatz.osm.pbf");
	auto result = setup.run_dijkstra(setup.requests[2]);
	setup.print_path(setup.requests[2], result);
}
