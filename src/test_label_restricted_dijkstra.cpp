#include <routingkit/test_builder.h>
#include <iostream>
using namespace RoutingKit;
using namespace std;

int main(int argc, char*argv[]){
	auto log_message = [](const std::string&msg){
		cout << msg << endl;
	};

	TestSetup setup = TestSetup("hd_neuenheim.osm.pbf");
	std::cout << human_readable_label(setup.requests[0].profile) << std::endl;
	return 0;
	auto result = setup.run_dijkstra(setup.requests[0]);
	setup.print_path(setup.requests[0], result);
}
