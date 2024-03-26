
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <random>
#include <cmath>
#include <ctime>
#include <fstream>
#include <fcntl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include "proto/lattle.pb.h"
#include "graph_utils/Header.h"
#include "instance_generator/InstanceGenerator.h"

using namespace std;
using namespace operations_research::lattle;

struct Parameters
{
    unsigned int random_seed;		// Random seed. Default value: 0

	int hubs_number;									// Number of hubs, i.e., of vertices in the graph
	int intial_numb_fully_connected_nodes;				// Parameter of the extended Barabási-Albert algorithm
	int new_connections_per_node;						// Parameter of the extended Barabási-Albert algorithm

	int time_horizon;									// Length of the time horizon
	int time_step;										// Time step to discretise the horizon

	int vehicle_number_per_step;						// Number of vehicles leaving every time step
	int max_vehicle_duration;							// Maximal number of arcs of the space-time network that a vehicle can perform
	int max_vehicle_capacity;							// Maximal capacity of the vehicles
	double vehicle_rand_sample;							// Parameter of the softmax distribution to sample the vehicles at each time step 
	
	int parcels_number;									// Number of parcels to sample
	int mean_parcel_path_length;						// Mean number of arcs in the space-time network that a parcel performs to reach its destination
	int min_parcel_weight;								// Minimal weight of a parcel
	int max_parcel_weight;								// Maximal weight of a parcel
	double parcel_weight_sample;						// Parameter of the Lomax distribution to sample parcel weights
	double parcel_start_sample;							// Parameter of the Boltzmann distribution to sample parcel starting hub
	int max_tries;										// Maximal number of tries to sample the destination of a parcel
 Parameters() :random_seed (0),hubs_number(100),
	intial_numb_fully_connected_nodes(2),
	new_connections_per_node(2),
	time_horizon(24),
	time_step(15),
	vehicle_number_per_step(10),
	max_vehicle_duration(5),
	max_vehicle_capacity(100),
	vehicle_rand_sample(0.01),
	parcels_number(50),
	mean_parcel_path_length(5),
	min_parcel_weight(1),
	max_parcel_weight(100),
	parcel_weight_sample(0.1),
	parcel_start_sample(0.1),
	max_tries(10){};
};

int main(int argc, char const* argv[]) {
    //TODO(support argv for all variables)
	int number_of_nodes = 10;
	if(argc >= 2){
		number_of_nodes = atoi(argv[1]);
	}
	int time_horizon = time_encoder(0, 0, 0, 24, 0, 0);
	unsigned int random_seed = ElRandom::GetSeed();
	ElRandom::SetSeed(random_seed);
	string instanceName = "logisticNetwork_"+to_string(number_of_nodes);
	string description = "";
	InstanceGenerator iGenerator(instanceName, description, random_seed);
	iGenerator.generate_logistic_network(number_of_nodes, time_horizon, 1, 3, 10, 5, 100.0, 0.01,10);
	return 0;
}
