
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

	// Printing information about how to use the code
	void display_help()
	{
		//TODO(aymanelotfi) : change this with the working parameters.
		std::cout << std::endl;
		std::cout << "-------------------------------------------------- PARAMETERS DOC ---------------------------------------------------" << std::endl;
		std::cout << "[-seed <int>] sets a fixed random seed. Defaults to 0                                                                                  " << std::endl;
		std::cout << "[-hubs <int>] sets the number of hubs. Defaults to 100                                                                                 " << std::endl;
		std::cout << "[-m0 <int>] sets the m0 param for the extended Barabási-Albert algorithm, m0 = initial number of nodes fully connected. Defaults to 2                                                                                 " << std::endl;
		std::cout << "[-m <int>] sets the m param for the extended Barabási-Albert algorithm, m = number of connection to add per node. Defaults to 2                                                                                 " << std::endl;
		std::cout << "[-th <int>] sets the length of the time horizon. Defaults to 24 hours                                                                                 " << std::endl;
		std::cout << "[-ts <int>] sets the step size to discretise the time horizon. Defaults to 15 minutes                                                                                 " << std::endl;
		std::cout << "[-veh <int>] sets the number of vehicles samples per time step. Defaults to 10                                                                                 " << std::endl;
		std::cout << "[-v_dur <int>] sets the maximal number of arcs of the space-time network that a vehicle can perform to reach the destination. Defaults to 5                                                                                 " << std::endl;
		std::cout << "[-cap <int>] sets the maximal capacity of the vehicles. Defaults to 100                                                                                 " << std::endl;
		std::cout << "[-v_sample <double>] sets the parameter in the softmax distribution to sample vehicles. Defaults to 0.01                                                                                 " << std::endl;
		
		std::cout << "[-parc <int>] sets the number of parcels to sample. Defaults to 50                                                                                 " << std::endl;
		std::cout << "[-p_dur <int>] sets the mean number of arcs of the space-time network that a parcel can perform to reach the destination. Defaults to 5                                                                                 " << std::endl;
		std::cout << "[-min_w <int>] sets the minimal value to sample the weight of a parcel. Defaults to 1                                                                                 " << std::endl;
		std::cout << "[-max_w <int>] sets the maximal value to sample the weight of a parcel. Defaults to 100                                                                                 " << std::endl;
		std::cout << "[-w_sample <double>] sets the parameter in the lomax distribution to sample parcel weights. Defaults to 0.1                                                                                 " << std::endl;
		std::cout << "[-s_sample <double>] sets the parameter in the Boltzmann distribution to sample the starting hub of the parcels. Defaults to 0.1                                                                                 " << std::endl;
		std::cout << "[-tries <int>] sets the maximal number of tries to sample the destination of the parcels. Defaults to 10                                                                                 " << std::endl;

		std::cout << "Call with: ./main Instance_Name [-flag_param param] ...               " << std::endl;
		
		std::cout << "--------------------------------------------------------------------------------------------------------------------------------" << std::endl;
		std::cout << std::endl;
	};
int main(int argc, char const* argv[]) {
    //TODO(support argv for all variables)
	Parameters p ;
	string instanceName ="";
	/*if (argc % 2 != 0 || argc > 35 || argc < 3) {		
			std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc << std::endl;
			display_help(); throw std::string("Incorrect line of command");
		}
		else {
			instanceName = std::string(argv[1]);
			for (int i = 2; i < argc; i += 2) {
				if (std::string(argv[i]) == "-seed")
					p.random_seed = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-hubs")
					p.hubs_number = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-m0")
					p.intial_numb_fully_connected_nodes = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-m")
					p.new_connections_per_node = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-th")
					p.time_horizon = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-ts")
					p.time_step = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-veh")
					p.vehicle_number_per_step = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-v_dur")
					p.max_vehicle_duration = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-cap")
					p.max_vehicle_capacity = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-v_sample")
					p.vehicle_rand_sample = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-parc")
					p.parcels_number = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-p_dur")
					p.mean_parcel_path_length = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-min_w")
					p.min_parcel_weight = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-max_w")
					p.max_parcel_weight = atoi(argv[i + 1]);
				else if (std::string(argv[i]) == "-w_sample")
					p.parcel_weight_sample = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-s_sample")
					p.parcel_start_sample = atof(argv[i + 1]);
				else if (std::string(argv[i]) == "-tries")
					p.max_tries = atoi(argv[i + 1]);
				else {
					std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i]) << std::endl;
					display_help(); throw std::string("Incorrect line of command");
				}
			}
		}*/

	int time_horizon = time_encoder(0, 0, 0, 24, 0, 0);
	unsigned int random_seed = ElRandom::GetSeed();
	ElRandom::SetSeed(620878630);
	instanceName = "instance_"+to_string(p.hubs_number);
	string description = "";
	cout<<"RANDOM SEED "<<ElRandom::GetSeed()<<endl;

	operations_research::lattle::Instance instance;
	instance.set_name(instanceName);
	InstanceGenerator iGenerator(instanceName, description, random_seed);
	iGenerator.generate_logistic_network(instance, p.hubs_number, 0.5, time_horizon, 1, 10, 2, 3, 3, 5, 100.0, 0.01, 10);
	iGenerator.generate_shipments(instance, 5, time_horizon, 5);
	cout<<"end\n";
	
	//output protobuffer file
	google::protobuf::TextFormat::Printer printer;
	string out;

	if (!printer.PrintToString(instance, &out)) {
		cerr << "Failed to write the instance file " << instanceName << "\n";
		exit(EXIT_FAILURE);
	}
	fstream output_instance("../generated_graphs/" + instanceName + ".textproto", ios::out);
	output_instance << out;
	output_instance.close();
	google::protobuf::ShutdownProtobufLibrary();

	return 0;
}
