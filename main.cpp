
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

namespace {
void CustomLogHandler(google::protobuf::LogLevel level, const char* filename, int line, const std::string& message) {
	std::cerr << "ERROR: " << level << " at " << filename << ":" << line << ": " << message << std::endl;
	}
}

int main(int argc, char const* argv[]) {


//   GOOGLE_PROTOBUF_VERIFY_VERSION;

//   google::protobuf::SetLogHandler(&CustomLogHandler);
//     // Parse protobuf text part.
//     LogisticsNetwork read_network;
// 	string network_file = argv[1];
// 	const char* network_cc = network_file.c_str();
// 	if (!std::filesystem::exists({ network_cc })) {
// 		std::cout << "File " << network_cc << " does not exist!\n";
// 		exit(EXIT_FAILURE);
// 	}

// 	fstream input_network(network_cc, ios::in);
// 	if (!input_network.is_open()) {
// 		std::cout << "Error while opening file " << network_cc << ", although the file exists!\n";
// 		exit(EXIT_FAILURE);
// 	}

// 	std::ostringstream ss_network;
// 	ss_network << input_network.rdbuf();
// 	if (!google::protobuf::TextFormat::ParseFromString(ss_network.str(), &read_network)) {
// 		cerr << "Failed to parse instance from file " << network_cc << "\n";
// 		exit(EXIT_FAILURE);
// 	}
//   cout << read_network.name() << endl;


// 	read_network.set_name("write_network");
// 	cout << read_network.name() << endl;

//     //output protobuffer file part
// 	string out;
// 	if (!google::protobuf::TextFormat::PrintToString(read_network, &out)) {
// 		cerr << "Failed to write the logistic network file "<< "\n";
// 		exit(EXIT_FAILURE);
// 	}
// 	fstream output_network("./" + read_network.name() + ".textproto", ios::out);
// 	output_network << out;
// 	output_network.close();

// 	// Optional:  Delete all global objects allocated by libprotobuf.
// 	google::protobuf::ShutdownProtobufLibrary();

   int time_horizon = time_encoder(0, 0, 0, 24, 0, 0);
	//unsigned int random_seed = 501844;
	unsigned int random_seed = ElRandom::GetSeed();
	ElRandom::SetSeed(random_seed);
	string instanceName = "logisticNetwork_1000_500_300";
	string description = "";
	
	InstanceGenerator iGenerator(instanceName, description, random_seed);
	iGenerator.generate_logistic_network(1000, time_horizon, 1, 3, 10, 5, 100.0, 0.01,10);


	return 0;
}
