
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

int main(int argc, char const* argv[]) {
   int time_horizon = time_encoder(0, 0, 0, 24, 0, 0);
	unsigned int random_seed = ElRandom::GetSeed();
	ElRandom::SetSeed(random_seed);
	string instanceName = "logisticNetwork_100_500_300";
	string description = "";

	InstanceGenerator iGenerator(instanceName, description, random_seed);
	iGenerator.generate_logistic_network(20, time_horizon, 1, 3, 10, 5, 100.0, 0.01,10);


	return 0;
}
