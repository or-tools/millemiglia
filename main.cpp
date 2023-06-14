// Copyright 2023-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//kls
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include <fstream>
#include <iostream>
#include <string>

#include "instance_generator/InstanceGenerator.h"
#include "proto/millemiglia.pb.h"

using namespace std;
using namespace millemiglia;

struct Parameters {
  string instance_name;  // Name of the instance
  
  int random_seed;       // Random seed
  
  int hubs_number;       // Number of hubs, i.e., of vertices in the graph

  double graph_density;  // Density of the graph

  int time_horizon;  // Length of the time horizon

  int nbr_lines;  // Number of lines in the network

  int max_length_line;  // Maximal length of a line

  int max_numb_rotations_per_line;  // Maximum number of rotations per line

  double max_vehicle_capacity;  // Maximal capacity of the vehicles

  int shipment_number;  // Number of shipments to sample

  int min_parcel_weight;  // Minimal weight of a parcel

  int max_parcel_weight;  // Maximal weight of a parcel

  int max_path_length;  // Maximal path length of a parcel

  Parameters()
      : instance_name("random_instance"),
        random_seed(ElRandom::GetSeed()),
        hubs_number(100),
        graph_density(0.05),
        time_horizon(time_encoder(0, 0, 0, 24, 0, 0)),
        nbr_lines(10),
        max_length_line(4),
        max_numb_rotations_per_line(10),
        max_vehicle_capacity(100),
        shipment_number(0),
        min_parcel_weight(1),
        max_parcel_weight(100),
        max_path_length(5) {};
};

// Printing information about how to use the code
void display_help() {
  std::cout << std::endl;
  std::cout << "-------------------------------------------------- PARAMETERS "
               "DOC ---------------------------------------------------"
            << std::endl;
  std::cout
      << "[instanceName] sets the instance name. Defaults to no_name_instance  "
         "            "
         "                                                                  "
      << std::endl;
  std::cout
      << "[-seed <int>] sets a fixed random seed. Defaults to 0                "
         "                                                                  "
      << std::endl;
  std::cout
      << "[-hubs_number <int>] sets the number of hubs. Defaults to 100               "
         "                                                                  "
      << std::endl;
  std::cout
      << "[-graph_density <double>] sets the density of the graph. Defaults to "
         "0.5                                                                "
      << std::endl;
  std::cout
      << "[-th <int>] sets the length of the time horizon. Defaults to 10     "
         "                                                                  "
      << std::endl;
  std::cout
      << "[-nbr_lines <int>] sets the number of lines in the network. Defaults "
         "to 10                                                             "
      << std::endl;
  std::cout
      << "[-max_length_line <int>] sets the maximal length of a line. Defaults "
         "to 10                                                             "
      << std::endl;
  std::cout
      << "[-max_numb_rotations_per_line <int>] sets the maximum number of "
         "rotations per line. Defaults to 3                                 "
      << std::endl;
  std::cout
      << "[-max_vehicle_capacity <int>] sets the maximal capacity of the "
         "vehicles. Defaults to 100                                         "
      << std::endl;
  std::cout
      << "[-vehicle_rand_sample <double>] sets the parameter of the softmax "
         "distribution to sample the vehicles at each time step. Defaults to "
         "0.01                                                              "
      << std::endl;
  std::cout
      << "[-shipments_number <int>] sets the number of shipments to sample. "
         "Defaults to 50                                                    "
      << std::endl;
  std::cout
      << "[-min_parcel_weight <int>] sets the minimal weight of a parcel. "
         "Defaults to 1                                                     "
      << std::endl;
  std::cout
      << "[-max_parcel_weight <int>] sets the maximal weight of a parcel. "
         "Defaults to 100                                                   "
      << std::endl;
  std::cout
      << "[-max_path_length <int>] sets the maximal path length of a parcel. "
         "Defaults to 10                                                    "
      << std::endl;
  std::cout
      << "[-max_tries <int>] sets the maximal number of tries to sample the "
         "destination of a parcel. Defaults to 10                           "
      << std::endl;
  std::cout
      << "---------------------------------------------------------------------"
         "-----------------------------------------------------------"
      << std::endl;
  std::cout << std::endl;
};
int main(int argc, char const* argv[]) {
  Parameters p;
  if (argc % 2 != 0 || argc > 35 || argc < 3) {
    std::cout << "----- NUMBER OF COMMANDLINE ARGUMENTS IS INCORRECT: " << argc
              << std::endl;
    display_help();
    throw std::string("Incorrect line of command");
  } else {
    p.instance_name = std::string(argv[1]);
    for (int i = 2; i < argc; i += 2) {
      if (std::string(argv[i]) == "-hubs_number") {
        p.hubs_number = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-graph_density") {
        p.graph_density = strtod(argv[i + 1], nullptr);
      } else if (std::string(argv[i]) == "-th") {
        p.time_horizon = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-nbr_lines") {
        p.nbr_lines = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-max_length_line") {
        p.max_length_line = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-max_numb_rotations_per_line") {
        p.max_numb_rotations_per_line = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-max_vehicle_capacity") {
        p.max_vehicle_capacity = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-shipments_number") {
        p.shipment_number = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-min_parcel_weight") {
        p.min_parcel_weight = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-max_parcel_weight") {
        p.max_parcel_weight = atoi(argv[i + 1]);
      } else if (std::string(argv[i]) == "-max_path_length") {
        p.max_path_length = atoi(argv[i + 1]);
      }  else if (std::string(argv[i]) == "-seed") {
        p.random_seed = atoi(argv[i + 1]);
      } else {
        std::cout << "----- ARGUMENT NOT RECOGNIZED: " << std::string(argv[i])
                  << std::endl;
        display_help();
        throw std::string("Incorrect line of command");
      }
    }
  }

  millemiglia::Instance instance;
  instance.set_name(p.instance_name);
  InstanceGenerator iGenerator(p.instance_name, "", p.random_seed);
  iGenerator.generate_logistic_network(
      instance, p.hubs_number, p.graph_density, p.time_horizon, p.nbr_lines,
      p.max_length_line, p.max_numb_rotations_per_line, p.max_vehicle_capacity);
  iGenerator.generate_shipments(instance, p.shipment_number, p.time_horizon,
                                p.max_path_length);

  google::protobuf::TextFormat::Printer printer;
  string out;

  if (!printer.PrintToString(instance, &out)) {
    cerr << "Failed to write the instance file " << p.instance_name << "\n";
    exit(EXIT_FAILURE);
  }
  fstream output_instance(
      "./generated_graphs/" + p.instance_name + ".textproto", ios::out);
  output_instance << out;
  output_instance.close();
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}