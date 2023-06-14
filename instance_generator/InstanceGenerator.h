// Copyright 2023-2024 Google LLC
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INSTANCEGENERATOR_H
#define INSTANCEGENERATOR_H

#include "../graph_utils/Header.h"
#include "../graph_utils/SpaceTimeNetwork.h"
#include "../graph_utils/Utils.h"


/**
 * InstanceGenerator
 * generate .textproto for the logisticsNetwork and
 * logisticsNetworkState.
 */

class InstanceGenerator {
 protected:
  /**
   *	name of the .textproto file
   */
  string name;
  /**
   * description contains info regarding the instance file that is generated.
   * Its content is print in a name_info.txt file
   */
  string description;

  /**
   * random_seed to use in the randomized steps
   */
  unsigned int random_seed;

 public:
  /**
   * \brief Constructor by default.
   */
  InstanceGenerator();
  /**
   * \brief Constructor by data. The call to the constructor
   */
  InstanceGenerator(const string& name, const string& description,
                    const unsigned int& random_seed);
  /**
   * \brief Copy constructor.
   *
   */
  InstanceGenerator(const InstanceGenerator& instanceGenerator);
  /**
   *	\brief	Operator assignment
   */
  InstanceGenerator& operator=(const InstanceGenerator& instanceGenerator);

  /**
   * \brief Destructor.
   *
   */
  ~InstanceGenerator();

  /**
   * build the logistic network and print the .textproto
   *	\param instance							instance object of the protobuf
   *	\param hubs_number 						number
   *of hubs
   *	\param graph_density				target density
   *of the space graph
   *	\param time_horizon time_horizon
   *	\param nbr_lines						number
   *of lines, i.e., number of paths in the space graph
   *	\param max_length_line					maximal lenght
   *of a line
   *	\param max_numb_rotations_per_line		maximal number of
   *rotations per line, i.e., given a line it is the number of paths in the
   *space-time graph whose projection onto the space-graph is the line
   *	\param max_travelling_time				maximal
   *travelling time between two hubs
   *	\param travelling_time_perturbation		perturbation 0<p<1 for
   *the travelling time t; the resulting travelling time is t'=t(1+-p). Default
   *p=0
   *	\param max_vehicle_capacity				maximal vehicle
   *capacity. default is 100
   *
   * 1) Randomly generate a (space) graph with hubs_number vertices and whose
   *density is graph_density via the extended Barabàsi-Albert algorithm
   *(https://www.degruyter.com/document/doi/10.1515/9781400841356/html). This
   *results in a few highly connected (large degree), while most depots only
   *have a few connections.
   *
   * 2) Randomly generate nbr_lines paths in the space
   *graph with length at most max_length_line. The arcs of a path are sampled
   *with discrete_distribution where the weights of (i,j) is deg(i)+deg(j) 
   *
   * 3) Sample the travelling times of the arcs with uniform distribution in
   *[1,max_travelling_time]
   *  
   * 4) Build the rotation for each line. Start-time is
   *randomly sampled with uniform distribution in [0,time_horizon[ Then,
   *durations are assigned to the arcs composing the line. A positive/negative
   *perturbation is applied with probability 0.5, the perturbatino is sampled
   *with uniform distribution in [0,travelling_time_perturbation]. 5) Randomly
   *generate a vehicle for each line-rotation. Vehicle capacity is
   *max_vehicle_capacity*f, where f is sampled with uniform distribution in
   *[0,1]
   */
  void generate_logistic_network(
      millemiglia::Instance& instance, const int& hubs_number,
      const double& graph_density, const int& time_horizon,
      const int& nbr_lines, const int& max_length_line,
      const int& max_numb_rotations_per_line,
      const double& max_vehicle_capacity = 100.0) const;

  /*
   * \brief build the random graph according to the extended BarabAsi-Albert
   * algorithm (see 1 in generate_logistic_network) The density of the built
   * graph is smaller than the given graph_density by at most 2/(hubs_numer -1).
   * \param hubs_number
   * \param graph_density
   * \return graph data structure
   */
  Graph build_random_graph(const int& hubs_number,
                           const double& graph_density) const;
  /*
   * \brief build the weights on the arcs of the graph to sample vehicles based
   * on the extremes degree: W_ij = (deg(i)+deg(j))
   *
   * \param random graph
   * \return weights
   */
  vector<double> build_arc_weights(const Graph& graph) const;
  vector<vector<double>> build_arc_weights_per_adjacency_lists(
      const Graph& graph, const vector<double>& arc_weights) const;

  /*
   * \brief add the hubs to the network
   *
   * \param network
   * \param graph
   */
  void add_hubs(millemiglia::LogisticsNetwork& network,
                const Graph& graph) const;
  /*
   * sample the line-rotations (see 4 in generate_logistic_network)
   *
   * \param network
   * \param graph
   * \param arc_weights
   * \param arc_weights_adj_lists
   * \param time_horizon
   * \param nbr_lines
   * \param max_length_line
   * \param max_numb_rotations_per_line
   * \param max_travelling_time
   * \param travelling_time_perturbation
   *
   * \return vehicle number
   */
  int generate_line_rotations(
      millemiglia::LogisticsNetwork& network,
      const Graph& graph, const vector<double>& arc_weights,
      const vector<vector<double>>& arc_weights_adj_lists,
      const int& time_horizon, const int& nbr_lines, const int& max_length_line,
      const int& max_numb_rotations_per_line) const;
  /*
   * sample the lines (see 2 in generate_logistic_network)
   *
   * \param graph
   * \param max_length_line
   * \param arc_weights
   * \param arc_weights_adj_lists
   * \return vehicle number
   */
  vector<int> generate_line(const Graph& graph, const int& max_length_line,
                            const vector<vector<double>>& arc_weights_adj_lists,
                            const vector<double>& arc_weights) const;
  /*
   * add the line rotations to the logistic network object
   *
   * \param network
   * \param graph
   * \param nbr_lines
   * \param line
   * \param rotations
   *
   */
  void add_line_rotation(millemiglia::LogisticsNetwork& network,
                         const Graph& graph, const int& line_nbr,
                         const vector<int>& line,
                         const vector<vector<int>>& rotations) const;
  /*
   * add the entry (x,y) in the distance matrix: NOT USED AS PER NOW
   *
   * \param network
   * \param x
   * \param y
   *
   */
  void add_distance_matrix_entry(
      millemiglia::LogisticsNetwork& network, const string& x,
      const string& y) const;
  /*
   * fill object millemiglia::DateTimeRange given
   * google::type::DateTime
   *
   * \param range
   * \param time
   *
   */
  void add_time_range(millemiglia::DateTimeRange& range,
                      const google::type::DateTime& time) const;
  /*
   * sample vehicle and add them to the network (see 5 in
   * generate_logistic_network)
   *
   * \param range
   * \param time
   *
   */
  void add_vehicles(millemiglia::LogisticsNetwork& network,
                    const int& vehicle_number,
                    const int& max_vehicle_capacity) const;

  /**
   *   build the logistic network and print the .textproto
   *
   *	\param instance						instance object
   *of the protobuf
   *	\param shipment_number 				number of shipments
   *	\param time_horizon					time_horizon
   *	\param max_path_length				maximal lenght of a path
   *(in terms of hubs) that a shipment can do to be delivered
   *	\param min_shipment_weight			minimal weight of a
   *shipment
   *	\param max_shipment_weight			maximal weight of a
   *shipment
   *	\param shipment_weight_shape		parameter to sample shipment
   *weights with Lomax distribution
   *  \details  
   *	1) Build the space-time network based on the lines and rotations
   *
   *	2) Sample the shipment weights based on the Lomax distribution
   *
   *	3) Build the hubs weights. Hub h weight is = number of vertices is the space graph - neighbours of h. 
   *
   *    4) Sample the shipments paths. The first
   *       vertex (h,t) is sampled as hub h with discrete distribution with weights in
   *       3 to prefer less connected hubs as starting point) and t with uniform
   *       distribution in [0,time_horizon[. We sample the next vertex from the
   *       neighbours of (h,t) and so on.. At each step, a path may be interrupted with
   *       probability 1/(max_path_length - current_path_length + 1).
   */
  void generate_shipments(millemiglia::Instance& instance,
                          const int& shipment_number, const int& time_horizon,
                          const int& max_path_length,
                          const int& min_shipment_weight = 1,
                          const int& max_shipment_weight = 100,
                          const double& shipment_weight_shape = 0.1) const;
  /**
    \brief
   * fill the shipment object and add it to the instance object
   *
   * \param instance
   * \param shipment_number
   * \param source_hub
   * \param destination_hub
   * \param departure_time
   * \param arrival_time
   * \param weight
   *
   */
  void add_shipment(millemiglia::Instance& instance,
                    const int& shipment_number, const string& source_hub,
                    const string& destination_hub, const int& departure_time,
                    const int& arrival_time, const int& weight) const;
  /*
   * fill the departure time (google::type::DateTime) of the shipment
   *
   * \param shipment
   * \param time_departure
   *
   */
  void add_departure_time(millemiglia::Shipment& shipment,
                          const google::type::DateTime& time_departure) const;
  /**
   * fill the arrival time (millemiglia::DateTimeRange) of the
   * shipment given the time_arrival object
   *
   * \param shipment
   * \param time_arrival
   *
   */
  void add_arrival_time(millemiglia::Shipment& shipment,
                        const google::type::DateTime& time_arrival) const;
};

#endif  // !INSTANCEGENERATOR_H