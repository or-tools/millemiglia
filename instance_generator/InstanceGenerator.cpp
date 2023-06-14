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

#include "InstanceGenerator.h"

InstanceGenerator::InstanceGenerator() {}

InstanceGenerator::InstanceGenerator(const string& name,
                                     const string& description,
                                     const unsigned int& random_seed) {
  this->name = name;
  this->description = description;
  this->random_seed = random_seed;
}

InstanceGenerator::~InstanceGenerator() {}

InstanceGenerator::InstanceGenerator(
    const InstanceGenerator& instanceGenerator) {
  this->name = instanceGenerator.name;
  this->description = instanceGenerator.description;
  this->random_seed = instanceGenerator.random_seed;
}

InstanceGenerator& InstanceGenerator::operator=(
    const InstanceGenerator& instanceGenerator) {
  this->name = instanceGenerator.name;
  this->description = instanceGenerator.description;
  this->random_seed = instanceGenerator.random_seed;
  return *this;
}

void InstanceGenerator::generate_logistic_network(
    millemiglia::Instance& instance, const int& hubs_number,
    const double& graph_density, const int& time_horizon, const int& nbr_lines,
    const int& max_length_line, const int& max_numb_rotations_per_line,
    const double& max_vehicle_capacity) const {
  string network_name = "network_" + to_string(hubs_number);
  instance.mutable_network()->set_name(network_name);

  // generate the transportation graph by means of the Barabasi-Albert
  // algorithm.

  Graph random_graph = build_random_graph(hubs_number, graph_density);

  // Fill the hubs structure in the logistic network object.
  add_hubs(*instance.mutable_network(), random_graph);

  // vehicle frequencies depend on the importance of an edge(degree of sender +
  // degree of receiver).
  vector<double> arc_weights = build_arc_weights(random_graph);
  vector<vector<double>> arc_weights_adj_lists =
      build_arc_weights_per_adjacency_lists(random_graph, arc_weights);

  // fill line_rotation structure in the logistic network object
  int number_of_vehicles = generate_line_rotations(
      *instance.mutable_network(), random_graph, arc_weights,
      arc_weights_adj_lists, time_horizon, nbr_lines, max_length_line,
      max_numb_rotations_per_line);

  // Sample vehicle capacities
  add_vehicles(*instance.mutable_network(), number_of_vehicles,
               max_vehicle_capacity);
}

Graph InstanceGenerator::build_random_graph(const int& hubs_number,
                                            const double& graph_density) const {
  // Define variables to store graph elements
  set<std::pair<int, int>> edges;  // List of edges (source, target)
  vector<int> repeated_nodes;      // Keeps track of nodes used for connections

  // Calculate the number of connections per hub to achieve the graph density.
  int new_connections_per_node = graph_density * (hubs_number - 1);
  // Create initial connections from the first hub to all other nodes
  for (int i = 1; i <= new_connections_per_node; i++) {
    edges.insert({1, i + 1});
    repeated_nodes.push_back(1);      // Add first hub to repeated nodes
    repeated_nodes.push_back(i + 1);  // Add connected node to repeated nodes
  }
  // Loop through remaining hubs
  int source = 2;
  while (source <= hubs_number) {
    // Get a random subset of previously used nodes for connections
    vector<int> targets =
        random_subset(repeated_nodes, new_connections_per_node);
    // Create connections from the current hub to the random subset
    for (auto node : targets) {
      edges.insert({node, source});
      repeated_nodes.push_back(source);  // Add current hub to repeated nodes
      repeated_nodes.push_back(node);    // Add connected node to repeated nodes
    }
    source++;
  }
  // Create the graph object with specified number of hubs
  Graph random_graph(hubs_number);
  for (int i = 1; i <= hubs_number; i++) {
    string name = "h_" + to_string(i);
    random_graph.add_vertex(name);
  }

  // Add edges to the graph using vertex names
  for (auto edge : edges) {
    string id1 = "h_" + to_string(edge.first);
    string id2 = "h_" + to_string(edge.second);
    random_graph.add_neighbour(id1, id2, "");
  }

  return random_graph;
}

vector<double> InstanceGenerator::build_arc_weights(const Graph& graph) const {
  vector<double> arc_weights((int)graph.get_arcs().size(), 0.0);
  for (int i = 0; i < (int)graph.get_arcs().size(); i++) {
    const Vertex& v1 = graph.get_vertex(graph.get_arcs().at(i).first);
    const Vertex& v2 = graph.get_vertex(graph.get_arcs().at(i).second);
    arc_weights.at(i) =
        (v1.get_neighbours_number() + v2.get_neighbours_number());
  }
  return arc_weights;
}

vector<vector<double>> InstanceGenerator::build_arc_weights_per_adjacency_lists(
    const Graph& graph, const vector<double>& arc_weights) const {
  vector<vector<double>> weights_per_adj_lists(graph.get_vertex_number(),
                                               vector<double>());
  for (int i = 0; i < (int)graph.get_vertices().size(); i++) {
    const Vertex& v = graph.get_vertex(i);
    for (const auto& arc : v.get_adjacency_list_out()) {
      int arc_pos = graph.get_arc_position(v.get_id(), arc.first);
      assert(arc_pos >= 0 && arc_pos < (int)arc_weights.size());
      weights_per_adj_lists.at(v.get_id()).push_back(arc_weights.at(arc_pos));
    }
  }
  return weights_per_adj_lists;
}

void InstanceGenerator::add_hubs(millemiglia::LogisticsNetwork& network,
                                 const Graph& graph) const {
  for (int i = 0; i < graph.get_vertex_number(); i++) {
    const Vertex& v = graph.get_vertex(i);
    millemiglia::Hub h;
    h.set_name(v.get_name());
    network.mutable_hubs()->Add(move(h));
  }
}

// TODO(aymanelotfi): Should we allow the same element to be present more than
// one time in the same path?

vector<int> InstanceGenerator::generate_line(
    const Graph& graph, const int& max_length_line,
    const vector<vector<double>>& arc_weights_adj_lists,
    const vector<double>& arc_weights) const {
  // generate a path in the random_graph
  vector<int> path;
  unordered_set<int> path_set;
  // draw max length of the path
  int path_length = ElRandom::Uniform(1, max_length_line);

  // initialise path with first edge and randomly pick an orientation
  int edge = ElRandom::Discrete(arc_weights);
  int id1 = graph.get_arcs().at(edge).first;
  int id2 = graph.get_arcs().at(edge).second;
  path.push_back(id1);
  path.push_back(id2);

  path_set.insert(id1);
  path_set.insert(id2);

  // complete the line up to its length or until there exists an outgoing arc
  int last_vertex = path.back();
  bool is_path_augmented = true;
  while (is_path_augmented && (int)path.size() < path_length &&
         !graph.get_vertex(last_vertex).get_adjacency_list_out().empty()) {
    // we allow max tries to draw a next vertex not already in the path and
    // whose duration is feasible
    int tries = 0;
    int edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
    int new_vertex =
        graph.get_vertex(last_vertex).get_out_going_by_position(edge);
    is_path_augmented = false;
    do {
      if (path_set.find(new_vertex) == path_set.end()) {
        last_vertex = new_vertex;
        path.push_back(new_vertex);
        path_set.insert(new_vertex);
        is_path_augmented = true;
        break;
      } else {
        edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
        new_vertex =
            graph.get_vertex(last_vertex).get_out_going_by_position(edge);
      }

      tries++;
    } while (tries < (int)arc_weights_adj_lists.at(last_vertex).size());
  }
  return path;
}

int InstanceGenerator::generate_line_rotations(
    millemiglia::LogisticsNetwork& network, const Graph& graph,
    const vector<double>& arc_weights,
    const vector<vector<double>>& arc_weights_adj_lists,
    const int& time_horizon, const int& nbr_lines, const int& max_length_line,
    const int& max_numb_rotations_per_line) const {
  int vehicles_number = 0;
  // build lines
  for (int i = 0; i < nbr_lines; i++) {
    // generate a path in the random_graph
    vector<int> line = generate_line(graph, max_length_line,
                                     arc_weights_adj_lists, arc_weights);
    vector<vector<int>> time_infos;

    // add rotations associated to the line
    int max_rotations = ElRandom::Uniform(1, max_numb_rotations_per_line);
    int nbr_hubs = (int)line.size();
    for (int j = 0; j < max_rotations; j++) {
      millemiglia::LineRotation rotation;
      string rotation_name = "l" + to_string(i + 1) + "_lr" + to_string(j + 1);
      vector<int> time_info;
      // sample the start time of the rotation
      int start_time = ElRandom::Uniform(1, time_horizon - nbr_hubs + 1);
      time_info.push_back(start_time);
      // max_duration defined to make sure we can sample nbr_hubs times without
      // exceeding the time horizon.
      assert(nbr_hubs > 1);
      int max_duration = (time_horizon - start_time) / (nbr_hubs - 1);
      for (int k = 2; k <= nbr_hubs; k++) {
        int duration = ElRandom::Uniform(1, max_duration);
        start_time += duration;
        assert(start_time <= time_horizon);
        time_info.push_back(start_time);
      }
      time_infos.push_back(time_info);
      vehicles_number++;
    }
    // fill rotation proto
    add_line_rotation(network, graph, i + 1, line, time_infos);
  }
  return vehicles_number;
}

void InstanceGenerator::add_line_rotation(
    millemiglia::LogisticsNetwork& network, const Graph& graph,
    const int& line_nbr, const vector<int>& line,
    const vector<vector<int>>& rotations) const {
  // build line
  millemiglia::Line line_proto;
  for (int i = 0; i < (int)line.size(); i++) {
    string name = graph.get_vertex(line.at(i)).get_name();
    line_proto.add_hub_ids(name);
  }

  // add rotations to line
  for (int i = 0; i < (int)rotations.size(); i++) {
    millemiglia::LineRotation rotation;
    string name = "l" + to_string(line_nbr) + "_lr" + to_string(i + 1);

    for (int j = 0; j + 1 < (int)rotations.at(i).size(); j++) {
      string start_hub = graph.get_vertex(line.at(j)).get_name();
      string end_hub = graph.get_vertex(line.at(j + 1)).get_name();
      int start_time = rotations.at(i).at(j);
      int end_time = rotations.at(i).at(j + 1);
      assert(start_time < end_time);

      // add departure
      millemiglia::DateTimeRange departure_time_range;
      google::type::DateTime time_departure = ::time_decoder(start_time);
      add_time_range(departure_time_range, time_departure);
      rotation.mutable_departure_times()->insert(
          {start_hub, departure_time_range});

      // add arrival
      millemiglia::DateTimeRange arrival_time_range;
      google::type::DateTime time_arrival = ::time_decoder(end_time);
      add_time_range(arrival_time_range, time_arrival);
      rotation.mutable_arrival_times()->insert({end_hub, arrival_time_range});
    }

    // add cost + number of vehicles
    int price = rotations.at(i).back() - rotations.at(i).front();
    assert(price > 0);
    rotation.set_name(name);
    rotation.set_fixed_price(static_cast<double>(price));
    rotation.set_maximum_number_vehicles(1);

    // add rotation to line
    line_proto.mutable_next_rotations()->Add(move(rotation));
  }
  line_proto.set_name("l" + to_string(line_nbr));
  network.mutable_lines()->Add(move(line_proto));
}

void InstanceGenerator::add_distance_matrix_entry(
    millemiglia::LogisticsNetwork& network, const string& x,
    const string& y) const {
  millemiglia::DistanceMatrixEntry entry;
  entry.set_source_hub(x);
  entry.set_destination_hub(y);
  millemiglia::ValueDimension distance_value,
      time_value;  // NOT CLEAR AT ALL
  distance_value.set_dimension("distance");
  distance_value.set_value(1);
  time_value.set_dimension("time");
  time_value.set_value(1);
  entry.mutable_weights()->Add(move(distance_value));
  entry.mutable_weights()->Add(move(time_value));
  network.mutable_distance_matrix()->Add(move(entry));
}

void InstanceGenerator::add_time_range(
    millemiglia::DateTimeRange& range,
    const google::type::DateTime& time) const {
  range.mutable_first_date()->set_year(time.year());
  range.mutable_last_date()->set_year(time.year());
  range.mutable_first_date()->set_month(time.month());
  range.mutable_last_date()->set_month(time.month());
  range.mutable_first_date()->set_day(time.day());
  range.mutable_last_date()->set_day(time.day());
  range.mutable_first_date()->set_hours(time.hours());
  range.mutable_last_date()->set_hours(time.hours());
  range.mutable_first_date()->set_minutes(time.minutes());
  range.mutable_last_date()->set_minutes(time.minutes());
}

void InstanceGenerator::add_vehicles(millemiglia::LogisticsNetwork& network,
                                     const int& vehicle_number,
                                     const int& max_vehicle_capacity) const {
  // build vehicle objects and add them to the logistic network
  for (int l = 0; l < vehicle_number; l++) {
    millemiglia::Vehicle v;
    millemiglia::ValueDimension value;
    int capacity = max_vehicle_capacity * ElRandom::Uniform(0.0, 1.0);
    value.set_dimension("weight");
    value.set_value(capacity);
    v.mutable_capacities()->Add(move(value));
    v.set_cost(capacity);
    v.set_name("v" + to_string(l + 1));
    network.mutable_vehicles()->Add(move(v));
  }
}

void InstanceGenerator::generate_shipments(
    millemiglia::Instance& instance, const int& shipment_number,
    const int& time_horizon, const int& max_path_length,
    const int& min_shipment_weight, const int& max_shipment_weight,
    const double& shipment_weight_shape) const {
  SpaceTimeNetwork st_network =
      SpaceTimeNetwork(instance.network(), time_horizon);

  // Sample shipments' weight
  vector<int> shipment_weights(shipment_number, 0);
  for (int j = 0; j < shipment_number; j++) {
    while (::strictly_less(shipment_weights.at(j), min_shipment_weight) ||
           ::strictly_greater(shipment_weights.at(j), max_shipment_weight)) {
      double lomax_draw = ElRandom::Lomax(
          static_cast<double>(min_shipment_weight), shipment_weight_shape);
      shipment_weights.at(j) = max(1, static_cast<int>(floor(lomax_draw)));
    }
  }
  sort(shipment_weights.rbegin(), shipment_weights.rend());

  // Compute weights to assign to the hubs which are used to sample starting
  // hubs for the shipments
  vector<double> hubs_weights(
      st_network.get_underlying_graph().get_vertex_number(),
      st_network.get_underlying_graph().get_vertex_number() + 1);
  for (const auto& vertex : st_network.get_underlying_graph().get_vertices()) {
    hubs_weights.at(vertex.get_id()) -=
        static_cast<double>(vertex.get_neighbours_number());
  }

  // Compute weights to assign to the arcs ((h,t),(h',t')) in the space-time
  // graph with h!=h' to sample a starting arc for the shipments
  vector<int> travelling_arcs;
  vector<double> travelling_arcs_weights;
  for (int i = 0; i < (int)st_network.get_arcs().size(); i++) {
    const ArcST& arc = st_network.get_arcs().at(i);
    int departure_hub =
        st_network.get_vertices().at(arc.get_departure_id()).get_id_in_graph();
    int arrival_hub =
        st_network.get_vertices().at(arc.get_arrival_id()).get_id_in_graph();
    if (departure_hub != arrival_hub) {
      travelling_arcs.push_back(i);
      travelling_arcs_weights.push_back(hubs_weights.at(departure_hub));
    }
  }

  // Build shipments
  int number = 0;
  for (int k = 0; k < shipment_number; k++) {
    int source_hub_id = -1;
    int destination_hub_id = -1;
    int departure_time = -1;
    int arrival_time = -1;

    // Sample starting arc in the space-time graph
    int start_arc_pos = ElRandom::Discrete(travelling_arcs_weights);
    const ArcST& arc =
        st_network.get_arcs().at(travelling_arcs.at(start_arc_pos));
    source_hub_id =
        st_network.get_vertices().at(arc.get_departure_id()).get_id_in_graph();
    departure_time =
        st_network.get_vertices().at(arc.get_departure_id()).get_time();
    destination_hub_id =
        st_network.get_vertices().at(arc.get_arrival_id()).get_id_in_graph();
    arrival_time =
        st_network.get_vertices().at(arc.get_arrival_id()).get_time();
    int last_vertex = arc.get_arrival_id();
    assert(source_hub_id!=destination_hub_id);

    // Sample a path in the st_network starting at the (source hub,
    // departure_time)
    int length = 2;
    while (true) {
      const VertexST& v = st_network.get_vertices().at(last_vertex);
      if ((int)v.get_adjacency_list_out().size() == 0) {
        break;
      }

      if (length >= max_path_length) {
        break;
      }

      int size = (int)v.get_adjacency_list_out().size() - 1;
      int pos = ElRandom::Uniform(0, size);
      unordered_map<int, vector<int>>::const_iterator finder =
          v.get_adjacency_list_out().begin();
      advance(finder, pos);
      const VertexST& next_vertex = st_network.get_vertices().at(finder->first);
      last_vertex = next_vertex.get_id();

      if (v.get_id_in_graph() != next_vertex.get_id_in_graph()) {
        length++;
      }

      // Cut the path?
      double prob_stop =
          1.0 / (static_cast<double>(max_path_length - length + 1));
      if (next_vertex.get_id_in_graph() != source_hub_id) {
        destination_hub_id = next_vertex.get_id_in_graph();
        arrival_time = next_vertex.get_time();
        assert(source_hub_id != destination_hub_id);
      }
      if (ElRandom::Bernoulli(prob_stop)) {
        break;
      }
    }
    number++;
    assert(source_hub_id!=destination_hub_id);
    string destination_name = st_network.get_underlying_graph()
                                  .get_vertex(destination_hub_id)
                                  .get_name();
    string source_name =
        st_network.get_underlying_graph().get_vertex(source_hub_id).get_name();
    assert(destination_name != source_name);
    add_shipment(instance, number, source_name, destination_name,
                 departure_time, arrival_time, shipment_weights.at(k));
  }
}

void InstanceGenerator::add_shipment(millemiglia::Instance& instance,
                                     const int& shipment_number,
                                     const string& source_hub,
                                     const string& destination_hub,
                                     const int& departure_time,
                                     const int& arrival_time,
                                     const int& weight) const {
  millemiglia::Shipment shipment;
  shipment.set_name("s_" + to_string(shipment_number));
  assert(source_hub != destination_hub);
  shipment.set_destination_hub(destination_hub);
  shipment.set_source_hub(source_hub);
  millemiglia::ValueDimension value;
  value.set_dimension("weight");
  value.set_value(weight);
  shipment.mutable_size()->Add(move(value));
  google::type::DateTime time_departure = ::time_decoder(departure_time);
  add_departure_time(shipment, time_departure);
  millemiglia::DateTimeRange arrival_time_range;
  google::type::DateTime time_arrival = ::time_decoder(arrival_time);
  add_arrival_time(shipment, time_arrival);
  instance.mutable_shipments()->Add(move(shipment));
}

void InstanceGenerator::add_departure_time(
    millemiglia::Shipment& shipment,
    const google::type::DateTime& time_departure) const {
  shipment.mutable_departure_time()->set_year(time_departure.year());
  shipment.mutable_departure_time()->set_month(time_departure.month());
  shipment.mutable_departure_time()->set_day(time_departure.day());
  shipment.mutable_departure_time()->set_hours(time_departure.hours());
  shipment.mutable_departure_time()->set_minutes(time_departure.minutes());
  shipment.mutable_departure_time()->set_seconds(time_departure.seconds());
}

void InstanceGenerator::add_arrival_time(
    millemiglia::Shipment& shipment,
    const google::type::DateTime& time_arrival) const {
  shipment.mutable_arrival_time()->mutable_first_date()->set_year(
      time_arrival.year());
  shipment.mutable_arrival_time()->mutable_first_date()->set_month(
      time_arrival.month());
  shipment.mutable_arrival_time()->mutable_first_date()->set_day(
      time_arrival.day());
  shipment.mutable_arrival_time()->mutable_first_date()->set_hours(
      time_arrival.hours());
  shipment.mutable_arrival_time()->mutable_first_date()->set_minutes(
      time_arrival.minutes());
  shipment.mutable_arrival_time()->mutable_first_date()->set_seconds(
      time_arrival.seconds());
  shipment.mutable_arrival_time()->mutable_last_date()->set_year(
      time_arrival.year());
  shipment.mutable_arrival_time()->mutable_last_date()->set_month(
      time_arrival.month());
  shipment.mutable_arrival_time()->mutable_last_date()->set_day(
      time_arrival.day());
  shipment.mutable_arrival_time()->mutable_last_date()->set_hours(
      time_arrival.hours());
  shipment.mutable_arrival_time()->mutable_last_date()->set_minutes(
      time_arrival.minutes());
  shipment.mutable_arrival_time()->mutable_last_date()->set_seconds(
      time_arrival.seconds());
}