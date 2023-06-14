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

#include <string>

#include "SpaceTimeNetwork.h"

#ifndef NDEBUG
#define ASSERT(condition, message)                                       \
  do {                                                                   \
    if (!(condition)) {                                                  \
      std::cerr << "Assertion `" #condition "` failed in " << __FILE__   \
                << " line " << __LINE__ << ": " << message << std::endl; \
      std::terminate();                                                  \
    }                                                                    \
  } while (false)
#else
#define ASSERT(condition, message) \
  do {                             \
  } while (false)
#endif

SpaceTimeNetwork::SpaceTimeNetwork() {
  VertexST::restart_id_counter();
  ArcST::restart_id_counter();
}

SpaceTimeNetwork::SpaceTimeNetwork(const string& network_file,
                                   const int& time_horizon) {
  VertexST::restart_id_counter();
  ArcST::restart_id_counter();
  parse_logistic_network(network_file);
  build_underlying_graph(this->network);
  build_vertices(time_horizon);
  build_arcs(this->network, time_horizon);
  build_lorries(this->network);
}

SpaceTimeNetwork::SpaceTimeNetwork(
    const millemiglia::LogisticsNetwork& network,
    const int& time_horizon) {
  VertexST::restart_id_counter();
  ArcST::restart_id_counter();
  build_underlying_graph(network);
  build_vertices(time_horizon);
  build_arcs(network, time_horizon);
}

void SpaceTimeNetwork::parse_logistic_network(const string& network_file) {
  const char* network_cc = network_file.c_str();
  if (!std::filesystem::exists({network_cc})) {
    std::cout << "File " << network_cc << " does not exist!\n";
    exit(EXIT_FAILURE);
  }

  fstream input_network(network_cc, ios::in);
  if (!input_network.is_open()) {
    std::cout << "Error while opening file " << network_cc
              << ", although the file exists!\n";
    exit(EXIT_FAILURE);
  }

  std::ostringstream ss_network;
  ss_network << input_network.rdbuf();
  if (!google::protobuf::TextFormat::ParseFromString(ss_network.str(),
                                                     &this->network)) {
    cerr << "Failed to parse instance from file " << network_cc << "\n";
    exit(EXIT_FAILURE);
  }
}

SpaceTimeNetwork::SpaceTimeNetwork(const SpaceTimeNetwork& spaceTimeNetwork) {
  this->underlying_graph = spaceTimeNetwork.underlying_graph;
  this->vertices = spaceTimeNetwork.vertices;
  this->arcs = spaceTimeNetwork.arcs;
  this->vertex_dictionary = spaceTimeNetwork.vertex_dictionary;
  this->lorries = spaceTimeNetwork.lorries;
  this->network = spaceTimeNetwork.network;
  this->adjacency_lists_arc_position =
      spaceTimeNetwork.adjacency_lists_arc_position;
}

SpaceTimeNetwork& SpaceTimeNetwork::operator=(
    const SpaceTimeNetwork& spaceTimeNetwork) {
  this->underlying_graph = spaceTimeNetwork.underlying_graph;
  this->vertices = spaceTimeNetwork.vertices;
  this->arcs = spaceTimeNetwork.arcs;
  this->vertex_dictionary = spaceTimeNetwork.vertex_dictionary;
  this->lorries = spaceTimeNetwork.lorries;
  this->network = spaceTimeNetwork.network;
  this->adjacency_lists_arc_position =
      spaceTimeNetwork.adjacency_lists_arc_position;
  return *this;
}

SpaceTimeNetwork::~SpaceTimeNetwork() {}

void SpaceTimeNetwork::build_underlying_graph(
    const millemiglia::LogisticsNetwork& network) {
  this->underlying_graph = Graph(network.hubs_size());
  // VERTICES
  for (auto hub : network.hubs()) {
    this->underlying_graph.add_vertex(hub.name());
  }
  // ARCS
  for (auto hub : network.hubs()) {
    for (auto line : network.lines()) {
      for (int i = 0; i < line.hub_ids().size() - 1; i++) {
        string start = line.hub_ids().at(i);
        string end = line.hub_ids().at(i + 1);
        if (start == hub.name()) {
          this->underlying_graph.add_neighbour(start, end, line.name());
        }
      }
    }
  }
}

void SpaceTimeNetwork::build_vertices(const int& time_horizon) {
  for (int i = 0; i < this->underlying_graph.get_vertices().size(); i++) {
    for (int t = 0; t <= time_horizon; t++) {
      add_vertexST(i, t);
    }
  }
}

void SpaceTimeNetwork::build_arcs(
    const millemiglia::LogisticsNetwork& network,
    const int& time_horizon) {
  // Travelling arcs
  for (const auto& line : network.lines()) {
    for (int i = 0; i < line.hub_ids().size() - 1; i++) {
      string departure_hub = line.hub_ids().at(i);
      string arrival_hub = line.hub_ids().at(i + 1);
      double cost = 0.0;
      for (const auto& rotations : line.next_rotations()) {
        millemiglia::DateTimeRange dt =
            rotations.departure_times().at(departure_hub);
        millemiglia::DateTimeRange at =
            rotations.arrival_times().at(arrival_hub);
        const VertexST& depVertex = get_vertex(departure_hub, dt);
        const VertexST& arrVertex = get_vertex(arrival_hub, at);
        int duration = arrVertex.get_time() - depVertex.get_time();
        if (i == 0) {
          cost = rotations.fixed_price();
        }
        ArcST arc = ArcST(depVertex.get_id(), arrVertex.get_id(), "travelling",
                          line.name(), rotations.name(), duration, cost);
        this->arcs.push_back(arc);
        add_to_adjacency_list_in(arc);
        add_to_adjacency_list_out(arc);
      }
    }
  }
  // Waiting arcs
  for (int i = 0; i < this->underlying_graph.get_vertices().size(); i++) {
    for (int t = 0; t <= time_horizon - 1; t++) {
      const VertexST& v1 = get_vertex(i, t);
      const VertexST& v2 = get_vertex(i, t + 1);
      ArcST arc = ArcST(v1.get_id(), v2.get_id(), "waiting", "", "", 1, 0.0);
      this->arcs.push_back(arc);
      add_to_adjacency_list_in(arc);
      add_to_adjacency_list_out(arc);
    }
  }
}

void SpaceTimeNetwork::build_lorries(
    const millemiglia::LogisticsNetwork& network) {
  for (const auto& vehicle : network.vehicles()) {
    Lorry lorry = Lorry(vehicle.name(), vehicle);
    this->lorries.push_back(lorry);
  }
}

void SpaceTimeNetwork::add_to_adjacency_list_out(const ArcST& arc) {
  this->vertices.at(arc.get_departure_id())
      .add_neighbour_out(arc.get_arrival_id(), arc.get_id());
}

void SpaceTimeNetwork::add_to_adjacency_list_in(const ArcST& arc) {
  this->vertices.at(arc.get_arrival_id())
      .add_neighbour_in(arc.get_departure_id(), arc.get_id());
}

const VertexST& SpaceTimeNetwork::get_vertex(
    const string& hub,
    const millemiglia::DateTimeRange& time) const {
  int graph_id = this->underlying_graph.get_vertex(hub).get_id();
  int encTime = ::time_encoder(time.first_date());
  const auto& finder_id = this->vertex_dictionary.find(graph_id);
  assert(finder_id != this->vertex_dictionary.end());
  const auto& finder_time = this->vertex_dictionary.at(graph_id).find(encTime);
  ASSERT(finder_time != this->vertex_dictionary.at(graph_id).end(),
         "encTime: " << encTime);
  assert(finder_time->second < (int)this->vertices.size());
  return this->vertices.at(finder_time->second);
}

const VertexST& SpaceTimeNetwork::get_vertex(const int& hub,
                                             const int& time) const {
  const auto& finder_id = this->vertex_dictionary.find(hub);
  assert(finder_id != this->vertex_dictionary.end());
  const auto& finder_time = this->vertex_dictionary.at(hub).find(time);
  assert(finder_time != this->vertex_dictionary.at(hub).end());
  assert(finder_time->second < this->vertices.size());
  return this->vertices.at(finder_time->second);
}

void SpaceTimeNetwork::add_vertexST(const int& hub, const int& time) {
  auto dict1 = this->vertex_dictionary.find(hub);
  bool isToInsert = false;
  if (dict1 == this->vertex_dictionary.end()) {
    isToInsert = true;
    unordered_map<int, int> aux;
    aux.insert(make_pair(time, this->vertices.size()));
    this->vertex_dictionary.insert(make_pair(hub, aux));
  } else if (dict1->second.find(time) == dict1->second.end()) {
    isToInsert = true;
    dict1->second.insert(make_pair(time, this->vertices.size()));
  }
  if (isToInsert) {
    VertexST vst = VertexST(hub, time);
    this->vertices.push_back(vst);
  }
}

void SpaceTimeNetwork::add_vertexST(
    const string& hub, const millemiglia::DateTimeRange& time,
    const int& time_horizon) {
  const Vertex& v = this->underlying_graph.get_vertex(hub);
  int encTime = ::time_encoder(time.first_date());
  assert(encTime <= time_horizon);
  add_vertexST(v.get_id(), encTime);
}

const millemiglia::LogisticsNetwork&
SpaceTimeNetwork::get_network() const {
  return this->network;
}

const Graph& SpaceTimeNetwork::get_underlying_graph() const {
  return this->underlying_graph;
}

const vector<VertexST>& SpaceTimeNetwork::get_vertices() const {
  return this->vertices;
}

const vector<ArcST>& SpaceTimeNetwork::get_arcs() const { return this->arcs; }

const string SpaceTimeNetwork::toString() const {
  string str = "VERTICES:\n";
  str.append("\tID\t(GRAPH_ID,TIME)\tTOP_POS\tIN_SIZE\tOUT_SIZE\n");
  for (int i = 0; i < this->vertices.size(); i++) {
    str.append("\t" + this->vertices.at(i).toString());
  }
  str.append("\nARCS:\n");
  str.append("\tID\t(DEP_ID,ARR_ID)\tTYPE\tTRAV_TIME\tCOST\t(LINE,ROT)\n");
  for (int i = 0; i < (int)this->arcs.size(); i++) {
    str.append("\t" + this->arcs.at(i).toString());
  }
  return str;
}
