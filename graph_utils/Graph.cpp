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

#include "Graph.h"

Graph::Graph() {
  vertices = std::vector<Vertex>();
  arcs_dictionary = std::vector<std::unordered_map<int, int>>();
  Vertex::restart_id_counter();
}

Graph::Graph(const int& hub_number) {
  vertices = std::vector<Vertex>(hub_number);
  arcs_dictionary = std::vector<std::unordered_map<int, int>>(hub_number);
  Vertex::restart_id_counter();
}

Graph::~Graph() {
  vertices.clear();
  Vertex::restart_id_counter();
}

Graph::Graph(const Graph& graph) {
  vertices = graph.vertices;
  vertex_dictionary = graph.vertex_dictionary;
  arcs = graph.arcs;
  arcs_dictionary = graph.arcs_dictionary;
}

Graph& Graph::operator=(const Graph& graph) {
  vertices = graph.vertices;
  vertex_dictionary = graph.vertex_dictionary;
  arcs = graph.arcs;
  arcs_dictionary = graph.arcs_dictionary;
  return *this;
}

void Graph::add_vertex(const std::string& name) {
  Vertex v = Vertex(name);
  assert((int)vertices.size() > v.get_id());
  vertices.at(v.get_id()) = v;
  vertex_dictionary.insert(std::make_pair(name, v.get_id()));
}

void Graph::add_neighbour(const std::string& key, const std::string& neighbour,
                          const std::string& line) {
  assert(vertex_dictionary.count(key) > 0);  // Assert key exists
  int key_id = vertex_dictionary.at(key);

  assert(vertex_dictionary.count(neighbour) >
         0);  // Assert neighbour exists
  int neighbour_id = vertex_dictionary.at(neighbour);

  vertices.at(key_id).add_neighbour_out(neighbour_id, line);
  vertices.at(neighbour_id).add_neighbour_in(key_id, line);

  arcs_dictionary.at(key_id).insert(
      std::make_pair(neighbour_id, (int)arcs.size()));
  arcs_dictionary.at(neighbour_id)
      .insert(std::make_pair(key_id, (int)arcs.size()));

  arcs.push_back(std::make_pair(key_id, neighbour_id));
}

const std::vector<Vertex>& Graph::get_vertices() const { return vertices; }

int Graph::get_vertex_number() const { return (int)vertices.size(); }

const Vertex& Graph::get_vertex(const int id) const {
  assert(id < (int)vertices.size());
  return vertices.at(id);
}

const Vertex& Graph::get_vertex(const std::string& name) const {
  auto finder = vertex_dictionary.find(name);
  assert(finder != vertex_dictionary.end());
  return vertices.at(finder->second);
}

const std::vector<std::pair<int, int>>& Graph::get_arcs() const { return arcs; }

int Graph::get_number_of_arcs() const { return arcs.size(); }

int Graph::get_arc_position(const int id1, const int id2) const {
  auto finder = arcs_dictionary.at(id1).find(id2);
  assert(finder != arcs_dictionary.at(id1).end());
  return finder->second;
}

const std::string Graph::toString() const {
  std::string str = "VERTICES:\n\tID\tNAME\tIN_SIZE\tOUT_SIZE\n";
  for (const auto & vertice : vertices) {
    str.append("\t" + vertice.toString());
  }
  str.append("ARC NUMBER:\t" + std::to_string(arcs.size()) + "\n");
  return str;
}