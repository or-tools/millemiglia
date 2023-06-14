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
  this->vertices = vector<Vertex>();
  this->arcs_dictionary = vector<unordered_map<int, int>>();
  Vertex::restart_id_counter();
}

Graph::Graph(const int& hub_number) {
  this->vertices = vector<Vertex>(hub_number);
  this->arcs_dictionary = vector<unordered_map<int, int>>(hub_number);
  Vertex::restart_id_counter();
}

Graph::~Graph() {
  this->vertices.clear();
  Vertex::restart_id_counter();
}

Graph::Graph(const Graph& graph) {
  this->vertices = graph.vertices;
  this->vertex_dictionary = graph.vertex_dictionary;
  this->arcs = graph.arcs;
  this->arcs_dictionary = graph.arcs_dictionary;
}

Graph& Graph::operator=(const Graph& graph) {
  this->vertices = graph.vertices;
  this->vertex_dictionary = graph.vertex_dictionary;
  this->arcs = graph.arcs;
  this->arcs_dictionary = graph.arcs_dictionary;
  return *this;
}

void Graph::add_vertex(const string& name) {
  Vertex v = Vertex(name);
  assert((int)this->vertices.size() > v.get_id());
  this->vertices.at(v.get_id()) = v;
  this->vertex_dictionary.insert(make_pair(name, v.get_id()));
}

void Graph::add_neighbour(const string& key, const string& neighbour,
                          const string& line) {
  assert(this->vertex_dictionary.count(key) > 0);  // Assert key exists
  int key_id = this->vertex_dictionary.at(key);

  assert(this->vertex_dictionary.count(neighbour) >
         0);  // Assert neighbour exists
  int neighbour_id = this->vertex_dictionary.at(neighbour);

  this->vertices.at(key_id).add_neighbour_out(neighbour_id, line);
  this->vertices.at(neighbour_id).add_neighbour_in(key_id, line);

  this->arcs_dictionary.at(key_id).insert(
      make_pair(neighbour_id, (int)this->arcs.size()));
  this->arcs_dictionary.at(neighbour_id)
      .insert(make_pair(key_id, (int)this->arcs.size()));

  this->arcs.push_back(make_pair(key_id, neighbour_id));
}

const vector<Vertex>& Graph::get_vertices() const { return this->vertices; }

const int Graph::get_vertex_number() const { return (int)this->vertices.size(); }

const Vertex& Graph::get_vertex(const int& id) const {
  assert(id < (int)this->vertices.size());
  return this->vertices.at(id);
}

const Vertex& Graph::get_vertex(const string& name) const {
  unordered_map<string, int>::const_iterator finder =
      this->vertex_dictionary.find(name);
  assert(finder != this->vertex_dictionary.end());
  return this->vertices.at(finder->second);
}

const vector<pair<int, int>>& Graph::get_arcs() const { return this->arcs; }

const int Graph::get_number_of_arcs() const { return this->arcs.size(); }

const int Graph::get_arc_position(const int& id1, const int& id2) const {
  unordered_map<int, int>::const_iterator finder =
      this->arcs_dictionary.at(id1).find(id2);
  assert(finder != this->arcs_dictionary.at(id1).end());
  return finder->second;
}

const string Graph::toString() const {
  string str = "VERTICES:\n";
  str.append("\tID\tNAME\tIN_SIZE\tOUT_SIZE\n");
  for (int i = 0; i < (int)this->vertices.size(); i++) {
    str.append("\t" + this->vertices.at(i).toString());
  }
  str.append("ARC NUMBER:\t" + to_string(this->arcs.size()) + "\n");
  return str;
}