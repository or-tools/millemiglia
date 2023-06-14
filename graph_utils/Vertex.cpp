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

#include "Vertex.h"

int Vertex::lastId;

Vertex::Vertex() {
  this->id = this->lastId;
  this->name = "Vertex_" + to_string(this->lastId);
  this->lastId++;
}

Vertex::Vertex(const string& name) {
  this->id = this->lastId;
  this->name = name;
  this->lastId++;
}

Vertex::~Vertex() {}

Vertex::Vertex(const Vertex& vertex) {
  this->id = vertex.id;
  this->name = vertex.name;
  this->adjacency_list_out = vertex.adjacency_list_out;
  this->adjacency_list_in = vertex.adjacency_list_in;
}

Vertex& Vertex::operator=(const Vertex& vertex) {
  this->id = vertex.id;
  this->name = vertex.name;
  this->adjacency_list_out = vertex.adjacency_list_out;
  this->adjacency_list_in = vertex.adjacency_list_in;
  return *this;
}

void Vertex::add_neighbour_out(const int& id, const string& line) {
  unordered_map<int, vector<string>>::iterator finder =
      this->adjacency_list_out.find(id);
  if (finder == this->adjacency_list_out.end()) {
    vector<string> aux = {line};
    this->adjacency_list_out.insert(make_pair(id, aux));
  } else {
    finder->second.push_back(line);
  }
}

void Vertex::add_neighbour_in(const int& id, const string& line) {
  unordered_map<int, vector<string>>::iterator finder =
      this->adjacency_list_in.find(id);
  if (finder == this->adjacency_list_in.end()) {
    vector<string> aux = {line};
    this->adjacency_list_in.insert(make_pair(id, aux));
  } else {
    finder->second.push_back(line);
  }
}

bool Vertex::is_neighbour_out(const int& id) const {
  return (this->adjacency_list_out.find(id) != this->adjacency_list_out.end());
}

bool Vertex::is_neighbour_in(const int& id) const {
  return (this->adjacency_list_in.find(id) != this->adjacency_list_in.end());
}

const int& Vertex::get_id() const { return this->id; }

const string& Vertex::get_name() const { return this->name; }

const unordered_map<int, vector<string>>& Vertex::get_adjacency_list_out()
    const {
  return this->adjacency_list_out;
}

const unordered_map<int, vector<string>>& Vertex::get_adjacency_list_in()
    const {
  return this->adjacency_list_in;
}

const int Vertex::get_out_going_by_position(const int& pos) const {
  assert(pos < (int)this->adjacency_list_out.size());
  unordered_map<int, vector<string>>::const_iterator it =
      this->adjacency_list_out.begin();
  advance(it, pos);
  return it->first;
}

const int Vertex::get_neighbours_number() const {
  return (int)this->adjacency_list_in.size() + (int)this->adjacency_list_out.size();
}

const vector<string> Vertex::get_lines_out(const int& id) const {
  if (is_neighbour_out(id)) {
    return this->adjacency_list_out.at(id);
  }
  return {};
}

const vector<string> Vertex::get_lines_in(const int& id) const {
  if (is_neighbour_in(id)) {
    return this->adjacency_list_in.at(id);
  }
  return {};
}

const string Vertex::toString() const {
  string str;
  str.append(to_string(this->id) + "\t" + this->name + "\t" +
             to_string((int)this->adjacency_list_in.size()) + "\t" +
             to_string((int)this->adjacency_list_out.size()) + "\n");
  return str;
}
