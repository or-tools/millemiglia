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
  id = lastId;
  name = "Vertex_" + std::to_string(lastId);
  lastId++;
}

Vertex::Vertex(const std::string& name) {
  id = lastId;
  this->name = name;
  lastId++;
}

Vertex::~Vertex() = default;

Vertex::Vertex(const Vertex& vertex) {
  this->id = vertex.id;
  this->name = vertex.name;
  this->adjacency_list_out = vertex.adjacency_list_out;
  this->adjacency_list_in = vertex.adjacency_list_in;
}

Vertex& Vertex::operator=(const Vertex& vertex) = default;

void Vertex::add_neighbour_out(const int& id, const std::string& line) {
  auto finder = adjacency_list_out.find(id);
  if (finder == adjacency_list_out.end()) {
    std::vector aux = {line};
    adjacency_list_out.insert(std::make_pair(id, aux));
  } else {
    finder->second.push_back(line);
  }
}

void Vertex::add_neighbour_in(const int& id, const std::string& line) {
  auto finder = adjacency_list_in.find(id);
  if (finder == adjacency_list_in.end()) {
    std::vector aux = {line};
    adjacency_list_in.insert(std::make_pair(id, aux));
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

const std::string& Vertex::get_name() const { return this->name; }

const std::unordered_map<int, std::vector<std::string>>& Vertex::get_adjacency_list_out()
    const {
  return this->adjacency_list_out;
}

const std::unordered_map<int, std::vector<std::string>>& Vertex::get_adjacency_list_in()
    const {
  return this->adjacency_list_in;
}

const int Vertex::get_out_going_by_position(const int pos) const {
  assert(pos < (int)this->adjacency_list_out.size());
  std::unordered_map<int, std::vector<std::string>>::const_iterator it =
      this->adjacency_list_out.begin();
  std::advance(it, pos);
  return it->first;
}

const int Vertex::get_neighbours_number() const {
  return (int)this->adjacency_list_in.size() + (int)this->adjacency_list_out.size();
}

const std::vector<std::string> Vertex::get_lines_out(const int id) const {
  if (is_neighbour_out(id)) {
    return this->adjacency_list_out.at(id);
  }
  return {};
}

const std::vector<std::string> Vertex::get_lines_in(const int id) const {
  if (is_neighbour_in(id)) {
    return adjacency_list_in.at(id);
  }
  return {};
}

const std::string Vertex::toString() const {
  std::string str;
  str.append(std::to_string(this->id) + "\t" + this->name + "\t" +
             std::to_string((int)this->adjacency_list_in.size()) + "\t" +
             std::to_string((int)this->adjacency_list_out.size()) + "\n");
  return str;
}
