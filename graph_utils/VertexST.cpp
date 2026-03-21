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

#include "VertexST.h"

int VertexST::lastId;

VertexST::VertexST() {
  this->id = -1;
  this->id_in_graph = -1;
  this->time = -1;
}

VertexST::VertexST(const int& id_in_graph, const int& time) {
  this->id = this->lastId;
  this->id_in_graph = id_in_graph;
  this->time = time;
  this->lastId++;
}

VertexST::~VertexST() {}

VertexST::VertexST(const VertexST& vertex) {
  this->id = vertex.id;
  this->id_in_graph = vertex.id_in_graph;
  this->time = vertex.time;
  this->adjacency_list_out = vertex.adjacency_list_out;
  this->adjacency_list_out_cost = vertex.adjacency_list_out_cost;
  this->adjacency_list_out_time = vertex.adjacency_list_out_time;
  this->adjacency_list_in = vertex.adjacency_list_in;
}

VertexST& VertexST::operator=(const VertexST& vertex) {
  this->id = vertex.id;
  this->id_in_graph = vertex.id_in_graph;
  this->time = vertex.time;
  this->adjacency_list_out = vertex.adjacency_list_out;
  this->adjacency_list_out_cost = vertex.adjacency_list_out_cost;
  this->adjacency_list_out_time = vertex.adjacency_list_out_time;
  this->adjacency_list_in = vertex.adjacency_list_in;
  return *this;
}

void VertexST::add_neighbour_out(const int& vId, const int& arcId) {
  auto finder = this->adjacency_list_out.find(vId);
  if (finder == this->adjacency_list_out.end()) {
    std::vector adj = {arcId};
    this->adjacency_list_out.insert(std::make_pair(vId, adj));
  } else {
    finder->second.push_back(arcId);
  }
}

void VertexST::add_neighbour_out_time(const int& vId,
                                      const std::vector<int>& arcIds) {
  this->adjacency_list_out_time.insert(std::make_pair(vId, arcIds));
}

void VertexST::add_neighbour_out_cost(const int& vId,
                                      const std::vector<int>& arcIds) {
  this->adjacency_list_out_cost.insert(std::make_pair(vId, arcIds));
}

void VertexST::add_neighbour_in(const int& vId, const int& arcId) {
  auto finder = this->adjacency_list_in.find(vId);
  if (finder == this->adjacency_list_in.end()) {
    std::vector adj = {arcId};
    this->adjacency_list_in.insert(std::make_pair(vId, adj));
  } else {
    finder->second.push_back(arcId);
  }
}

int VertexST::get_id() const { return id; }

int VertexST::get_id_in_graph() const { return id_in_graph; }

int VertexST::get_time() const { return time; }

const std::unordered_map<int, std::vector<int>>& VertexST::get_adjacency_list_out()
    const {
  return this->adjacency_list_out;
}

const std::unordered_map<int, std::vector<int>>& VertexST::get_adjacency_list_out_cost()
    const {
  return this->adjacency_list_out_cost;
}

const std::unordered_map<int, std::vector<int>>& VertexST::get_adjacency_list_out_time()
    const {
  return this->adjacency_list_out_time;
}

const std::unordered_map<int, std::vector<int>>& VertexST::get_adjacency_list_in() const {
  return this->adjacency_list_in;
}

std::string VertexST::toString() const {
  std::string str;
  str.append(std::to_string(this->id) + "\t(" + std::to_string(this->id_in_graph) + "," +
             std::to_string(this->time) + ")\t" + "\t\t" +
             std::to_string((int)this->adjacency_list_in.size()) + "\t" +
             std::to_string((int)this->adjacency_list_out.size()) + "\n");
  return str;
}
