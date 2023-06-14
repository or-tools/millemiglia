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

#include "Lorry.h"

int Lorry::lastId;

Lorry::Lorry(string name, const millemiglia::Vehicle& vehicle)
    : vehicle(vehicle) {
  this->id = this->lastId;
  this->name = name;
  for (const auto& capacity : this->vehicle.capacities()) {
    this->capacities.push_back(capacity.value());
  }
  this->lastId++;
}

Lorry::~Lorry() {
  this->capacities.clear();
  this->path.clear();
  this->loaded_commodities.clear();
  this->loads_per_arc.clear();
}

Lorry::Lorry(const Lorry& lorry) : vehicle(lorry.vehicle) {
  this->id = lorry.id;
  this->name = lorry.name;
  this->path = lorry.path;
  this->capacities = lorry.capacities;
  this->loaded_commodities = lorry.loaded_commodities;
  this->loads_per_arc = lorry.loads_per_arc;
}

Lorry& Lorry::operator=(const Lorry& lorry) {
  this->id = lorry.id;
  this->name = lorry.name;
  this->path = lorry.path;
  this->capacities = lorry.capacities;
  this->loaded_commodities = lorry.loaded_commodities;
  this->loads_per_arc = lorry.loads_per_arc;
  return *this;
}

void Lorry::add_commodity(const int& commodity_id, const vector<int>& sizes,
                          const int& arc) {
  const auto finder = this->loaded_commodities.find(arc);
  if (finder == this->loaded_commodities.end()) {
    vector<int> aux = {commodity_id};
    this->loaded_commodities.insert(make_pair(arc, aux));
    this->loads_per_arc.insert(make_pair(arc, sizes));
  } else {
    finder->second.push_back(commodity_id);
    for (int i = 0; i < sizes.size(); i++) {
      this->loads_per_arc.at(arc).at(i) += sizes.at(i);
    }
  }
}

const int& Lorry::get_id() const { return this->id; }

const string& Lorry::get_name() const { return this->name; }

const millemiglia::Vehicle& Lorry::get_proto_vehicle() const {
  return this->vehicle;
}

const vector<int>& Lorry::get_path() const { return this->path; }

const vector<int>& Lorry::get_capacities() const { return this->capacities; }

const unordered_map<int, vector<int>>& Lorry::get_loaded_commodities() const {
  return this->loaded_commodities;
}

const unordered_map<int, vector<int>>& Lorry::get_loads_per_arc() const {
  return this->loads_per_arc;
}

const string Lorry::toString() const {
  string str;
  str.append("LORRY: " + this->name + "\n");
  str.append("\tCAPACITIES:\t");
  for (const auto& cap : this->capacities) {
    str.append(to_string(cap) + " ");
  }
  str.append("\n\tPATH:\t");
  for (int i = 0; i < this->path.size(); i++) {
    str.append(to_string(this->path.at(i)) + " ");
  }
  str.append("\n");
  return str;
}
