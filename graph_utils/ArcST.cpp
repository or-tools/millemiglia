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

#include "ArcST.h"

int ArcST::lastId;

ArcST::ArcST() {
  id = -1;
  departure_id = -1;
  arrival_id = -1;
}

ArcST::ArcST(const int departure_id, const int arrival_id, const std::string& type,
             const std::string& line, const std::string& rotation,
             const int travelling_time, const double& cost) {
  id = lastId;
  this->departure_id = departure_id;
  this->arrival_id = arrival_id;
  this->type = type;
  this->line = line;
  this->rotation = rotation;
  this->travelling_time = travelling_time;
  this->cost = cost;
  lastId++;
}

ArcST::~ArcST() = default;

ArcST::ArcST(const ArcST& arcST) {
  this->id = arcST.id;
  this->departure_id = arcST.departure_id;
  this->arrival_id = arcST.arrival_id;
  this->type = arcST.type;
  this->line = arcST.line;
  this->rotation = arcST.rotation;
  this->travelling_time = arcST.travelling_time;
  this->cost = arcST.cost;
}

ArcST& ArcST::operator=(const ArcST& arcST) = default;

int ArcST::get_id() const { return id; }

int ArcST::get_departure_id() const { return departure_id; }

int ArcST::get_arrival_id() const { return arrival_id; }

const std::string& ArcST::get_type() const { return type; }

const std::string& ArcST::get_line() const { return line; }

const std::string& ArcST::get_rotation() const { return rotation; }

int ArcST::get_travelling_time() const { return travelling_time; }

const double& ArcST::get_cost() const { return cost; }

std::string ArcST::toString() const {
  std::string str;
  str.append(std::to_string(id) + "\t(" + std::to_string(departure_id) + "," +
             std::to_string(arrival_id) + ")\t" + type + "\t" +
             std::to_string(travelling_time) + "\t" + std::to_string(cost) +
             "\t(" + line + "," + rotation + ")" + "\n");
  return str;
}
