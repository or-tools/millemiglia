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

#ifndef LORRY_H
#define LORRY_H

#pragma once

#include "Header.h"
#include "Utils.h"

/**
 *	\class Vehicle.
 *	\brief	This class represent a vehicle
 */

class Lorry {
 protected:
  int id;
  string name;

  /**
   *	vehicle object
   */
  const millemiglia::Vehicle& vehicle;
  /**
   *	path: arcs in the st_network traversed by the vehicle
   */
  vector<int> path;
  /**
   *	capacities
   */
  vector<int> capacities;
  /**
   *	loaded_commodities: for each arc in path, the commodities on that arc
   */
  unordered_map<int, vector<int>> loaded_commodities;
  /**
   *	residual capacity: for each arc in path, the residual capacities of the
   *vehicle on that arc
   */
  unordered_map<int, vector<int>> loads_per_arc;

  /**
   *	vertex counter to assign unique id
   */
  static int lastId;

 public:
  /**
   * \brief Constructor by data.
   *
   * \param number of hubs
   */
  Lorry(string name, const millemiglia::Vehicle& vehicle);
  /**
   * \brief Copy constructor.
   *
   */
  Lorry(const Lorry& lorry);
  /**
   *	\brief	Operator assignment
   */
  Lorry& operator=(const Lorry& lorry);

  /**
   * \brief Destructor.
   *
   */
  ~Lorry();

  /**
   *	\brief add a commodity in the vehicle, modify residual capacity
   *
   *	\param commodity : Commodity
   *	\param arc : int
   */
  void add_commodity(const int& commodity_id, const vector<int>& sizes,
                     const int& arc);

  static void restart_id_counter() { lastId = 0; }

  const int& get_id() const;
  const string& get_name() const;
  const millemiglia::Vehicle& get_proto_vehicle() const;
  const vector<int>& get_path() const;
  const vector<int>& get_capacities() const;
  const unordered_map<int, vector<int>>& get_loaded_commodities() const;
  const unordered_map<int, vector<int>>& get_loads_per_arc() const;

  const string toString() const;
};

#endif  // !LORRY_H
