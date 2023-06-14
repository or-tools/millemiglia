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

#ifndef VERTEX_H
#define VERTEX_H

#pragma once

#include "Header.h"
#include "Utils.h"

/**
 *	\class Vertex.
 *	\brief	This class represent a vertex in the graph
 */

class Vertex {
 protected:
  /**
   *	vertex id from 1,...,hub_number
   */
  int id;

  /**
   *	vertex name in the protocol buffer
   */
  string name;

  /**
   * @brief  
   * Outgoing arcs adjacency map: key = id of the vertex to reach, value =
   *vector of "line" that use arc (this,key)
   */
  unordered_map<int, vector<string>> adjacency_list_out;

  /** \brief 
   *
   *	ingoing arcs adjacency map: key = id of the vertex to reach, value =
   *vector of "line" that use arc (key,this)
   */
  unordered_map<int, vector<string>> adjacency_list_in;

  /**
   *	vertex counter to assign unique id
   */
  static int lastId;

 public:
  /**
   * \brief Constructor by default.
   */
  Vertex();
  /**
   * \brief Constructor by name.
   *
   * \param name
   */
  Vertex(const string& name);
  /**
   * \brief Copy constructor.
   *
   */
  Vertex(const Vertex& vertex);
  /**
   *	\brief	Operator assignment
   */
  Vertex& operator=(const Vertex& vertex);
  /**
   * \brief Destructor.
   *
   */
  ~Vertex();

  /**
   * \brief Add ingoing arc (id,this) to the adjacency list.
   *
   *	\param id
   *	\param line of the network protocal buffer network file
   */
  void add_neighbour_in(const int& id, const string& line);

  /**
   * \brief Add outgoing arc (this,id) to the adjacency list.
   *
   *	\param id
   *	\param line of the network protocal buffer network file
   */
  void add_neighbour_out(const int& id, const string& line);

  /**
   * \brief Test whether (this,id) exists
   *
   *	\param id
   *	\return true if (this,id) exists, false otherwise
   */
  bool is_neighbour_out(const int& id) const;

  /**
   * \brief Test whether (id,this) exists
   *
   *	\param id
   *	\return true if (id,this) exists, false otherwise
   */
  bool is_neighbour_in(const int& id) const;

  static void restart_id_counter() { lastId = 0; }

  const int& get_id() const;
  const string& get_name() const;
  const unordered_map<int, vector<string>>& get_adjacency_list_out() const;
  const unordered_map<int, vector<string>>& get_adjacency_list_in() const;
  const int get_out_going_by_position(const int& pos) const;
  const int get_neighbours_number() const;
  const vector<string> get_lines_out(const int& id) const;
  const vector<string> get_lines_in(const int& id) const;

  const string toString() const;
};

#endif  // !GRAPH_H