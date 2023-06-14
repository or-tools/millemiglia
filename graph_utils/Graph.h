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

#ifndef GRAPH_H
#define GRAPH_H

#pragma once

#include "Header.h"
#include "Utils.h"
#include "Vertex.h"

/**
 *   \class Graph.
 *   \brief  This class represents a directed graph data structure,
 *           commonly used for modeling networks where connections
 *           between elements (vertices) have a direction.
 *
 *   The class provides functionalities to:
 *     - Manage vertices (in our case hubs) and their names.
 *     - Add directed connections (arcs) between vertices.
 *     - Efficiently access vertices and their neighbors.
 *     - Retrieve information about the graph structure.
 */

class Graph {
 protected:
  /**
   *	each vertex correspond to a hub
   */
  vector<Vertex> vertices;

  /**
   *	key = name, value = id
   */
  unordered_map<string, int> vertex_dictionary;

  /**
   *	number of arcs
   */
  vector<pair<int, int>> arcs;

  /**
   *	key = name, value = id
   */
  vector<unordered_map<int, int>> arcs_dictionary;

 public:
  /**
   * \brief Constructor by default.
   */
  Graph();
  /**
   * \brief Constructor by data.
   *
   * \param number of hubs
   */
  Graph(const int& hub_number);
  /**
   * \brief Copy constructor.
   *
   */
  Graph(const Graph& graph);
  /**
   *	\brief	Operator assignment
   */
  Graph& operator=(const Graph& graph);
  /**
   * \brief Destructor.
   *
   */
  ~Graph();

  /**
   *	\brief add a vertex
   *
   *	\param hub name
   */
  void add_vertex(const string& name);
  /**
   *	\brief add a neighbour to key both (key,neighbour) and (neighbour,key)
   *
   *	\param key
   *	\param neighbour
   *	\param line in the network protocol buffer that uses arc (key,
   *neighbour)
   */
  void add_neighbour(const string& key, const string& neighbour,
                     const string& line);

  const vector<Vertex>& get_vertices() const;
  const int get_vertex_number() const;
  const Vertex& get_vertex(const int& id) const;
  const Vertex& get_vertex(const string& name) const;
  const vector<pair<int, int>>& get_arcs() const;
  const int get_number_of_arcs() const;
  const int get_arc_position(const int& id1, const int& id2) const;

  const string toString() const;
};

#endif  // !GRAPH_H