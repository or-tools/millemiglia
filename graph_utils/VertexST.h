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

#ifndef VERTEXST_H
#define VERTEXST_H

#pragma once

#include "Header.h"
#include "Utils.h"

/**
 *	\class VertexST.
 *	\brief	This class represent a vertex in the space-time network, it is a
 *pair (i,t), where i is the vertex id in the graph and t is a time
 */

class VertexST {
 protected:
  /**
   *	vertex id in the space-time network
   */
  int id;

  /**
   *	vertex id (hub) in the underlying graph
   */
  int id_in_graph;

  /**
   *	time the encoding of the discretised time
   */
  int time;

  /**
   *	arc = (this,id)  key = id, value = vector of arcs to go from this to id
   */
  unordered_map<int, vector<int>> adjacency_list_out;
  /**
   *	adjacency_list_out arcs are sorted by increasing cost
   */
  unordered_map<int, vector<int>> adjacency_list_out_cost;
  /**
   *	adjacency_list_out arcs are sorted by increasing travelling time
   */
  unordered_map<int, vector<int>> adjacency_list_out_time;
  /**
   *	arc = (id,this)   key = id, value = vector of arcs to go from id to this
   */
  unordered_map<int, vector<int>> adjacency_list_in;

  /**
   *	vertex counter to assign unique id
   */
  static int lastId;

 public:
  /**
   * \brief Constructor by default.
   */
  VertexST();
  /**
   * \brief Constructor by name.
   *
   * \param id_in_graph
   * \param time
   */
  VertexST(const int& id_in_graph, const int& time);
  /**
   * \brief Copy constructor.
   *
   */
  VertexST(const VertexST& vertex);
  /**
   *	\brief	Operator assignment
   */
  VertexST& operator=(const VertexST& vertex);

  /**
   * \brief Destructor.
   *
   */
  ~VertexST();

  /**
   * \brief Add arc=(this,vId) id to the adjacency list of the out going arcs
   *
   * \param vId
   * \param arcId
   */
  void add_neighbour_out(const int& vId, const int& arcId);
  void add_neighbour_out_time(const int& vId, const vector<int>& arcIds);
  void add_neighbour_out_cost(const int& vId, const vector<int>& arcIds);

  /**
   * \brief Add arc=(vId,this) id to the adjacency list of the out going arcs
   *
   * \param vId
   * \param arcId
   */
  void add_neighbour_in(const int& vId, const int& arcId);

  static void restart_id_counter() { lastId = 0; }

  const int& get_id() const;
  const int& get_id_in_graph() const;
  const int& get_time() const;
  const unordered_map<int, vector<int>>& get_adjacency_list_out() const;
  const unordered_map<int, vector<int>>& get_adjacency_list_out_cost() const;
  const unordered_map<int, vector<int>>& get_adjacency_list_out_time() const;
  const unordered_map<int, vector<int>>& get_adjacency_list_in() const;

  const string toString() const;
};

#endif  // !VERTEXTS_H