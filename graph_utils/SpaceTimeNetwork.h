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

#ifndef SpaceTimeNetwork_H
#define SpaceTimeNetwork_H

#pragma once

#include "ArcST.h"
#include "Graph.h"
#include "Header.h"
#include "Lorry.h"
#include "Utils.h"
#include "VertexST.h"

/**
 *	\class SpaceTimeNetwork.
 *	\brief	This class represents the space-time network
 */

class SpaceTimeNetwork {
 protected:
  /**
   *	logistics network object
   */
  millemiglia::LogisticsNetwork network;

  /**
   *	underlying graph (only space dimension)
   */
  Graph underlying_graph;

  /**
   *	each vertex correspond to a (hub,time)
   */
  vector<VertexST> vertices;

  /**
   *	each arc correspond to a movement in the space-time network
   */
  vector<ArcST> arcs;

  /**
   *	key1 = hub id in graph, key2 = encoded time, value = vertex id in ST
   *network
   */
  unordered_map<int, unordered_map<int, int>> vertex_dictionary;

  /**
   *	current arc processed in an adjacency list = the space-time network is a
   *multi-graph used in the k-shortest path algorithm
   */
  vector<unordered_map<int, int>> adjacency_lists_arc_position;

  /**
   *	available lorries
   */
  vector<Lorry> lorries;

 public:
  /**
   * \brief Constructor by default.
   */
  SpaceTimeNetwork();
  /**
   * \brief Constructor by data: string path to LogisticsNetwork protocol buffer
   * and time horizon
   *
   * \param network protocal buffer object
   */
  SpaceTimeNetwork(const string& network_file, const int& time_horizon);

  SpaceTimeNetwork(const millemiglia::LogisticsNetwork& network,
                   const int& time_horizon);

  /**
   * \brief Copy constructor.
   *
   */
  SpaceTimeNetwork(const SpaceTimeNetwork& spaceTimeNetwork);
  /**
   *	\brief	Operator assignment
   */
  SpaceTimeNetwork& operator=(const SpaceTimeNetwork& spaceTimeNetwork);

  /**
   * \brief Destructor.
   *
   */
  ~SpaceTimeNetwork();

  /**
   *	\brief parse the network proto buffer and build the associated object.
   *		Build object space-time network
   *
   *	\param path to the logistics network file
   */
  void parse_logistic_network(const string& network_file);

  /**
   *	\brief Build graph object
   *
   *	\param network protocal buffer object
   */
  void build_underlying_graph(
      const millemiglia::LogisticsNetwork& network);

  /**
   *	\brief Build vector of vertices (i,t) for each hub i and each time t
   *
   *	\param time_horizon
   */
  void build_vertices(const int& time_horizon);

  /**
   *	\brief Build vector of arcs
   *
   *	\param network protocal buffer object
   */
  void build_arcs(const millemiglia::LogisticsNetwork& network,
                  const int& time_horizon);

  /**
   *	\brief Build vector of vehicles
   *
   *	\param network protocal buffer object
   */
  void build_lorries(
      const millemiglia::LogisticsNetwork& network);

  /**
   *	\brief Build vector of vertices
   *
   *	\param hub name
   *	\param time object
   */
  void add_vertexST(const string& hub,
                    const millemiglia::DateTimeRange& time,
                    const int& time_horizon);
  /**
   *	\brief Build vector of vertices
   *
   *	\param hub_id
   *	\param time encoded time
   */
  void add_vertexST(const int& hub, const int& time);

  /**
   *	\brief Add outgoing arc to adjacency list of a vertex
   *
   *	\param arc
   */
  void add_to_adjacency_list_out(const ArcST& arc);
  /**
   *	\brief Add ingoing arc to adjacency list of a vertex
   *
   *	\param arc
   */
  void add_to_adjacency_list_in(const ArcST& arc);


  const millemiglia::LogisticsNetwork& get_network() const;
  const Graph& get_underlying_graph() const;
  const vector<VertexST>& get_vertices() const;
  const vector<ArcST>& get_arcs() const;
  const VertexST& get_vertex(
      const string& hub,
      const millemiglia::DateTimeRange& time) const;
  const VertexST& get_vertex(const int& hub, const int& time) const;

  const string toString() const;
};

#endif  // !SpaceTimeNetwork_H