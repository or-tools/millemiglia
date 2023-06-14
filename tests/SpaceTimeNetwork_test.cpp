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

#include <gtest/gtest.h>

#include "../graph_utils/SpaceTimeNetwork.h"

using namespace millemiglia;

TEST(SpaceTimeNetworkTest, BuildVertices) {
  LogisticsNetwork network;
  int time_horizon = 5;
  SpaceTimeNetwork stn(network, time_horizon);
  // Creating a network of 10 hubs.
  for (int i = 0; i < 10; i++) {
    Hub h;
    h.set_name("node_" + to_string(i));
    network.mutable_hubs()->Add(move(h));
  }
  // Check that the correct number of vertices were created.
  ASSERT_EQ(stn.get_vertices().size(),
            stn.get_underlying_graph().get_vertices().size() * time_horizon);

  // Check that the vertices have the correct graph and time values.
  for (int i = 0; i < stn.get_underlying_graph().get_vertices().size(); i++) {
    for (int t = 0; t < time_horizon; t++) {
      const VertexST& vertex = stn.get_vertex(i, t);
      ASSERT_EQ(vertex.get_id_in_graph(), i);
      ASSERT_EQ(vertex.get_time(), t);
    }
  }
}

// TODO(aymanelotfi) : Improve this by supporting get vertexST by id.
TEST(SpaceTimeNetworkTest, BuildArcs) {
  LogisticsNetwork network;
  int time_horizon = 5;
  SpaceTimeNetwork stn(network, time_horizon);

  // Check that the correct number of arcs were created.
  int num_travelling_arcs = 0;
  int num_waiting_arcs = 0;
  for (const auto& line : network.lines()) {
    num_travelling_arcs +=
        (line.hub_ids().size() - 1) * line.next_rotations().size();
  }
  num_waiting_arcs =
      stn.get_underlying_graph().get_vertices().size() * (time_horizon - 1);
  ASSERT_EQ(stn.get_arcs().size(), num_travelling_arcs + num_waiting_arcs);
}