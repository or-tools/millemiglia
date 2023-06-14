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

#include "../graph_utils/Graph.h"

TEST(Graph, DefaultConstructor) {
  Graph graph;
  EXPECT_EQ(graph.get_vertex_number(), 0);
  EXPECT_TRUE(graph.get_arcs().empty());
}

TEST(Graph, ConstructorWithNumberHubs) {
  Graph graph(3);
  EXPECT_EQ(graph.get_vertex_number(), 3);
  EXPECT_TRUE(graph.get_arcs().empty());
  // Check if vertex IDs are assigned sequentially
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(graph.get_vertex(i).get_id(), i);
  }
}

TEST(Graph, AddVertex) {
  Graph graph(1);
  graph.add_vertex("A");
  EXPECT_EQ(graph.get_vertex_number(), 1);
  EXPECT_EQ(graph.get_vertex(0).get_name(), "A");
}

TEST(Graph, AddNeighbourSimple) {
  Graph graph(2);
  graph.add_vertex("A");
  graph.add_vertex("B");
  graph.add_neighbour("A", "B", "");
  EXPECT_EQ(graph.get_vertex_number(), 2);
  EXPECT_EQ(graph.get_vertex(0).get_adjacency_list_out().size(), 1);
  EXPECT_EQ(graph.get_vertex(1).get_adjacency_list_in().size(), 1);
  EXPECT_EQ(graph.get_arcs().size(), 1);
}

TEST(Graph, GetVertex) {
  Graph graph(2);
  graph.add_vertex("A");
  graph.add_vertex("B");
  EXPECT_EQ(graph.get_vertex(0).get_name(), "A");
  EXPECT_EQ(graph.get_vertex("B").get_id(), 1);
}

TEST(Graph, GetArcs) {
  Graph graph(3);
  graph.add_vertex("A");
  graph.add_vertex("B");
  graph.add_vertex("C");
  graph.add_neighbour("A", "B", "");
  graph.add_neighbour("B", "C", "");

  EXPECT_EQ(graph.get_arcs().size(), 2);
  EXPECT_EQ(graph.get_arcs()[0], make_pair(0, 1));
  EXPECT_EQ(graph.get_arcs()[1], make_pair(1, 2));
}

TEST(Graph, GetArcPosition) {
  Graph graph(2);
  graph.add_vertex("A");
  graph.add_vertex("B");
  graph.add_neighbour("A", "B", "");
  EXPECT_EQ(graph.get_arc_position(0, 1), 0);
}
