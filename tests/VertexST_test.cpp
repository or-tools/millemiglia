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

#include "../graph_utils/VertexST.h"

TEST(VertexSTTest, Constructor) {
  // Test default constructor
  VertexST v1;
  EXPECT_EQ(v1.get_id(), -1);
  EXPECT_EQ(v1.get_id_in_graph(), -1);
  EXPECT_EQ(v1.get_time(), -1);

  // Test constructor with parameters
  VertexST v2(10, 5);
  EXPECT_EQ(v2.get_id(), 0);
  EXPECT_EQ(v2.get_id_in_graph(), 10);
  EXPECT_EQ(v2.get_time(), 5);
}

TEST(VertexSTTest, CopyConstructor) {
  VertexST v1(10, 5);
  v1.add_neighbour_out(1, 2);

  // Create a copy of v1
  VertexST v2(v1);

  // Verify that the copies are equal
  EXPECT_EQ(v2.get_id(), v1.get_id());
  EXPECT_EQ(v2.get_id_in_graph(), v1.get_id_in_graph());
  EXPECT_EQ(v2.get_time(), v1.get_time());
  EXPECT_EQ(v2.get_adjacency_list_out(), v1.get_adjacency_list_out());
}

TEST(VertexSTTest, AssignmentOperator) {
  VertexST v1(10, 5);
  v1.add_neighbour_out(1, 2);

  // Create a new vertex
  VertexST v2;

  // Assign v1 to v2
  v2 = v1;

  // Verify that the assignment was successful
  EXPECT_EQ(v2.get_id(), v1.get_id());
  EXPECT_EQ(v2.get_id_in_graph(), v1.get_id_in_graph());
  EXPECT_EQ(v2.get_time(), v1.get_time());
  EXPECT_EQ(v2.get_adjacency_list_out(), v1.get_adjacency_list_out());
}

TEST(VertexSTTest, AddNeighbourOut) {
  VertexST v;
  v.add_neighbour_out(1, 2);
  v.add_neighbour_out(1, 3);
  v.add_neighbour_out(2, 4);

  // Verify that the neighbours were added correctly
  EXPECT_EQ(v.get_adjacency_list_out().size(), 2);
  EXPECT_GT(v.get_adjacency_list_out().count(1), 0);
  EXPECT_GT(v.get_adjacency_list_out().count(2), 0);
  EXPECT_EQ(v.get_adjacency_list_out().at(1).size(), 2);
  EXPECT_EQ(v.get_adjacency_list_out().at(2).size(), 1);
}

TEST(VertexSTTest, AddNeighbourIn) {
  VertexST v;
  v.add_neighbour_in(1, 2);
  v.add_neighbour_in(1, 3);
  v.add_neighbour_in(2, 4);

  // Verify that the neighbours were added correctly
  EXPECT_EQ(v.get_adjacency_list_in().size(), 2);
  EXPECT_GT(v.get_adjacency_list_in().count(1), 0);
  EXPECT_GT(v.get_adjacency_list_in().count(2), 0);
  EXPECT_EQ(v.get_adjacency_list_in().at(1).size(), 2);
  EXPECT_EQ(v.get_adjacency_list_in().at(2).size(), 1);
}
