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

#include <string>
#include <utility>

#include "../graph_utils/Utils.h"
#include "../instance_generator/InstanceGenerator.h"

TEST(GraphInstanceGeneratorTest, BuildRandomGraph) {
  // Create an instance generator
  InstanceGenerator generator;

  // Build a random graph with 10 hubs and 0.5 density
  Graph graph = generator.build_random_graph(100, 0.5);
  int n_vertices = graph.get_vertex_number();
  // Assert the number of vertices is correct
  EXPECT_EQ(100, n_vertices);
  double density =
      (double)graph.get_number_of_arcs() / (n_vertices * (n_vertices - 1));
  double error = 0.02;
  // Assert the number of edges is correct
  EXPECT_GT(density, 0.5 - error);
  EXPECT_LE(density, 0.5 + error);
}

TEST(InstanceGeneratorTest, BuildArcWeights) {
  Graph graph(4);

  // Add vertices
  graph.add_vertex("v1");
  graph.add_vertex("v2");
  graph.add_vertex("v3");
  graph.add_vertex("v4");
  // Add edges
  graph.add_neighbour("v1", "v2", "");
  graph.add_neighbour("v1", "v3", "");
  graph.add_neighbour("v2", "v3", "");
  graph.add_neighbour("v3", "v4", "");
  InstanceGenerator generator("test_instance", "test description", 100);
  vector<double> arc_weights = generator.build_arc_weights(graph);

  EXPECT_EQ(arc_weights.size(), 4);
  EXPECT_EQ(arc_weights[0], 4);
  EXPECT_EQ(arc_weights[1], 5);
  EXPECT_EQ(arc_weights[2], 5);
  EXPECT_EQ(arc_weights[3], 4);
}

TEST(InstanceGeneratorTest, GenerateLine) {
  // Test with a simple graph
  int hubs_number = 10;
  Graph graph(10);
  for (int k = 1; k <= hubs_number; k++) graph.add_vertex("v_" + to_string(k));
  for (int k = 1; k <= hubs_number; k++) {
    for (int e = k + 1; e <= hubs_number; e++) {
      graph.add_neighbour("v_" + to_string(k), "v_" + to_string(e), "");
    }
  }

  InstanceGenerator generator("test_instance", "test description", 100);
  int max_length_line = 5;
  vector<vector<double>> arc_weights_adj_lists =
      generator.build_arc_weights_per_adjacency_lists(
          graph, generator.build_arc_weights(graph));
  vector<double> arc_weights = generator.build_arc_weights(graph);

  vector<int> line = generator.generate_line(
      graph, max_length_line, arc_weights_adj_lists, arc_weights);

  EXPECT_GE(line.size(), 1);
  EXPECT_LE(line.size(), max_length_line);

  vector<pair<int, int>> edges = graph.get_arcs();
  auto has_edge = [&edges](int u, int v) {
    return find(edges.begin(), edges.end(), make_pair(u, v)) != edges.end();
  };

  // Check that all nodes in the line are connected in the graph
  for (int i = 0; i < (int)line.size() - 1; ++i) {
    EXPECT_TRUE(has_edge(line[i], line[i + 1]));
  }
}

// Test that the GraphInstanceGeneratorTest method populates the provided
// instance.
TEST(GraphInstanceGeneratorTest, GenerateLogisticNetwork) {
  // Create an instance object.
  millemiglia::Instance instance;
  InstanceGenerator generator;

  // Generate the network with the following parameters:
  generator.generate_logistic_network(
      instance, 20 /* hubs */, 0.5 /* density */, 10 /* time horizon */,
      4 /* lines */, 5 /* max length line */, 5 /* max rotations per line */,
      1000 /* max vehicle capacity */);
  // Verify that the instance has been populated.
  EXPECT_EQ(instance.network().name(),
            "network_20");  // Network name should be "network_<hubs_number>"
  EXPECT_EQ(instance.network().hubs_size(), 20);  // Should have 5 hubs
  EXPECT_EQ(instance.network().lines_size(), 4);  // Should have 2 lines
}

TEST(GraphInstanceGeneratorTest, GenerateLineRotations_MultipleLinesRotations) {
  // Create a graph with 5 hubs.
  Graph graph(5);

  for (int i = 1; i <= 5; i++) {
    graph.add_vertex("h_" + to_string(i));
  }
  // Add edges.
  for (int i = 1; i < 5; i++) {
    graph.add_neighbour("h_" + to_string(i), "h_" + to_string(i + 1), "");
  }
  InstanceGenerator generator;

  // Generate lines with more lines and rotations
  int num_lines = 2;
  int max_length = 4;
  int max_rotations = 2;
  int time_horizon = 20;
  millemiglia::LogisticsNetwork network;
  vector<double> arc_weights = generator.build_arc_weights(graph);
  vector<vector<double>> arc_weights_adj_lists =
      generator.build_arc_weights_per_adjacency_lists(graph, arc_weights);

  int num_vehicles = generator.generate_line_rotations(
      network, graph, arc_weights, arc_weights_adj_lists, time_horizon,
      num_lines, max_length, max_rotations);

  // Verify the number of vehicles (should depend on rotations)
  EXPECT_GT(num_vehicles, 0);  // Expect at least some vehicles for rotations

  EXPECT_EQ(network.lines_size(), num_lines);

  for (int i = 0; i < network.lines_size(); ++i) {
    const millemiglia::Line& line = network.lines(i);

    // Verify the number of hubs in the line (should be within max_length)
    EXPECT_LE(line.hub_ids_size(), max_length + 1);
    EXPECT_GT(line.hub_ids_size(),
              1);  // Line should have at least 2 hubs (start and end)

    // Verify the number of rotations for the line (should be at most
    // max_rotations)
    EXPECT_LE(line.next_rotations_size(), max_rotations);

    // Loop through each rotation of the line and verify its structure
    for (int j = 0; j < line.next_rotations_size(); ++j) {
      const millemiglia::LineRotation& rotation = line.next_rotations(j);

      EXPECT_EQ(rotation.departure_times_size(), line.hub_ids_size() - 1);
      EXPECT_EQ(rotation.arrival_times_size(), line.hub_ids_size() - 1);

      // TODO(aymanelotfi):Verify that departure times are within the time
      // horizon
    }
  }
}

TEST(InstanceGeneratorTest, GenerateShipments) {
  // Create an instance with a small network and time horizon.
  millemiglia::Instance instance;
  InstanceGenerator generator;
  // Generate the network with the following parameters:
  generator.generate_logistic_network(
      instance, 5 /* hubs */, 0.5 /* density */, 10 /* time horizon */,
      4 /* lines */, 5 /* max length line */, 5 /* max rotations per line */,
      1000 /* max vehicle capacity */);

  // Set up the parameters for shipment generation.
  const int shipment_number = 10;
  const int max_path_length = 6;
  const int min_shipment_weight = 1;
  const int max_shipment_weight = 5;
  const double shipment_weight_shape = 2.0;
  const int time_horizon = 10;

  //  Create an InstanceGenerator object and generate shipments.
  generator.generate_shipments(instance, shipment_number, time_horizon,
                               max_path_length, min_shipment_weight,
                               max_shipment_weight, shipment_weight_shape);

  // Verify that the correct number of shipments were generated with enough
  // tries.
  ASSERT_EQ(instance.shipments_size(), shipment_number);

  // Verify that the shipments have valid time and weight values.
  for (const auto& shipment : instance.shipments()) {
    ASSERT_NE(shipment.source_hub(), shipment.destination_hub());
    ASSERT_LT(time_encoder(shipment.departure_time()), time_horizon);
    ASSERT_GT(time_encoder(shipment.arrival_time().first_date()),
              time_encoder(shipment.departure_time()));
    ASSERT_LE(time_encoder(shipment.arrival_time().first_date()), time_horizon);

    for (auto weight : shipment.size()) {
      ASSERT_GE(weight.value(), min_shipment_weight);
      ASSERT_LE(weight.value(), max_shipment_weight);
    }
  }
}