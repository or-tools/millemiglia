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

#include "../graph_utils/Lorry.h"

using namespace millemiglia;

TEST(LorryTest, Constructor) {
  Vehicle vehicle;
  Lorry lorry("test_lorry", vehicle);

  EXPECT_EQ(lorry.get_id(), 0);
  EXPECT_EQ(lorry.get_name(), "test_lorry");
  EXPECT_TRUE(lorry.get_path().empty());
  EXPECT_TRUE(lorry.get_capacities().empty());
  EXPECT_TRUE(lorry.get_loaded_commodities().empty());
  EXPECT_TRUE(lorry.get_loads_per_arc().empty());
}

TEST(LorryTest, ConstructorWithCapacities) {
  Vehicle vehicle;
  vehicle.add_capacities()->set_value(100);
  vehicle.add_capacities()->set_value(200);

  Lorry lorry("test_lorry", vehicle);

  EXPECT_EQ(lorry.get_capacities().size(), 2);
  EXPECT_EQ(lorry.get_capacities()[0], 100);
  EXPECT_EQ(lorry.get_capacities()[1], 200);
}

TEST(LorryTest, AddCommodity) {
  Vehicle vehicle;
  Lorry lorry("test_lorry", vehicle);

  lorry.add_commodity(1, {10, 20}, 0);
  lorry.add_commodity(2, {30, 40}, 1);

  EXPECT_EQ(lorry.get_loaded_commodities().at(0).size(), 1);
  EXPECT_EQ(lorry.get_loaded_commodities().at(0)[0], 1);
  EXPECT_EQ(lorry.get_loads_per_arc().at(0)[0], 10);
  EXPECT_EQ(lorry.get_loads_per_arc().at(0)[1], 20);
  EXPECT_EQ(lorry.get_loaded_commodities().at(1).size(), 1);
  EXPECT_EQ(lorry.get_loaded_commodities().at(1)[0], 2);
  EXPECT_EQ(lorry.get_loads_per_arc().at(1)[0], 30);
  EXPECT_EQ(lorry.get_loads_per_arc().at(1)[1], 40);
}
TEST(LorryTest, AddCommodityMultipleTimes) {
  Vehicle vehicle;
  Lorry lorry("test_lorry", vehicle);
  lorry.add_commodity(1, {10, 20}, 0);
  lorry.add_commodity(1, {30, 40}, 0);
  EXPECT_EQ(lorry.get_loaded_commodities().at(0).size(), 2);
  EXPECT_EQ(lorry.get_loaded_commodities().at(0)[0], 1);
  EXPECT_EQ(lorry.get_loaded_commodities().at(0)[1], 1);
  EXPECT_EQ(lorry.get_loads_per_arc().at(0)[0], 40);
  EXPECT_EQ(lorry.get_loads_per_arc().at(0)[1], 60);
}
