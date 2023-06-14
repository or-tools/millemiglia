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

#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "proto/millemiglia.pb.h"

using namespace std;
using namespace millemiglia;

constexpr auto EPSILON = 1.0e-6; /**< Precision variable.*/
constexpr auto OMEGA = 1.0e+6;   /**< Precision variable.*/
constexpr auto TIMESTEP =
    15; /**< Discretise the horizon (1day) with a step of 15min.*/

#endif  // HEADER_H
