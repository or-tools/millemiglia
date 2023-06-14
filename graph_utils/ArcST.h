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

#ifndef ARCST_H
#define ARCST_H

#pragma once

#include "Header.h"
#include "Utils.h"

/**
 *	\class ArcST.
 *	\brief	This class represent an arc in the space-time graph
 */

class ArcST {
 protected:
  /**
   *	arc id from 1,...,arc_number
   */
  int id;

  /**
   *	departure id
   */
  int departure_id;

  /**
   *	arrival id
   */
  int arrival_id;

  /**
   *	type = "travelling", "waiting"
   */
  string type;

  /**
   *	cost = arc cost; if this is the first arc of a line-rotation, the cost
   *is equal to the entire cost, otherwise it is zero
   */
  double cost;

  /**
   *	travelling_time = time to travel the arc, if type = "travelling",
   *otherwise it is zero
   */
  int travelling_time;

  /**
   *	line associated with the arc
   */
  string line;

  /**
   *	rotation associated with the arc
   */
  string rotation;

  /**
   *	arc counter to assign unique id
   */
  static int lastId;

 public:
  /**
   * \brief Constructor by default.
   */
  ArcST();
  /**
   * \brief Constructor by data.
   *
   * \param departure_id
   * \param arrival_id
   * \param line
   * \param rotation
   * \param travelling_time
   * \param cost
   */
  ArcST(const int& departure_id, const int& arrival_id, const string& type,
        const string& line, const string& rotation,
        const int& travelling_time = 0.0, const double& cost = 0.0);
  /**
   * \brief Copy constructor.
   *
   */
  ArcST(const ArcST& arc);
  /**
   *	\brief	Operator assignment
   */
  ArcST& operator=(const ArcST& arc);

  /**
   * \brief Destructor.
   *
   */
  ~ArcST();

  static void restart_id_counter() { lastId = 0; }

  const int& get_id() const;
  const int& get_departure_id() const;
  const int& get_arrival_id() const;
  const string& get_type() const;
  const string& get_line() const;
  const string& get_rotation() const;
  const int& get_travelling_time() const;
  const double& get_cost() const;

  const string toString() const;
};

#endif  // !GRAPH_H