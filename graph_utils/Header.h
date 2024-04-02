/*****************************************************************//**
 * \file   Header.h
 * \brief  Contains the common includes of all the files in this project.
 *
 * \author mpetris
 * \date   October 2023
 *********************************************************************/

#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <filesystem>
#include <random>
#include <cmath>
#include <ctime>

#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>
#include "proto/lattle.pb.h"
//#include <armadillo>

using namespace std;
using namespace operations_research::lattle;
//using namespace arma;

constexpr auto		   EPSILON = 1.0e-6;	/**< Precision variable.*/
constexpr auto		   OMEGA = 1.0e+6;		/**< Precision variable.*/
constexpr auto		   TIMESTEP = 15;		/**< Discretise the horizon (1day) with a step of 15min.*/

#endif //HEADER_H
