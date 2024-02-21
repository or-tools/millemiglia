#ifndef LORRY_H
#define LORRY_H

#pragma once

#include "Header.h"
#include "Utils.h"

/**
*	\class Vehicle.
*	\brief	This class represent a vehicle
*/


class Lorry {

protected:

	int id;
	string name;

	/**
	*	vehicle object
	*/
	const operations_research::lattle::Vehicle& vehicle;
	/**
	*	path: arcs in the st_network traversed by the vehicle
	*/
	vector<int> path;
	/**
	*	capacities
	*/
	vector<int> capacities;
	/**
	*	vehicle current position in the st_network
	*/
	int current_position;
	/**
	*	loaded_commodities: for each arc in path, the commodities on that arc
	*/
	unordered_map<int, vector<int>> loaded_commodities;
	/**
	*	residual capacity: for each arc in path, the residual capacities of the vehicle on that arc
	*/
	unordered_map<int, vector<int>> loads_per_arc;

	/**
	*	vertex counter to assign unique id
	*/
	static int lastId;

public:

	/**
	 * \brief Constructor by data.
	 *
	 * \param number of hubs
	 */
	Lorry(const string& name, const operations_research::lattle::Vehicle& vehicle);
	/**
	 * \brief Copy constructor.
	 *
	 */
	Lorry(const Lorry& lorry);
	/**
	*	\brief	Operator assignment
	*/
	Lorry& operator=(const Lorry& lorry);

	/**
	 * \brief Destructor.
	 *
	 */
	~Lorry();

	/**
	*	\brief add a commodity in the vehicle, modify residual capacity
	*
	*	\param commodity : Commodity
	*	\param arc : int
	*/
	void add_commodity(const int& commodity_id, const vector<int>& sizes, const int& arc);

	static void restart_id_counter() {
		lastId = 0;
	}

	const int& get_id() const;
	const string& get_name() const;
	const operations_research::lattle::Vehicle& get_proto_vehicle() const;
	const vector<int>& get_path() const;
	const vector<int>& get_capacities() const;
	const int& get_current_position() const;
	const unordered_map<int, vector<int>>& get_loaded_commodities() const;
	const unordered_map<int, vector<int>>& get_loads_per_arc() const;

	void set_current_position(const int& pos);

	const string toString() const;

};

#endif // !LORRY_H