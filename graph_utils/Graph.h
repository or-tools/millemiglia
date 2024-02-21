#ifndef GRAPH_H
#define GRAPH_H

#pragma once

#include "Header.h"
#include "Utils.h"

#include "Vertex.h"

/**
*	\class InstanceParser.
*	\brief	This class parse the network, the parcel and the state files. Additional data structure are contained
*/


class Graph {

protected:

	/**
	*	each vertex correspond to a hub
	*/
	vector<Vertex> vertices;

	/**
	*	key = name, value = id
	*/
	unordered_map<string, int> vertex_dictionary;

	/**
	*	number of arcs
	*/
	vector<pair<int, int>> arcs;

	/**
	*	key = name, value = id
	*/
	vector<unordered_map<int, int>> arcs_dictionary;

public:

	/**
	 * \brief Constructor by default.
	 */
	Graph();
	/**
	 * \brief Constructor by data.
	 * 
	 * \param number of hubs
	 */
	Graph(const int& hub_number);
	/**
	 * \brief Copy constructor.
	 *
	 */
	Graph(const Graph& graph);
	/**
	*	\brief	Operator assignment
	*/
	Graph& operator=(const Graph& graph);
	/**
	 * \brief Destructor.
	 *
	 */
	~Graph();

	/**
	*	\brief add a vertex
	*
	*	\param hub name
	*/
	void add_vertex(const string& name);
	/**
	*	\brief add a neighbour to key both (key,neighbour) and (neighbour,key)
	*
	*	\param key
	*	\param neighbour
	*	\param line in the network protocol buffer that uses arc (key, neighbour)
	*/
	void add_neighbour(const string& key, const string& neighbour, const string& line);

	const vector<Vertex>& get_vertices() const;
	const int get_vertex_number() const;
	const Vertex& get_vertex(const int& id) const;
	const Vertex& get_vertex(const string& name) const;
	const vector<pair<int, int>>& get_arcs() const;
	const int get_arc_position(const int& id1, const int& id2) const;

	const string toString() const;

};

#endif // !GRAPH_H