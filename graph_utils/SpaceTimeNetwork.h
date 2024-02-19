#ifndef SpaceTimeNetwork_H
#define SpaceTimeNetwork_H

#pragma once

#include "Header.h"
#include "Utils.h"

#include "Graph.h"
#include "VertexST.h"
#include "ArcST.h"
#include "Lorry.h"
#include "DepthFirstSearch.h"

/**
*	\class SpaceTimeNetwork.
*	\brief	This class represents the space-time network
*/


class SpaceTimeNetwork {

protected:

	/**
	*	logistics network object
	*/
	operations_research::lattle::LogisticsNetwork network;

	/**
	*	underlying graph (only space dimension)
	*/
	Graph underlying_graph;

	/**
	*	each vertex correspond to a (hub,time)
	*/
	vector<VertexST> vertices;

	/**
	*	each arc correspond to a movement in the space-time network
	*/
	vector<ArcST> arcs;

	/**
	*	key1 = hub id in graph, key2 = encoded time, value = vertex id in ST network
	*/
	unordered_map<int, unordered_map<int, int>> vertex_dictionary;

	/**
	*	A topological order of the vertices of the space-time network which is a directed acyclic graph
	*/
	vector<int> topological_order;

	/**
	*	Adjacency lists used in the graph algorithms, e.g., dfs, Yen, spp
	*		the space-time network is a multigraph, instead adjacency_lists_graph_algorithms contains only one arc between two vertices.
	*		adjacency_lists_graph_algorithms[u] = adj list of vertex u = pairs (v,cost) where (u,v) is an arc in the network and cost is its associated cost
	*		This data structure is mutable 
	*/
	vector<unordered_map<int,double>> adjacency_lists_graph_algorithms;
	/**
	*	current arc processed in an adjacency list = the space-time network is a multi-graph used in the k-shortest path algorithm
	*/
	vector<unordered_map<int, int>> adjacency_lists_arc_position;

	/**
	*	available lorries
	*/
	vector<Lorry> lorries;

public:

	/**
	 * \brief Constructor by default.
	 */
	SpaceTimeNetwork();
	/**
	 * \brief Constructor by data: string path to LogisticsNetwork protocol buffer and time horizon 
	 * 
	 * \param network protocal buffer object
	 */
	SpaceTimeNetwork(const string& network_file, const int& time_horizon);
	/**
	 * \brief Copy constructor.
	 *
	 */
	SpaceTimeNetwork(const SpaceTimeNetwork& spaceTimeNetwork);
	/**
	*	\brief	Operator assignment
	*/
	SpaceTimeNetwork& operator=(const SpaceTimeNetwork& spaceTimeNetwork);

	/**
	 * \brief Destructor.
	 *
	 */
	~SpaceTimeNetwork();

	/**
	*	\brief parse the network proto buffer and build the associated object.
	*		Build object space-time network
	*
	*	\param path to the logistics network file
	*/
	void parse_logistic_network(const string& network_file);

	/**
	*	\brief Build graph object
	*
	*	\param network protocal buffer object
	*/
	void build_underlying_graph(const operations_research::lattle::LogisticsNetwork& network);

	/**
	*	\brief Build vector of vertices (i,t) for each hub i and each time t
	*
	*	\param time_horizon 
	*/
	void build_vertices(const int& time_horizon);

	/**
	*	\brief Build vector of arcs
	*
	*	\param network protocal buffer object
	*/
	void build_arcs(const operations_research::lattle::LogisticsNetwork& network, const int& time_horizon);

	/**
	*	\brief Build vector of vehicles
	*
	*	\param network protocal buffer object
	*/
	void build_lorries(const operations_research::lattle::LogisticsNetwork& network);

	/**
	*	\brief Build vector of vertices
	*
	*	\param hub name
	*	\param time object 
	*/
	void add_vertexST(const string& hub, const operations_research::lattle::DateTimeRange& time, const int& time_horizon);
	/**
	*	\brief Build vector of vertices
	*
	*	\param hub_id
	*	\param time encoded time
	*/
	void add_vertexST(const int& hub, const int& time);

	/**
	*	\brief Add outgoing arc to adjacency list of a vertex
	*
	*	\param arc
	*/
	void add_to_adjacency_list_out(const ArcST& arc);
	/**
	*	\brief Add ingoing arc to adjacency list of a vertex
	*
	*	\param arc
	*/
	void add_to_adjacency_list_in(const ArcST& arc);
	/**
	*	\brief Sort arcs in adjacency lists by increasing cost, time
	*/
	void sort_adjacency_list_out();
	/**
	*	\brief Build topological order as in "Cormen, Thomas H., et al. Introduction to algorithms. MIT press, 2022."
	*		call Depth-First Search and order the vertices by decreasing DFS finish time
	*/
	void compute_topological_order();
	/**
	*	\brief Get vertices (hub,t) where t >= time 
	*
	*	\param hub : int
	*	\param time : int
	*	\return vertices ids: vector<int>
	*/
	const vector<int> get_hub_time_after_t(const int& hub, const int& time) const;
	/**
	*	\brief Get vertices (hub,t) where t <= time
	*
	*	\param hub : int
	*	\param time : int
	*	\return vertices ids: vector<int>
	*/
	const vector<int> get_hub_time_before_t(const int& hub, const int& time) const;
	/**
	*	\brief for each adjacency list the arc position is set to zero
	*/
	void initialise_adjacency_lists_arc_position();
	/**
	*	\brief for each adjacency list graph algorithm
	*/
	void initialise_adjacency_lists_graph_algorithms(const string& type);
	/**
	*	\brief restore value of arc (u,v) in adjacency list. 
	*		If type is length set value to 1, otherwise set it to the best arc in the multi-graph
	*	
	*	\param type of value (lenght,cost, travelling time)
	*	\param vertex id u
	*	\param vertex id v
	*/
	void restore_adjacency_lists_graph_value(const string& type, const int& u, const int& v);
	/**
	*	\brief modify value of arc (u,v) in adjacency list.
	*		If type is length do nothing, otherwise if there is another arc between u and v fix the values to those of that arc
	*		update list arc position container as well.
	*		if there is no other arc, set the value to infinity
	*
	*	\param type of value (lenght,cost, travelling time)
	*	\param vertex id u
	*	\param vertex id v
	*	\param position of the arc to modify in the adjacency list
	*/
	void modify_adjacency_lists_graph_value(const string& type, const int& u, const int& v, const int& position_in_adj_list);


	const operations_research::lattle::LogisticsNetwork& get_network() const;
	const Graph& get_underlying_graph() const;
	const vector<VertexST>& get_vertices() const;
	const vector<ArcST>& get_arcs() const;
	const VertexST& get_vertex(const string& hub, const operations_research::lattle::DateTimeRange& time) const;
	const VertexST& get_vertex(const int& hub, const int& time) const; 
	const vector<unordered_map<int, double>>& get_adjacency_lists_graph_algorithms() const;
	const vector<unordered_map<int, int>>& get_adjacency_lists_arc_position() const;
	const vector<int>& get_topologiacal_order() const;

	const string toString() const;

};

#endif // !SpaceTimeNetwork_H