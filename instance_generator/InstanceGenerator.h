#ifndef INSTANCEGENERATOR_H
#define INSTANCEGENERATOR_H



#include "../graph_utils/Header.h"
#include "../graph_utils/Utils.h"

#include "../graph_utils/SpaceTimeNetwork.h"


/**
*	\class InstanceGenerator.
*	\brief	This class generate .textproto for the logisticsNetwork and logisticsNetworkState.
*/


class InstanceGenerator {

protected:

	/**
	*	name of the .textproto file
	*/
	string name;
	/**
	* description contains info regarding the instance file that is generated. Its content is print in a name_info.txt file
	*/
	string description;

	/**
	* random_seed to use in the randomized steps
	*/
	unsigned int random_seed;

public:

	/**
	 * \brief Constructor by default.
	 */
	InstanceGenerator();
	/**
	 * \brief Constructor by data. The call to the constructor 
	 */
	InstanceGenerator(const string& name, const string& description, const unsigned int& random_seed);
	/**
	 * \brief Copy constructor.
	 *
	 */
	InstanceGenerator(const InstanceGenerator& instanceGenerator);
	/**
	*	\brief	Operator assignment
	*/
	InstanceGenerator& operator=(const InstanceGenerator& instanceGenerator);

	/**
	 * \brief Destructor.
	 *
	 */
	~InstanceGenerator();

	/**
	*	\brief build the logistic network and print the .textproto
	* 
	*	\param hubs_number 
	*	\param timesteps 
	*	\param num_vehicles_per_step			number of vehicles leaving every time step (deterministic)
	*	\param max_vehicle_duration				maximal number of time steps a vehicle takes to reach its destination. The actual number is sampled uniformly from {1,...,`max_vehicle_duration`}
	*	\param max_vehicle_capacity				maximum scalar capacity of vehicles. The actual capacity is sampled uniformly from [0,`max_vehicle_capacity`)
	*	\param unit_capacities					if `True`, all vehicle capacities are set to 1. The argument `max_vehicle_capacity` is ignored in this case
	*	\param vehicle_sampling_inv_temp		when sampling the vehicles for a given time step, probabilities for the vehicles are given by `softmax(vehicle_sampling_inv_temp* (degree_vehicle_sender + degree_vehicle_receiver))`. 
	*											This way, vehicles connecting two high-degree nodes are sampled more frequently
	*	\param network_connectivity				`m` parameter of the extended Barab�si-Albert model 
	*	\param initial_number_connected_nodes	`m0` parameter of the extended Barab�si-Albert model
	* 
	*	First, generates a (space) graph of depots and connections between them according to
	*	the extended Barab�si-Albert algorithm (https://www.degruyter.com/document/doi/10.1515/9781400841356/html). 
	*	The algorithm is found in https://github.com/VictorSeven/CNetwork
	*	This results in a few highly connected (large degree), while most depots only have a few connections. 
	*	The time-expanded representation is then generated from the original graph by
	*	sampling a pre-specified number of vehicles at each time step, each of which
	*	starts at the current time step and ends a random number of time steps later
	*	(taking care to not exceed the specified total number of time steps). 
	* 
	*	In terms of lattle.proto objects.
	*	A line corresponds to paths either equal to (i-j), FOR EACH arc (i,j) of the graph.
	*	A rotation corresponds to a sampled vehicle. Remark that we MUST assign at least one vehicle to each rotation.
	*/
	void generate_logistic_network(const int& hubs_number, const int& time_horizon, const int& dimension_number, const int& max_length_line, const int& num_vehicles_per_step, const int& max_vehicle_duration,
		const double& max_vehicle_capacity = 100.0 , const double& vehicle_sampling_inv_temp = 0.01, 
		const int& new_connections_per_node = 2) const;

	/*
	* \brief build the random graph according to the extended Barab�si-Albert algorithm (https://www.degruyter.com/document/doi/10.1515/9781400841356/html). 
	* 	The algorithm is found in https://github.com/VictorSeven/CNetwork
	*	This results in a few highly connected (large degree), while most depots only have a few connections. 
	*
	* \param hubs_number : int
	* \param new_connections_per_node : int
	* \return graph data structure
	*/
	Graph build_random_graph(const int& hubs_number, const int& new_connections_per_node) const;

	/*
	* \brief build the weights on the arcs of the graph to sample vehicles based on the extremes degree: W_ij = vehicle_sampling_inv_temp * (deg(i)+deg(j)) 
	*
	* \param random graph : Graph 
	* \param vehicle_sampling_inv_temp : see generate_logistic_network doc
	* \return weights : vector<double>
	*/
	vector<double> build_arc_weights(const Graph& graph, const double& vehicle_sampling_inv_temp) const;
	vector<vector<double>> build_arc_weights_per_adjacency_lists(const Graph& graph, const vector<double>& arc_weights) const;

	void add_dimensions(operations_research::lattle::LogisticsNetwork& network, const int& dimension_number) const;
	void add_hubs(operations_research::lattle::LogisticsNetwork& network, const Graph& graph) const;
	/*
	* add the line rotations to the logistic network: sample randomly + insertion
	* 
	* \param network
	* \param graph
	* \param max_length_line
	* \param time_horizon : int
	* \param num_vehicles_per_step : int
	* \param arcs_number_graph : int
	* \param arc_weights 
	* \param arc_weights_adj_lists 
	* 
	* \return number of vehicles 
	*/
	int add_line_rotations(operations_research::lattle::LogisticsNetwork& network, const Graph& graph, const int& max_length_line,
		const int& time_horizon, const int& num_vehicles_per_step, const int& max_vehicle_duration, 
		const vector<double>& arc_weights, const vector<vector<double>>& arc_weights_adj_lists) const;
	void add_line_rotation(operations_research::lattle::LogisticsNetwork& network, const Graph& graph, 
		const int& line_numb, const vector<int>& line, const vector<vector<int>>& rotations) const;
	
	
	void add_distance_matrix_entry(operations_research::lattle::LogisticsNetwork& network, const string& x, const string& y) const;
	void add_time_range(operations_research::lattle::DateTimeRange& range, const google::type::DateTime& time) const;
	void add_vehicles(operations_research::lattle::LogisticsNetwork& network, const int& vehicle_number, const int& max_vehicle_capacity) const;

	/*
	* \brief Adds random parcels to a time-expanded transportation network.

	* The network may be created using `make_random_network`, whose outputs can be
	* passed as `state, network, distances`. This function adds parcels to the
	* network by first sampling a parcel weight (from a truncated Pareto
	* distribution [1]), a start node from the time expanded network, and then
	* sampling trucks to make a parcel route. Routes end probabilistically after
	* each truck to give an average route length of `mean_route_length`. The
	* time-expanded network (`state`) should normally be pruned (using
	* `prune_network` with `prune_parcels=False`) before adding parcels with this
	* function. This method ensures that the network is completely solvable.
	*
	*	\param rng: Random number generator.
	*	\param network: the logistic network where to add the parcels
	*	\param distances: Resistance distance matrix of `network`, as returned by
	*	`make_random_network`. This is used for sampling trucks in such a way that
	*	 they are more likely to bring the parcel further from their start hub. How
	*	much this is taken into account is conrolled by the `dist_inv_temp` parameter.
	*	\param num_parcels: Number of parcels to be included in the network.
	*	\param mean_route_length: Average length of a parcel route (in time steps). After
	*	 each time step of a parcel route, it is randomly decided whether to
	*	terminate the route, with probability 1 / `mean_route_length`. This way, a
	*	parcel route lasts `sum((1 - 1 / mean_route_length)**k)` time steps in
	*	expectation. This geometric sum approaches `mean_route_length` as the
	*	number of time steps in `state` becomes larger.
	*	\param min_parcel_weight: The scale (`m`) parameter of the Pareto distribution [1]
	*	 from which parcel weights are sampled.
	*	\param max_parcel_weight: The maximum weight of parcels. The parcel weights are
	*	sampled from a truncated Pareto distribution using rejection sampling to
	*	keep weights below `max_parcel_weight`.
	*	\param parcel_weight_shape: The shape (`a`) parameter of the Pareto distribution
	*	[1] from which parcel weights are sampled.
	*	\param unit_weights: If `True`, all parcel weights are set to 1 instead of being
	*	sampled from a Pareto distribution.
	*	\param start_inv_temp: The start node hub is sampled from a Boltzmann distribution
	*	with exponent `-start_inv_temp * degree(hub)` to preferably start at hubs
	*	 with lower degree.
	*	\param dist_inv_temp: When sampling parcel routes it is often posssible to choose
	*	 between multiple trucks. In these cases, the truck is chosen among the
	*	available trucks by sampling from a Boltzmann distribution with exponent
	*	`dist_inv_temp * dist_from_start` (where the distance to the start node is
	*	measured in terms of resistance distance, i.e. `distances`). This
	*	encourages parcels to go further from their start nodes.
	*	\param max_tries: If, after a parcel route has been sampled, this parcel route
	*	doesn't actually use any real trucks (instead, the parcel just stays at
	*	its start hub), we try again. This is done for a maximum of `max_tries`
	*	times (after which a 'staying' parcel route will simply be accepted). The
	*	parcel weight is reduced by 10% each try to make it easier to place the
	*	parcel.
	*	\param cut_capacities: After all parcels have been placed in the network, there is
	*	normally still capacity left in some trucks. `cut_capacities` is a number
	*	between 0 and 1 that specifies how much of this excess capacity should be
	*	removed (by reducing the trucks' capacities). Thus, 1 means all excess
	*	capacity is cut and 0 means no capacity is cut.
	*	
	*	The new state (time-expanded network), now with `num_parcels` parcels added,
	*	as well a dictionary mapping each parcel id to a list of nodes (in
	*	(location, time) format) representing a solution route for that parcel.
	*/
	void generate_parcels(const operations_research::lattle::LogisticsNetwork& network, const SpaceTimeNetwork& st_network, const int& parcel_number,
		const int& timesteps, const int& mean_path_length, const int& min_parcel_weight = 1, const int& max_parcel_weight = 100,
		const double& parcel_weight_shape = 0.1, const bool& unit_weights = false, const double& start_inv_temp = 0.1, const double& dist_inv_temp = 0.1,
		const int& max_tries = 50, const double& cut_capacities = 0.0) const;

};

#endif // !INSTANCEGENERATOR_H