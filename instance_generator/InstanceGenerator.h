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
	*	\param instance							instance object of the protobuf 
	*	\param hubs_number 						number of hubs
	*	\param rnd_graph_density				target density of the space graph
	*	\param time_horizon						time_horizon
	*	\param line_numb						number of lines, i.e., number of paths in the space graph
	*	\param max_length_line					maximal lenght of a line
	*	\param max_numb_rotations_per_line		maximal number of rotations per line, i.e., given a line it is the number of paths in the space-time graph whose projection onto the space-graph is the line
	*	\param max_travelling_time				maximal travelling time between two hubs
	*	\param travelling_time_perturbation		perturbation 0<p<1 for the travelling time t; the resulting travelling time is t'=t(1+-p). Default p=0 
	*	\param max_vehicle_capacity				maximal vehicle capacity. default is 100
	* 
	*	1) Randomly generate a (space) graph with hubs_number vertices and whose density is rnd_graph_density via 
	*		the extended Barabàsi-Albert algorithm (https://www.degruyter.com/document/doi/10.1515/9781400841356/html). 
	*		This results in a few highly connected (large degree), while most depots only have a few connections. 
	*	2) Randomly generate line_numb paths in the space graph with length at most max_length_line. 
	*		The arcs of a path are sampled with discrete_distribution where the weights of (i,j) is deg(i)+deg(j)
	*	3) Sample the travelling times of the arcs with uniform distribution in [1,max_travelling_time]
	*	4) Build the rotation for each line. Start-time is randomly sampled with uniform distribution in [0,time_horizon[ Then, durations are assigned to the arcs composing the line.
	*		A positive/negative perturbation is applied with probability 0.5, the perturbatino is sampled with uniform distribution in [0,travelling_time_perturbation]. 
	*	5) Randomly generate a vehicle for each line-rotation. Vehicle capacity is max_vehicle_capacity*f, where f is sampled with uniform distribution in [0,1]
	*/
	void generate_logistic_network(operations_research::lattle::Instance& instance, const int& hubs_number, const double& rnd_graph_density, const int& time_horizon, 
	 const int& line_numb, const int& max_length_line, const int& max_numb_rotations_per_line, const int& max_travelling_time,
	 const double& travelling_time_perturbation = 0.0, const double& max_vehicle_capacity = 100.0) const;

	/*
	* \brief build the random graph according to the extended Barab�si-Albert algorithm (see 1 in generate_logistic_network)
	* \param hubs_number 
	* \param rnd_graph_density 
	* \return graph data structure
	*/
	Graph build_random_graph(const int& hubs_number, const double& rnd_graph_density) const;
	/*
	* \brief build the weights on the arcs of the graph to sample vehicles based on the extremes degree: W_ij = (deg(i)+deg(j)) 
	*
	* \param random graph 
	* \return weights 
	*/
	vector<double> build_arc_weights(const Graph& graph) const;
	vector<vector<double>> build_arc_weights_per_adjacency_lists(const Graph& graph, const vector<double>& arc_weights) const;
	/*
	* \brief add dimensions to the network
	*
	* \param network
	*/
	void add_dimensions(operations_research::lattle::LogisticsNetwork& network) const;
	/*
	* \brief add the hubs to the network
	*
	* \param network 
	* \param graph 
	*/
	void add_hubs(operations_research::lattle::LogisticsNetwork& network, const Graph& graph) const;
	/*
	* sample the line-rotations (see 4 in generate_logistic_network)
	* 
	* \param network
	* \param graph
	* \param arc_weights
	* \param arc_weights_adj_lists 
	* \param time_horizon 
	* \param line_numb 
	* \param max_length_line 
	* \param max_numb_rotations_per_line 
	* \param max_travelling_time 
	* \param travelling_time_perturbation 
	* 
	* \return vehicle number
	*/
	int add_line_rotations(operations_research::lattle::LogisticsNetwork& network, const Graph& graph, const vector<double>& arc_weights, const vector<vector<double>>& arc_weights_adj_lists,
	 const int& time_horizon, const int& line_numb, const int& max_length_line, const int& max_numb_rotations_per_line, const int& max_travelling_time, const double& travelling_time_perturbation) const;
	/*
	* sample the lines (see 2 in generate_logistic_network)
	* 
	* \param graph
	* \param max_length_line
	* \param arc_weights
	* \param arc_weights_adj_lists 
	* 
	* \return vehicle number
	*/
	vector<int> generate_line(const Graph& graph, const int& max_length_line, const vector<vector<double>>& arc_weights_adj_lists, const vector<double>& arc_weights) const;	
	/*
	* sample the travelling times and store them in travelling_times (see 3 in generate_logistic_network)
	*	For each arc in the line, add the travelling time in travelling_times, unless it was already sampled
	* 
	* \param travelling_times
	* \param line
	* \param max_travelling_time
	* 
	* \return vehicle number
	*/
	void add_travelling_times(map<pair<int,int>, int>& travelling_times, const vector<int>& line, const int& max_travelling_time) const;
	/*
	* add the line rotations to the logistic network object
	* 
	* \param network
	* \param graph
	* \param line_numb
	* \param line 
	* \param rotations 
	* 
	*/
	void add_line_rotation(operations_research::lattle::LogisticsNetwork& network, const Graph& graph,
	const int& line_numb, const vector<int>& line, const vector<vector<int>>& rotations) const;
	/*
	* add the entry (x,y) in the distance matrix: NOT USED AS PER NOW
	* 
	* \param network
	* \param x
	* \param y
	* 
	*/
	void add_distance_matrix_entry(operations_research::lattle::LogisticsNetwork& network, const string& x, const string& y) const;
	/*
	* fill object operations_research::lattle::DateTimeRange given google::type::DateTime
	* 
	* \param range
	* \param time
	* 
	*/
	void add_time_range(operations_research::lattle::DateTimeRange& range, const google::type::DateTime& time) const;
	/*
	* sample vehicle and add them to the network (see 5 in generate_logistic_network)
	* 
	* \param range
	* \param time
	* 
	*/
	void add_vehicles(operations_research::lattle::LogisticsNetwork& network, const int& vehicle_number, const int& max_vehicle_capacity) const;

	/**
	*	\brief build the logistic network and print the .textproto
	* 
	*	\param instance						instance object of the protobuf 
	*	\param shipment_number 				number of shipments
	*	\param time_horizon					time_horizon
	*	\param max_path_length				maximal lenght of a path (in terms of hubs) that a shipment can do to be delivered
	*	\param min_shipment_weight			minimal weight of a shipment
	*	\param max_shipment_weight			maximal weight of a shipment
	*	\param shipment_weight_shape		parameter to sample shipment weights with Lomax distribution 
	*	\param max_tries					maximal tries to sample the shipment path
	* 
	*	1) Build the space-time network based on the lines and rotations
	*	2) Sample the shipment weights based on the Lomax distribution
	*	3) Build the hubs weights. Hub h weight is = number of vertices is the space graph - neighbours of h. 
	*	4) Sample the shipments paths. The first vertex (h,t) is sampled as hub h with discrete distribution with weights in 3 (to prefer less connected hubs as starting point)
	*		and t with uniform distribution in [0,time_horizon[. We sample the next vertex from the neighbours of (h,t) and so on..
	*		At each step, a path may be interrupted with probability 1/(max_path_length - current_path_length + 1).
	*/
	void generate_shipments(operations_research::lattle::Instance& instance, const int& shipment_number,
		const int& time_horizon, const int& max_path_length, const int& min_shipment_weight = 1, const int& max_shipment_weight = 100,
		const double& shipment_weight_shape = 0.1, const int& max_tries = 50) const;
	/*
	* fill the shipment object and add it to the instance object
	* 
	* \param instance
	* \param shipment_number
	* \param source_hub
	* \param destination_hub 
	* \param departure_time 
	* \param arrival_time 
	* \param weight 
	* 
	*/
	void add_shipment(operations_research::lattle::Instance& instance, const int& shipment_number, 
	const string& source_hub, const string& destination_hub, const int& departure_time, const int& arrival_time, const int& weight) const;
	/*
	* fill the departure time (google::type::DateTime) of the shipment
	* 
	* \param shipment
	* \param time_departure
	* 
	*/
	void add_departure_time(operations_research::lattle::Shipment& shipment, const google::type::DateTime& time_departure) const;
	/*
	* fill the arrival time (operations_research::lattle::DateTimeRange) of the shipment given the time_arrival object
	* 
	* \param shipment
	* \param time_arrival
	* 
	*/
	void add_arrival_time(operations_research::lattle::Shipment& shipment, const google::type::DateTime& time_arrival) const;

};

#endif // !INSTANCEGENERATOR_H