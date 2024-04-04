#include "InstanceGenerator.h"

InstanceGenerator::InstanceGenerator() {
}

InstanceGenerator::InstanceGenerator(const string& name, const string& description, const unsigned int& random_seed) {
	this->name = name;
	this->description = description;
	this->random_seed = random_seed;
}

InstanceGenerator::~InstanceGenerator() {
}

InstanceGenerator::InstanceGenerator(const InstanceGenerator& instanceGenerator) {
	this->name = instanceGenerator.name;
	this->description = instanceGenerator.description;
	this->random_seed = instanceGenerator.random_seed;
}

InstanceGenerator& InstanceGenerator::operator=(const InstanceGenerator& instanceGenerator) {
	this->name = instanceGenerator.name;
	this->description = instanceGenerator.description;
	this->random_seed = instanceGenerator.random_seed;
	return *this;
}

void InstanceGenerator::generate_logistic_network(operations_research::lattle::Instance& instance, const int& hubs_number, const int& time_horizon, 
	const int& dimension_number, const int& max_length_line, const int& num_vehicles_per_step, const int& max_vehicle_duration,
	const double& max_vehicle_capacity, const double& vehicle_sampling_inv_temp,
	const int& new_connections_per_node) const {

	//Build the protobuffer associated with the network
	operations_research::lattle::LogisticsNetwork logisticsNetwork;
	logisticsNetwork.set_name(this->name);

	//add dimensions
	add_dimensions(logisticsNetwork, dimension_number);
	
	//generate the transportation graph by means of the Barabasi-Albert algorithm.
	Graph random_graph = build_random_graph(hubs_number,new_connections_per_node);

	//Fill the hubs structure in the logistic network object.
	add_hubs(logisticsNetwork, random_graph);

	//vehicle frequencies depend on the importance of an edge(degree of sender + degree of receiver).
	vector<double> arc_weights = build_arc_weights(random_graph, vehicle_sampling_inv_temp);
	vector<vector<double>> arc_weights_adj_lists = build_arc_weights_per_adjacency_lists(random_graph, arc_weights);

	//fill line_rotation structure in the logistic network object
	int vehicle_number = add_line_rotations(logisticsNetwork, random_graph, max_length_line, time_horizon, num_vehicles_per_step, max_vehicle_duration, arc_weights, arc_weights_adj_lists);
	
	//Sample vehicle capacities
	add_vehicles(logisticsNetwork, vehicle_number, max_vehicle_capacity);

	instance.set_allocated_network(&logisticsNetwork);

}

Graph InstanceGenerator::build_random_graph(const int& hubs_number, const int& new_connections_per_node ) const {
	vector<std::pair<int,int>> edges;
	vector<int> repeated_nodes;
	for(int i = 1; i <= new_connections_per_node ; i++){
		edges.push_back({1,i+1});
		repeated_nodes.push_back(1);
		repeated_nodes.push_back(i+1);	
	}
    
	int source = new_connections_per_node + 1;
     
	 while(source <= hubs_number){
		// random subset of repeated nodes.
		vector<int> targets = random_subset(repeated_nodes, new_connections_per_node);
		for(auto node: targets){
			edges.push_back({node,source});
			repeated_nodes.push_back(source);
			repeated_nodes.push_back(node);
		}
		source++;
	 }

     Graph random_graph(hubs_number);
	for (int i = 1; i <= hubs_number; i++) {
		string name = "h_" + to_string(i);
		random_graph.add_vertex(name);
	}

	for (auto edge:edges) {
		string id1 = "h_" + to_string(edge.first);
		string id2 = "h_" + to_string(edge.second);
		random_graph.add_neighbour(id1, id2, "");
	}

	return random_graph;
}

vector<double> InstanceGenerator::build_arc_weights(const Graph& graph, const double& vehicle_sampling_inv_temp) const {
	vector<double> arc_weights(graph.get_arcs().size(), 0.0);
	for (int i = 0; i < graph.get_arcs().size(); i++) {
		const Vertex& v1 = graph.get_vertex(graph.get_arcs().at(i).first);
		const Vertex& v2 = graph.get_vertex(graph.get_arcs().at(i).second);
		arc_weights.at(i) = vehicle_sampling_inv_temp * (v1.get_neighbours_number() + v2.get_neighbours_number());
	}
	return arc_weights;
}

vector<vector<double>> InstanceGenerator::build_arc_weights_per_adjacency_lists(const Graph& graph, const vector<double>& arc_weights) const {
	vector<vector<double>> weights_per_adj_lists(graph.get_vertex_number(), vector<double>());
	for (int i = 0; i < graph.get_vertices().size(); i++) {
		const Vertex& v = graph.get_vertex(i);
		for (const auto& arc : v.get_adjacency_list_out()) {
			int arc_pos = graph.get_arc_position(v.get_id(), arc.first);
			assert(arc_pos >= 0 && arc_pos < arc_weights.size());
			weights_per_adj_lists.at(v.get_id()).push_back(arc_weights.at(arc_pos));
		}
	}
	return weights_per_adj_lists;
}

void InstanceGenerator::add_dimensions(operations_research::lattle::LogisticsNetwork& network, const int& dimension_number) const {
	operations_research::lattle::ValueDimension distance, time;
	operations_research::lattle::ValueDimension weight;
	distance.set_dimension("distance"); 
	distance.set_value(1); 
	time.set_dimension("time"); 
	time.set_value(1);
	weight.set_dimension("weight"); 
	weight.set_value(1);
	network.mutable_dimensions()->Add(move(distance));
	network.mutable_dimensions()->Add(move(time));
	network.mutable_dimensions()->Add(move(weight));
	if (dimension_number > 1) {
		operations_research::lattle::ValueDimension pallet;
		pallet.set_dimension("pallet");
		network.mutable_dimensions()->Add(move(pallet));
	}
}

void InstanceGenerator::add_hubs(operations_research::lattle::LogisticsNetwork& network, const Graph& graph) const {
	for (int i = 0; i < graph.get_vertex_number(); i++) {
		const Vertex& v = graph.get_vertex(i);
		operations_research::lattle::Hub h;
		h.set_name(v.get_name());
		network.mutable_hubs()->Add(move(h));
	}
}

int InstanceGenerator::add_line_rotations(operations_research::lattle::LogisticsNetwork& network, const Graph& graph, const int& max_length_line,
	const int& time_horizon, const int& num_vehicles_per_step, const int& max_vehicle_duration, 
	const vector<double>& arc_weights, const vector<vector<double>>& arc_weights_adj_lists) const {
	// vehicles for time-expanded network (sample one time step at a time).
	map<vector<int>, vector<vector<int>>> rotations_per_line;
	vector<double> expo = exponential(arc_weights);
	int vehicle_number = 0;
	for (int t = 1; t < time_horizon; t++) {
		// Sample vehicles and add to the network
		for (int l = 0; l < num_vehicles_per_step; l++) {
			//generate a path in the random_graph + durations s.t. the time horizon is not exceeded 
			vector<int> path;
			unordered_set<int> path_set;
			int path_length = ElRandom::Uniform(1, max_length_line);
			//initialise path with first edge
			int edge = ElRandom::Discrete(expo);
			int id1 = graph.get_arcs().at(edge).first;
			int id2 = graph.get_arcs().at(edge).second;
			path_set.insert(id1);
			path_set.insert(id2);
			if (ElRandom::Uniform(0.0, 1.0) < 0.5) {
				path.push_back(id1);
				path.push_back(id2);
			}
			else {
				path.push_back(id2);
				path.push_back(id1);
			}
			int last_vertex = path.back();
			int duration = ElRandom::Uniform(0, max_vehicle_duration) + 1;
			vector<int> visits_time = { t, t + duration };

			bool is_path_augmented = true;
			while (is_path_augmented && path.size() < path_length && visits_time.back() < time_horizon && 
				!graph.get_vertex(last_vertex).get_adjacency_list_out().empty()) {
				//we allow max tries to draw a next vertex not already in the path and whose duration is feasible
				int tries = 0;
				int duration = ElRandom::Uniform(0, max_vehicle_duration) + 1;
				int edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
				int new_vertex = graph.get_vertex(last_vertex).get_out_going_by_position(edge);
				is_path_augmented = false;
				do {

					if (visits_time.back() + duration < time_horizon && path_set.find(new_vertex) == path_set.end()) {
						last_vertex = new_vertex;
						path.push_back(new_vertex);
						path_set.insert(new_vertex);
						visits_time.push_back(visits_time.back() + duration);
						is_path_augmented = true;
						break;
					}
					else {
						duration = ElRandom::Uniform(0, max_vehicle_duration-1) + 1;
						edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
						new_vertex = graph.get_vertex(last_vertex).get_out_going_by_position(edge);
					}

					tries++;
				} while (tries < arc_weights_adj_lists.at(last_vertex).size());


			}

			//store the path and the time info in the structure
			if (visits_time.back() < time_horizon) {
				vehicle_number++;
				map<vector<int>, vector<vector<int>>>::iterator finder = rotations_per_line.find(path);
				if (finder == rotations_per_line.end()) {
					vector<vector<int>> aux = { visits_time };
					rotations_per_line.insert(make_pair(path, aux));
				}
				else {
					finder->second.push_back(visits_time);
				}
			}
		}

	}

	int counter = 1;
	for (const auto& line_rotations : rotations_per_line) {
		add_line_rotation(network, graph, counter, line_rotations.first, line_rotations.second);
		counter++;
	}

	return vehicle_number;
}

vector<int> InstanceGenerator::generate_line(const Graph& graph, const int& max_length_line, const vector<vector<double>>& arc_weights_adj_lists, const vector<double>& expo) const{
	//generate a path in the random_graph 
	vector<int> path;
	unordered_set<int> path_set;
	//draw max length of the path
	int path_length = ElRandom::Uniform(1, max_length_line);
	//initialise path with first edge and randomly pick an orientation
	int edge = ElRandom::Discrete(expo);
	int id1 = graph.get_arcs().at(edge).first;
	int id2 = graph.get_arcs().at(edge).second;
	path_set.insert(id1);
	path_set.insert(id2);
	if (ElRandom::Uniform(0.0, 1.0) < 0.5) {
		path.push_back(id1);
		path.push_back(id2);
	}
	else {
		path.push_back(id2);
		path.push_back(id1);
	}

	//complete the line up to its length or until there exists an outgoing arc
	int last_vertex = path.back();
	bool is_path_augmented = true;
	while (is_path_augmented && path.size() < path_length && 
				!graph.get_vertex(last_vertex).get_adjacency_list_out().empty()) {
		//we allow max tries to draw a next vertex not already in the path and whose duration is feasible
		int tries = 0;
		int edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
		int new_vertex = graph.get_vertex(last_vertex).get_out_going_by_position(edge);
		is_path_augmented = false;
		do {
			if (path_set.find(new_vertex) == path_set.end()) {
				last_vertex = new_vertex;
				path.push_back(new_vertex);
				path_set.insert(new_vertex);
				is_path_augmented = true;
				break;
			}
			else {
				edge = ElRandom::Discrete(arc_weights_adj_lists.at(last_vertex));
				new_vertex = graph.get_vertex(last_vertex).get_out_going_by_position(edge);
			}

			tries++;
		} while (tries < arc_weights_adj_lists.at(last_vertex).size());
	}
	return path;
}

int InstanceGenerator::add_line_rotations(operations_research::lattle::LogisticsNetwork& network, const Graph& graph, const vector<double>& arc_weights, const vector<vector<double>>& arc_weights_adj_lists,
	 const int& time_horizon, const int& line_numb, const int& max_length_line, const int& max_numb_rotations_per_line, const int& max_time_duration) const {

	vector<double> expo = exponential(arc_weights);
	int vehicle_number = 0;
	//build lines
	for (int i = 0; i < line_numb; i++) {

		//generate a path in the random_graph 
		vector<int> line = generate_line(graph, max_length_line, arc_weights_adj_lists, expo);
		int line_number = i + 1;
		vector<vector<int>> time_infos;
		
		//add rotations associated to the line
		int max_rotations = ElRandom::Uniform(1, max_numb_rotations_per_line);

		for(int j=0; j<max_rotations; j++){

			operations_research::lattle::LineRotation rotation;
			string rotation_name = "l" + to_string(i + 1) + "_lr" + to_string(j + 1);

			int tries = 0;
			vector<int> time_info;
			bool is_rotation_good = false;
			while (tries < 50 && !is_rotation_good){
				is_rotation_good = true;
				int start_time = ElRandom::Uniform(1, time_horizon);
				time_info = {start_time};
				for(int l=0; l<line.size() - 1; l++) {
					int duration = ElRandom::Uniform(0, max_time_duration) + 1;
					int time = time_info.back() + duration;
					if(time < time_horizon){
						time_info.push_back(time);
					}else{
						is_rotation_good = false;
					}
				}
			}

			if(is_rotation_good){
				time_infos.push_back(time_info);
				vehicle_number++;
			}

		}

		//fill rotation proto
		if(!time_infos.empty()){
			add_line_rotation(network, graph, line_number, line, time_infos);
		}
	}
	return vehicle_number;
}

void InstanceGenerator::add_line_rotation(operations_research::lattle::LogisticsNetwork& network, const Graph& graph,
	const int& line_numb, const vector<int>& line, const vector<vector<int>>& rotations) const {

	//build line
	operations_research::lattle::Line line_proto;
	for (int i = 0; i < line.size(); i++) {
		string name = graph.get_vertex(line.at(i)).get_name();
		line_proto.add_hub_ids(name);
	}

	//add rotations to line
	for (int i = 0; i < rotations.size(); i++) {
		operations_research::lattle::LineRotation rotation;
		string name = "l" + to_string(line_numb) + "_lr" + to_string(i + 1);

		for (int j = 0; j < rotations.at(i).size() - 1; j++) {

			string start_hub = graph.get_vertex(line.at(j)).get_name();
			string end_hub = graph.get_vertex(line.at(j + 1)).get_name();
			int start_time = rotations.at(i).at(j);
			int end_time = rotations.at(i).at(j + 1);
			assert(start_time < end_time);

			//add departure 
			operations_research::lattle::DateTimeRange departure_time_range;
			google::type::DateTime time_departure = ::time_decoder(start_time);
			add_time_range(departure_time_range, time_departure);
			rotation.mutable_departure_times()->insert({ start_hub, departure_time_range });

			//add arrival 
			operations_research::lattle::DateTimeRange arrival_time_range;
			google::type::DateTime time_arrival = ::time_decoder(end_time);
			add_time_range(arrival_time_range, time_arrival);
			rotation.mutable_arrival_times()->insert({ end_hub, arrival_time_range });

		}

		//add cost + number of vehicles
		int price = rotations.at(i).back() - rotations.at(i).front();
		assert(price > 0);
		rotation.set_name(name);
		rotation.mutable_fixed_price()->mutable_separable()->set_constant_price(static_cast<double>(price));
		rotation.mutable_maximum_number_vehicles()->set_start_value(1);
		rotation.mutable_maximum_number_vehicles()->set_end_value(1);

		//add rotation to line
		line_proto.mutable_next_rotations()->Add(move(rotation));

	}
    line_proto.set_name("l" + to_string(line_numb));
	network.mutable_lines()->Add(move(line_proto));

}

void InstanceGenerator::add_distance_matrix_entry(operations_research::lattle::LogisticsNetwork& network, const string& x, const string& y) const {
	operations_research::lattle::DistanceMatrixEntry entry;
	entry.set_source_hub(x);
	entry.set_destination_hub(y);
	operations_research::lattle::ValueDimension distance_value, time_value;				//NOT CLEAR AT ALL
	distance_value.set_dimension("distance"); distance_value.set_value(1);
	time_value.set_dimension("time"); time_value.set_value(1);
	entry.mutable_weights()->Add(move(distance_value));
	entry.mutable_weights()->Add(move(time_value));
	network.mutable_distance_matrix()->Add(move(entry));
}

void InstanceGenerator::add_time_range(operations_research::lattle::DateTimeRange& range, const google::type::DateTime& time) const {
	range.mutable_first_date()->set_year(time.year());
	range.mutable_last_date()->set_year(time.year());
	range.mutable_first_date()->set_month(time.month());
	range.mutable_last_date()->set_month(time.month());
	range.mutable_first_date()->set_day(time.day());
	range.mutable_last_date()->set_day(time.day());
	range.mutable_first_date()->set_hours(time.hours());
	range.mutable_last_date()->set_hours(time.hours());
	range.mutable_first_date()->set_minutes(time.minutes());
	range.mutable_last_date()->set_minutes(time.minutes());
}

void InstanceGenerator::add_vehicles(operations_research::lattle::LogisticsNetwork& network, const int& vehicle_number, const int& max_vehicle_capacity) const {
	//build vehicle objects and add them to the logistic network					
	for (int l = 0; l < vehicle_number; l++) {
		operations_research::lattle::Vehicle v;
		operations_research::lattle::ValueDimension value;
		int capacity = max_vehicle_capacity * ElRandom::Uniform(0.0, 1 - EPSILON); //capacities drawn uniformly from [0,max_vehicle_capacity)
		value.set_dimension("weight");
		value.set_value(capacity);
		v.mutable_capacities()->Add(move(value));
		v.mutable_cost()->mutable_separable()->set_constant_price(capacity);
		v.set_name("v" + to_string(l + 1));
		network.mutable_vehicles()->Add(move(v));
	}
}

void InstanceGenerator::generate_shipments(operations_research::lattle::Instance& instance, const SpaceTimeNetwork& st_network, 
	const int& time_horizon, const int& shipment_number, const int& max_path_length, const int& min_shipment_weight, const int& max_shipment_weight, 
	const double& shipment_weight_shape, const double& start_inv_temp, const double& dist_inv_temp,
	const int& max_tries, const double& cut_capacities) const {

	// Sample shipments' weight
	vector<int> shipment_weights(shipment_number, 0);
	for (int j = 0; j < shipment_number; j++) {
			while (::strictly_less(shipment_weights.at(j), min_shipment_weight) || ::strictly_greater(shipment_weights.at(j), max_shipment_weight)) {
				double lomax_draw = ElRandom::Lomax(static_cast<double>(min_shipment_weight), shipment_weight_shape);
				shipment_weights.at(j) = max(1, static_cast<int>(floor(lomax_draw)));
			}
		}
	sort(shipment_weights.rbegin(), shipment_weights.rend());

	// Compute weights to assign to the hubs which are used to sample starting hubs for the shipments
	vector<double> hubs_weights(st_network.get_underlying_graph().get_vertex_number(), -1);
	for (const auto& vertex : st_network.get_underlying_graph().get_vertices()) {
		hubs_weights.at(vertex.get_id()) = static_cast<double>(vertex.get_neighbours_number());
	}

	// Build shipments
	int number = 0;
	for (int k = 0; k < shipment_number; k++){

		bool success = false;
		int tries = 0;
		int source_hub_id = -1;
		int destination_hub_id = -1;
		int departure_time = -1;
		int arrival_time = -1;

		while (tries <= max_tries && !success) {

			// Sample starting hub
			source_hub_id = ElRandom::Discrete(hubs_weights);
			departure_time = ElRandom::Uniform(0, time_horizon);
			int start_vertex_id = st_network.get_vertex(source_hub_id, departure_time).get_id();
			int last_vertex = start_vertex_id;

			// Sample a path in the st_network starting at the (source hub, departure_time) 
			int length = 1;
			while (true) {

				const VertexST& v = st_network.get_vertices().at(last_vertex);
				if (v.get_adjacency_list_out().size() == 0) {
					break;
				}

			    if (length >= max_path_length) {
					break;
				}

				int size = v.get_adjacency_list_out().size() - 1;
				int pos = ElRandom::Uniform(0, size);
				unordered_map<int, vector<int>>::const_iterator finder = v.get_adjacency_list_out().begin();
				advance(finder, pos);
				const VertexST& next_vertex = st_network.get_vertices().at(finder->first);
				last_vertex = next_vertex.get_id();

				if(v.get_id_in_graph() != next_vertex.get_id_in_graph()){
					length++;
				}

				// Cut the path?
				double prob_stop = 1.0 / (static_cast<double>(max_path_length - length + 1));
				bool is_to_stop = ElRandom::Bernoulli(prob_stop);
				if (is_to_stop) {
					if (v.get_id_in_graph() != next_vertex.get_id_in_graph()) {
						success = true;
						destination_hub_id = next_vertex.get_id_in_graph();
						arrival_time = next_vertex.get_time();
						break;
					}
				}

			}

			if(success){
				number++;
				string destination_name = st_network.get_underlying_graph().get_vertex(destination_hub_id).get_name();
				string source_name = st_network.get_underlying_graph().get_vertex(source_hub_id).get_name();
				add_shipment(instance, number, source_name, destination_name, departure_time, arrival_time, shipment_weights.at(k));
				break;
			}

			tries++;



		}


	}

}

void InstanceGenerator::add_shipment(operations_research::lattle::Instance& instance, const int& shipment_number, 
	const string& source_hub, const string& destination_hub, const int& departure_time, const int& arrival_time, const int& weight) const {
	operations_research::lattle::Shipment shipment;
	shipment.set_name("p_" + to_string(shipment_number));
	assert(source_hub != destination_hub);
	shipment.set_destination_hub(destination_hub);
	shipment.set_source_hub(source_hub);
	operations_research::lattle::ValueDimension value;
	value.set_dimension("weight");
	value.set_value(weight);
	shipment.mutable_size()->Add(move(value));
	google::type::DateTime time_departure = ::time_decoder(departure_time);
	shipment.set_allocated_departure_time(&time_departure);
	operations_research::lattle::DateTimeRange arrival_time_range;
	google::type::DateTime time_arrival = ::time_decoder(arrival_time);
	add_time_range(arrival_time_range, time_arrival);
	shipment.set_allocated_arrival_time(&arrival_time_range);
	instance.mutable_shipments()->Add(move(shipment));
}