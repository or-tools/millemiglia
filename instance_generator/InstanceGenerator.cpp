#include "InstanceGenerator.h"
#include "CNetwork-master/source/CNet.cpp"

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

void InstanceGenerator::generate_logistic_network(const int& hubs_number, const int& time_horizon, 
	const int& dimension_number, const int& max_length_line, const int& num_vehicles_per_step, const int& max_vehicle_duration,
	const double& max_vehicle_capacity, const double& vehicle_sampling_inv_temp, const int& intial_numb_fully_connected_nodes,
	const int& new_connections_per_node) const {

	//Build the protobuffer associated with the network
	operations_research::lattle::LogisticsNetwork logisticsNetwork;
	logisticsNetwork.set_name(this->name);

	//add dimensions
	add_dimensions(logisticsNetwork, dimension_number);
	
	//generate the transportation graph by means of the Barabási-Albert algorithm with the implementation of Newman 
	Graph random_graph = build_random_graph(hubs_number, intial_numb_fully_connected_nodes, new_connections_per_node);

	//Fill the hubs structure in the logistic network object
	add_hubs(logisticsNetwork, random_graph);

	//vehicle frequencies depend on the importance of an edge(degree of sender + degree of receiver).
	vector<double> arc_weights = build_arc_weights(random_graph, vehicle_sampling_inv_temp);
	vector<vector<double>> arc_weights_adj_lists = build_arc_weights_per_adjacency_lists(random_graph, arc_weights);

	//fill line_rotation structure in the logistic network object
	int vehicle_number = add_line_rotations(logisticsNetwork, random_graph, max_length_line, time_horizon, num_vehicles_per_step, max_vehicle_duration, arc_weights, arc_weights_adj_lists);
	
	//Sample vehicle capacities
	add_vehicles(logisticsNetwork, vehicle_number, max_vehicle_capacity);

	//output protobuffer file
	string out;
	if (!google::protobuf::TextFormat::PrintToString(logisticsNetwork, &out)) {
		cerr << "Failed to write the logistic network file " << this->name << "\n";
		exit(EXIT_FAILURE);
	}
	fstream output_network("..\\test_cases\\" + this->name + ".textproto", ios::out);
	output_network << out;
	output_network.close();
	
	google::protobuf::ShutdownProtobufLibrary();

}

Graph InstanceGenerator::build_random_graph(const int& hubs_number, const int& intial_numb_fully_connected_nodes, const int& new_connections_per_node) const {
	CNetwork graph_rand(hubs_number);
	clock_t start = clock();
	graph_rand.create_albert_barabasi(hubs_number, intial_numb_fully_connected_nodes, new_connections_per_node, ElRandom::GetSeed());
	clock_t end = clock();
	double dur = (float)(end - start) / CLOCKS_PER_SEC;

	assert(hubs_number == graph_rand.get_node_count());

	//build our data structure to hold the random graph
	Graph graph = Graph(graph_rand.get_node_count());
	for (int i = 0; i < graph_rand.get_node_count(); i++) {
		string name = "h_" + to_string(i + 1);
		graph.add_vertex(name);
	}

	for (int i = 0; i < graph_rand.get_link_count(); i++) {
		string id1 = "h_" + to_string(graph_rand.adjm[i].x + 1);
		string id2 = "h_" + to_string(graph_rand.adjm[i].y + 1);
		graph.add_neighbour(id1, id2, "");
	}

	//cout << graph.mean_degree() << endl;

	/*ofstream mystream;
	mystream.open("random_graph_" + to_string(hubs_number) + ".txt", ios::trunc);
	mystream << "hubs number\t"<<hubs_number << endl;
	mystream << "mean degree\t"<< graph.mean_degree() << endl;
	mystream << "time generation\t"<< dur << endl;
	for (int i = 0; i < hubs_number; i++) {
		for (int j = 0; j < graph.get_neighs_out(i).size(); j++) {
			mystream << i << " " << graph.get_neighs_out(i).at(j) << endl;
		}
	}
	mystream.close();

	exit(EXIT_SUCCESS);*/

	return graph;
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
		network.mutable_hubs()->insert({ v.get_name(),h});
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
						duration = ElRandom::Uniform(0, max_vehicle_duration) + 1;
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
		rotation.mutable_fixed_price()->mutable_separable()->set_constant_price(static_cast<double>(price));
		rotation.mutable_maximum_number_vehicles()->set_start_value(1);
		rotation.mutable_maximum_number_vehicles()->set_end_value(1);

		//add rotation to line
		line_proto.mutable_next_rotations()->insert({ name,rotation });

	}

	network.mutable_lines()->insert({ "l" + to_string(line_numb), line_proto });

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
		network.mutable_vehicles()->insert({ "v" + to_string(l + 1),v });
	}
}

void InstanceGenerator::generate_parcels(const operations_research::lattle::LogisticsNetwork& network, const SpaceTimeNetwork& st_network, 
	const int& timesteps, const int& parcel_number, const int& mean_path_length, const int& min_parcel_weight, const int& max_parcel_weight, 
	const double& parcel_weight_shape, const bool& unit_weights, const double& start_inv_temp, const double& dist_inv_temp,
	const int& max_tries, const double& cut_capacities) const {

	//cout << "parcel\tsource\tdest\tweight\n";

	// Add parcels to the network
	vector<int> parcel_weights;
	default_random_engine generator(this->random_seed);
	uniform_real_distribution<double> distribution(0.0, 1.0);
	if (unit_weights) {
		parcel_weights = vector<int>(parcel_number, 1);
	}
	else {
		parcel_weights = vector<int>(parcel_number, 0);
		for (int j = 0; j < parcel_number; j++) {
			while (::strictly_less(parcel_weights.at(j), min_parcel_weight) || ::strictly_greater(parcel_weights.at(j), max_parcel_weight)) {
				double lomax_draw = ElRandom::Lomax(static_cast<double>(min_parcel_weight), parcel_weight_shape);
				parcel_weights.at(j) = max(1, static_cast<int>(floor(lomax_draw)));
			}
		}
	}
	sort(parcel_weights.rbegin(), parcel_weights.rend());

	// Compute weights to assign to the hubs which are used to sample starting hubs for the parcels
	vector<double> hubs_weights(st_network.get_underlying_graph().get_vertex_number(), -start_inv_temp);
	for (const auto& vertex : st_network.get_underlying_graph().get_vertices()) {
		hubs_weights.at(vertex.get_id()) *= static_cast<double>(vertex.get_neighbours_number());
	}
	vector<double> expo = exponential(hubs_weights);
	default_random_engine generator_hub(this->random_seed);
	discrete_distribution<int> distribution_hub(expo.begin(), expo.end());

	int end_time = timesteps - mean_path_length;

	// Sample paths for all parcels: start and end point for the parcels
	vector<int> source_hubs(parcel_number, -1);
	vector<int> destination_hubs(parcel_number, -1);
	default_random_engine generator_time(this->random_seed);
	default_random_engine generator_next(this->random_seed);
	default_random_engine generator_stop(this->random_seed);
	double prob_success = 1.0 / static_cast<double>(mean_path_length);
	int parcel_counter = 0;
	for (int j = 0; j < parcel_number; j++) {
		bool success = false;
		int tries = 0;

		while (tries <= max_tries && !success) {

			//select hub
			int hub = distribution_hub(generator_hub);
			//select time
			vector<int> available_vertices = st_network.get_hub_time_before_t(hub, end_time);
			uniform_int_distribution<int> distribution_time(0, available_vertices.size() - 1);
			int v_id = available_vertices.at(distribution_time(generator_time));
			source_hubs.at(j) = v_id;

			//build a route
			int length = 1;
			while (true) {
				const VertexST& v = st_network.get_vertices().at(v_id);

				if (v.get_adjacency_list_out().size() <= 0) {
					break;
				}

				uniform_int_distribution<int> distribution_next(0, v.get_adjacency_list_out().size() - 1);
				int pos = distribution_next(generator_next);

				if (length >= timesteps / 2.0) {
					break;
				}

				vector<int> awful;
				for (const auto& next : v.get_adjacency_list_out()) {
					awful.push_back(next.first);
				}

				v_id = awful.at(pos);
				length++;
				binomial_distribution<int> distribution_stop(length, prob_success);
				if (distribution_stop(generator_stop) > 0) {
					const VertexST& arr = st_network.get_vertices().at(v_id);
					if (arr.get_id_in_graph() != hub) {
						success = true;
						destination_hubs.at(j) = v_id;
						break;
					}
				}

			}

			tries++;

		}

		if (success) {
			operations_research::lattle::Parcel parcel;
			parcel.set_parcel("p_" + to_string(parcel_counter));
			const VertexST& source = st_network.get_vertices().at(source_hubs.at(j));
			int source_hub_id = source.get_id_in_graph();
			const VertexST& destination = st_network.get_vertices().at(destination_hubs.at(j));
			int destination_hub_id = destination.get_id_in_graph();
			string destination_name = st_network.get_underlying_graph().get_vertex(destination_hub_id).get_name();
			string source_name = st_network.get_underlying_graph().get_vertex(source_hub_id).get_name();
			assert(source_name != destination_name);
			parcel.set_destination_hub(destination_name);
			parcel.set_source_hub(source_name);
			operations_research::lattle::ValueDimension value;
			value.set_dimension("weight");
			value.set_value(parcel_weights.at(j));
			parcel.mutable_size()->Add(move(value));
			/*cout << j << "\t" << st_network.get_underlying_graph().get_vertex(source_hub_id).get_name() << "\t" << st_network.get_underlying_graph().get_vertex(destination_hub_id).get_name() << "\t" << parcel_weights.at(j);
			cout << "\n";*/

			//output protobuffer file
			string out;
			if (!google::protobuf::TextFormat::PrintToString(parcel, &out)) {
				cerr << "Failed to write the parcel file " << "parcels" << "\n";
				exit(EXIT_FAILURE);
			}
			string parcel_file = "..\\test_cases\\parcel_" + to_string(parcel_counter) + ".textproto";
			fstream output_network(parcel_file, ios::out);
			output_network << out;
			output_network.close();

			parcel_counter++;

		}

		

	}

	google::protobuf::ShutdownProtobufLibrary();

}