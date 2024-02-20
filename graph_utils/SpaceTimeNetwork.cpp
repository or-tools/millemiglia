#include "SpaceTimeNetwork.h"

SpaceTimeNetwork::SpaceTimeNetwork() {
	VertexST::restart_id_counter();
	ArcST::restart_id_counter();
}

SpaceTimeNetwork::SpaceTimeNetwork(const string& network_file, const int& time_horizon) {
	VertexST::restart_id_counter();
	ArcST::restart_id_counter();
	parse_logistic_network(network_file);
	build_underlying_graph(this->network);
	cout << this->underlying_graph.toString() << endl;
	build_vertices(time_horizon);
	cout << "\n\nSORT ADJACENCY LISTS FOR GRAPH ALGORITHMS --- STARTING\n\n";
	this->sort_adjacency_list_out();
	cout << "\n\nSORT ADJACENCY LISTS FOR GRAPH ALGORITHMS --- COMPLETED\n\n";
	build_arcs(this->network, time_horizon);
	build_lorries(this->network);
	cout << toString() << endl;
	cout << "\n\nCOMPUTE ADJACENCY LISTS FOR GRAPH ALGORITHMS --- STARTING\n\n";
	this->initialise_adjacency_lists_graph_algorithms("length");
	cout << "\n\nCOMPUTE TOPOLOGICAL ORDER --- STARTING\n\n";
}

void SpaceTimeNetwork::parse_logistic_network(const string& network_file) {
	const char* network_cc = network_file.c_str();
	if (!std::filesystem::exists({ network_cc })) {
		std::cout << "File " << network_cc << " does not exist!\n";
		exit(EXIT_FAILURE);
	}

	fstream input_network(network_cc, ios::in);
	if (!input_network.is_open()) {
		std::cout << "Error while opening file " << network_cc << ", although the file exists!\n";
		exit(EXIT_FAILURE);
	}

	std::ostringstream ss_network;
	ss_network << input_network.rdbuf();
	if (!google::protobuf::TextFormat::ParseFromString(ss_network.str(), &this->network)) {
		cerr << "Failed to parse instance from file " << network_cc << "\n";
		exit(EXIT_FAILURE);
	}

}

SpaceTimeNetwork::SpaceTimeNetwork(const SpaceTimeNetwork& spaceTimeNetwork) {
	this->underlying_graph = spaceTimeNetwork.underlying_graph;
	this->vertices = spaceTimeNetwork.vertices;
	this->arcs = spaceTimeNetwork.arcs;
	this->vertex_dictionary = spaceTimeNetwork.vertex_dictionary;
	this->lorries = spaceTimeNetwork.lorries;
	this->network = spaceTimeNetwork.network;
	this->adjacency_lists_graph_algorithms = spaceTimeNetwork.adjacency_lists_graph_algorithms;
	this->adjacency_lists_arc_position = spaceTimeNetwork.adjacency_lists_arc_position;
}

SpaceTimeNetwork& SpaceTimeNetwork::operator=(const SpaceTimeNetwork& spaceTimeNetwork) {
	this->underlying_graph = spaceTimeNetwork.underlying_graph;
	this->vertices = spaceTimeNetwork.vertices;
	this->arcs = spaceTimeNetwork.arcs;
	this->vertex_dictionary = spaceTimeNetwork.vertex_dictionary;
	this->lorries = spaceTimeNetwork.lorries;
	this->network = spaceTimeNetwork.network;
	this->adjacency_lists_graph_algorithms = spaceTimeNetwork.adjacency_lists_graph_algorithms;
	this->adjacency_lists_arc_position = spaceTimeNetwork.adjacency_lists_arc_position;
	return *this;
}

SpaceTimeNetwork::~SpaceTimeNetwork() {
}

void SpaceTimeNetwork::build_underlying_graph(const operations_research::lattle::LogisticsNetwork& network) {
	this->underlying_graph = Graph(network.hubs_size());
	//VERTICES
	for (google::protobuf::Map<string, operations_research::lattle::Hub>::const_iterator hub = network.hubs().begin();
		hub != network.hubs().end(); hub++) {
		this->underlying_graph.add_vertex(hub->first);
	}
	//ARCS
	for (google::protobuf::Map<string, operations_research::lattle::Hub>::const_iterator hub = network.hubs().begin();
		hub != network.hubs().end(); hub++) {
		//cout << "HUB " << hub->first << endl;
		for (google::protobuf::Map<string, operations_research::lattle::Line>::const_iterator line = network.lines().begin();
			line != network.lines().end(); line++) {
			//cout << "\tLINE " << line->first << endl;
			for (int i = 0; i < line->second.hub_ids().size() - 1; i++) {
				string start = line->second.hub_ids().at(i);
				string end = line->second.hub_ids().at(i + 1);
				//cout << "\t\t" << start << " " << end << endl;
				if (start == hub->first) {
					this->underlying_graph.add_neighbour(start, end, line->first);
				}
			}
		}
	}
}

void SpaceTimeNetwork::build_vertices(const int& time_horizon) {
	for (int i = 0; i < this->underlying_graph.get_vertices().size(); i++) {
		for (int t = 0; t < time_horizon; t++) {
			add_vertexST(i, t);
		}
	}
	/*for (const auto& line : network.lines()) {
		for (int i = 0; i < line.second.hub_ids().size() - 1; i++) {
			string departure_hub = line.second.hub_ids().at(i);
			string arrival_hub = line.second.hub_ids().at(i + 1);
			for (const auto& rotations : line.second.next_rotations()) {
				operations_research::lattle::DateTimeRange dt = rotations.second.departure_times().at(departure_hub);
				operations_research::lattle::DateTimeRange at = rotations.second.departure_times().at(arrival_hub);
				add_vertexST(departure_hub, dt, time_horizon);
				add_vertexST(arrival_hub, at, time_horizon);
			}
		}
	}*/
}

void SpaceTimeNetwork::build_arcs(const operations_research::lattle::LogisticsNetwork& network, const int& time_horizon) {
	//Travelling arcs
	for (const auto& line : network.lines()) {
		for (int i = 0; i < line.second.hub_ids().size() - 1; i++) {
			string departure_hub = line.second.hub_ids().at(i);
			string arrival_hub = line.second.hub_ids().at(i + 1);
			//cout << departure_hub << "\t" << arrival_hub << endl;
			double cost = 0.0;
			for (const auto& rotations : line.second.next_rotations()) {
				operations_research::lattle::DateTimeRange dt = rotations.second.departure_times().at(departure_hub);
				operations_research::lattle::DateTimeRange at = rotations.second.arrival_times().at(arrival_hub);
				const VertexST& depVertex = get_vertex(departure_hub, dt);
				const VertexST& arrVertex = get_vertex(arrival_hub, at);
				int duration = arrVertex.get_time() - depVertex.get_time();
				if (i == 0) {
					cost = rotations.second.fixed_price().separable().constant_price();;
				}
				ArcST arc = ArcST(depVertex.get_id(), arrVertex.get_id(), "travelling", line.first, rotations.first, duration, cost);
				this->arcs.push_back(arc);
				add_to_adjacency_list_in(arc);
				add_to_adjacency_list_out(arc);
			}
		}
	}
	//Waiting arcs
	for (int i = 0; i < this->underlying_graph.get_vertices().size(); i++) {
		for (int t = 0; t < time_horizon - 1; t++) {
			const VertexST& v1 = get_vertex(i, t);
			const VertexST& v2 = get_vertex(i, t + 1);
			ArcST arc = ArcST(v1.get_id(), v2.get_id(), "waiting", "", "", 1, 0.0);
			this->arcs.push_back(arc);
			add_to_adjacency_list_in(arc);
			add_to_adjacency_list_out(arc);
		}
	}
}

void SpaceTimeNetwork::sort_adjacency_list_out() {
	//sort adjacency lists by cost and by time
	for (auto& v : this->vertices) {
		for (const auto& adj : v.get_adjacency_list_out()) {
			vector<pair<int, int>> sort_time(adj.second.size());
			vector<pair<double, int>> sort_cost(adj.second.size());
			for (int i = 0; i < adj.second.size(); i++) {
				int arc = adj.second.at(i);
				sort_time.at(i) = make_pair(this->arcs.at(arc).get_travelling_time(), arc);
				sort_cost.at(i) = make_pair(this->arcs.at(arc).get_cost(), arc);
			}
			sort(sort_cost.begin(), sort_cost.end());
			sort(sort_time.begin(), sort_time.end());
			vector<int> adj_time(adj.second.size());
			vector<int> adj_cost(adj.second.size());
			for (int i = 0; i < adj.second.size(); i++) {
				adj_time.at(i) = sort_time.at(i).second;
				adj_cost.at(i) = sort_cost.at(i).second;
			}
			v.add_neighbour_out_time(adj.first, adj_time);
			v.add_neighbour_out_cost(adj.first, adj_cost);
		}

	}
}

void SpaceTimeNetwork::build_lorries(const operations_research::lattle::LogisticsNetwork& network) {
	for (const auto& vehicle : network.vehicles()) {
		Lorry lorry = Lorry(vehicle.first, vehicle.second);
		this->lorries.push_back(lorry);
	}
}

void SpaceTimeNetwork::add_to_adjacency_list_out(const ArcST& arc) {
	this->vertices.at(arc.get_departure_id()).add_neighbour_out(arc.get_arrival_id(), arc.get_id());
}

void SpaceTimeNetwork::add_to_adjacency_list_in(const ArcST& arc) {
	this->vertices.at(arc.get_arrival_id()).add_neighbour_in(arc.get_departure_id(), arc.get_id());
}



const vector<int> SpaceTimeNetwork::get_hub_time_after_t(const int& hub, const int& time) const {
	vector<int> v_ids;
	const auto& finder_id = this->vertex_dictionary.find(hub);
	assert(finder_id != this->vertex_dictionary.end());
	for (unordered_map<int, int>::const_iterator it = finder_id->second.begin(); it != finder_id->second.end(); it++) {
		if (it->first >= time) {
			v_ids.push_back(it->second);
		}
	}
	return v_ids;
}

const vector<int> SpaceTimeNetwork::get_hub_time_before_t(const int& hub, const int& time) const {
	vector<int> v_ids;
	const auto& finder_id = this->vertex_dictionary.find(hub);
	assert(finder_id != this->vertex_dictionary.end());
	for (unordered_map<int, int>::const_iterator it = finder_id->second.begin(); it != finder_id->second.end(); it++) {
		if (it->first <= time) {
			v_ids.push_back(it->second);
		}
	}
	return v_ids;
}

const VertexST& SpaceTimeNetwork::get_vertex(const string& hub, const operations_research::lattle::DateTimeRange& time) const {
	int graph_id = this->underlying_graph.get_vertex(hub).get_id();
	int encTime = ::time_encoder(time.first_date());
	const auto& finder_id = this->vertex_dictionary.find(graph_id);
	assert(finder_id != this->vertex_dictionary.end());
	const auto& finder_time = this->vertex_dictionary.at(graph_id).find(encTime);
	assert(finder_time != this->vertex_dictionary.at(graph_id).end());
	assert(finder_time->second < this->vertices.size());
	return this->vertices.at(finder_time->second);
}

const VertexST& SpaceTimeNetwork::get_vertex(const int& hub, const int& time) const {
	const auto& finder_id = this->vertex_dictionary.find(hub);
	assert(finder_id != this->vertex_dictionary.end());
	const auto& finder_time = this->vertex_dictionary.at(hub).find(time);
	assert(finder_time != this->vertex_dictionary.at(hub).end());
	assert(finder_time->second < this->vertices.size());
	return this->vertices.at(finder_time->second);
}

const vector<unordered_map<int, double>>& SpaceTimeNetwork::get_adjacency_lists_graph_algorithms() const {
	return this->adjacency_lists_graph_algorithms;
}

const vector<unordered_map<int, int>>& SpaceTimeNetwork::get_adjacency_lists_arc_position() const {
	return this->adjacency_lists_arc_position;
}


void SpaceTimeNetwork::add_vertexST(const int& hub, const int& time) {
	auto dict1 = this->vertex_dictionary.find(hub);
	bool isToInsert = false;
	if (dict1 == this->vertex_dictionary.end()) {
		isToInsert = true;
		unordered_map<int, int> aux; aux.insert(make_pair(time, this->vertices.size()));
		this->vertex_dictionary.insert(make_pair(hub, aux));
	}
	else if (dict1->second.find(time) == dict1->second.end()) {
		isToInsert = true;
		dict1->second.insert(make_pair(time, this->vertices.size()));
	}
	if (isToInsert) {
		VertexST vst = VertexST(hub, time);
		this->vertices.push_back(vst);
	}
}

void SpaceTimeNetwork::add_vertexST(const string& hub, const operations_research::lattle::DateTimeRange& time, const int& time_horizon) {
	const Vertex& v = this->underlying_graph.get_vertex(hub);
	int encTime = ::time_encoder(time.first_date());
	assert(encTime < time_horizon);
	add_vertexST(v.get_id(), encTime);
}

void SpaceTimeNetwork::initialise_adjacency_lists_arc_position() {
	this->adjacency_lists_arc_position = vector<unordered_map<int, int>>(this->vertices.size());
	for (const auto& vertex : this->vertices) {
		for (const auto& neighbour : vertex.get_adjacency_list_out()) {
			this->adjacency_lists_arc_position.at(vertex.get_id()).insert(make_pair(neighbour.first, 0));
		}
	}
}

void SpaceTimeNetwork::initialise_adjacency_lists_graph_algorithms(const string& type) {
	this->adjacency_lists_graph_algorithms = vector<unordered_map<int, double>>(this->vertices.size());
	if (type == "length") {
		for (const auto& vertex : this->vertices) {
			for (const auto& neighbour : vertex.get_adjacency_list_out()) {
				this->adjacency_lists_graph_algorithms.at(vertex.get_id()).insert(make_pair(neighbour.first, 1.0));
			}
		}
	}
	else if (type == "cost") {
		for (const auto& vertex : this->vertices) {
			for (const auto& neighbour : vertex.get_adjacency_list_out_cost()) {
				int arcId = neighbour.second.front();
				this->adjacency_lists_graph_algorithms.at(vertex.get_id()).insert(make_pair(neighbour.first, this->arcs.at(arcId).get_cost()));
			}
		}
	}
	else if (type == "time") {
		for (const auto& vertex : this->vertices) {
			for (const auto& neighbour : vertex.get_adjacency_list_out_time()) {
				int arcId = neighbour.second.front();
				this->adjacency_lists_graph_algorithms.at(vertex.get_id()).insert(make_pair(neighbour.first, this->arcs.at(arcId).get_travelling_time()));
			}
		}
	}
	else {
		cout << "\n\nWRONG TYPE in initialise_adjacency_lists_graph_algorithms\n\n";
		exit(EXIT_FAILURE);
	}
}

void SpaceTimeNetwork::restore_adjacency_lists_graph_value(const string& type, const int& u, const int& v) {
	double new_value = 1.0;
	this->adjacency_lists_arc_position.at(u).at(v) = 0;
	if (type == "cost") {
		int arcId = this->vertices.at(u).get_adjacency_list_out_cost().at(v).at(0);
		new_value = this->arcs.at(arcId).get_cost();
	}
	else if (type == "time") {
		int arcId = this->vertices.at(u).get_adjacency_list_out_time().at(v).at(0);
		new_value = this->arcs.at(arcId).get_travelling_time();
	}
	this->adjacency_lists_graph_algorithms.at(u).at(v) = new_value;
}

void SpaceTimeNetwork::modify_adjacency_lists_graph_value(const string& type, const int& u, const int& v, const int& position_in_adj_list) {
	double new_value = ::OMEGA * ::OMEGA;
	if (type != "length" && position_in_adj_list < this->vertices.at(u).get_adjacency_list_out().at(v).size() - 1) {
		int current_arc_position = position_in_adj_list + 1;
		this->adjacency_lists_arc_position.at(u).at(v) = current_arc_position;
		if (type == "cost") {
			int arcId = this->vertices.at(u).get_adjacency_list_out_cost().at(v).at(current_arc_position);
			new_value = this->arcs.at(arcId).get_cost();
		}
		else if (type == "time") {
			int arcId = this->vertices.at(u).get_adjacency_list_out_time().at(v).at(current_arc_position);
			new_value = this->arcs.at(arcId).get_travelling_time();
		}
	}
	this->adjacency_lists_graph_algorithms.at(u).at(v) = new_value;
}

const operations_research::lattle::LogisticsNetwork& SpaceTimeNetwork::get_network() const {
	return this->network;
}

const Graph& SpaceTimeNetwork::get_underlying_graph() const {
	return this->underlying_graph;
}

const vector<VertexST>& SpaceTimeNetwork::get_vertices() const {
	return this->vertices;
}

const vector<ArcST>& SpaceTimeNetwork::get_arcs() const {
	return this->arcs;
}

const string SpaceTimeNetwork::toString() const {
	string str = "VERTICES:\n";
	str.append("\tID\t(GRAPH_ID,TIME)\tTOP_POS\tIN_SIZE\tOUT_SIZE\n");
	for (int i = 0; i < this->vertices.size(); i++) {
		str.append("\t" + this->vertices.at(i).toString());
	}
	str.append("\nARCS:\n");
	str.append("\tID\t(DEP_ID,ARR_ID)\tTYPE\tTRAV_TIME\tCOST\t(LINE,ROT)\n");
	for (int i = 0; i < this->arcs.size(); i++) {
		str.append("\t" + this->arcs.at(i).toString());
	}
	return str;
}


