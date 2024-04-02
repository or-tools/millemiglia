#include "Graph.h"

Graph::Graph() {
	Vertex::restart_id_counter();
}

Graph::Graph(const int& hub_number) {
	this->vertices = vector<Vertex>(hub_number);
	this->arcs_dictionary = vector<unordered_map<int, int>>(hub_number);
	Vertex::restart_id_counter();
}

Graph::~Graph() {
	this->vertices.clear();
}

Graph::Graph(const Graph& graph) {
	this->vertices = graph.vertices;
	this->vertex_dictionary = graph.vertex_dictionary;
	this->arcs = graph.arcs;
	this->arcs_dictionary = graph.arcs_dictionary;
}

Graph& Graph::operator=(const Graph& graph) {
	this->vertices = graph.vertices;
	this->vertex_dictionary = graph.vertex_dictionary;
	this->arcs = graph.arcs;
	this->arcs_dictionary = graph.arcs_dictionary;
	return *this;
}

void Graph::add_vertex(const string& name) {
	Vertex v = Vertex(name);
	this->vertices.at(v.get_id()) = v;
	this->vertex_dictionary.insert(make_pair(name, v.get_id()));
}

void Graph::add_neighbour(const string& key, const string& neighbour, const string& line) {
	int key_id = this->vertex_dictionary.at(key);
	int neighbour_id = this->vertex_dictionary.at(neighbour);
	this->vertices.at(key_id).add_neighbour_out(neighbour_id, line);
	this->vertices.at(neighbour_id).add_neighbour_in(key_id, line);
	int min_id = min(key_id, neighbour_id);
	int max_id = max(key_id, neighbour_id);
	this->arcs_dictionary.at(key_id).insert(make_pair(neighbour_id, this->arcs.size()));
	this->arcs_dictionary.at(neighbour_id).insert(make_pair(key_id, this->arcs.size()));
	this->arcs.push_back(make_pair(min_id, max_id));
}

const vector<Vertex>& Graph::get_vertices() const {
	return this->vertices;
}

const int Graph::get_vertex_number() const {
	return this->vertices.size();
}

const Vertex& Graph::get_vertex(const int& id) const {
	assert(id < this->vertices.size());
	return this->vertices.at(id);
}

const Vertex& Graph::get_vertex(const string& name) const {
	unordered_map<string, int>::const_iterator finder = this->vertex_dictionary.find(name);
	assert(finder != this->vertex_dictionary.end());
	return this->vertices.at(finder->second);
}

const vector<pair<int, int>>& Graph::get_arcs() const {
	return this->arcs;
}

const int Graph::get_arc_position(const int& id1, const int& id2) const {
	unordered_map<int, int>::const_iterator finder = this->arcs_dictionary.at(id1).find(id2);
	assert(finder != this->arcs_dictionary.at(id1).end());
	return finder->second;
}

const string Graph::toString() const {
	string str = "VERTICES:\n";
	str.append("\tID\tNAME\tIN_SIZE\tOUT_SIZE\n");
	for (int i = 0; i < this->vertices.size(); i++) {
		str.append("\t" + this->vertices.at(i).toString());
	}
	str.append("ARC NUMBER:\t" + to_string(this->arcs.size()) + "\n");
	return str;
}