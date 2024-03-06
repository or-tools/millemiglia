#include "VertexST.h"

int VertexST::lastId;

VertexST::VertexST() {
	this->id = -1;
	this->id_in_graph = -1;
	this->time = -1;
	this->topological_position = -1;
}

VertexST::VertexST(const int& id_in_graph, const int& time) {
	this->id = this->lastId;
	this->id_in_graph = id_in_graph;
	this->time = time;
	this->topological_position = -1;
	this->lastId++;
}

VertexST::~VertexST() {
}

VertexST::VertexST(const VertexST& vertex) {
	this->id = vertex.id;
	this->id_in_graph = vertex.id_in_graph;
	this->time = vertex.time;
	this->topological_position = vertex.topological_position;
	this->adjacency_list_out = vertex.adjacency_list_out;
	this->adjacency_list_out_cost = vertex.adjacency_list_out_cost;
	this->adjacency_list_out_time = vertex.adjacency_list_out_time;
	this->adjacency_list_in = vertex.adjacency_list_in;
}

VertexST& VertexST::operator=(const VertexST& vertex) {
	this->id = vertex.id;
	this->id_in_graph = vertex.id_in_graph;
	this->time = vertex.time;
	this->topological_position = vertex.topological_position;
	this->adjacency_list_out = vertex.adjacency_list_out;
	this->adjacency_list_out_cost = vertex.adjacency_list_out_cost;
	this->adjacency_list_out_time = vertex.adjacency_list_out_time;
	this->adjacency_list_in = vertex.adjacency_list_in;
	return *this;
}

void VertexST::add_neighbour_out(const int& vId, const int& arcId) {
	auto finder = this->adjacency_list_out.find(vId);
	if (finder == this->adjacency_list_out.end()) {
		vector<int> adj = { arcId };
		this->adjacency_list_out.insert(make_pair(vId, adj));
	}
	else {
		finder->second.push_back(arcId);
	}
}

void VertexST::add_neighbour_out_time(const int& vId, const vector<int>& arcIds) {
	this->adjacency_list_out_time.insert(make_pair(vId, arcIds));
}

void VertexST::add_neighbour_out_cost(const int& vId, const vector<int>& arcIds) {
	this->adjacency_list_out_cost.insert(make_pair(vId, arcIds));
}

void VertexST::add_neighbour_in(const int& vId, const int& arcId) {
	auto finder = this->adjacency_list_in.find(vId);
	if (finder == this->adjacency_list_in.end()) {
		vector<int> adj = { arcId };
		this->adjacency_list_in.insert(make_pair(vId, adj));
	}
	else {
		finder->second.push_back(arcId);
	}
}

const int& VertexST::get_id() const {
	return this->id;
}

const int& VertexST::get_id_in_graph() const {
	return this->id_in_graph;
}

const int& VertexST::get_time() const {
	return this->time;
}

const int& VertexST::get_topological_position() const {
	return this->topological_position;
}

const unordered_map<int, vector<int>>& VertexST::get_adjacency_list_out() const {
	return this->adjacency_list_out;
}

const unordered_map<int, vector<int>>& VertexST::get_adjacency_list_out_cost() const {
	return this->adjacency_list_out_cost;
}

const unordered_map<int, vector<int>>& VertexST::get_adjacency_list_out_time() const {
	return this->adjacency_list_out_time;
}

const unordered_map<int, vector<int>>& VertexST::get_adjacency_list_in() const {
	return this->adjacency_list_in;
}

void VertexST::set_topological_position(const int& pos) {
	this->topological_position = pos;
}

const string VertexST::toString() const {
	string str;
	str.append(to_string(this->id) + "\t(" + to_string(this->id_in_graph) + "," + to_string(this->time) + ")\t" + to_string(this->topological_position) + "\t\t" +
		to_string(this->adjacency_list_in.size()) + "\t" + to_string(this->adjacency_list_out.size()) + "\n");
	return str;
}
