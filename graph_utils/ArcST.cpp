#include "ArcST.h"

int ArcST::lastId;

ArcST::ArcST() {
	this->id = -1;
	this->departure_id = -1;
	this->arrival_id = -1;
}

ArcST::ArcST(const int& departure_id, const int& arrival_id, const string& type, const string& line, const string& rotation, const int& travelling_time, const double& cost) {
	this->id = this->lastId;
	this->departure_id = departure_id;
	this->arrival_id = arrival_id;
	this->type = type;
	this->line = line;
	this->rotation = rotation;
	this->travelling_time = travelling_time;
	this->cost = cost;
	this->lastId++;
}

ArcST::~ArcST() {
}

ArcST::ArcST(const ArcST& arcST) {
	this->id = arcST.id;
	this->departure_id = arcST.departure_id;
	this->arrival_id = arcST.arrival_id;
	this->type = arcST.type;
	this->line = arcST.line;
	this->rotation = arcST.rotation;
	this->travelling_time = arcST.travelling_time;
	this->cost = arcST.cost;
}

ArcST& ArcST::operator=(const ArcST& arcST) {
	this->id = arcST.id;
	this->departure_id = arcST.departure_id;
	this->arrival_id = arcST.arrival_id;
	this->type = arcST.type;
	this->line = arcST.line;
	this->rotation = arcST.rotation;
	this->travelling_time = arcST.travelling_time;
	this->cost = arcST.cost;
	return *this;
}



const int& ArcST::get_id() const {
	return this->id;
}

const int& ArcST::get_departure_id() const {
	return this->departure_id;
}

const int& ArcST::get_arrival_id() const {
	return this->arrival_id;
}

const string& ArcST::get_type() const {
	return this->type;
}

const string& ArcST::get_line() const {
	return this->line;
}

const string& ArcST::get_rotation() const {
	return this->rotation;
}

const int& ArcST::get_travelling_time() const {
	return this->travelling_time;
}

const double& ArcST::get_cost() const {
	return this->cost;
}

const string ArcST::toString() const {
	string str;
	str.append(to_string(this->id) + "\t(" + to_string(this->departure_id) + "," + to_string(this->arrival_id) + ")\t" +
		this->type + "\t" + to_string(this->travelling_time) + "\t" + to_string(this->cost)
		+ "\t(" + this->line + "," + this->rotation + ")" + "\n");
	return str;
}
