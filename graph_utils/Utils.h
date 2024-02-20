/**
*	\file Utils.h
*	\brief	Contains useful functions needed in the code.
*	\author	Matteo Petris
*	\date	February 2023
*/
#ifndef UTILS_H
#define UTILS_H

#include "Header.h"
#include "Elrandom.h"

//METHODS FOR FLOATING POINT NUMBERS
extern inline bool strictly_less(double x, double y);
extern inline bool less(double x, double y);
extern inline bool equal(double x, double y);
extern inline bool strictly_greater(double x, double y);
extern inline bool greater(double x, double y);
extern inline double minimum(double x, double y);
extern inline double maximum(double x, double y);
extern inline double computeFractionalPart(double x);
extern inline double rounder(double x);
extern inline double rounderToNDecimals(double x, unsigned int precision);

/**
 * \brief given a year, month, day, hour, minute, second covert to an integer according to a time discretisation
 *
 * \param year : int
 * \param month : int
 * \param day : int
 * \param hour : int
 * \param minute : int
 * \param second : int
 * \return the encode : int.
 */
extern inline int time_encoder(const int& year, const int& month, const int& day, const int& hour, const int& minute, const int& second);
/**
 * \brief given a google::type::DateTime object covert to an integer according to a time discretisation
 *
 * \param time : google::type::DateTime
 * \return the encode : int.
 */
extern inline int time_encoder(const google::type::DateTime& time);
/**
 * \brief given an integer google::type::DateTime object covert to an integer according to a time discretisation
 *
 * \param time_encoded : int
 * \return the decoded time : google::type::DateTime.
 */
extern inline google::type::DateTime time_decoder(const int& time_encoded);

//RANDOM GENERATORS
/**
 * \brief Pick randomly an integer in the interval[_min, _max] by uniform distribution.
 *
 * \param generator : mt19937
 * \return The random integer : int.
 */
extern inline int randomIntBetween(int _min, int _max, mt19937& generator);

/**
 * \brief compute the softmax function of python. sigma(x)_j = e^(x_j)/ sum_k e^(x_k)
 * 
 * \param x : vector of double
 * \return softmax of x : vector of double.
 */
extern inline vector<double> softmax(const vector<double>& x);

/**
 * \brief compute the "exponential" of the vector. sigma(x)_j = e^(x_j)
 *
 * \param x : vector of double
 * \return exponential of x : vector of double.
 */
extern inline vector<double> exponential(const vector<double>& x);

//OPERATIONS WITH VECTORS
/**
 * \brief extract subvector from a vector
 *
 * \param v : vector 
 * \param v : first_position 
 * \param v : last_position 
 * \return subvector of v from first_position to last_position.
 */
template <typename T>																
static vector<T> extract_subvector(const vector<T> v, const int& first_position, const int& last_position) {
	vector<T> subvector(v.begin() + first_position, v.begin() + last_position + 1);
	return subvector;
}
/**
 * \brief concatenate v1 with v2
 *
 * \param v1 : vector 1
 * \param v2 : vector 2
 * \return (v1,v2)
 */
template <typename T>
static vector<T> concatenate_vectors(const vector<T> v1, const vector<T> v2) {
	vector<T> concatenate;
	concatenate.reserve(v1.size() + v2.size());
	concatenate.insert(concatenate.end(), v1.begin(), v1.end());
	concatenate.insert(concatenate.end(), v2.begin(), v2.end());
	return concatenate;
}
/**
 * \brief check whether v1 is the first part of v2
 *
 * \param v1 : vector 1
 * \param v2 : vector 2
 * \return true if v2=(v1)+..., false otherwise
 */
template <typename T>
static bool is_first_subvector(const vector<T> v1, const vector<T> v2) {
	if (v2.size() < v1.size()) {
		return false;
	}
	for (int i = 0; i < v1.size(); i++) {
		if (v2.at(i) != v1.at(i)) {
			return false;
		}
	}
	return true;
}

vector<int> random_subset(vector<int> &v,int n){
  set<int> targets;
  while((int)targets.size() < n){
	 int index = ElRandom::Uniform(0, (int)v.size()-1);
	 targets.insert(v[index]);
  }
  return vector<int>(targets.begin(),targets.end());
}
#endif //UTILS_H