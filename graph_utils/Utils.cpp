#include "Utils.h"

inline bool strictly_less(double x, double y) {

	bool result = false;
	result = ((y - x) > EPSILON) ? true : false;
	return result;

};

inline bool less(double x, double y) {

	bool result = false;
	result = ((y - x) > -EPSILON) ? true : false;
	return result;

};

inline bool equal(double x, double y) {

	bool result = false;
	result = (fabs(y - x) <= EPSILON) ? true : false;
	return result;

};

inline bool strictly_greater(double x, double y) {

	bool result = false;
	result = ((x - y) > EPSILON) ? true : false;
	return result;

};

inline bool greater(double x, double y) {

	bool result = false;
	result = ((x - y) > -EPSILON) ? true : false;
	return result;

};

inline double minimum(double x, double y) {

	if ((x - y) > EPSILON)
		return y;
	else
		return x;

};

inline double maximum(double x, double y) {

	if ((x - y) > EPSILON)
		return x;
	else
		return y;

};

inline double computeFractionalPart(double x) {

	double toFloor = x + ::EPSILON;
	double toCeil = x - ::EPSILON;
	return  min(x - floor(toFloor), ceil(toCeil) - x);

};

inline double rounder(double x) {

	if (x < 0) {
		double toCeil = x - 0.5;
		return ceil(toCeil);
	}
	double toFloor = x + 0.5;
	return floor(toFloor);

};

inline double rounderToNDecimals(double x, unsigned int precision) {

	double pow_10 = pow(10.0f, precision);
	return rounder(x * pow_10) / pow_10;

}

inline int time_encoder(const int& year, const int& month, const int& day, const int& hour, const int& minute, const int& second) {
	double tot_time_in_min = minute + hour * 60;
	double min = static_cast<double>(tot_time_in_min) / ::TIMESTEP;
	int encoding = -1;
	if (::less(min - floor(min), 0.5)) {
		encoding = floor(min);
	}
	else {
		encoding = ceil(min);
	}
	return encoding;
}

inline int time_encoder(const google::type::DateTime& time) {
	double tot_time_in_min = time.minutes() + time.hours() * 60;
	double min = static_cast<double>(tot_time_in_min) / ::TIMESTEP;
	int encoding = -1;
	if (::less(min - floor(min), 0.5)) {
		encoding = floor(min);
	}
	else {
		encoding = ceil(min);
	}
	return encoding;
}

inline google::type::DateTime time_decoder(const int& time_encoded) {
	google::type::DateTime time;
	time.set_year(2023);
	time.set_month(3);
	time.set_day(10);
	int to_min = time_encoded * 15;
	double hours = to_min / 60.0;
	int hours_int = floor(hours);
	time.set_hours(hours_int);
	double fract_part = hours - hours_int;
	int minutes_int = fract_part * 60;
	time.set_minutes(minutes_int);
	return time;
}

inline int randomIntBetween(int _min, int _max, mt19937& generator) {

	int minimum = min(_min, _max);
	int maximum = max(_min, _max);
	uniform_int_distribution<int>  distr(minimum, maximum);

	//return distr(generator);
	return minimum + (rand() % (int)(maximum - minimum + 1));

}

inline vector<double> softmax(const vector<double>& x) {
	vector<double> sm(x.size(), -1.0);
	double den = 0.0;
	for (int i = 0; i < x.size(); i++) {
		den += exp(x.at(i));
	}
	for (int i = 0; i < x.size(); i++) {
		sm.at(i) = exp(x.at(i)) / den;
	}
	return sm;
}

inline vector<double> exponential(const vector<double>& x) {
	vector<double> expo(x.size(), -1.0);
	for (int i = 0; i < x.size(); i++) {
		expo.at(i) = exp(x.at(i));
	}
	return expo;
}