/*
 * AMODTypes.hpp
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef AMODTYPES_HPP_
#define AMODTYPES_HPP_

#include <functional>


namespace  AMODBase {

enum CarStatus { FREE, HIRED, ONCALL, PARKED, UNKNOWN };
enum ReturnCode{ FAILED, SUCCESS, CAR_NOT_PARKED, SOURCE_EQUALS_DESTINATION };

// Stores Positions of objects
struct Position {
	double x, y;
	bool valid;
	Position() : x(0), y(0), valid(true) {};
	Position(double xpos, double ypos) : x(xpos), y(ypos), valid(true) {};

	bool operator==(const Position &other) const {
		return (x == other.x) && (y == other.y) && valid && other.valid;
	}

};

} /* namespace AMOD */



namespace std {

  template <>
  struct hash<AMODBase::Position>
  {
    std::size_t operator()(const AMODBase::Position& p) const
    {
    	std::hash<int> hash_fn;
    	std::size_t hash_val = hash_fn(p.x);
    	return ( (hash_val << 4) ^ (hash_val >> 28) ^ hash_fn(p.y) );
    }
  };

}

#endif /* AMODTYPES_HPP_ */
