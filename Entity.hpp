/*
 * Entity.hpp
 * Entity is a base class for any entity that exists in the simulation world (e.g., AMOD vehicles
 * locations (nodes) and road segments (segments)). All objects must have an id_ and position_.
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Entity_HPP_
#define Entity_HPP_

#include "Types.hpp"
#include <string>

namespace amod {

class Entity {
public:
	Entity(): id_(0) {};
	Entity(int id, std::string name, Position pos) : id_(id), name_(name), position_(pos) {};
	virtual ~Entity() {};

	virtual Position getPosition() const { return position_; };
	virtual void setPosition(const Position &p) { position_ = p; };

	virtual int getId() const { return id_; };
	virtual void setId(const int id) { id_ = id; };

	virtual std::string getName() const { return name_;};
	virtual void setName(std::string name ) { name_ = name; };

private:
	int id_;
	std::string name_;
	Position position_;
};

} /* namespace AMOD */

#endif /* Entity_HPP_ */
