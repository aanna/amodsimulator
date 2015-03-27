/*
 * AMODObject.hpp
 * AMODObject is a base class for any entity that exists in the simulation world (e.g., AMOD vehicles
 * locations (nodes) and road segments (segments)). All objects must have an id_ and position_.
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef AMODOBJECT_HPP_
#define AMODOBJECT_HPP_

#include "AMODTypes.hpp"
#include <string>

namespace AMODBase {

class AMODObject {
public:
	AMODObject(): id_(0) {};
	virtual ~AMODObject() {};

	virtual Position getPosition() const { return position_; };
	virtual void setPosition(const Position &p) { position_ = p; };

	virtual int getID() const { return id_; };
	virtual void setID(const int id) { id_ = id; };

	virtual std::string getName() const { return name_;};
	virtual std::string setName(std::string name ) { name_ = name; };

private:
	int id_;
	std::string name_;
	Position position_;
};

} /* namespace AMOD */

#endif /* AMODOBJECT_HPP_ */
