/*
 * Location.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "Location.hpp"

namespace amod {

Location::Location() : capacity_(0) {
	// nothing to do
}

Location::Location(int id, std::string name, Position pos, int capacity):
	Entity(id, name, pos), capacity_(capacity) {
	// nothing to do here
}

Location::~Location() {
	// nothing to do here
}

void Location::addVehicleId(int veh_id) {
	vehicle_ids_.insert(veh_id);
}

void Location::removeVehicleId(int veh_id) {
	vehicle_ids_.erase(veh_id);
}

void Location::getVehicleIds(std::unordered_set<int> *veh_ids) {
	*veh_ids = vehicle_ids_;
}

void Location::clearVehicleIds() {
	vehicle_ids_.clear();
}

int Location::getNumVehicles() {
	return vehicle_ids_.size();
}

void Location::setCapacity(int capacity) {
	capacity_ = capacity;
}

int Location::getCapacity() const {
	return capacity_;
}

} /* namespace amod */
