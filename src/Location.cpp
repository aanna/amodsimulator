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

void Location::getVehicleIds(std::unordered_set<int>::const_iterator *bitr, std::unordered_set<int>::const_iterator *eitr) {
	*bitr = vehicle_ids_.begin();
	*eitr = vehicle_ids_.end();
}

void Location::clearVehicleIds() {
	vehicle_ids_.clear();
}

int Location::getNumVehicles() const {
	return (int) vehicle_ids_.size();
}


int Location::getNumCustomers() const {
	return customer_ids_.size();
}

void Location::getCustomerIds(std::unordered_set<int> *cust_ids) {
	*cust_ids = customer_ids_;
}

void Location::getCustomerIds(std::unordered_set<int>::const_iterator *bitr, std::unordered_set<int>::const_iterator *eitr) {
	*bitr = customer_ids_.begin();
	*eitr = customer_ids_.end();
}


void Location::addCustomerId(int cust_id) {
	customer_ids_.insert(cust_id);
}

void Location::removeCustomerId(int cust_id) {
	customer_ids_.erase(cust_id);
}

void Location::clearCustomerIds() {
	customer_ids_.clear();
}

void Location::setCapacity(int capacity) {
	capacity_ = capacity;
}

int Location::getCapacity() const {
	return capacity_;
}

} /* namespace amod */
