/*
 * Customer.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "Customer.hpp"

namespace amod {


Customer::~Customer() {
	return;
}

Customer::Customer(int id, std::string name, amod::Position pos, int loc_id, int assigned_vehicle, bool in_vehicle, Customer::Status status) :
		Entity(id, name, pos), veh_id_(assigned_vehicle), status_(status), location_id_(loc_id) {
	return;
}

void Customer::setStatus(Status s) {
	status_ = s;
}

CustomerStatus Customer::getStatus() const {
	return status_;
}

void Customer::setInVehicle() {
	status_ = IN_VEHICLE;
}

bool Customer::isInVehicle() {
	return (status_ == IN_VEHICLE || status_ == WAITING_FOR_DROPOFF);
}

void Customer::setAssignedVehicleId(int veh_id) {
	veh_id_ = veh_id;
}

void Customer::clearAssignedVehicleId() {
	veh_id_ = 0;
}

int Customer::getAssignedVehicleId() {
	return veh_id_;
}

int Customer::getLocationId() {
    return location_id_;
}

void Customer::setLocationId(int loc_id) {
    location_id_ = loc_id;
}

} /* namespace AMODBase */
