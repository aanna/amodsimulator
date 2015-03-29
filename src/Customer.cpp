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

Customer::Customer(int id, std::string name, amod::Position pos, int assigned_vehicle, bool in_vehicle) :
		Entity(id, name, pos), veh_id_(assigned_vehicle), in_vehicle_(in_vehicle) {
	return;
}

void Customer::setInVehicle(bool in_vehicle) {
	in_vehicle_ = in_vehicle;
}

bool Customer::isInVehicle() {
	return in_vehicle_;
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

} /* namespace AMODBase */
