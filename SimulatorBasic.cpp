/*
 * SimulatorBasic.cpp
 *
 *  Created on: Mar 27, 2015
 *      Author: haroldsoh
 */

#include "SimulatorBasic.hpp"

namespace amod {

SimulatorBasic::SimulatorBasic() {
	// TODO Auto-generated constructor stub

}

SimulatorBasic::~SimulatorBasic() {
	// TODO Auto-generated destructor stub
}

amod::ReturnCode  SimulatorBasic::init(amod::World *world_state) {
	state_ = *world_state;
}

amod::ReturnCode  SimulatorBasic::update(amod::World *world_state) {
	// updates the world state

	simulateVehicles(world_state);
	simulatePickups(world_state);
	simulateDropoffs(world_state);

}

void simulateVehicles(amod::World *world_state);
void simulatePickups(amod::World *world_state);
void simulateDropoffs(amod::World *world_state);


amod::ReturnCode SimulatorBasic::dispatchVehicle(int veh_id, const amod::Position &to) {

}

amod::ReturnCode SimulatorBasic::pickupCustomer(int veh_id, int cust_id) {

}

amod::ReturnCode SimulatorBasic::dropoffCustomer(int veh_id, int cust_id) {

}

amod::ReturnCode SimulatorBasic::serviceBooking(int veh_id, const amod::Booking &booking) {
	// TODO
	return amod::SUCCESS;
}

double SimulatorBasic::getDrivingDistance(const amod::Position &from, const amod::Position &to) {
	return getDistance(from, to);
}

double SimulatorBasic::getDistance(const amod::Position &from, const amod::Position &to) {
	return sqrt(pow( from.x - to.x ,2.0) + pow( from.y - to.y ,2.0));
}

} /* namespace amod */
