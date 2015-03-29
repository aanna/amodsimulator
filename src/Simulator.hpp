/*
 * Simulator.h
 *
 *  Created on: Mar 23, 2015
 *      Author: haroldsoh
 */

#ifndef Simulator_H_
#define Simulator_H_

#include "Types.hpp"
#include "Entity.hpp"
#include "Vehicle.hpp"
#include "World.hpp"
#include "Booking.hpp"
#include <vector>


namespace amod {

class Simulator {
public:
	Simulator() {};

	virtual ~Simulator() {};

	virtual amod::ReturnCode init(amod::World *world_state) = 0;
    virtual amod::ReturnCode update(amod::World *world_state) = 0;

    // low level commands
    virtual amod::ReturnCode dispatchVehicle(int veh_id, const amod::Position &to) = 0;
    virtual amod::ReturnCode pickupCustomer(int veh_id, int cust_id) = 0;
    virtual amod::ReturnCode dropoffCustomer(int veh_id, int cust_id) = 0;

    // med level commands
    virtual amod::ReturnCode serviceBooking(const amod::Booking &booking) = 0;

    // distance functions
    virtual double getDrivingDistance(const amod::Position &from, const amod::Position &to) = 0;
    virtual double getDistance(const amod::Position &from, const amod::Position &to) = 0;

};

} /* namespace AMOD */

#endif /* Simulator_H_ */
